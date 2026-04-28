use core::cell::Cell;
use defmt::error;
use crate::devices::hall_sensor_3144::{LEFT_TICKS_TOTAL, RIGHT_TICKS_TOTAL};
use core::f32::consts::PI;
use core::sync::atomic::{AtomicBool, Ordering};
use defmt::{error, trace, warn};
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::gpio::{Level, Output, Pin, Speed};
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_stm32::timer::{Channel, GeneralInstance4Channel};
use embassy_stm32::{Peri, peripherals};
use embassy_stm32::{Peri, PeripheralType, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel as SyncChannel;
use embassy_time::{Duration, Timer};
use core::f32::consts::{PI, TAU};
use crate::positioning::CURRENT_STATE;
use crate::positioning::odometry::{WHEEL_RADIUS, WHEEL_BASE, TICKS_PER_REVOLUTION};
use crate::utils::DurationUtils;

pub const DT: f32 = 0.02; // 20ms control loop iteration length

#[derive(Clone, Copy, Debug, Default)]
pub struct PathPoint {
    pub x: f32,
    pub y: f32,
    pub theta: f32,
}

pub static PATH_CHANNEL: SyncChannel<CriticalSectionRawMutex, PathPoint, 32> = SyncChannel::new();

/// Global flag set by the overcurrent protection task to immediately halt motors
pub static OVERCURRENT_FAULT: AtomicBool = AtomicBool::new(false);

use crate::positioning::odometry::{TICKS_PER_REVOLUTION, WHEEL_BASE, WHEEL_RADIUS};

// NOTE regarding concurrency design:
// By using `AtomicI32` for incremental odometry ticks we avoid dropped packets.
// For the planned Path tracking, an MPMC Channel acts as a perfect trajectory FIFO buffer.
// The navigation algorithms can stream upcoming `PathPoint`s.
// If the channel is full, the sender will suspend (await) until the motors consume points physically.

/// PI Controller Proportional Gain.
/// Pushes current directly against the immediate speed error gap.
/// If the robot gets pushed, or slowed down dynamically by a turn, Kp ramps PWM.
pub const KP: f32 = 0.5;

/// PI Controller Integral Gain.
/// Sums up persistent error over time to correct steady-state offsets.
/// Extremely helpful for ensuring the micromouse achieves the commanded velocity eventually
/// despite battery voltage sag preventing standard PWM ratios from reaching top speed.
pub const KI: f32 = 0.1;

fn angle(angle: f32) -> f32 {
    (angle + PI) % TAU - PI
}

/// Asynchronous task that consumes a trajectory over `PATH_CHANNEL`,
/// derives continuous PI-controlled PWM to both differential wheels,
/// and corrects physical deviations dynamically using Odometry atomic ticks.
#[embassy_executor::task]
pub async fn motor_controller_task(
    mut left_motor: Motor<'static, peripherals::TIM3>,
    mut right_motor: Motor<'static, peripherals::TIM2>,
) {
    let distance_per_tick = 2.0 * PI * WHEEL_RADIUS / TICKS_PER_REVOLUTION;

    let mut last_left_ticks = LEFT_TICKS_TOTAL.load(Ordering::Relaxed);
    let mut last_right_ticks = RIGHT_TICKS_TOTAL.load(Ordering::Relaxed);

    let mut left_integral = 0.0;
    let mut right_integral = 0.0;

    //let mut last_target = PathPoint::default();
    let mut has_first_point = false;
    let mut fault_logged = false;

    loop {
        if OVERCURRENT_FAULT.load(Ordering::Relaxed) {
            if !fault_logged {
                error!("Motors halted due to overcurrent fault!");
                fault_logged = true;
            }
            left_motor.brake();
            right_motor.brake();
            Timer::after(Duration::from_millis(100)).await;
            continue;
        }

        let mut target_lin = 0.0;
        let mut target_ang = 0.0;

        // Try reading next path point
        if let Ok(next_point) = PATH_CHANNEL.try_receive() {
            let state = CURRENT_STATE.lock(Cell::get);
            let estim_pos = PathPoint {x: state.x, y: state.y, theta: state.theta};

            if has_first_point {
                // Calculate required velocities to get from last target to this target in exactly DT.
                let dx = next_point.x - estim_pos.x;
                let dy = next_point.y - estim_pos.y;
                let d_theta = angle(next_point.theta - estim_pos.theta);

                target_lin = micromath::F32Ext::sqrt(dx * dx + dy * dy) / DT;
                // If robot goes backwards, we'd need a sign check here, but for micromouse forward splines:
                target_ang = d_theta / DT;
            }
            //last_target = next_point;
            has_first_point = true;
        } else {
            // Un-set if we run out of path points (stops smoothly)
            has_first_point = false;
            left_integral = 0.0;
            right_integral = 0.0;
        }

        // 1. Calculate Target Wheel Velocities
        // Differential drive kinematics: V_left = V - (W * d)/2, V_right = V + (W * d)/2
        let target_left_v = target_lin - (target_ang * WHEEL_BASE / 2.0);
        let target_right_v = target_lin + (target_ang * WHEEL_BASE / 2.0);

        // 2. Read actual ticks to get Current Wheel Velocities
        let current_left_ticks = LEFT_TICKS_TOTAL.load(Ordering::Relaxed);
        let current_right_ticks = RIGHT_TICKS_TOTAL.load(Ordering::Relaxed);

        let delta_left = current_left_ticks - last_left_ticks;
        let delta_right = current_right_ticks - last_right_ticks;

        last_left_ticks = current_left_ticks;
        last_right_ticks = current_right_ticks;

        // (Warning: With only 2 ticks per revolution, this measured velocity will be jittery
        //  on small dt. Real tests may require low-pass filtering this speed!)
        let actual_left_v = (delta_left as f32 * distance_per_tick) / DT;
        let actual_right_v = (delta_right as f32 * distance_per_tick) / DT;

        // 3. PI Controller for both wheels
        // --- Left Wheel Compute ---
        let left_error = target_left_v - actual_left_v;
        left_integral += left_error * DT;
        let left_out = (KP * left_error) + (KI * left_integral);

        // --- Right Wheel Compute ---
        let right_error = target_right_v - actual_right_v;
        right_integral += right_error * DT;
        let right_out = (KP * right_error) + (KI * right_integral);

        // 4. Apply to Motors
        // Stop completely if target is strictly exactly 0 to avoid jitter integration windup
        if target_lin == 0.0 && target_ang == 0.0 {
            left_motor.brake();
            right_motor.brake();
        } else {
            left_motor.set_speed(left_out.clamp(-1.0, 1.0));
            right_motor.set_speed(right_out.clamp(-1.0, 1.0));
        }

        // Wait for next control interval
        20.ms_timer().await;
    }
}

pub struct Motor<'d, T: GeneralInstance4Channel> {
    in_a: Output<'d>,
    in_b: Output<'d>,
    pwm: SimplePwm<'d, T>,
    pwm_channel: Channel,
}

impl<'d, T: GeneralInstance4Channel> Motor<'d, T> {
    pub fn new(
        in_a_pin: Peri<'d, impl Pin>,
        in_b_pin: Peri<'d, impl Pin>,
        pwm: SimplePwm<'d, T>,
        pwm_channel: Channel,
    ) -> Self {
        let in_a = Output::new(in_a_pin, Level::Low, Speed::Medium);
        let in_b = Output::new(in_b_pin, Level::Low, Speed::Medium);

        Self {
            in_a,
            in_b,
            pwm,
            pwm_channel,
        }
    }

    pub fn set_speed(&mut self, speed: f32) {
        assert!(
            -1.0 <= speed && speed <= 1.0,
            "Speed must be between -1.0 and 1.0"
        );

        if speed == 0.0 {
            self.brake();
            return;
        }

        if speed > 0.0 {
            self.in_a.set_high();
            self.in_b.set_low();
        } else {
            self.in_a.set_low();
            self.in_b.set_high();
        }

        // --- DEADBAND COMPENSATION ---
        // DC Motors cannot run below a certain PWM threshold (approx 20%).
        // We remap the logical (0.0, 1.0] speed to [MIN_PWM, 1.0].
        const MIN_PWM: f32 = 0.20;
        let abs_speed = speed.abs();
        let effective_speed = MIN_PWM + abs_speed * (1.0 - MIN_PWM);

        let duty = (self.pwm.max_duty_cycle() as f32 * effective_speed) as u32;

        let mut pwm_channel = self.pwm.channel(self.pwm_channel);
        pwm_channel.set_duty_cycle(duty);

        if !pwm_channel.is_enabled() {
            pwm_channel.enable();
        }
    }

    pub fn brake(&mut self) {
        self.in_a.set_high();
        self.in_b.set_high();
        let mut pwm_channel = self.pwm.channel(self.pwm_channel);
        pwm_channel.disable();
        pwm_channel.set_duty_cycle(0);
    }

    pub fn neutral(&mut self) {
        self.in_a.set_low();
        self.in_b.set_low();
        let mut pwm_channel = self.pwm.channel(self.pwm_channel);
        pwm_channel.disable();
        pwm_channel.set_duty_cycle(0);
    }
}

#[embassy_executor::task]
pub async fn overcurrent_protection_task(
    mut adc_module: Adc<'static, peripherals::ADC2>,
    mut sense_motor1: Peri<'static, peripherals::PA0>,
    mut sense_motor2: Peri<'static, peripherals::PA1>,
) -> ! {
    // Constantes basées sur la datasheet VNH2SP30
    const K: f32 = 11370.0; // Ratio typique
    const R_SENSE: f32 = 1500.0; // Résistance sur le shield en Ohms
    const ADC_MAX: f32 = 4095.0; // Résolution 12-bit
    const V_REF: f32 = 3.3; // Tension de référence Nucleo

    loop {
        // Motor 1
        let mut max_raw1 = 0;
        for _ in 0..20 {
            let raw = adc_module.blocking_read(&mut sense_motor1, SampleTime::CYCLES144);
            if raw > max_raw1 {
                max_raw1 = raw;
            }
        }

        // Motor 2
        let mut max_raw2 = 0;
        for _ in 0..20 {
            let raw = adc_module.blocking_read(&mut sense_motor2, SampleTime::CYCLES144);
            if raw > max_raw2 {
                max_raw2 = raw;
            }
        }

        let v_sense1 = (max_raw1 as f32 / ADC_MAX) * V_REF;
        let v_sense2 = (max_raw2 as f32 / ADC_MAX) * V_REF;

        let current_amps1 = (v_sense1 / R_SENSE) * K;
        let current_amps2 = (v_sense2 / R_SENSE) * K;

        if current_amps1 > 4.0 || current_amps2 > 4.0 {
            error!("Overcurrent ! M1: {} A, M2: {} A", current_amps1, current_amps2);
            OVERCURRENT_FAULT.store(true, Ordering::Relaxed);

            // Wait 5 seconds before attempting recovery
            Timer::after(Duration::from_secs(5)).await;

            // Optional auto-recovery:
            // OVERCURRENT_FAULT.store(false, Ordering::Relaxed);
        }

        20.ms_timer().await;
    }
}
