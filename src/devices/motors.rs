use core::cell::Cell;
use defmt::{error, trace, warn};
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::gpio::{Level, Output, Pin, Speed};
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_stm32::timer::{Channel, GeneralInstance4Channel};
use embassy_stm32::{Peri, peripherals, PeripheralType};
use embassy_sync::channel::Channel as SyncChannel;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use crate::devices::hall_sensor_3144::{LEFT_TICKS_TOTAL, RIGHT_TICKS_TOTAL};
use core::sync::atomic::Ordering;
use embassy_time::{Duration, Timer};
use core::f32::consts::{PI, TAU};
use crate::positioning::CURRENT_STATE;

pub const DT: f32 = 0.02; // 20ms control loop iteration length

#[derive(Clone, Copy, Debug, Default)]
pub struct PathPoint {
    pub x: f32,
    pub y: f32,
    pub theta: f32,
}

pub static PATH_CHANNEL: SyncChannel<CriticalSectionRawMutex, PathPoint, 32> = SyncChannel::new();

use crate::positioning::odometry::{WHEEL_RADIUS, WHEEL_BASE, TICKS_PER_REVOLUTION};

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
    mut right_motor: Motor<'static, peripherals::TIM4>, // assuming TIM4 or whatever they are typed
) {
    let distance_per_tick = 2.0 * PI * WHEEL_RADIUS / TICKS_PER_REVOLUTION;

    let mut last_left_ticks = LEFT_TICKS_TOTAL.load(Ordering::Relaxed);
    let mut last_right_ticks = RIGHT_TICKS_TOTAL.load(Ordering::Relaxed);

    let mut left_integral = 0.0;
    let mut right_integral = 0.0;

    //let mut last_target = PathPoint::default();
    let mut has_first_point = false;

    loop {
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
        Timer::after(Duration::from_millis(20)).await;
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
        spawner: &Spawner,
        in_a_pin: Peri<'d, impl Pin>,
        in_b_pin: Peri<'d, impl Pin>,
        pwm: SimplePwm<'d, T>,
        pwm_channel: Channel,
        adc_module: Adc<'static, peripherals::ADC2>,
        current_sense_pin: Peri<'static, peripherals::PA4>,
        enable_pin: Peri<'static, impl Pin>,
    ) -> Self {
        let in_a = Output::new(in_a_pin, Level::Low, Speed::Medium);
        let in_b = Output::new(in_b_pin, Level::Low, Speed::Medium);
        let enable_output = Output::new(enable_pin, Level::Low, Speed::Medium);

        spawner
            .spawn(overcurrent_protection_task(
                adc_module,
                current_sense_pin,
                enable_output,
            ))
            .unwrap();

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

        let duty = (self.pwm.max_duty_cycle() as f32 * speed.abs()) as u32;

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
    mut current_sense_pin: Peri<'static, peripherals::PA4>,
    mut enable_pin: Output<'static>,
) -> ! {
    enable_pin.set_high();

    // Constantes basées sur la datasheet VNH2SP30
    const K: f32 = 11370.0; // Ratio typique
    const R_SENSE: f32 = 1500.0; // Résistance sur le shield en Ohms
    const ADC_MAX: f32 = 4095.0; // Résolution 12-bit
    const V_REF: f32 = 3.3; // Tension de référence Nucleo

    loop {
        // Read the raw 12-bit value from the ADC
        // Take a burst of 10 samples to ensure we catch the PWM "ON" phase
        let mut max_raw = 0;
        for _ in 0..20 {
            let raw = adc_module.blocking_read(&mut current_sense_pin, SampleTime::CYCLES144);
            if raw > max_raw {
                max_raw = raw;
            }
        }

        // 1. Convert raw ADC value to Voltage
        // $V_{SENSE} = (raw / 4095) * 3.3V$
        let v_sense = (max_raw as f32 / ADC_MAX) * V_REF;

        // 2. Convert Voltage to Sense Current (using Ohm's Law: I = V/R)
        // $I_{SENSE} = V_{SENSE} / 1500\Omega$
        let i_sense = v_sense / R_SENSE;

        // 3. Convert Sense Current to actual Motor Current
        // $I_{OUT} = I_{SENSE} * K$
        let current_amps = i_sense * K;

        if current_amps > 4.0 {
            error!("Overcurrent ! {} A (raw: {})", current_amps, max_raw);
            enable_pin.set_low();
            panic!("Overcurrent detected");
        }

        // trace!("current: {} A (raw: {})", current_amps, max_raw);

        Timer::after(Duration::from_millis(20)).await;
    }
}
