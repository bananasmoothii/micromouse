use crate::devices::mpu9250::LATEST_MPU;
use crate::positioning::odometry::{WHEEL_BASE, left_wheel_velocity, right_wheel_velocity};
use crate::positioning::CURRENT_STATE;
use core::cell::Cell;
use micromath::F32Ext;
use crate::utils::DurationUtils;
use core::f32::consts::{PI, TAU};
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use defmt::error;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::gpio::{Level, Output, Pin, Speed};
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_stm32::timer::{Channel, GeneralInstance4Channel};
use embassy_stm32::{Peri, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel as SyncChannel;
use embassy_time::{Duration, Instant, Timer};

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

// NOTE regarding concurrency design:
// By using `AtomicI32` for incremental odometry ticks we avoid dropped packets.
// For the planned Path tracking, an MPMC Channel acts as a perfect trajectory FIFO buffer.
// The navigation algorithms can stream upcoming `PathPoint`s.
// If the channel is full, the sender will suspend (await) until the motors consume points physically.

/// PI Controller Proportional Gain.
/// Pushes current directly against the immediate speed error gap.
/// If the robot gets pushed, or slowed down dynamically by a turn, Kp ramps PWM.
pub const KP: f32 = 0.20;

/// PI Controller Integral Gain.
/// Sums up persistent error over time to correct steady-state offsets.
/// Extremely helpful for ensuring the micromouse achieves the commanded velocity eventually
/// despite battery voltage sag preventing standard PWM ratios from reaching top speed.
pub const KI: f32 = 0.02;

/// Feedforward gain: maps desired wheel linear velocity (m/s) to an estimated
/// PWM command in the -1.0..1.0 range. Multiply a wheel velocity by this
/// constant to obtain the open-loop PWM that would approximately produce that
/// speed on the motors. The PI controller then corrects residual error.
///
/// This value is hardware-dependent and should be tuned experimentally.
/// As a starting point it assumes 1.0 PWM corresponds to ~0.8 m/s wheel speed
/// (so FF_V_TO_PWM = 1.0 / 0.8 = 1.25).
// Feedforward stored as atomic bits so it can be calibrated at runtime.
// Tune: if robot is too slow increase, if too fast decrease (= 1/max_speed_at_full_pwm).
static FF_V_TO_PWM_BITS: AtomicU32 = AtomicU32::new(0.2f32.to_bits());

#[inline]
pub fn get_ff_v_to_pwm() -> f32 {
    f32::from_bits(FF_V_TO_PWM_BITS.load(Ordering::Relaxed))
}

pub fn set_ff_v_to_pwm(v: f32) {
    FF_V_TO_PWM_BITS.store(v.to_bits(), Ordering::Relaxed);
}

fn angle(angle: f32) -> f32 {
    (angle + PI) % TAU - PI
}

/// Output PWM smoothing: EMA weight on the freshly-computed raw PWM.
/// Lower = slower but smoother; higher = more responsive but noisier.
/// Equivalent time constant: tau = DT * (1 - EMA_ALPHA) / EMA_ALPHA.
/// At 0.25: tau ≈ 60 ms.  At 0.40: tau ≈ 30 ms.
const EMA_ALPHA: f32 = 0.30;

/// Velocity measurement smoothing before PI.
/// Prevents per-tick estimation jitter (2 ticks/rev → v oscillates as elapsed grows)
/// from driving hard PI corrections.  Higher = faster response but noisier error signal.
const V_EMA_ALPHA: f32 = 0.40;

/// Position correction gains (added on top of trajectory velocity).
/// Forward error (robot behind waypoint) → add forward speed.
/// Lateral error → steer toward waypoint.
/// Heading error → correct orientation.
const KP_FWD: f32 = 0.6; // (m/s) per meter behind
const KP_LAT: f32 = 0.8; // (rad/s) per meter lateral offset — reduced to limit noise from EKF jitter
const KP_HDG: f32 = 0.6; // (rad/s) per radian heading error

/// If the robot is more than this far behind its current waypoint, pause consuming
/// new waypoints so the trajectory doesn't race ahead of reality.
const MAX_LAG_M: f32 = 0.12;

/// Gyro angular rate feedback gain. Scales the difference between commanded and
/// actual yaw rate (rad/s) to an additive angular velocity correction (rad/s).
/// Increase to react more aggressively to wheel imbalance; decrease if oscillatory.
const K_ANG: f32 = 0.10;

#[embassy_executor::task]
pub async fn motor_controller_task(
    mut left_motor: Motor<'static, peripherals::TIM3>,
    mut right_motor: Motor<'static, peripherals::TIM2>,
) {
    let mut left_integral = 0.0f32;
    let mut right_integral = 0.0f32;
    let mut last_waypoint: Option<PathPoint> = None;
    let mut prev_left_out = 0.0f32;
    let mut prev_right_out = 0.0f32;
    let mut smooth_left_v = 0.0f32;
    let mut smooth_right_v = 0.0f32;
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

        let mut target_lin = 0.0f32;
        let mut target_ang = 0.0f32;

        // Read robot position once per loop for lag check and position correction.
        let state = CURRENT_STATE.lock(Cell::get);
        let (sin_t, cos_t) = state.theta.sin_cos();

        // Only advance to the next waypoint if the robot is not too far behind the
        // current one. This keeps the trajectory from racing ahead of reality.
        let should_advance = match &last_waypoint {
            None => true,
            Some(wp) => {
                let lag = (wp.x - state.x) * cos_t + (wp.y - state.y) * sin_t;
                lag < MAX_LAG_M
            }
        };

        if should_advance {
            if let Ok(next_point) = PATH_CHANNEL.try_receive() {
                if let Some(lp) = last_waypoint {
                    let dx = next_point.x - lp.x;
                    let dy = next_point.y - lp.y;
                    target_lin = micromath::F32Ext::sqrt(dx * dx + dy * dy) / DT;
                    target_ang = angle(next_point.theta - lp.theta) / DT;
                }
                last_waypoint = Some(next_point);
            } else {
                // Trajectory finished.
                last_waypoint = None;
                left_integral = 0.0;
                right_integral = 0.0;
                smooth_left_v = 0.0;
                smooth_right_v = 0.0;
            }
        }
        // When should_advance = false, target stays 0 and position correction below
        // drives the robot toward its current waypoint at a controlled rate.

        // Soft position correction toward the current waypoint.
        // Adds to (not replaces) the trajectory velocity so corrections are bounded.
        if let Some(wp) = &last_waypoint {
            let err_x = wp.x - state.x;
            let err_y = wp.y - state.y;
            let err_fwd = err_x * cos_t + err_y * sin_t;
            let err_lat = -err_x * sin_t + err_y * cos_t;
            let err_hdg = angle(wp.theta - state.theta);

            target_lin = (target_lin + KP_FWD * err_fwd).clamp(0.0f32, 0.8f32);
            target_ang = (target_ang + KP_LAT * err_lat + KP_HDG * err_hdg).clamp(-3.0f32, 3.0f32);
        }

        // Gyro angular rate feedback: correct target_ang with the difference between
        // commanded and actual yaw rate. gyro[2] is negated to match our coord convention
        // (IMU is mounted upside-down; fusion.rs applies the same sign flip).
        let actual_omega = LATEST_MPU
            .lock(|c| c.get())
            .map(|d| -d.gyro[2])
            .unwrap_or(0.0);
        let corrected_ang = target_ang + K_ANG * (target_ang - actual_omega);

        // Per-wheel velocity targets from differential drive kinematics.
        let target_left_v = target_lin - corrected_ang * WHEEL_BASE / 2.0;
        let target_right_v = target_lin + corrected_ang * WHEEL_BASE / 2.0;

        // Actual wheel velocities from inter-tick timestamps, then low-pass filtered.
        // Filtering before the PI prevents per-tick velocity estimation jitter
        // (elapsed_us grows every frame even without a new tick) from driving hard corrections.
        let now_us = Instant::now().as_micros() as u32;
        smooth_left_v = V_EMA_ALPHA * left_wheel_velocity(now_us)
            + (1.0 - V_EMA_ALPHA) * smooth_left_v;
        smooth_right_v = V_EMA_ALPHA * right_wheel_velocity(now_us)
            + (1.0 - V_EMA_ALPHA) * smooth_right_v;

        // Feedforward + PI (PI error uses smoothed velocity to suppress noise)
        let ff = get_ff_v_to_pwm();

        let left_error = target_left_v - smooth_left_v;
        left_integral = (left_integral + left_error * DT).clamp(-0.5, 0.5);
        let left_raw = target_left_v * ff + KP * left_error + KI * left_integral;

        let right_error = target_right_v - smooth_right_v;
        right_integral = (right_integral + right_error * DT).clamp(-0.5, 0.5);
        let right_raw = target_right_v * ff + KP * right_error + KI * right_integral;

        // EMA output filter: smooths the physical PWM command, removing frame-to-frame jitter.
        // Direction lock: output sign must match target sign to prevent the PI from reversing
        // a forward-commanded wheel during deceleration.
        let left_out = (EMA_ALPHA * left_raw + (1.0 - EMA_ALPHA) * prev_left_out)
            .clamp(
                if target_left_v < 0.0 { -1.0 } else { 0.0 },
                if target_left_v > 0.0 { 1.0 } else { 0.0 },
            );
        let right_out = (EMA_ALPHA * right_raw + (1.0 - EMA_ALPHA) * prev_right_out)
            .clamp(
                if target_right_v < 0.0 { -1.0 } else { 0.0 },
                if target_right_v > 0.0 { 1.0 } else { 0.0 },
            );

        if last_waypoint.is_none() {
            left_motor.brake();
            right_motor.brake();
            prev_left_out = 0.0;
            prev_right_out = 0.0;
        } else {
            left_motor.set_speed(left_out);
            right_motor.set_speed(right_out);
            prev_left_out = left_out;
            prev_right_out = right_out;
        }

        20.ms_timer().await;
    }
}

pub struct Motor<'d, T: GeneralInstance4Channel> {
    in_a: Output<'d>,
    in_b: Output<'d>,
    pwm: SimplePwm<'d, T>,
    pwm_channel: Channel,
    forward_flag: &'static AtomicBool,
}

impl<'d, T: GeneralInstance4Channel> Motor<'d, T> {
    pub fn new(
        in_a_pin: Peri<'d, impl Pin>,
        in_b_pin: Peri<'d, impl Pin>,
        pwm: SimplePwm<'d, T>,
        pwm_channel: Channel,
        forward_flag: &'static AtomicBool,
    ) -> Self {
        let in_a = Output::new(in_a_pin, Level::Low, Speed::Medium);
        let in_b = Output::new(in_b_pin, Level::Low, Speed::Medium);

        Self {
            in_a,
            in_b,
            pwm,
            pwm_channel,
            forward_flag,
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
            self.forward_flag.store(true, Ordering::Relaxed);
            self.in_a.set_high();
            self.in_b.set_low();
        } else {
            self.forward_flag.store(false, Ordering::Relaxed);
            self.in_a.set_low();
            self.in_b.set_high();
        }

        // --- DEADBAND COMPENSATION ---
        // DC Motors cannot run below a certain PWM threshold (approx 20%).
        // We remap the logical (0.0, 1.0] speed to [MIN_PWM, 1.0].
        const MIN_PWM: f32 = 0.10;
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

/*/// Calibration task: measures wheel speed vs PWM using the left wheel only and
/// computes a single feedforward value used for both wheels. This simplifies
/// the calibration under the assumption left/right FF are equivalent.
#[embassy_executor::task]
pub async fn ff_calibration_task(
    mut left_motor: Motor<'static, peripherals::TIM3>,
    mut _right_motor: Motor<'static, peripherals::TIM2>,
) {
    // Logical PWM steps to test (above expected deadband). Adjust if needed.
    const STEPS: [f32; 4] = [0.25, 0.40, 0.60, 0.80];
    const SETTLE_MS: u64 = 300;
    const MEASURE_MS: u64 = 500;

    let distance_per_tick = 2.0 * PI * WHEEL_RADIUS / TICKS_PER_REVOLUTION;

    // Helper to average valid slopes
    fn compute_average_slope(slopes: &[f32]) -> Option<f32> {
        let mut sum = 0.0f32;
        let mut cnt = 0u32;
        for &s in slopes.iter() {
            if s > 1e-6 {
                sum += s;
                cnt += 1;
            }
        }
        if cnt == 0 { None } else { Some(sum / (cnt as f32)) }
    }

    let mut left_slopes = [0.0f32; 4];
    let mut left_count = 0usize;

    for &pwm in STEPS.iter() {
        left_motor.set_speed(pwm);
        Timer::after(Duration::from_millis(SETTLE_MS)).await;

        let t0 = LEFT_TICKS_TOTAL.load(Ordering::Relaxed);
        Timer::after(Duration::from_millis(MEASURE_MS)).await;
        let t1 = LEFT_TICKS_TOTAL.load(Ordering::Relaxed);

        let delta = t1 - t0;
        if delta > 0 {
            let vel = (delta as f32 * distance_per_tick) / (MEASURE_MS as f32 / 1000.0);
            let slope = vel / pwm;
            if left_count < left_slopes.len() {
                left_slopes[left_count] = slope;
                left_count += 1;
            }
            trace!("FF calib left: pwm={} vel={} slope={}", pwm, vel, slope);
        } else {
            trace!("FF calib left: pwm={} no ticks", pwm);
        }
        left_motor.brake();
        Timer::after(Duration::from_millis(100)).await;
    }

    let left_avg = compute_average_slope(&left_slopes[..left_count]);

    if let Some(ls) = left_avg {
        if ls > 1e-6 {
            let ff = 1.0 / ls;
            set_ff_v_to_pwm(ff);
            trace!("FF calibration complete: ff={} (left_slope={})", ff, ls);
        } else {
            warn!("FF calibration failed: left slope too small");
        }
    } else {
        warn!("FF calibration failed: no valid slopes measured");
    }

    // Ensure motors are stopped at the end
    left_motor.neutral();
}
*/
#[embassy_executor::task]
pub async fn overcurrent_protection_task(
    mut adc_module: Adc<'static, peripherals::ADC2>,
    mut sense_motor1: Peri<'static, peripherals::PA4>,
    mut sense_motor2: Peri<'static, peripherals::PB0>,
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

        // Both channels at saturation → sense pins are floating (motor driver unpowered).
        // A real overcurrent on small motors stays well below 15 A.
        // if current_amps1 < 15.0 || current_amps2 < 15.0 {
        if current_amps1 > 4.0 || current_amps2 > 4.0 {
            error!("Overcurrent ! M1: {} A, M2: {} A", current_amps1, current_amps2);
            OVERCURRENT_FAULT.store(true, Ordering::Relaxed);

            // Wait 5 seconds before attempting recovery
            Timer::after(Duration::from_secs(5)).await;

            // Optional auto-recovery:
            // OVERCURRENT_FAULT.store(false, Ordering::Relaxed);
        }
        // }

        20.ms_timer().await;
    }
}
