use crate::devices::buzzer::{BUZZER_CHANNEL, BuzzerTask};
use crate::devices::hall_sensor_3144::{LEFT_TICKS_CUMULATIVE, RIGHT_TICKS_CUMULATIVE};
use crate::devices::motors::Motor;
use crate::flash_log;
use crate::positioning::CURRENT_POS;
use crate::positioning::odometry::{DISTANCE_PER_TICK, TICKS_PER_REVOLUTION};
use crate::trajectory::{TrajectorySegment, UPDATE_INTERVAL_MS};
use crate::utils::{CellMutexUtils, DurationUtils, HertzUtils, MathUtils};
use alloc::format;
use core::sync::atomic::Ordering::Relaxed;
use core::sync::atomic::{AtomicI32, AtomicU32};
use defmt::{debug, info, trace};
use embassy_stm32::peripherals::{TIM1, TIM2, TIM3};
use futures_util::future::err;
use core::f32::consts::PI;
use micromath::F32Ext;

const MAX_SPEED_M_S: f32 = 0.75;

// PI controller parameters — tune empirically on hardware
// Start with KI=0, raise KP until slight oscillation, back off ~30%, then add KI.
const KP: f32 = 0.13; // (m/s output) / (m/s error) — dimensionless
const KI: f32 = 0.00; // per tick; keep KI * typical_error << MAX_I

// ^ also consider tweaking V_ALPHA in fusion.rs when tweaking these consts ^
// (smoothes out speed from odometry)

// D term is intentionally omitted. The derivative term amplifies high-frequency noise in the
// velocity signal — and our velocity is quantized to 0–2 ticks per 20 ms window, making the
// sample-to-sample difference almost pure noise. A D term would produce large, random kick
// corrections every cycle and destabilize the loop. The EMA in fusion.rs already provides
// implicit damping (it smooths out sudden changes before they reach the controller), which
// gives the lag-reduction benefit of D without amplifying the quantization noise.

/// P correction capped at X m/s per tick
const MAX_P: f32 = 0.1;
/// I correction (integral anti-windup)
const MAX_I: f32 = 0.05;

/// Straight-line heading PI: keeps the robot on its initial bearing.
/// Negated because CW rotation (right turn) increases theta on this hardware (gyro z-axis down).
const KP_STRAIGHT: f32 = 0.8;
const KI_STRAIGHT: f32 = 0.3; // (m/s) per (rad·s) of accumulated heading error
const MAX_HEADING_INTEGRAL: f32 = 0.1; // caps integral wind-up at ~±0.1 m/s authority
const MAX_STEERING: f32 = 0.15;

/// We need this to stop at a precise distance
const DECELERATION_SPEED: f32 = 0.8;

/// Accelerates to max speed (if possible), maintains it, then decelerates.
/// Acceleration is done purely through PI control, deceleration is commanded gradually (still
/// through PI control to satisfy DECELERATION_SPEED.
/// Implementation is distance-based, not time-based
pub struct StraightLine {
    pub distance: f32,
    pub out_speed: f32,
}

impl TrajectorySegment for StraightLine {
    async fn execute<'a>(
        &self,
        motor_left: &mut Motor<'a, TIM3>,
        motor_right: &mut Motor<'a, TIM2>,
        override_start_speed: Option<f32>,
    ) {
        let mut motors = Motors {
            motor_left,
            motor_right,
        };

        let start_pos = CURRENT_POS.get();
        let initial_theta = start_pos.theta;
        let left_ticks_start = LEFT_TICKS_CUMULATIVE.load(Relaxed);
        let right_ticks_start = RIGHT_TICKS_CUMULATIVE.load(Relaxed);
        let target_ticks = (self.distance / DISTANCE_PER_TICK).round() as i32;
        let start_speed = override_start_speed
            .unwrap_or(start_pos.v_forward)
            .clamp(-MAX_SPEED_M_S, MAX_SPEED_M_S);

        // ΔE = mgΔx = 1/2 m Δv² -> Δx = (v_max² - v_final²) / 2a  (with g = a)
        let decel_distance_full_speed =
            (MAX_SPEED_M_S.square() - self.out_speed.square()) / (2.0 * DECELERATION_SPEED);
        let (max_reachable_speed, decel_distance) = if decel_distance_full_speed <= self.distance {
            (MAX_SPEED_M_S, decel_distance_full_speed)
        } else {
            // v_max² = 2aΔx + v_final²    (with Δx = self.distance)
            (
                (2.0 * DECELERATION_SPEED * self.distance + self.out_speed.square()).sqrt(),
                self.distance,
            )
        };
        let decel_start_distance = self.distance - decel_distance;

        let mut target_speed = max_reachable_speed;
        let mut commanded_speed = start_speed;
        let mut integral = 0.0f32;
        let mut heading_integral = 0.0f32;

        loop {
            let current_pos = CURRENT_POS.get();
            let left_ticks = LEFT_TICKS_CUMULATIVE.load(Relaxed) - left_ticks_start;
            let right_ticks = RIGHT_TICKS_CUMULATIVE.load(Relaxed) - right_ticks_start;
            // Tick-based distance: more accurate than EKF position for stop condition
            let avg_ticks = (left_ticks + right_ticks) / 2;
            let distance = avg_ticks as f32 * DISTANCE_PER_TICK;
            // Heading error: how much the robot has rotated from its initial direction.
            // Normalized to ±π so wrap-around near ±180° doesn't produce spurious corrections.
            let mut heading_error = current_pos.theta - initial_theta;
            if heading_error > PI { heading_error -= 2.0 * PI; }
            if heading_error < -PI { heading_error += 2.0 * PI; }
            heading_integral = (heading_integral + heading_error * (UPDATE_INTERVAL_MS as f32 / 1000.0))
                .clamp(-MAX_HEADING_INTEGRAL, MAX_HEADING_INTEGRAL);
            let steering = (-(KP_STRAIGHT * heading_error + KI_STRAIGHT * heading_integral))
                .clamp(-MAX_STEERING, MAX_STEERING);

            let speed_error = target_speed - current_pos.v_forward;

            if avg_ticks >= target_ticks {
                flash_log!(
                    "StraightLine ({}/{}m): target: {}, current: {}, error: {}, commanded: {}, P: 0.0000, I: 0.0000, steer: 0.0000",
                    format!("{:.2}", distance).as_str(),
                    format!("{:.2}", self.distance).as_str(),
                    format!("{:.2}", target_speed).as_str(),
                    format!("{:.2}", current_pos.v_forward).as_str(),
                    format!("{:.2}", speed_error).as_str(),
                    format!("{:.2}", commanded_speed).as_str(),
                );
                flash_log!(
                    "Distance reached: left {} ticks ({} turns), right {} ticks ({} turns), speed error: {}",
                    left_ticks,
                    left_ticks / TICKS_PER_REVOLUTION as i32,
                    right_ticks,
                    right_ticks / TICKS_PER_REVOLUTION as i32,
                    current_pos.v_forward - self.out_speed,
                );
                motors.set_speed(self.out_speed);
                break;
            }

            // update target speed
            if distance >= decel_start_distance {
                target_speed -= DECELERATION_SPEED * (UPDATE_INTERVAL_MS as f32 / 1000.0);
            }

            let proportional = (KP * speed_error).clamp(-MAX_P, MAX_P);
            integral = (integral + KI * speed_error).clamp(-MAX_I, MAX_I);
            commanded_speed = (commanded_speed + proportional + integral)
                .clamp(-1.5 * MAX_SPEED_M_S, 1.5 * MAX_SPEED_M_S);
            flash_log!(
                "StraightLine ({}/{}m): target: {}, current: {}, error: {}, commanded: {}, P: {}, I: {}, steer: {}, hdg: {}deg",
                format!("{:.2}", distance).as_str(),
                format!("{:.2}", self.distance).as_str(),
                format!("{:.2}", target_speed).as_str(),
                format!("{:.2}", current_pos.v_forward).as_str(),
                format!("{:.2}", speed_error).as_str(),
                format!("{:.2}", commanded_speed).as_str(),
                format!("{:.4}", proportional).as_str(),
                format!("{:.4}", integral).as_str(),
                format!("{:.4}", steering).as_str(),
                format!("{:.1}", heading_error.to_degrees()).as_str(),
            );

            motors.set_speed_steered(commanded_speed, steering);

            if speed_error < 0.0 && target_speed < max_reachable_speed {
                let _ = BUZZER_CHANNEL.try_send(BuzzerTask {
                    freq: 2000.hz(),
                    duration: 20.ms(),
                });
            }

            UPDATE_INTERVAL_MS.ms_timer().await;
        }
    }
}

struct Motors<'r, 'd> {
    motor_left: &'r mut Motor<'d, TIM3>,
    motor_right: &'r mut Motor<'d, TIM2>,
}

impl Motors<'_, '_> {
    fn set_speed(&mut self, speed: f32) {
        self.motor_left.set_speed(speed);
        self.motor_right.set_speed(speed);
    }

    /// Apply base speed with a steering correction: left gets -steering, right gets +steering.
    /// Positive steering turns left (right wheel faster); negative turns right.
    fn set_speed_steered(&mut self, speed: f32, steering: f32) {
        self.motor_left.set_speed(speed - steering);
        self.motor_right.set_speed(speed + steering);
    }
}
