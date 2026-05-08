use crate::devices::motors::Motor;
use crate::positioning::CURRENT_POS;
use crate::trajectory::{TrajectorySegment, UPDATE_INTERVAL_MS};
use crate::utils::{CellMutexUtils, DurationUtils, MathUtils};
use alloc::format;
use core::sync::atomic::{AtomicI32, AtomicU32};
use core::sync::atomic::Ordering::Relaxed;
use defmt::debug;
use embassy_stm32::peripherals::{TIM1, TIM2, TIM3};

const MAX_SPEED_M_S: f32 = 0.5;
const ACCELERATION_M_S2: f32 = 0.5; // and deceleration

// PI controller parameters — tune empirically on hardware
// Start with KI=0, raise KP until slight oscillation, back off ~30%, then add KI.
const KP: f32 = 0.5;  // (m/s output) / (m/s error) — dimensionless
const KI: f32 = 0.02; // per tick; keep KI * typical_error << MAX_I

/// P correction capped at ±0.1 m/s per tick
const MAX_P: f32 = 0.1;
/// I correction capped at ±0.05 m/s (integral anti-windup)
const MAX_I: f32 = 0.05;

/// Accelerates to max speed, maintains it, then decelerates.
/// This results in a trapezoidal speed profile.
/// Implementation is distance-based, not time-based
pub struct StraightLine {
    pub distance: f32,
    pub out_speed: f32,
}

static LAST_INTEGRAL: AtomicU32 = AtomicU32::new(0f32.to_bits());

impl TrajectorySegment for StraightLine {
    async fn execute(&self, motor_left: &mut Motor<'_, TIM3>, motor_right: &mut Motor<'_, TIM2>, override_start_speed: Option<f32>) {
        let start_pos = CURRENT_POS.get();
        let start_speed = override_start_speed.unwrap_or(start_pos.v_forward);

        // also for deceleration
        let max_accel_time = (MAX_SPEED_M_S - start_speed) / ACCELERATION_M_S2;
        // basic integration
        let max_accel_distance = ACCELERATION_M_S2 / 2.0 * max_accel_time.square()
            + start_speed * max_accel_time;

        let max_decel_time = (MAX_SPEED_M_S - self.out_speed) / ACCELERATION_M_S2;
        let max_decel_distance =
            ACCELERATION_M_S2 / 2.0 * max_decel_time.square() + self.out_speed * max_decel_time;

        let accel_distance = if max_accel_distance + max_decel_distance <= self.distance {
            max_accel_distance
        } else {
            // see straigh_line_math.md
            self.distance / 2.0
                + (self.out_speed.square() - start_speed.square())
                / (4.0 * ACCELERATION_M_S2)
        };
        let decel_start_distance = if max_accel_distance + max_decel_distance <= self.distance {
            self.distance - max_decel_distance
        } else {
            accel_distance
        };

        let v_change_per_tick = ACCELERATION_M_S2 * UPDATE_INTERVAL_MS as f32 / 1000.0;

        let mut integral = f32::from_bits(LAST_INTEGRAL.load(Relaxed));

        let mut target_speed = start_pos.v_forward;
        let mut i = 0u32;
        loop {
            let current_pos = CURRENT_POS.get();
            let distance = current_pos.distance_from(&start_pos);

            if distance >= self.distance {
                break;
            }

            // update target speed
            let mut speed_set = true;
            if distance < accel_distance {
                target_speed += v_change_per_tick;
            } else if distance >= decel_start_distance {
                target_speed -= v_change_per_tick;
            } else {
                speed_set = false;
            }

            let speed_error = target_speed - current_pos.v_forward;

            if speed_error.abs() >= 0.04 && i >= 3 {
                // Use PI control
                let proportional = (KP * speed_error).clamp(-MAX_P, MAX_P);
                integral += KI * speed_error;
                integral = integral.clamp(-MAX_I, MAX_I);
                let new_speed = target_speed + proportional + integral;
                debug!(
                    "StraightLine: PI control: target_speed: {} m/s, current_speed: {} m/s, error: {} m/s, new_speed: {} m/s, P: {} m/s, I: {} m/s",
                    format!("{:.2}", target_speed).as_str(),
                    format!("{:.2}", current_pos.v_forward).as_str(),
                    format!("{:.2}", speed_error).as_str(),
                    format!("{:.2}", new_speed).as_str(),
                    format!("{:.6}", proportional).as_str(),
                    format!("{:.6}", integral).as_str()
                );

                motor_left.set_speed(new_speed);
                motor_right.set_speed(new_speed);
            } else if speed_set {
                // use standard feedforward - more reactive
                debug!(
                    "StraightLine: standard feedforward: target_speed: {} m/s",
                    format!("{:.2}", target_speed).as_str()
                );
                motor_left.set_speed(target_speed);
                motor_right.set_speed(target_speed);
            }

            UPDATE_INTERVAL_MS.ms_timer().await;
            i += 1;
        }
        LAST_INTEGRAL.store(integral.to_bits(), Relaxed);
    }
}
