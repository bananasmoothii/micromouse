use crate::devices::buzzer::{BUZZER_CHANNEL, BuzzerTask};
use crate::devices::motors::Motor;
use crate::positioning::CURRENT_POS;
use crate::trajectory::{TrajectorySegment, UPDATE_INTERVAL_MS};
use crate::utils::{CellMutexUtils, DurationUtils, HertzUtils, MathUtils};
use alloc::format;
use core::sync::atomic::Ordering::Relaxed;
use core::sync::atomic::{AtomicI32, AtomicU32};
use defmt::{debug, trace};
use embassy_stm32::peripherals::{TIM1, TIM2, TIM3};
use futures_util::future::err;
use micromath::F32Ext;

const MAX_SPEED_M_S: f32 = 0.5;

// PI controller parameters — tune empirically on hardware
// Start with KI=0, raise KP until slight oscillation, back off ~30%, then add KI.
const KP: f32 = 0.05; // (m/s output) / (m/s error) — dimensionless
const KI: f32 = 0.00; // per tick; keep KI * typical_error << MAX_I

/// P correction capped at X m/s per tick
const MAX_P: f32 = 0.1;
/// I correction (integral anti-windup)
const MAX_I: f32 = 0.05;

/// Accelerates to max speed, maintains it, then decelerates.
/// This results in a trapezoidal speed profile.
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
        let start_speed = override_start_speed
            .unwrap_or(start_pos.v_forward)
            .clamp(-MAX_SPEED_M_S, MAX_SPEED_M_S);

        // Effective deceleration the PI can sustain: max correction per tick × tick rate.
        // Tuning MAX_P or MAX_I automatically adjusts the braking distance.
        let effective_decel_m_s2: f32 = (MAX_P + MAX_I) / (UPDATE_INTERVAL_MS as f32 / 1000.0);

        // ΔE = mgΔx = 1/2 m Δv² -> Δx = (v_max² - v_final²) / 2a  (with g = a)
        let decel_distance_full_speed =
            (MAX_SPEED_M_S.square() - self.out_speed.square()) / (2.0 * effective_decel_m_s2);
        let (max_reachable_speed, decel_distance) = if decel_distance_full_speed <= self.distance {
            (MAX_SPEED_M_S, decel_distance_full_speed)
        } else {
            // v_max² = 2aΔx + v_final²    (with Δx = self.distance)
            (
                (2.0 * effective_decel_m_s2 * self.distance + self.out_speed.square()).sqrt(),
                self.distance,
            )
        };

        let mut target_speed = max_reachable_speed;
        let mut commanded_speed = start_speed;
        let mut integral = 0.0f32;

        loop {
            let current_pos = CURRENT_POS.get();
            let distance = current_pos.distance_from(&start_pos);

            if distance >= self.distance {
                break;
            }

            // update target speed
            if distance >= self.distance - decel_distance {
                target_speed = self.out_speed;
            }
            let speed_error = target_speed - current_pos.v_forward;

            let proportional = (KP * speed_error).clamp(-MAX_P, MAX_P);
            integral = (integral + KI * speed_error).clamp(-MAX_I, MAX_I);
            commanded_speed = (commanded_speed + proportional + integral);
            // .clamp(-max_reachable_speed, max_reachable_speed);
            debug!(
                "StraightLine ({}/{}m): target_speed: {} m/s, current_speed: {} m/s, error: {} m/s, commanded_speed: {} m/s, P: {} m/s, I: {} m/s",
                format!("{:.2}", distance).as_str(),
                format!("{:.2}", self.distance).as_str(),
                format!("{:.2}", target_speed).as_str(),
                format!("{:.2}", current_pos.v_forward).as_str(),
                format!("{:.2}", speed_error).as_str(),
                format!("{:.2}", commanded_speed).as_str(),
                format!("{:.4}", proportional).as_str(),
                format!("{:.4}", integral).as_str(),
            );

            motors.set_speed(commanded_speed);

            if speed_error < 0.0 {
                let _ = BUZZER_CHANNEL.try_send(BuzzerTask {
                    freq: (((speed_error + 1.5) * 500.0) as u32).hz(),
                    duration: 10.ms(),
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
}
