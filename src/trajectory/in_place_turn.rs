use alloc::boxed::Box;
use crate::devices::motors::Motor;
use crate::flash_log;
use crate::positioning::CURRENT_POS;
use crate::positioning::odometry::WHEEL_BASE;
use crate::trajectory::{FusionMode, TrajectorySegment, UPDATE_INTERVAL_MS, set_current_fusion_mode};
use crate::utils::{CellMutexUtils, DurationUtils};
use alloc::format;
use core::f32::consts::PI;
use embassy_stm32::peripherals::{TIM2, TIM3};

// Position-form PI on angle error → wheel tangential speed (m/s per radian).
const KP_TURN: f32 = 0.6;
const KI_TURN: f32 = 0.4; // (m/s) per (rad·s) of accumulated angle error

// Anti-windup: caps the integral contribution so it can't drive speed beyond max alone.
const MAX_INTEGRAL_CONTRIB: f32 = 0.2; // m/s

// Speed is clamped between MIN_TURN_SPEED and MAX_TURN_SPEED.
// Empirically: both motors need at least 0.75 m/s to overcome static friction in the
// backward direction during a pivot turn.
const MIN_TURN_SPEED: f32 = 0.9;  // needed to break static friction at turn start
const MAX_TURN_SPEED: f32 = 1.0;
const APPROACH_SPEED: f32 = 0.5;  // gentler final approach; less inertia at motor cutoff

// Within FINAL_ZONE_RAD of the target, switch to APPROACH_SPEED.
const FINAL_ZONE_RAD: f32 = 30f32.to_radians();

// Empirical angular deceleration after motor cut (Coulomb friction model: coast = ω²/2α).
// Recalibrated from log: cut at reported ω≈23 rad/s, actual coast≈30.9° → α_real≈504.
// α is set BELOW α_real so the cut triggers one 20 ms iteration earlier: the loop's
// discretisation makes remaining jump from ~31° to ~4° in one step, so inflating coast_est
// is the only way to catch the earlier iteration. Tune: overshoot → lower α, undershoot → raise α.
const FRICTION_DECEL: f32 = 520.0; // rad/s²

// Minimum cut-early angle used when ω is near zero (avoids never stopping).
const DONE_THRESHOLD_MIN_RAD: f32 = 3f32.to_radians();

/// In-place (pivot) turn: left and right wheels spin in opposite directions.
/// Turns at a tangent speed between MIN_TURN_SPEED and MAX_TURN_SPEED
///
/// `angle`: target rotation in radians. Positive = CCW (left), negative = CW (right).
pub struct InPlaceTurn {
    pub angle: f32,
}

impl InPlaceTurn {
    /// Convenience constructor accepting degrees.
    pub fn from_degrees(degrees: f32) -> Self {
        Self { angle: degrees.to_radians() }
    }
}

#[async_trait::async_trait]
impl TrajectorySegment for InPlaceTurn {
    fn fusion_mode(&self) -> FusionMode {
        FusionMode::Pivot
    }

    async fn execute<'a>(
        &self,
        motor_left: &mut Motor<'a, TIM3>,
        motor_right: &mut Motor<'a, TIM2>,
    ) {
        set_current_fusion_mode(FusionMode::Pivot);
        let start_pos = CURRENT_POS.get();
        let dt = UPDATE_INTERVAL_MS as f32 / 1000.0;
        let mut angle_integral = 0.0f32;

        // Track rotation relative to start rather than comparing absolute theta values.
        // This ensures direction is always determined by self.angle's sign — gyro drift in
        // the startup delay can shift the absolute theta by ±90°+, which would flip the
        // target_theta normalization and randomly reverse the intended turn direction.
        let mut rotated = 0.0f32;
        let mut prev_theta = start_pos.theta;

        loop {
            let current_pos = CURRENT_POS.get();

            // Accumulated rotation since execute() was called, handling ±π wrap.
            let mut d_theta = current_pos.theta - prev_theta;
            if d_theta > PI { d_theta -= 2.0 * PI; }
            if d_theta < -PI { d_theta += 2.0 * PI; }
            rotated += d_theta;
            prev_theta = current_pos.theta;

            // How much further the robot needs to rotate (positive = still needs CCW).
            let angle_remaining = self.angle - rotated;
            let omega_current = d_theta / dt; // rad/s, positive = CCW

            let coast_estimate = (omega_current * omega_current) / (2.0 * FRICTION_DECEL);
            if angle_remaining.abs() <= coast_estimate.max(DONE_THRESHOLD_MIN_RAD) {
                motor_left.set_speed(0.0);
                motor_right.set_speed(0.0);
                // set_current_fusion_mode(FusionMode::Idle);
                flash_log!(
                    "InPlaceTurn done: target {}deg, final error {}deg",
                    format!("{:.1}", self.angle.to_degrees()).as_str(),
                    format!("{:.2}", angle_remaining.to_degrees()).as_str(),
                );
                break;
            }

            // Accumulate integral; clamp to prevent windup beyond what KI can contribute.
            angle_integral = (angle_integral + angle_remaining * dt)
                .clamp(-MAX_INTEGRAL_CONTRIB / KI_TURN, MAX_INTEGRAL_CONTRIB / KI_TURN);

            let pi_output = KP_TURN * angle_remaining + KI_TURN * angle_integral;
            let effective_max = if angle_remaining.abs() < FINAL_ZONE_RAD {
                APPROACH_SPEED
            } else {
                MAX_TURN_SPEED
            };
            let effective_min = effective_max.min(MIN_TURN_SPEED);
            let turn_speed = pi_output.abs().clamp(effective_min, effective_max);

            // Commanded angular speed: both wheels at ±turn_speed, so ω = 2v / WHEEL_BASE.
            let omega_commanded = pi_output.signum() * 2.0 * turn_speed / WHEEL_BASE;

            flash_log!(
                "InPlaceTurn: remaining {}deg, coast_est {}deg, pi_out {}, turn_speed {}, integral {}, cur_omega {}d/s, cmd_omega {}d/s",
                format!("{:.2}", angle_remaining.to_degrees()).as_str(),
                format!("{:.2}", coast_estimate.to_degrees()).as_str(),
                format!("{:.3}", pi_output).as_str(),
                format!("{:.3}", turn_speed).as_str(),
                format!("{:.4}", angle_integral).as_str(),
                format!("{:.1}", omega_current.to_degrees()).as_str(),
                format!("{:.1}", omega_commanded.to_degrees()).as_str(),
            );

            // pi_output > 0 → turn CCW (left): left backward, right forward.
            // pi_output < 0 → turn CW (right): left forward, right backward.
            let (left_speed, right_speed) = if pi_output > 0.0 {
                (-turn_speed, turn_speed)
            } else {
                (turn_speed, -turn_speed)
            };

            motor_left.set_speed(left_speed);
            motor_right.set_speed(right_speed);

            UPDATE_INTERVAL_MS.ms_timer().await;
        }
    }
}
