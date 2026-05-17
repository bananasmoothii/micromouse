use crate::devices::hall_sensor_3144::{LEFT_TICKS_CUMULATIVE, RIGHT_TICKS_CUMULATIVE};
use crate::devices::motors::{MIN_USABLE_SPEED, Motor};
use crate::devices::vl53lxx::MeasurementData;
use crate::devices::vl53lxx::vl53l1x::{
    VL53L1X_45D_LEFT_WATCH, VL53L1X_45D_RIGHT_WATCH, VL53L1X_MIDDLE_WATCH,
};
use embassy_time::{Duration, Instant};
use crate::flash_log;
use crate::positioning::CURRENT_POS;
use crate::positioning::odometry::{DISTANCE_PER_TICK, TICKS_PER_REVOLUTION};
use crate::trajectory::{
    FusionMode, TrajectorySegment, UPDATE_INTERVAL_MS, set_current_fusion_mode,
};
use crate::utils::{CellMutexUtils, DurationUtils, HertzUtils, MathUtils};
use alloc::boxed::Box;
use alloc::format;
use core::f32::consts::PI;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering::Relaxed;
use embassy_stm32::peripherals::{TIM2, TIM3};
use micromath::F32Ext;

const MAX_SPEED_M_S: f32 = 1.0;

// --- Speed controller: position-form PI ---
//
//   commanded_speed = KP * error + KI * integral(error)
//
// The integral eliminates steady-state speed error: if the motor consistently runs slower
// than commanded (due to friction or motor variability), the integral winds up until the
// commanded speed is high enough to close the gap. Anti-windup clamps the integral so it
// cannot contribute more than MAX_SPEED_M_S on its own.
//
// Tune KP first (set KI=0): raise until speed tracks target with minimal lag, back off if it
// oscillates. Then raise KI until steady-state error disappears.
const KP: f32 = 0.5;
const KI: f32 = 9.0;

/// KI * integral cap
const MAX_INTEGRAL_CONTRIB: f32 = MAX_SPEED_M_S;

/// Straight-line heading PI: keeps the robot on its initial bearing.
/// Negated because CW rotation (right turn) increases theta on this hardware (gyro z-axis down).
const KP_STEERING: f32 = 0.4;
const KI_STEERING: f32 = 1.5; // (m/s) per (rad·s) of accumulated heading error
const MAX_STEERING_INTEGRAL: f32 = 0.1; // caps integral wind-up at ~±0.1 m/s authority
const MAX_STEERING: f32 = 0.30;

/// Wall-following steering — uses the two 45° diagonal ToF sensors to keep the robot centred.
///
/// Geometry (see [`crate::dimensions`]): the diagonal sensors form a "U" with its centre on the
/// robot's front edge, on the centreline. Each ray starts at that U-centre and goes outward at
/// `DIAGONAL_ANGLE`. The U absorbs the physical sensor offset, so we can treat the rays as
/// originating exactly at the front-centre point with no other corrections.
///
/// When the robot is centred laterally, a diagonal ray reaches the side wall at:
///     `d_centered = (cell_size / 2) / sin(angle)`
/// If the robot drifts laterally by Δ (positive = drifted LEFT), the "looking-left" reading
/// shortens and the "looking-right" reading lengthens by the same amount:
///     `left  = (cell_half − Δ) / sin(angle)`
///     `right = (cell_half + Δ) / sin(angle)`
/// Solving for Δ:
///     `Δ = (right − left) · sin(angle) / 2`   (both sensors valid)
///     `Δ = cell_half − left · sin(angle)`     (left only)
///     `Δ = right · sin(angle) − cell_half`    (right only)
///
/// Front-wall confusion: if there's a wall directly ahead in the current cell, both diagonals
/// hit IT instead — at ~`front_clearance / cos(angle)` mm when centred. The middle sensor
/// detects this and vetoes the diagonals via [`FRONT_WALL_VETO_M`].
use crate::dimensions::{FRONT_CLEARANCE_CENTERED, LAB_CELL_HALF, LATERAL_CLEARANCE};

const SIN_ANGLE: f32 = core::f32::consts::FRAC_1_SQRT_2; // sin(45°) = cos(45°) = 1/√2

/// Range of trustworthy diagonal readings, derived from "robot somewhere in the cell".
/// Lower bound: drifted hard against the wall on the sensed side.
/// Upper bound: drifted hard away from it, with some slack for the next cell's wall.
const WALL_DIAG_MIN_M: f32 = (LAB_CELL_HALF - LATERAL_CLEARANCE) / SIN_ANGLE; // ≈ 0.071 m
const WALL_DIAG_MAX_M: f32 = (LAB_CELL_HALF + LATERAL_CLEARANCE) / SIN_ANGLE + 0.05; // ≈ 0.234 m
/// Middle-sensor cutoff for the "front wall is close, ignore diagonals" veto: anything closer
/// than this is the current cell's front wall, not the next one (which lives ≥ cell_size away).
const FRONT_WALL_VETO_M: f32 = FRONT_CLEARANCE_CENTERED + 0.06; // ≈ 0.085 m

/// (m/s of steering) per (m of lateral drift).
const KP_WALL_STEER: f32 = 1.0;
const WALL_STALE: Duration = Duration::from_millis(200);

/// Minimum commanded speed during deceleration — lower than MIN_USABLE_SPEED so friction dominates.
/// Requires the MIN_USABLE_PWM snap in motors.rs to be removed.
const DECEL_MIN_SPEED: f32 = MIN_USABLE_SPEED / 2.0;

const ACCELERATION: f32 = 2.0; // m/s²

/// We need this to stop at a precise distance
const DECELERATION: f32 = 2.5;

/// when entering deceleration phase, apply this factor to I
const DECEL_I_FACTOR: f32 = 0.4;

const BRAKE_DISTANCE: f32 = 0.00;

static HEADING_INTEGRAL: AtomicU32 = AtomicU32::new(0f32.to_bits());

/// Accelerates to max speed (if possible), maintains it, then decelerates.
/// Acceleration is done purely through PI control, deceleration is commanded gradually (still
/// through PI control to satisfy DECELERATION_SPEED.
/// Implementation is distance-based, not time-based
pub struct StraightLine {
    pub distance: f32,
    pub out_speed: f32,
}

#[async_trait::async_trait]
impl TrajectorySegment for StraightLine {
    fn fusion_mode(&self) -> FusionMode {
        FusionMode::Straight
    }

    async fn execute<'a>(
        &self,
        motor_left: &mut Motor<'a, TIM3>,
        motor_right: &mut Motor<'a, TIM2>,
    ) {
        set_current_fusion_mode(FusionMode::Straight);
        let mut motors = Motors {
            motor_left,
            motor_right,
        };

        let start_pos = CURRENT_POS.get();
        let initial_theta = start_pos.theta;
        let left_ticks_start = LEFT_TICKS_CUMULATIVE.load(Relaxed);
        let right_ticks_start = RIGHT_TICKS_CUMULATIVE.load(Relaxed);
        let target_ticks = (self.distance / DISTANCE_PER_TICK).round() as i32;

        // ΔE = mgΔx = 1/2 m Δv² -> Δx = (v_max² - v_final²) / 2a  (with g = a)
        let decel_distance_full_speed =
            (MAX_SPEED_M_S.square() - self.out_speed.square()) / (2.0 * DECELERATION);
        let (max_reachable_speed, decel_distance) = if decel_distance_full_speed <= self.distance {
            (MAX_SPEED_M_S, decel_distance_full_speed)
        } else {
            // v_max² = 2aΔx + v_final²    (with Δx = self.distance)
            (
                (2.0 * DECELERATION * self.distance + self.out_speed.square()).sqrt(),
                self.distance,
            )
        };
        let decel_start_distance = self.distance - decel_distance;

        let mut speed_integral = 0.0f32;
        let mut heading_integral = f32::from_bits(HEADING_INTEGRAL.load(Relaxed));
        let mut in_decel = false;

        let mut left_rcv = VL53L1X_45D_LEFT_WATCH
            .receiver()
            .expect("ToF Watch receivers exhausted (N=4)");
        let mut right_rcv = VL53L1X_45D_RIGHT_WATCH
            .receiver()
            .expect("ToF Watch receivers exhausted (N=4)");
        let mut middle_rcv = VL53L1X_MIDDLE_WATCH
            .receiver()
            .expect("ToF Watch receivers exhausted (N=4)");

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
            if heading_error > PI {
                heading_error -= 2.0 * PI;
            }
            if heading_error < -PI {
                heading_error += 2.0 * PI;
            }
            heading_integral = (heading_integral
                + heading_error * (UPDATE_INTERVAL_MS as f32 / 1000.0))
                .clamp(-MAX_STEERING_INTEGRAL, MAX_STEERING_INTEGRAL);
            let steer_p = -(KP_STEERING * heading_error);
            let steer_i = -(KI_STEERING * heading_integral);

            // Fetch latest diagonal/middle snapshots and filter by freshness.
            let now = Instant::now();
            let fresh = |snap: &crate::devices::vl53lxx::vl53l1x::DistanceSnapshot| {
                now.checked_duration_since(snap.at)
                    .map(|d| d < WALL_STALE)
                    .unwrap_or(false)
            };
            let left_snap = left_rcv.try_get().filter(fresh);
            let right_snap = right_rcv.try_get().filter(fresh);
            let middle_snap = middle_rcv.try_get().filter(fresh);

            // Convert raw mm readings (i16) to meters at the boundary; treat ≤0 as "no return".
            let to_m = |s: &crate::devices::vl53lxx::vl53l1x::DistanceSnapshot| {
                let mm = s.data.get_distance_mm();
                if mm > 0 { Some(mm as f32 / 1000.0) } else { None }
            };
            let left_m = left_snap.as_ref().and_then(to_m);
            let right_m = right_snap.as_ref().and_then(to_m);
            let middle_m = middle_snap.as_ref().and_then(to_m);

            // Veto diagonals when the middle sensor sees a close front wall: the diagonals
            // are almost certainly hitting that same front wall, not the side wall we want.
            let front_wall_veto = middle_m.map(|m| m < FRONT_WALL_VETO_M).unwrap_or(false);

            // A diagonal reading is "valid as a side wall" iff in [MIN, MAX] and not vetoed.
            let valid = |m: Option<f32>| -> Option<f32> {
                m.filter(|&d| !front_wall_veto && d >= WALL_DIAG_MIN_M && d <= WALL_DIAG_MAX_M)
            };
            let left_valid = valid(left_m);
            let right_valid = valid(right_m);

            // drift > 0 means the robot has drifted toward its LEFT (sensor mounted on the right
            // side sees a shorter range; sensor on the left side sees a longer one).
            // Sign for steering: drifted left → must steer right → negative steering.
            let drift_m: f32 = match (left_valid, right_valid) {
                (Some(l), Some(r)) => (r - l) * SIN_ANGLE / 2.0,
                (Some(l), None) => LAB_CELL_HALF - l * SIN_ANGLE,
                (None, Some(r)) => r * SIN_ANGLE - LAB_CELL_HALF,
                (None, None) => 0.0,
            };
            let wall_steer = -KP_WALL_STEER * drift_m;

            let steering = (steer_p + steer_i + wall_steer).clamp(-MAX_STEERING, MAX_STEERING);

            if avg_ticks >= target_ticks {
                flash_log!(
                    "Distance reached: left {} ticks ({} turns), right {} ticks ({} turns), speed error: {}",
                    left_ticks,
                    left_ticks / TICKS_PER_REVOLUTION as i32,
                    right_ticks,
                    right_ticks / TICKS_PER_REVOLUTION as i32,
                    current_pos.v_forward - self.out_speed,
                );
                motors.set_speed(self.out_speed);
                // keep straight in case out_speed != 0
                // set_current_fusion_mode(FusionMode::Idle);
                break;
            }

            // Acceleration ramp: ramps from 0 up to max_reachable_speed using kinematics.
            let accel_target = (MIN_USABLE_SPEED.square() + 2.0 * ACCELERATION * distance)
                .sqrt()
                .min(max_reachable_speed);

            // Deceleration ramp: distance-based, always reaches out_speed at self.distance.
            // Integral resets once on decel entry to clear cruise windup.
            // Deceleration always takes precedence (min of both ramps).
            let decel_target = if distance >= decel_start_distance {
                if !in_decel {
                    in_decel = true;
                    speed_integral *= DECEL_I_FACTOR;
                }
                (max_reachable_speed
                    - (distance - decel_start_distance) * max_reachable_speed / decel_distance)
                    .max(MIN_USABLE_SPEED)
            } else {
                f32::INFINITY
            };

            let target_speed = accel_target.min(decel_target);

            let speed_error = target_speed - current_pos.v_forward;

            speed_integral = (speed_integral + speed_error * (UPDATE_INTERVAL_MS as f32 / 1000.0))
                .clamp(-MAX_INTEGRAL_CONTRIB / KI, MAX_INTEGRAL_CONTRIB / KI);
            let p = KP * speed_error;
            let i = KI * speed_integral;

            let in_brake = self.distance - distance <= BRAKE_DISTANCE;
            let sign = self.distance.signum();
            let pi_out = p + i;
            let min_speed = if in_decel { DECEL_MIN_SPEED } else { MIN_USABLE_SPEED };
            let commanded_speed = if in_brake {
                0.0
            } else {
                pi_out.clamp(sign * min_speed, sign * 1.5 * MAX_SPEED_M_S)
            };
            let active_steering = if in_brake { 0.0 } else { steering };
            flash_log!(
                "StraightLine ({}/{}m): target: {}{}, current: {}, error: {}, commanded: {}, p: {}, i: {}, steer: {}, steer_p: {}, steer_i: {}, wall_steer: {}, L: {} M: {} R: {} (drift_mm: {}), hdg: {}deg",
                format!("{:.2}", distance).as_str(),
                format!("{:.2}", self.distance).as_str(),
                format!("{:.2}", target_speed).as_str(),
                if in_decel { " (decel)" } else { "" },
                format!("{:.2}", current_pos.v_forward).as_str(),
                format!("{:.2}", speed_error).as_str(),
                format!("{:.2}", commanded_speed).as_str(),
                format!("{:.4}", p).as_str(),
                format!("{:.4}", i).as_str(),
                format!("{:.4}", active_steering).as_str(),
                format!("{:.4}", steer_p).as_str(),
                format!("{:.4}", steer_i).as_str(),
                format!("{:.4}", wall_steer).as_str(),
                left_m.map(|m| (m * 1000.0) as i32).unwrap_or(-1),
                middle_m.map(|m| (m * 1000.0) as i32).unwrap_or(-1),
                right_m.map(|m| (m * 1000.0) as i32).unwrap_or(-1),
                format!("{:.1}", drift_m * 1000.0).as_str(),
                format!("{:.1}", heading_error.to_degrees()).as_str(),
            );

            motors.set_speed_steered(commanded_speed, active_steering);

            UPDATE_INTERVAL_MS.ms_timer().await;
        }
        HEADING_INTEGRAL.store(heading_integral.to_bits(), Relaxed);
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
