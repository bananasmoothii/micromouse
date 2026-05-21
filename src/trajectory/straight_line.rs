use crate::Box;
use crate::devices::hall_sensor_3144::{LEFT_TICKS_CUMULATIVE, RIGHT_TICKS_CUMULATIVE};
use crate::devices::motors::{MIN_USABLE_SPEED, Motor};
use crate::devices::vl53lxx::vl53l1x::{
    DistReceiver, VL53L1X_45D_LEFT_WATCH, VL53L1X_45D_RIGHT_WATCH, VL53L1X_MIDDLE_WATCH,
};
use crate::dimensions::{LAB_CELL, LAB_CELL_HALF, LATERAL_CLEARANCE, ROBOT_WIDTH};
use crate::flash_log;
use crate::positioning::CURRENT_POS;
use crate::positioning::odometry::{DISTANCE_PER_TICK, TICKS_PER_REVOLUTION};
use crate::trajectory::straight_line::StraightLineGoal::Distance;
use crate::trajectory::{TrajectorySegment, UPDATE_INTERVAL_MS};
use crate::utils::{CellMutexUtils, MathUtils};
use alloc::format;
use core::f32::consts::{PI, SQRT_2};
use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering::Relaxed;
use embassy_stm32::peripherals::{TIM2, TIM3};
use embassy_time::{Duration, Instant, Timer};
use micromath::F32Ext;

// consider also changing MIN_PULSE_WIDTH_CYCLES in hall_sensor_3144.rs if you change this
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
/// Steering is a dimensionless wheel-speed asymmetry (see `set_speed_steered`); negated because
/// CW rotation increases theta on this hardware.
const KP_STEERING: f32 = 0.6;
const KI_STEERING: f32 = 1.0;
const MAX_STEERING_INTEGRAL: f32 = 0.15;
/// When initial_heading_error is non-zero, treat the error as if it had been
/// integrating for this many seconds to produce the seeded heading_integral.
const HEADING_INTEGRAL_SEED_TIME: f32 = 0.5;
const MAX_STEERING: f32 = 0.30;

/// per tick
const SINGLE_WALL_STEERING_ANGLE_MULTIPLIER: f32 = 0.70;
const MAX_WALL_STEERING: f32 = 0.4;
/// Approach-rate gain for wall avoidance (rad per m/s of closing speed).
const KD_WALL: f32 = 0.2;
/// Angular-rate damping gain: opposes rotation to prevent overshoot and spinning.
const KD_HEADING: f32 = 0.05;

/// Minimum commanded speed during deceleration — lower than MIN_USABLE_SPEED so friction dominates.
/// Requires the MIN_USABLE_PWM snap in motors.rs to be removed.
const DECEL_MIN_SPEED: f32 = MIN_USABLE_SPEED / 2.0;

const ACCELERATION: f32 = 2.0; // m/s²

/// We need this to stop at a precise distance
const DECELERATION: f32 = 2.5;

/// when entering deceleration phase, apply this factor to I
const DECEL_I_FACTOR: f32 = 0.4;

const BRAKE_DISTANCE: f32 = 0.00;

const BRAKE_DISTANCE_TO_WALL: f32 = 0.01;

/// VL53L1X pipelines 3-4 measurements before reporting; at 66 ms/measurement that
/// is ~250 ms of structural lag between the physical event and the value we read.
/// Subtract v × this constant from each accepted M reading so effective_remaining
/// reflects where the robot actually is, not where it was 250 ms ago.
const SENSOR_LAG_S: f32 = 0.250;

static HEADING_INTEGRAL: AtomicU32 = AtomicU32::new(0f32.to_bits());

pub enum StraightLineGoal {
    Distance(f32),
    DistanceToFrontWall(f32),
}

/// Accelerates to max speed (if possible), maintains it, then decelerates.
/// Acceleration is done purely through PI control, deceleration is commanded gradually (still
/// through PI control to satisfy DECELERATION_SPEED.
/// Implementation is distance-based, not time-based
pub struct StraightLine {
    pub goal: StraightLineGoal,
    pub out_speed: f32,
    /// Angular error at segment start in radians: actual heading − target heading.
    /// Non-zero when a preceding `InPlaceTurn` left a residual error.
    /// Absorbed into `initial_theta` so the heading P term sees the true error
    /// from tick one, and used to pre-seed the heading integral.
    /// Set to `0.0` when no correction is needed (e.g. segment starts aligned).
    pub initial_heading_error: f32,
}

#[async_trait::async_trait]
impl TrajectorySegment for StraightLine {
    async fn execute<'a>(
        &self,
        motor_left: &mut Motor<'a, TIM3>,
        motor_right: &mut Motor<'a, TIM2>,
    ) {
        let mut motors = Motors {
            motor_left,
            motor_right,
        };

        let mut left_rcv = VL53L1X_45D_LEFT_WATCH
            .receiver()
            .expect("ToF Watch receivers exhausted (N=4)");
        let mut right_rcv = VL53L1X_45D_RIGHT_WATCH
            .receiver()
            .expect("ToF Watch receivers exhausted (N=4)");
        let mut middle_rcv = VL53L1X_MIDDLE_WATCH
            .receiver()
            .expect("ToF Watch receivers exhausted (N=4)");

        let start_pos = CURRENT_POS.get();
        // Absorb the known start-of-segment heading error so the controller tries to
        // maintain the *target* direction, not the (mis-aligned) actual start heading.
        let initial_theta = start_pos.theta - self.initial_heading_error;
        let left_ticks_start = LEFT_TICKS_CUMULATIVE.load(Relaxed);
        let right_ticks_start = RIGHT_TICKS_CUMULATIVE.load(Relaxed);
        let mut target_ticks = if let Distance(d) = self.goal {
            (d / DISTANCE_PER_TICK).round() as i32
        } else {
            i32::MAX
        };

        // For DistanceToFrontWall: seed total_distance from the current front sensor reading if
        // available so the accel ramp starts correctly; fall back to INFINITY (robot accelerates
        // freely until the first in-loop update).
        let mut total_distance: f32 = match &self.goal {
            Distance(d) => *d,
            StraightLineGoal::DistanceToFrontWall(stop_offset) => middle_rcv
                .try_get()
                .filter(|it| it.is_newer_than(SIDE_WALL_STALE_MEASUREMENT))
                .and_then(|snap| {
                    let r = snap.data.0.range_milli_meter as f32 / 1000.0 - *stop_offset;
                    if r > 0.0 { Some(r) } else { None }
                })
                .unwrap_or(f32::INFINITY),
        };
        let mut last_front_at: Option<Instant> = None;

        // ΔE = mgΔx = 1/2 m Δv² -> Δx = (v_max² - v_final²) / 2a  (with g = a)
        let decel_distance_full_speed =
            (MAX_SPEED_M_S.square() - self.out_speed.square()) / (2.0 * DECELERATION);
        let (max_reachable_speed, decel_distance) =
            if decel_distance_full_speed <= total_distance {
                (MAX_SPEED_M_S, decel_distance_full_speed)
            } else {
                // v_max² = 2aΔx + v_final²    (with Δx = total_distance)
                (
                    (2.0 * DECELERATION * total_distance + self.out_speed.square()).sqrt(),
                    total_distance,
                )
            };
        let decel_start_distance = total_distance - decel_distance;

        let mut speed_integral = 0.0f32;
        // When a heading offset is provided (after a turn), reset the persisted integral
        // (it came from a different corridor direction) and pre-seed with the known error.
        // Otherwise carry the integral over from the previous segment (same direction run).
        let mut heading_integral = if self.initial_heading_error.abs() > 0.002 {
            (self.initial_heading_error * HEADING_INTEGRAL_SEED_TIME)
                .clamp(-MAX_STEERING_INTEGRAL, MAX_STEERING_INTEGRAL)
        } else {
            f32::from_bits(HEADING_INTEGRAL.load(Relaxed))
        };
        let mut in_decel = false;
        let mut left_rate_buf = WallRateBuffer::new();
        let mut right_rate_buf = WallRateBuffer::new();
        // ToF-based remaining for DistanceToFrontWall: updated on each fresh M reading,
        // extrapolated via encoder delta between readings to sidestep encoder slip.
        let mut tof_remaining_m: Option<f32> = None;
        let mut distance_at_tof = 0.0f32;

        // Fixed-cadence control loop: wake on absolute deadlines so iteration period stays at
        // UPDATE_INTERVAL_MS regardless of work time inside the loop. If work overruns the
        // period, Timer::at returns immediately and we re-align on the next tick.
        let period = Duration::from_millis(UPDATE_INTERVAL_MS);
        let mut next_tick = Instant::now() + period;

        loop {
            let current_pos = CURRENT_POS.get();
            let left_ticks = LEFT_TICKS_CUMULATIVE.load(Relaxed) - left_ticks_start;
            let right_ticks = RIGHT_TICKS_CUMULATIVE.load(Relaxed) - right_ticks_start;
            // Tick-based distance: more accurate than EKF position for stop condition
            let avg_ticks = (left_ticks + right_ticks) / 2;
            let distance = avg_ticks as f32 * DISTANCE_PER_TICK;

            // DistanceToFrontWall: every new front-sensor reading updates the target.
            // Using the same ramp formula as the pre-loop init, applied to the new
            // total distance so accel and decel ramps stay consistent.
            if let StraightLineGoal::DistanceToFrontWall(stop_offset) = &self.goal {
                if let Some(snap) = middle_rcv
                    .try_get()
                    .filter(|it| it.is_newer_than(SIDE_WALL_STALE_MEASUREMENT))
                    .filter(|snap| last_front_at != Some(snap.at))
                {
                    last_front_at = Some(snap.at);
                    let d_front = snap.data.0.range_milli_meter as f32 / 1000.0;
                    let remaining = (d_front - *stop_offset).max(0.0);
                    // Sensor lag compensation: the measurement was taken ~SENSOR_LAG_S ago.
                    // The robot moved v × SENSOR_LAG_S since then; subtract that so the seed
                    // for encoder extrapolation reflects the actual current position.
                    let lag_dist = current_pos.v_forward * SENSOR_LAG_S;
                    let remaining_adj = (remaining - lag_dist).max(0.0);
                    tof_remaining_m = Some(remaining_adj);
                    distance_at_tof = distance;
                    // Update total_distance for logging and in_brake guard; do NOT update
                    // decel_distance / max_reachable_speed — encoder over-counting makes
                    // total_distance drift upward on each reading, which would silently
                    // widen the decel window and push the ramp floor into MIN_USABLE_SPEED.
                    total_distance = distance + remaining_adj;
                    target_ticks = (total_distance / DISTANCE_PER_TICK).round() as i32;
                }
            }

            // If the front sensor has gone stale while we hold a stored target, that target
            // was likely set from a wrong surface (e.g. a side wall seen during a mis-aligned
            // entry). Reset everything so the next valid front reading starts fresh.
            if tof_remaining_m.is_some()
                && matches!(&self.goal, StraightLineGoal::DistanceToFrontWall(_))
                && middle_rcv
                .try_get()
                .filter(|it| it.is_newer_than(SIDE_WALL_STALE_MEASUREMENT))
                .is_none()
            {
                tof_remaining_m = None;
                total_distance = f32::INFINITY;
                target_ticks = i32::MAX;
                in_decel = false;
            }

            // Best estimate of remaining distance to stop:
            // For DistanceToFrontWall with a valid ToF reading: ToF remaining minus encoder delta
            // since that reading (over-counts, but error resets every ~66 ms on next reading).
            // For Distance or stale M: encoder-based (original behaviour).
            let effective_remaining = match (&self.goal, tof_remaining_m) {
                (StraightLineGoal::DistanceToFrontWall(_), Some(r)) => {
                    (r - (distance - distance_at_tof)).max(0.0)
                }
                _ => total_distance - distance,
            };

            let tof_stop = matches!(&self.goal, StraightLineGoal::DistanceToFrontWall(_))
                && tof_remaining_m.is_some()
                && effective_remaining <= BRAKE_DISTANCE_TO_WALL;
            if avg_ticks >= target_ticks || tof_stop {
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
                break;
            }

            // Acceleration ramp: ramps from 0 up to max_reachable_speed using kinematics.
            let accel_target = (MIN_USABLE_SPEED.square() + 2.0 * ACCELERATION * distance)
                .sqrt()
                .min(max_reachable_speed);

            // Deceleration ramp: always reaches out_speed at the stop point.
            // For DistanceToFrontWall with a valid ToF reading: driven by effective_remaining
            // (ToF truth, extrapolated by encoder between readings) so encoder slip cannot
            // cause the ramp to fire late or the robot to blow through the target.
            // For Distance or stale M: original encoder-based ramp, unchanged.
            let decel_target = if matches!(&self.goal, StraightLineGoal::DistanceToFrontWall(_))
                && tof_remaining_m.is_some()
            {
                if effective_remaining <= decel_distance && !in_decel {
                    in_decel = true;
                    speed_integral *= DECEL_I_FACTOR;
                }
                // Once in_decel, always apply the proportional ramp. When a corrected ToF reading
                // shows more remaining than the encoder said (encoder over-counted), the formula
                // gives > max_reachable_speed which gets clamped — no re-acceleration.
                if in_decel {
                    (max_reachable_speed * effective_remaining / decel_distance).max(DECEL_MIN_SPEED)
                } else {
                    f32::INFINITY
                }
            } else if distance >= decel_start_distance {
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

            let in_brake = total_distance - distance <= BRAKE_DISTANCE;
            let sign = total_distance.signum();
            let pi_out = p + i;
            // For DistanceToFrontWall: when the robot is above the decel target (active braking),
            // cut the motor floor to 0 so friction dominates rather than fighting it at
            // DECEL_MIN_SPEED. This lets PI command ~0 → stronger braking, matching the 2.5 m/s²
            // deceleration assumption. Distance(x) keeps DECEL_MIN_SPEED (encoder-based, unchanged).
            let min_speed = if in_decel
                && matches!(&self.goal, StraightLineGoal::DistanceToFrontWall(_))
                && decel_target < f32::INFINITY
                && current_pos.v_forward > decel_target
            {
                0.0
            } else if in_decel {
                DECEL_MIN_SPEED
            } else {
                MIN_USABLE_SPEED
            };
            let commanded_speed = if in_brake {
                0.0
            } else {
                pi_out.clamp(sign * min_speed, sign * 1.5 * MAX_SPEED_M_S)
            };

            // Heading error: how much the robot has rotated from its initial direction.
            // Normalized to ±π so wrap-around near ±180° doesn't produce spurious corrections.
            let mut heading_error = current_pos.theta - initial_theta;
            let left_ts = Self::get_wall_dist(&mut left_rcv);
            let right_ts = Self::get_wall_dist(&mut right_rcv);

            if let Some((d, at)) = left_ts {
                left_rate_buf.push_if_new(d, at);
            }
            if let Some((d, at)) = right_ts {
                right_rate_buf.push_if_new(d, at);
            }
            let left_dist = left_ts.map(|(d, _)| d);
            let right_dist = right_ts.map(|(d, _)| d);

            // Blend wall-proximity correction into heading_error additively so that accumulated
            // theta drift is never discarded. The clamp bounds the wall term alone; the combined
            // total is then bounded by MAX_STEERING at the steer output.
            {
                let h_l = left_dist
                    .and_then(|d| Self::get_heading_error(d))
                    .unwrap_or(0.0);
                let h_r = right_dist
                    .and_then(|d| Self::get_heading_error(d))
                    .unwrap_or(0.0);
                // Both walls contribute → full centering force (PREFERRED_CLEARANCE target).
                // Only one wall → conservative avoidance: only correct when actually closer
                // than the natural corridor center (LATERAL_CLEARANCE target). This prevents
                // overshooting into open space when the opposing wall is not yet visible.
                let wall_correction = if h_l > 0.0 && h_r > 0.0 {
                    h_l - h_r
                } else {
                    let sl = left_dist
                        .map(|d| Self::get_single_wall_heading_error(d))
                        .unwrap_or(0.0);
                    let sr = right_dist
                        .map(|d| Self::get_single_wall_heading_error(d))
                        .unwrap_or(0.0);
                    sl - sr
                };
                // During decel the robot is legitimately converging on walls (end of cell), so
                // high-gain wall correction oscillates against L/R asymmetry every 20 ms. Back
                // off to 30% so the gyro-based theta term dominates, preventing ±12° swings.
                let wall_scale = if in_decel { 0.3 } else { 1.0 };
                heading_error +=
                    (wall_correction * wall_scale).clamp(-MAX_WALL_STEERING, MAX_WALL_STEERING);
            }
            // Approach-rate correction: disabled during decel because the robot is approaching
            // the front wall legitimately, spiking approach rates and adding noise to heading.
            if !in_decel {
                heading_error +=
                    KD_WALL * (left_rate_buf.approach_rate() - right_rate_buf.approach_rate());
            }
            if heading_error > PI {
                heading_error -= 2.0 * PI;
            }
            if heading_error < -PI {
                heading_error += 2.0 * PI;
            }
            heading_integral = (heading_integral
                + heading_error * (UPDATE_INTERVAL_MS as f32 / 1000.0))
                .clamp(-MAX_STEERING_INTEGRAL, MAX_STEERING_INTEGRAL);
            let steer_p = -(KP_STEERING * heading_error) - KD_HEADING * current_pos.omega;
            let steer_i = -(KI_STEERING * heading_integral);
            let steering = (steer_p + steer_i).clamp(-MAX_STEERING, MAX_STEERING);

            let active_steering = if in_brake { 0.0 } else { steering };
            let middle_dist = middle_rcv
                .try_get()
                .filter(|it| it.is_newer_than(SIDE_WALL_STALE_MEASUREMENT))
                .map(|snap| snap.data.0.range_milli_meter as f32 / 1000.0);
            flash_log!(
                "StraightLine ({}/{}m): target: {}{}, current: {}, error: {}, commanded: {}, p: {}, i: {}, steer_p: {}, L: {} M: {} R: {}, L_rate: {} R_rate: {}, hdg: {}deg, theta: {}deg",
                format!("{:.2}", distance).as_str(),
                format!("{:.2}", total_distance).as_str(),
                format!("{:.2}", target_speed).as_str(),
                if in_decel { " (decel)" } else { "" },
                format!("{:.2}", current_pos.v_forward).as_str(),
                format!("{:.2}", speed_error).as_str(),
                format!("{:.2}", commanded_speed).as_str(),
                format!("{:.4}", p).as_str(),
                format!("{:.4}", i).as_str(),
                format!("{:.4}", steer_p).as_str(),
                format!("{:.3}", left_dist.unwrap_or(f32::NAN)).as_str(),
                format!("{:.3}", middle_dist.unwrap_or(f32::NAN)).as_str(),
                format!("{:.3}", right_dist.unwrap_or(f32::NAN)).as_str(),
                format!("{:.3}", left_rate_buf.approach_rate()).as_str(),
                format!("{:.3}", right_rate_buf.approach_rate()).as_str(),
                format!("{:.1}", heading_error.to_degrees()).as_str(),
                format!("{:.1}", current_pos.theta.to_degrees()).as_str(),
            );

            motors.set_speed_steered(commanded_speed, active_steering);

            Timer::at(next_tick).await;
            next_tick += period;
        }
        HEADING_INTEGRAL.store(heading_integral.to_bits(), Relaxed);
    }
}

const WALL_RATE_BUFFER_SIZE: usize = 3;

/// Ring buffer of the last N wall-distance samples (one entry per new sensor reading).
/// Approach rate is computed as (oldest − newest) / elapsed, so positive = wall closing in.
struct WallRateBuffer {
    slots: [Option<(f32, Instant)>; WALL_RATE_BUFFER_SIZE],
    head: usize,
    last_at: Option<Instant>,
}

impl WallRateBuffer {
    const fn new() -> Self {
        Self {
            slots: [None; WALL_RATE_BUFFER_SIZE],
            head: 0,
            last_at: None,
        }
    }

    /// Only records the sample if its timestamp is newer than the last one stored,
    /// so consecutive loop iterations with the same sensor reading don't inflate the buffer.
    fn push_if_new(&mut self, dist: f32, at: Instant) {
        if self.last_at == Some(at) {
            return;
        }
        self.last_at = Some(at);
        self.slots[self.head] = Some((dist, at));
        self.head = (self.head + 1) % WALL_RATE_BUFFER_SIZE;
    }

    /// Returns approach rate in m/s (positive = getting closer).
    /// Uses the oldest and newest valid entries in the buffer.
    /// Returns 0.0 if fewer than 2 samples are available or the newest is stale.
    fn approach_rate(&self) -> f32 {
        let mut oldest: Option<(f32, Instant)> = None;
        let mut newest: Option<(f32, Instant)> = None;
        for slot in &self.slots {
            if let Some((d, t)) = *slot {
                if oldest.map_or(true, |(_, ot)| t < ot) {
                    oldest = Some((d, t));
                }
                if newest.map_or(true, |(_, nt)| t > nt) {
                    newest = Some((d, t));
                }
            }
        }
        match (oldest, newest) {
            (Some((d_old, t_old)), Some((d_new, t_new))) if t_new > t_old => {
                // Treat the rate as zero if no fresh sample arrived recently.
                if Instant::now()
                    .checked_duration_since(t_new)
                    .map_or(true, |age| age >= SIDE_WALL_STALE_MEASUREMENT)
                {
                    return 0.0;
                }
                let elapsed_s = t_new
                    .checked_duration_since(t_old)
                    .map(|dur| dur.as_micros() as f32 / 1_000_000.0)
                    .unwrap_or(0.0);
                if elapsed_s > 0.001 {
                    (d_old - d_new) / elapsed_s
                } else {
                    0.0
                }
            }
            _ => 0.0,
        }
    }
}

const SIDE_WALL_STALE_MEASUREMENT: Duration = Duration::from_millis(150);
const FOLLOWED_WALL_MAX_DIST: f32 = LAB_CELL - ROBOT_WIDTH;

impl StraightLine {
    /// Gets the distance in meters to the wall plus its sample timestamp, if data is fresh enough.
    /// Distance is returned straight (lateral component only, not diagonal).
    fn get_wall_dist(rcv: &mut DistReceiver) -> Option<(f32, Instant)> {
        rcv.try_get()
            .filter(|it| it.is_newer_than(SIDE_WALL_STALE_MEASUREMENT))
            .map(|snap| {
                (
                    snap.data.0.range_milli_meter as f32 / 1000.0 / SQRT_2,
                    snap.at,
                )
            })
    }

    fn get_heading_error(distance_middle_to_wall: f32) -> Option<f32> {
        let remaining = (distance_middle_to_wall - ROBOT_WIDTH / 2.0).max(0.0);
        const PREFERRED_CLEARANCE: f32 = LATERAL_CLEARANCE * 2.5;
        if remaining < PREFERRED_CLEARANCE {
            Some(
                SINGLE_WALL_STEERING_ANGLE_MULTIPLIER * (PREFERRED_CLEARANCE - remaining)
                    / PREFERRED_CLEARANCE,
            )
        } else {
            None
        }
    }

    /// Single-wall variant: uses LATERAL_CLEARANCE as the target instead of PREFERRED_CLEARANCE.
    /// Fires only when the robot is actually closer than the natural corridor center, so a robot
    /// at the center of the corridor (remaining ≈ LATERAL_CLEARANCE) gets zero correction.
    fn get_single_wall_heading_error(distance_middle_to_wall: f32) -> f32 {
        let remaining = (distance_middle_to_wall - ROBOT_WIDTH / 2.0).max(0.0);
        const PREFERRED_CLEARANCE: f32 = LATERAL_CLEARANCE * 1.1;
        if remaining < PREFERRED_CLEARANCE {
            SINGLE_WALL_STEERING_ANGLE_MULTIPLIER * (PREFERRED_CLEARANCE - remaining)
                / PREFERRED_CLEARANCE
        } else {
            0.0
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

    /// Apply base speed with a multiplicative steering correction:
    ///   left  = speed * (1 - steering)
    ///   right = speed * (1 + steering)
    /// `steering` is a dimensionless fraction. Positive turns left (right wheel faster);
    /// negative turns right. Curvature is constant per steering unit; correction authority
    /// scales with speed so the controller doesn't over-rotate when slowing down.
    fn set_speed_steered(&mut self, speed: f32, steering: f32) {
        self.motor_left.set_speed(speed * (1.0 - steering));
        self.motor_right.set_speed(speed * (1.0 + steering));
    }
}
