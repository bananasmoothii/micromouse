//! Sensor fusion: two scalar Kalman filters combining gyro, odometry, and magnetometer.
//!
//! ## Architecture
//! This is **not** a full EKF. A proper EKF propagates a joint `[x, y, θ]` state vector with
//! a 3×3 covariance matrix through the nonlinear motion model using a Jacobian. Cross-covariance
//! between heading and position is tracked. This implementation instead uses two *independent*
//! scalar Kalman filters — one for `θ` and one for `(x, y)` — with the nonlinear rotation
//! (cos/sin of `θ`) applied ad-hoc. It is simpler to tune and sufficient for a micromouse.
//!
//! ## Heading filter (`θ`)
//! - **Predict**: integrate gyro `d_theta` (clamped to reject EMI spikes). `P += Q_THETA_GYRO`.
//! - **Update 1 (odometry)**: only when both wheels have seen at least two consecutive ticks
//!   (so a single asymmetric tick doesn't create a ~7.7° spurious turn heading update).
//!   Measurement noise scales with angular rate squared to discount skid-steering slip in turns.
//! - **Update 2 (magnetometer)**: large `R_THETA_MAG` keeps the correction gentle — motor-induced
//!   hard-iron changes during movement cause large transient errors, so the mag is mainly useful
//!   when the robot is stationary between moves.
//!
//! ## Position filter `(x, y)`
//! - **Predict**: integrate `v_center × cos/sin(θ) × dt`. `P += Q_XY_VEL`.
//! - **Update**: cumulative odometry global position (more stable than velocity over multiple steps).
//!
//! ## Tuning
//! All `Q_*` and `R_*` constants are documented with the physical reasoning behind their values.
//! Start with `R` values (measurement noise) and adjust `Q` (process noise) until the filter
//! settles at a speed that matches your application's dynamics.

use crate::devices::hall_sensor_3144::{LEFT_TICK_INTERVAL_CYCLES, RIGHT_TICK_INTERVAL_CYCLES};
use crate::positioning::mpu::MpuResult;
use crate::positioning::types::{MovementDelta, PositionState};
use core::f32::consts::PI;
use core::sync::atomic::Ordering;
use Ordering::Relaxed;
use micromath::F32Ext;
use crate::utils::MathUtils;

// --- Heading (θ) filter ---

/// Process noise from gyro integration: how much heading uncertainty grows per 20 ms step.
///   MPU9250 gyro noise ≈ 0.01 rad/s (std dev)
///   variance per step = (0.01 rad/s)² × 0.02 s = 2×10⁻⁶ rad²  (std dev ≈ 0.0014 rad ≈ 0.08°)
/// We use 1×10⁻⁴ (10× larger) to add margin for temperature drift and bias instability.
const Q_THETA_GYRO: f32 = 0.0001;

/// Hard cap on gyro d_theta per step to reject EMI spikes from motor startup current transients.
/// Physical max at full turn speed: 2 × 1.0 / 0.078 × 0.022 s ≈ 0.56 rad (≈ 32°).
/// With ±2000 DPS gyro, max possible per step = 2000° × π/180 × 0.022 s ≈ 0.77 rad (44°).
/// Clamp at 40° sits above the physical max and well below the 44° gyro ceiling.
const MAX_D_THETA_PER_STEP: f32 = 0.70;

/// Measurement noise when using odometry as a heading source — straight-line driving.
/// On a straight, both wheels roll without lateral slip so odometry heading is trustworthy.
const R_THETA_ODOM_STRAIGHT: f32 = 0.5;

/// Scales how fast odometry heading noise grows with angular rate.
/// R_odom = R_THETA_ODOM_STRAIGHT * (1 + SKID_K * ω²)
/// Derived so that at ω ≈ 13 rad/s (pivot turn) R reaches ~50 (gyro fully dominates):
///   (50 / 0.5 - 1) / 13² = 99 / 169 ≈ 0.586
const SKID_K: f32 = 0.586;

/// Measurement noise when using the magnetometer as a heading source.
/// Motor current changes the local hard iron field during operation, causing ~30–80° of error
/// during turns even after static calibration. R=10 keeps the correction gentle enough not
/// to corrupt active turns while still correcting gyro drift during stationary phases.
const R_THETA_MAG: f32 = 1000.0;

// --- Position (XY) filter ---

/// Process noise from velocity integration: how much position uncertainty grows per 20 ms step.
///   speed uncertainty ≈ 5% at 0.5 m/s  →  0.5 × 0.05 = 0.025 m/s error
///   position error per step = 0.025 m/s × 0.02 s = 0.5 mm
///   variance = (0.0005 m)² = 2.5×10⁻⁷ m²  (std dev = 0.5 mm per step)
/// We use 1×10⁻⁴ m² (std dev ≈ 1 cm) to include wheel slip and heading error contributions.
const Q_XY_VEL: f32 = 0.0001;

/// Measurement noise when using cumulative odometry as a position source.
/// Wheel encoders are accurate on straights (≈1% distance error), but skid-steering slip
/// and heading errors accumulate. Over 1 m of travel expect ~1–3 cm of drift → variance ≈ 0.0001–0.001 m².
/// We use 0.02 m² (std dev ≈ 14 cm) as a conservative starting point — tighten once tested.
const R_XY_ODOM: f32 = 0.02;

/// Two-filter sensor fusion state. One instance lives in [`crate::positioning::positioning_task`].
pub struct SensorFusion {
    state: PositionState,
    /// Cumulative odometry position in global frame, used as the absolute position measurement.
    odom_global_x: f32,
    odom_global_y: f32,
    /// Error covariance for heading: grows with Q_THETA_GYRO each step, shrinks on each measurement.
    p_theta_gyro: f32,
    /// Error covariance for XY position: grows with Q_XY_VEL each step, shrinks on each measurement.
    p_xy_odom: f32,
}

impl SensorFusion {
    pub fn new() -> Self {
        Self {
            state: PositionState::default(),
            odom_global_x: 0.0,
            odom_global_y: 0.0,
            p_theta_gyro: 1.0, // start uncertain — filter converges within the first few cycles
            p_xy_odom: 1.0,
        }
    }

    /// Runs one fusion cycle with the given odometry and IMU increments.
    /// Returns the updated [`PositionState`] (also stored internally for the next call).
    /// Call every 20 ms, immediately after [`crate::positioning::odometry::get_odom_delta`].
    pub fn update(&mut self, odom_delta: MovementDelta, mpu: MpuResult) -> PositionState {
        let omega_signed = if mpu.dt > 0.0 { mpu.d_theta / mpu.dt } else { 0.0 };
        let omega = omega_signed.abs();
        let r_theta_odom = R_THETA_ODOM_STRAIGHT * (1.0 + SKID_K * omega.square());
        // --- Heading Kalman filter ---

        // Predict: gyro gives us d_theta directly; uncertainty grows by Q_THETA_GYRO.
        // Clamp to reject EMI spikes from motor current transients — real motion can't
        // exceed MAX_D_THETA_PER_STEP; anything larger is electrical noise, not rotation.
        self.state.theta += mpu.d_theta.clamp(-MAX_D_THETA_PER_STEP, MAX_D_THETA_PER_STEP);
        self.p_theta_gyro += Q_THETA_GYRO;

        // Update from odometry — only once both wheels have seen at least two consecutive ticks
        // (interval > 0). Before that, an imbalanced single tick on one wheel looks like a
        // ~7.7° spurious turn (1 tick ≈ 10.5 mm of wheel travel ÷ 78 mm track), which is
        // still much larger than per-step gyro noise. Gyro-only is more accurate during
        // the first revolution after a stop.
        let odom_synced = LEFT_TICK_INTERVAL_CYCLES.load(Relaxed) > 0
            && RIGHT_TICK_INTERVAL_CYCLES.load(Relaxed) > 0;
        if odom_synced {
            let k_theta_odom = self.p_theta_gyro / (self.p_theta_gyro + r_theta_odom);
            self.state.theta += k_theta_odom * (odom_delta.d_theta - mpu.d_theta);
            self.p_theta_gyro *= 1.0 - k_theta_odom;
        }

        // Update from magnetometer (absolute heading — corrects gyro drift over time).
        // Hard iron calibration in mpu.rs makes relative_mag track actual heading.
        // R_THETA_MAG is set high so motor-induced field changes during turns don't corrupt fusion;
        // the correction is most effective during stationary phases between moves.
        let mut z_mag = mpu.relative_mag - self.state.theta;
        while z_mag > PI { z_mag -= 2.0 * PI; }
        while z_mag < -PI { z_mag += 2.0 * PI; }
        let k_theta_mag = self.p_theta_gyro / (self.p_theta_gyro + R_THETA_MAG);
        self.state.theta += k_theta_mag * z_mag;
        self.p_theta_gyro *= 1.0 - k_theta_mag;

        while self.state.theta > PI { self.state.theta -= 2.0 * PI; }
        while self.state.theta < -PI { self.state.theta += 2.0 * PI; }

        let (sin_theta, cos_theta) = self.state.theta.sin_cos();

        // --- Position Kalman filter ---

        // Forward velocity from tick-count odometry: odom_delta.dx is the wheel arc accumulated
        // over the 20 ms control window, so dividing by dt gives the true average speed over
        // that window with no aliasing artefacts.
        //
        // An earlier approach used inter-tick timestamps (DISTANCE_PER_TICK / tick_interval).
        // That gave better instantaneous resolution in theory, but with only 12 ticks/rev the
        // inter-tick period (~11 ms at 0.5 m/s) aliases against the 20 ms sample period,
        // producing large periodic spikes that corrupted the PI speed controller.
        //
        // The tick-count method's only downside is quantisation: at 0.75 m/s you get 1–2 ticks
        // per window, so the raw reading snaps between 0.52 and 1.05 m/s. The EMA below
        // smooths this; V_ALPHA can be higher than with the old method because the noise is
        // bounded rather than spiky.
        let v_center = if mpu.dt > 0.0 { odom_delta.dx / mpu.dt } else { 0.0 };
        const V_ALPHA: f32 = 0.5;
        self.state.v_forward = V_ALPHA * v_center + (1.0 - V_ALPHA) * self.state.v_forward;

        // Predict: integrate velocity; uncertainty grows by Q_XY_VEL
        self.state.x += v_center * cos_theta * mpu.dt;
        self.state.y += v_center * sin_theta * mpu.dt;
        self.p_xy_odom += Q_XY_VEL;

        // Update from cumulative odometry (more stable than velocity over multiple steps)
        self.odom_global_x += odom_delta.dx * cos_theta;
        self.odom_global_y += odom_delta.dx * sin_theta;
        let k_xy_odom = self.p_xy_odom / (self.p_xy_odom + R_XY_ODOM);
        self.state.x += k_xy_odom * (self.odom_global_x - self.state.x);
        self.state.y += k_xy_odom * (self.odom_global_y - self.state.y);
        self.p_xy_odom *= 1.0 - k_xy_odom;

        self.state.omega = omega_signed;
        self.state
    }
}
