use crate::devices::hall_sensor_3144::{LEFT_TICK_INTERVAL_US, RIGHT_TICK_INTERVAL_US};
use crate::positioning::mpu::MpuResult;
use crate::positioning::odometry::{left_wheel_velocity, right_wheel_velocity};
use crate::positioning::types::{MovementDelta, PositionState};
use core::f32::consts::PI;
use core::sync::atomic::Ordering;
use embassy_time::Instant;
use micromath::F32Ext;

// Note: this is NOT a proper EKF. A real EKF maintains a full [x, y, θ] state vector with a 3×3
// covariance matrix and propagates uncertainty through the nonlinear motion model via a Jacobian.
// What we have instead is two independent scalar Kalman filters (one for θ, one for XY) with the
// nonlinear rotation (cos/sin of θ) applied ad-hoc. Cross-covariance between θ and XY is ignored.
// This is simpler to tune and sufficient for a micromouse, but worth knowing if results disappoint.

// --- How a scalar Kalman filter works ---
// Each filter has one state value and one uncertainty estimate P.
// Every step:
//   Predict: state += model_update;  P += Q   (Q = how much uncertainty we add per step)
//   Update:  K = P / (P + R)                  (K = Kalman gain, 0..1)
//            state += K * (measurement - state)
//            P *= (1 - K)
// When P >> R → K ≈ 1 → trust measurement completely.
// When P << R → K ≈ 0 → ignore measurement, keep current estimate.
// Q and R are tuning parameters you set based on how noisy each source is.

// --- Heading (θ) filter ---

/// Process noise from gyro integration: how much heading uncertainty grows per 20 ms step.
///   MPU9250 gyro noise ≈ 0.01 rad/s (std dev)
///   variance per step = (0.01 rad/s)² × 0.02 s = 2×10⁻⁶ rad²  (std dev ≈ 0.0014 rad ≈ 0.08°)
/// We use 1×10⁻⁴ (10× larger) to add margin for temperature drift and bias instability.
const Q_THETA_GYRO: f32 = 0.0001;

/// Measurement noise when using odometry as a heading source.
/// In a pure rolling differential drive, odometry heading would be accurate to a few degrees.
/// With skid-steering, lateral slip in curves adds large errors — easily ±20–40° (0.35–0.70 rad),
/// so variance ≈ 0.1–0.5 rad². We use 0.5 to keep gyro dominant during turns.
/// Lower this if odometry heading proves reliable on your surface.
const R_THETA_ODOM: f32 = 0.5;

/// Measurement noise when using the magnetometer as a heading source.
/// DC motors generate strong, variable magnetic fields. Expect ±20–40° of noise near them,
/// which gives variance ≈ 0.1–0.5 rad². We use 1.5 (std dev ≈ ±70°) to be conservative —
/// if the magnetometer is heavily disturbed, raising this toward 5.0+ is appropriate.
const R_THETA_MAG: f32 = 1.5;

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

    pub fn update(&mut self, odom_delta: MovementDelta, mpu: MpuResult) -> PositionState {
        // --- Heading Kalman filter ---

        // Predict: gyro gives us d_theta directly; uncertainty grows by Q_THETA_GYRO
        self.state.theta += mpu.d_theta;
        self.p_theta_gyro += Q_THETA_GYRO;

        // Update from odometry — only once both wheels have seen at least two consecutive ticks
        // (interval > 0). Before that, a single tick looks like a ~46° turn on a straight line;
        // gyro-only is far more accurate during those first few revolutions.
        let odom_synced = LEFT_TICK_INTERVAL_US.load(Ordering::Relaxed) > 0
            && RIGHT_TICK_INTERVAL_US.load(Ordering::Relaxed) > 0;
        if odom_synced {
            let k_theta_odom = self.p_theta_gyro / (self.p_theta_gyro + R_THETA_ODOM);
            self.state.theta += k_theta_odom * (odom_delta.d_theta - mpu.d_theta);
            self.p_theta_gyro *= 1.0 - k_theta_odom;
        }

        // Update from magnetometer (absolute heading — prevents gyro drift from accumulating)
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

        // Velocity from per-wheel tick timestamps. The effective period =
        // max(elapsed_since_last_tick, last_tick_interval), so velocity naturally decreases
        // as the robot slows without needing another tick to fire.
        let now_us = Instant::now().as_micros() as u32;
        let v_center = (left_wheel_velocity(now_us) + right_wheel_velocity(now_us)) / 2.0;
        // EMA to smooth tick-rate aliasing (12 ticks/rev fires every ~23 ms at 0.45 m/s,
        // close to the 20 ms sample period — raw readings are very noisy).
        const V_ALPHA: f32 = 0.3; // 0 = frozen, 1 = raw; lower = smoother but more lag
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

        self.state
    }
}
