use core::f32::consts::PI;
use crate::positioning::types::{MovementDelta, Position2D};
use crate::positioning::mpu::MpuResult;
use micromath::F32Ext;

pub struct SensorFusion {
    state: Position2D,

    // Internal trackers to accumulate raw odometry as our absolute position measurement
    odom_global_x: f32,
    odom_global_y: f32,

    /// `p_theta` (Error Covariance): Represents our uncertainty in the current orientation estimate.
    /// Units: Radians Squared (rad^2).
    /// It grows during prediction (due to gyro drift) and shrinks when we get a measurement.
    p_theta: f32,

    /// `q_theta` (Process Noise Covariance): How much uncertainty we add per step because of integration drift.
    /// Units: Radians Squared (rad^2) per step.
    /// *Tuning*: Increase if the real gyro drifts heavily. Decrease if you trust the gyro almost perfectly.
    q_theta: f32,

    /// `r_theta_odom` (Odometry Measurement Noise): Represents the expected noise/slip in wheel odometry.
    /// Units: Radians Squared (rad^2).
    /// *Tuning*: Increase if the wheels frequently slip/skid or if the axle track width is imprecise. Decrease if odometry is extremely precise.
    r_theta_odom: f32,

    /// `r_theta_mag` (Magnetometer Measurement Noise): Magnetic noise is typically high inside a robot with motors.
    /// Units: Radians Squared (rad^2).
    /// *Tuning*: Decrease if the compass is electrically isolated and accurate (fixes long-term drift faster). Keep very high (e.g. 10.0+) if motors severely distort the magnetic field.
    r_theta_mag: f32,

    /// Covariance tracking for our X/Y coordinates
    p_xy: f32,
    /// Acceleration process noise (how much the accelerometer double-integration drifts laterally)
    q_xy: f32,
    /// Odometry absolute measurement noise. Lower value means we trust the odometry distance more.
    r_xy_odom: f32,
}

impl SensorFusion {
    pub fn new() -> Self {
        Self {
            state: Position2D::default(),
            odom_global_x: 0.0,
            odom_global_y: 0.0,
            p_theta: 1.0,
            q_theta: 0.001,
            r_theta_odom: 0.05,
            r_theta_mag: 10.0, // High noise -> weak but steady long-term correction
            p_xy: 1.0,
            q_xy: 0.005, // Acceleration noise per step
            r_xy_odom: 0.02, // Trust bounds for wheel slippage and chunky resolution
        }
    }

    pub fn update(&mut self, odom_delta: MovementDelta, mpu: MpuResult) -> Position2D {
        let dt = mpu.dt;
        let mpu_delta = mpu.delta;
        let accel_x = mpu.accel_x;
        let accel_y = mpu.accel_y;
        let mag_heading = mpu.relative_mag;

        // --- 1D Kalman Filter for Theta ---

        // 1. Predict (using MPU gyro delta)
        self.state.theta += mpu_delta.dtheta;
        self.p_theta += self.q_theta;

        // 2. Update (using Odometry delta)
        let z_odom = odom_delta.dtheta - mpu_delta.dtheta; // Measurement residual

        let s_odom = self.p_theta + self.r_theta_odom; // Innovation covariance
        let k_odom = self.p_theta / s_odom; // Kalman gain

        self.state.theta += k_odom * z_odom;
        self.p_theta = (1.0 - k_odom) * self.p_theta;

        // 3. Update (using Magnetometer absolute heading)
        let mut z_mag = mag_heading - self.state.theta;
        while z_mag > PI { z_mag -= 2.0 * PI; }
        while z_mag < -PI { z_mag += 2.0 * PI; }

        let s_mag = self.p_theta + self.r_theta_mag;
        let k_mag = self.p_theta / s_mag;

        self.state.theta += k_mag * z_mag;
        self.p_theta = (1.0 - k_mag) * self.p_theta;

        // Normalize theta to [-PI, PI] to keep math clean
        while self.state.theta > PI { self.state.theta -= 2.0 * PI; }
        while self.state.theta < -PI { self.state.theta += 2.0 * PI; }

        let (sin_theta, cos_theta) = self.state.theta.sin_cos();

        // --- 2D Position Kalman Filter (Using Accelerometer & Odometry) ---
        // Because Odometry has incredibly low resolution (2 ticks per rev) and wheels slip on turns,
        // we use the accelerometer to PREDICT rapid lateral and forward motion between ticks.
        // Odometry is our periodic MEASUREMENT correction pulling the position back to reality.

        let a_x = accel_x * 9.81; // Convert g to m/s^2
        let a_y = accel_y * 9.81;

        // 1. Predict (Using Accelerometer)
        // Integration Method:
        // - Velocity uses standard Euler integration ("left rectangles" rule): v_new = v_old + a * dt
        // - Position uses the precise constant-acceleration kinematic equation: p_new = p_old + v_old * dt + 0.5 * a * dt^2
        let a_x_global = a_x * cos_theta - a_y * sin_theta;
        let a_y_global = a_x * sin_theta + a_y * cos_theta;

        self.state.x += self.state.v_x * dt + 0.5 * a_x_global * dt * dt;
        self.state.y += self.state.v_y * dt + 0.5 * a_y_global * dt * dt;

        self.state.v_x += a_x_global * dt;
        self.state.v_y += a_y_global * dt;

        // Prevent integration from growing to infinity from generic sensor bias by softly decaying
        self.state.v_x *= 0.95;
        self.state.v_y *= 0.95;

        self.p_xy += self.q_xy;

        // 2. Update (Using Odometry)
        // Map the new chunky ticks incrementally into our ideal global odometer mapping
        let delta_center = odom_delta.dx;
        self.odom_global_x += delta_center * cos_theta;
        self.odom_global_y += delta_center * sin_theta;

        let z_x = self.odom_global_x - self.state.x;
        let z_y = self.odom_global_y - self.state.y;

        let s_xy = self.p_xy + self.r_xy_odom;
        let k_xy = self.p_xy / s_xy;

        self.state.x += k_xy * z_x;
        self.state.y += k_xy * z_y;

        // When the position jumps due to an odometry tick correction, give the velocity a bump
        // so it smoothly merges back with the physical reality.
        if dt > 0.0 {
            self.state.v_x += (k_xy * z_x) * 0.5 / dt;
            self.state.v_y += (k_xy * z_y) * 0.5 / dt;
        }

        self.p_xy = (1.0 - k_xy) * self.p_xy;

        self.state.clone()
    }
}
