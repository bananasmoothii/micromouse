use crate::positioning::mpu::MpuResult;
use crate::positioning::odometry::{left_wheel_velocity, right_wheel_velocity};
use crate::positioning::types::{MovementDelta, Position2D};
use core::f32::consts::PI;
use embassy_time::Instant;
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
    /// Position process noise per step (uncertainty growth between odometry corrections)
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
            r_theta_odom: 1.0,  // High value → gyro dominates; odometry barely corrects solo-tick noise
            r_theta_mag: 5.0,   // Slightly more mag trust to compensate for reduced odom correction
            p_xy: 1.0,
            q_xy: 0.005,
            r_xy_odom: 0.02,
        }
    }

    pub fn update(&mut self, odom_delta: MovementDelta, mpu: MpuResult) -> Position2D {
        let dt = mpu.dt;
        let mpu_delta = mpu.delta;
        let mag_heading = mpu.relative_mag;

        // --- 1D Kalman Filter for Theta ---

        // 1. Predict (using MPU gyro delta)
        self.state.theta += mpu_delta.d_theta;
        self.p_theta += self.q_theta;

        // 2. Update (using Odometry delta)
        let z_odom = odom_delta.d_theta - mpu_delta.d_theta;
        let k_odom = self.p_theta / (self.p_theta + self.r_theta_odom);
        self.state.theta += k_odom * z_odom;
        self.p_theta = (1.0 - k_odom) * self.p_theta;

        // 3. Update (using Magnetometer absolute heading)
        let mut z_mag = mag_heading - self.state.theta;
        while z_mag > PI {
            z_mag -= 2.0 * PI;
        }
        while z_mag < -PI {
            z_mag += 2.0 * PI;
        }
        let k_mag = self.p_theta / (self.p_theta + self.r_theta_mag);
        self.state.theta += k_mag * z_mag;
        self.p_theta = (1.0 - k_mag) * self.p_theta;

        // Normalize theta to [-PI, PI]
        while self.state.theta > PI {
            self.state.theta -= 2.0 * PI;
        }
        while self.state.theta < -PI {
            self.state.theta += 2.0 * PI;
        }

        let (sin_theta, cos_theta) = self.state.theta.sin_cos();

        // --- 2D Position Kalman Filter ---

        // 1. Velocity from per-wheel tick timestamps.
        //    The effective period = max(elapsed_since_last_tick, last_tick_interval) so velocity
        //    naturally decreases as the robot slows down without needing another tick to fire.
        //    This is inherently adaptive: faster → shorter period → responsive;
        //    slower → longer period → smooth.
        let now_us = Instant::now().as_micros() as u32;
        let v_left = left_wheel_velocity(now_us);
        let v_right = right_wheel_velocity(now_us);
        let v_center = (v_left + v_right) / 2.0;
        self.state.v_x = v_center * cos_theta;
        self.state.v_y = v_center * sin_theta;

        // 2. Predict position using smoothed velocity
        self.state.x += self.state.v_x * dt;
        self.state.y += self.state.v_y * dt;
        self.p_xy += self.q_xy;

        // 3. Correct position using cumulative odometry
        let delta_center = odom_delta.dx;
        self.odom_global_x += delta_center * cos_theta;
        self.odom_global_y += delta_center * sin_theta;

        let z_x = self.odom_global_x - self.state.x;
        let z_y = self.odom_global_y - self.state.y;

        let k_xy = self.p_xy / (self.p_xy + self.r_xy_odom);
        self.state.x += k_xy * z_x;
        self.state.y += k_xy * z_y;
        self.p_xy = (1.0 - k_xy) * self.p_xy;

        self.state.clone()
    }
}
