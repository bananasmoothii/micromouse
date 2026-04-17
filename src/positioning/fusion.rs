use crate::positioning::types::{MovementDelta, Position2D};
use micromath::F32Ext;

pub struct SensorFusion {
    state: Position2D,
    alpha: f32, // Complementary filter tuning constant
}

impl SensorFusion {
    pub fn new(alpha: f32) -> Self {
        Self {
            state: Position2D::default(),
            alpha,
        }
    }

    pub fn update(&mut self, odom_delta: MovementDelta, mpu_delta: MovementDelta) -> Position2D {
        // Simple complementary filter for delta theta
        let combined_dtheta =
            self.alpha * mpu_delta.dtheta + (1.0 - self.alpha) * odom_delta.dtheta;

        let delta_center = odom_delta.dx; // dx assumes center distance from odometry

        self.state.theta += combined_dtheta;
        self.state.x += delta_center * self.state.theta.cos();
        self.state.y += delta_center * self.state.theta.sin(); // Corrected for rotation! (often delta_center * sin(theta + combined_dtheta/2))

        self.state.clone()
    }
}
