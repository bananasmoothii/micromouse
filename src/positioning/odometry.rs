use crate::positioning::types::MovementDelta;
use core::f32::consts::PI;

#[derive(Clone, Copy, Debug)]
pub struct OdometryConfig {
    /// meters
    pub wheel_radius: f32,
    /// meters (distance between left and right wheels)
    pub wheel_base: f32,

    pub ticks_per_revolution: f32,
}

impl Default for OdometryConfig {
    fn default() -> Self {
        Self {
            wheel_radius: 0.020,
            wheel_base: 0.078,
            ticks_per_revolution: 2.0, // Very low resolution, requires accelerometer interpolation
        }
    }
}

pub struct OdometryProcessor {
    config: OdometryConfig,
    left_ticks: i32,
    right_ticks: i32,
    distance_per_tick: f32,
}

impl OdometryProcessor {
    pub fn new(config: OdometryConfig) -> Self {
        let distance_per_tick = 2.0 * PI * config.wheel_radius / config.ticks_per_revolution;
        Self {
            config,
            left_ticks: 0,
            right_ticks: 0,
            distance_per_tick,
        }
    }

    /// Update tick counts. Should be called when wheel sensors trigger.
    pub fn add_ticks(&mut self, left_delta: i32, right_delta: i32) {
        self.left_ticks += left_delta;
        self.right_ticks += right_delta;
    }

    /// Processes accumulated ticks to calculate movement delta and calls callback.
    pub fn update(&mut self) -> MovementDelta {
        let d_left = self.left_ticks as f32 * self.distance_per_tick;
        let d_right = self.right_ticks as f32 * self.distance_per_tick;

        self.left_ticks = 0;
        self.right_ticks = 0;

        let d_center = (d_right + d_left) / 2.0;
        let d_theta = (d_right - d_left) / self.config.wheel_base;

        let delta = MovementDelta {
            dx: d_center,
            dy: 0.0, // Assumes local straight-line movement for this step
            dtheta: d_theta,
        };

        delta
    }
}
