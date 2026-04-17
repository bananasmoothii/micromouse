use crate::positioning::types::MovementDelta;
use defmt::{info, trace};
use embassy_time::Instant;
use mpu9250::MargMeasurements; // Ensure `mpu9250` in Cargo.toml has this

pub struct MpuProcessor {
    last_yaw: f32, // Rad
    last_update_time: Option<Instant>,
}

impl MpuProcessor {
    pub fn new() -> Self {
        Self {
            last_yaw: 0.0,
            last_update_time: None,
        }
    }

    /// Process raw data from MPU9250 to determine rotational change in yaw (heading).
    pub fn update(&mut self, _accel: [f32; 3], mut gyro: [f32; 3], _mag: [f32; 3]) -> MovementDelta {
        let now = Instant::now();

        // Invert Y and Z per upside-down configuration
        gyro[1] = -gyro[1];
        gyro[2] = -gyro[2];

        // Ensure time elapsed is known
        let dt = if let Some(last_time) = self.last_update_time {
            let elapsed = now.duration_since(last_time).as_micros();
            elapsed as f32 / 1_000_000.0 // Convert to seconds
        } else {
            0.0 // First update
        };

        // Angular velocity around Z is essentially yaw rate
        let yaw_rate = gyro[2]; // assuming rad/s, check sensor configuration defaults

        let delta_yaw = yaw_rate * dt;

        self.last_update_time = Some(now);

        let delta = MovementDelta {
            dx: 0.0,
            dy: 0.0,
            dtheta: delta_yaw,
        };

        delta
    }
}
