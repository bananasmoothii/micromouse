use crate::positioning::types::MovementDelta;
use embassy_time::Instant;

pub struct MpuResult {
    pub delta: MovementDelta,
    pub relative_mag: f32,
    pub dt: f32,
    pub accel_x: f32,
    pub accel_y: f32,
}

pub struct MpuProcessor {
    last_update_time: Option<Instant>,
    initial_mag_heading: Option<f32>,
}

impl MpuProcessor {
    pub fn new() -> Self {
        Self {
            last_update_time: None,
            initial_mag_heading: None,
        }
    }

    /// Process raw data from MPU9250 to determine rotational change in yaw (heading).
    pub fn update(&mut self, mut accel: [f32; 3], mut gyro: [f32; 3], mut mag: [f32; 3]) -> MpuResult {
        let now = Instant::now();

        // Invert Y and Z per upside-down configuration
        gyro[1] = -gyro[1];
        gyro[2] = -gyro[2];
        mag[1] = -mag[1];
        mag[2] = -mag[2];
        accel[1] = -accel[1];
        accel[2] = -accel[2];

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

        // Compute absolute magnetometer heading
        let absolute_mag = micromath::F32Ext::atan2(mag[1], mag[0]);
        let initial = *self.initial_mag_heading.get_or_insert(absolute_mag);

        // Normalize compass heading relative to start direction [-PI, PI]
        let mut relative_mag = absolute_mag - initial;
        while relative_mag > core::f32::consts::PI { relative_mag -= 2.0 * core::f32::consts::PI; }
        while relative_mag < -core::f32::consts::PI { relative_mag += 2.0 * core::f32::consts::PI; }

        let delta = MovementDelta {
            dx: 0.0,
            dy: 0.0,
            dtheta: delta_yaw,
        };

        MpuResult {
            delta,
            relative_mag,
            dt,
            accel_x: accel[0],
            accel_y: accel[1],
        }
    }
}
