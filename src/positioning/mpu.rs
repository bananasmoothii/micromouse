use core::f32::consts::PI;
use embassy_time::Instant;
use micromath::F32Ext;

#[derive(Default)]
pub struct MpuResult {
    pub d_theta: f32,
    /// Compass bearing normalized relative to the robot's startup orientation, in [-PI, PI].
    pub relative_mag: f32,
    pub dt: f32,
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

    pub fn update(&mut self, mut gyro: [f32; 3], mut mag: [f32; 3]) -> MpuResult {
        let now = Instant::now();

        // Invert Y and Z per upside-down mounting
        gyro[1] = -gyro[1];
        gyro[2] = -gyro[2];
        mag[1] = -mag[1];
        mag[2] = -mag[2];

        let dt = if let Some(last_time) = self.last_update_time {
            now.duration_since(last_time).as_micros() as f32 / 1_000_000.0
        } else {
            0.0
        };
        self.last_update_time = Some(now);

        let d_theta = gyro[2] * dt;

        let absolute_mag = mag[1].atan2(mag[0]);
        let initial = *self.initial_mag_heading.get_or_insert(absolute_mag);
        let mut relative_mag = absolute_mag - initial;
        while relative_mag > PI { relative_mag -= 2.0 * PI; }
        while relative_mag < -PI { relative_mag += 2.0 * PI; }

        MpuResult { d_theta, relative_mag, dt }
    }
}
