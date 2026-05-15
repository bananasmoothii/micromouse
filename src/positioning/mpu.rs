use core::f32::consts::PI;
use embassy_time::Instant;
use micromath::F32Ext;

#[derive(Default)]
pub struct MpuResult {
    pub d_theta: f32,
    pub relative_mag: f32,
    pub dt: f32,
}

pub struct MpuProcessor {
    last_update_time: Option<Instant>,
    initial_mag_heading: Option<f32>,
    hard_iron_x: f32,
    hard_iron_y: f32,
}

impl MpuProcessor {
    pub fn new(hard_iron: (f32, f32)) -> Self {
        Self {
            last_update_time: None,
            initial_mag_heading: None,
            hard_iron_x: hard_iron.0,
            hard_iron_y: hard_iron.1,
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

        // Subtract hard iron offset before computing heading.
        let cal_x = mag[0] - self.hard_iron_x;
        let cal_y = mag[1] - self.hard_iron_y;
        let absolute_mag = cal_y.atan2(cal_x);
        // Guard against NaN from a bad mag reading so it never poisons the fused theta.
        let relative_mag = if absolute_mag.is_finite() {
            let initial = *self.initial_mag_heading.get_or_insert(absolute_mag);
            let mut r = absolute_mag - initial;
            while r > PI { r -= 2.0 * PI; }
            while r < -PI { r += 2.0 * PI; }
            r
        } else {
            0.0 // hold zero until a valid reading arrives
        };

        MpuResult {
            d_theta,
            relative_mag,
            dt,
        }
    }
}
