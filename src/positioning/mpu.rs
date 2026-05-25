//! MPU-9250 data post-processing: axis remapping, gyro integration, and magnetometer heading.
//!
//! ## Axis convention
//! The IMU is mounted upside-down on the PCB. Y and Z axes are inverted in software so that
//! the sensor's output matches the robot's right-hand coordinate frame (z pointing up).
//!
//! ## Hard-iron calibration
//! A static hard-iron offset `(hard_iron_x, hard_iron_y)` is subtracted from every
//! magnetometer reading.  The offset is estimated at startup from 3 consecutive readings
//! with the robot stationary (see [`crate::positioning`]).  This compensates for the DC
//! magnetic bias caused by the PCB traces and permanent magnets in the drive motors.
//!
//! ## Relative magnetometer heading
//! The absolute compass heading is computed as `atan2(cal_y, cal_x)`.  To make it independent
//! of the robot's starting orientation, the *first* finite reading is saved as `initial_mag_heading`
//! and all subsequent readings are expressed as the difference from that baseline.
//! The output `relative_mag` is wrapped to `(−π, π]`.

use core::f32::consts::PI;
use embassy_time::Instant;
use micromath::F32Ext;

/// Output of one [`MpuProcessor::update`] call.
#[derive(Default)]
pub struct MpuResult {
    /// Heading change in radians from gyro integration over `dt` (positive = CCW).
    pub d_theta: f32,
    /// Magnetometer heading relative to startup orientation, wrapped to (−π, π] radians.
    pub relative_mag: f32,
    /// Time since the previous call in seconds. Zero on the first call.
    pub dt: f32,
}

/// Stateful processor for raw MPU-9250 samples. One instance lives in [`crate::positioning::positioning_task`].
pub struct MpuProcessor {
    last_update_time: Option<Instant>,
    /// Absolute magnetometer heading at first valid reading — used as the zero reference.
    initial_mag_heading: Option<f32>,
    hard_iron_x: f32,
    hard_iron_y: f32,
}

impl MpuProcessor {
    /// Creates a new processor with the given static hard-iron offset `(x, y)`.
    pub fn new(hard_iron: (f32, f32)) -> Self {
        Self {
            last_update_time: None,
            initial_mag_heading: None,
            hard_iron_x: hard_iron.0,
            hard_iron_y: hard_iron.1,
        }
    }

    /// Processes one sample from the IMU and returns the derived heading increments.
    ///
    /// `gyro` and `mag` are raw sensor values in the sensor's native units
    /// (rad/s for gyro, µT for mag).
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
