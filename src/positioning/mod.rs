pub mod fusion;
pub mod mpu;
pub mod odometry;
pub mod types;

use self::types::MovementDelta;
use defmt::info;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use mpu9250::MargMeasurements;
use crate::devices::mpu9250::MPU_CHANNEL;
use crate::devices::hall_sensor_3144::{ODOM_LEFT_CHANNEL, ODOM_RIGHT_CHANNEL};

#[embassy_executor::task]
pub async fn positioning_task() {
    let mut mpu_proc = mpu::MpuProcessor::new();
    let mut odom_proc = odometry::OdometryProcessor::new(odometry::OdometryConfig::default());
    let mut fusion_proc = fusion::SensorFusion::new();

    loop {
        // Collect data from sensors
        let mut mpu_result = mpu::MpuResult {
            delta: MovementDelta { dx: 0.0, dy: 0.0, d_theta: 0.0 },
            relative_mag: 0.0,
            dt: 0.02,
            accel_x: 0.0,
            accel_y: 0.0,
        };

        let odom_delta;
        let mut updated = false;

        // Non-blocking drain configures the frame
        while let Ok(data) = MPU_CHANNEL.try_receive() {
            mpu_result = mpu_proc.update(data.accel, data.gyro, data.mag);
            updated = true;
        }

        let mut left_ticks = 0;
        let mut right_ticks = 0;
        while let Ok(ticks) = ODOM_LEFT_CHANNEL.try_receive() {
            left_ticks += ticks;
        }
        while let Ok(ticks) = ODOM_RIGHT_CHANNEL.try_receive() {
            right_ticks += ticks;
        }

        if left_ticks != 0 || right_ticks != 0 {
            odom_proc.add_ticks(left_ticks, right_ticks);
            updated = true;
        }
        odom_delta = odom_proc.update();

        if updated {
            let state = fusion_proc.update(odom_delta, mpu_result);
            info!("Position -> X: {}, Y: {}, Theta: {}", state.x, state.y, state.theta);
        }

        // Processing loop rate
        embassy_time::Timer::after_millis(20).await;
    }
}
