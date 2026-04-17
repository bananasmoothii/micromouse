pub mod fusion;
pub mod mpu;
pub mod odometry;
pub mod types;

use self::types::MovementDelta;
use defmt::info;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use mpu9250::MargMeasurements;

// Using Channels makes life cleaner than handling callbacks within callbacks.
pub static MPU_CHANNEL: Channel<CriticalSectionRawMutex, MargMeasurements<[f32; 3]>, 4> = Channel::new();
pub static ODOM_LEFT_CHANNEL: Channel<CriticalSectionRawMutex, i32, 4> = Channel::new();
pub static ODOM_RIGHT_CHANNEL: Channel<CriticalSectionRawMutex, i32, 4> = Channel::new();

#[embassy_executor::task]
pub async fn positioning_task() {
    let mut mpu_proc = mpu::MpuProcessor::new();
    let mut odom_proc = odometry::OdometryProcessor::new(odometry::OdometryConfig::default());
    let mut fusion_proc = fusion::SensorFusion::new();

    loop {
        // Collect data from sensors
        let mut mpu_delta = MovementDelta { dx: 0.0, dy: 0.0, dtheta: 0.0 };
        let mut mag_heading = 0.0;
        let mut accel_x = 0.0;
        let mut accel_y = 0.0;
        let mut loop_dt = 0.01; // initial default fallback

        let odom_delta;
        let mut updated = false;

        // Non-blocking drain configures the frame
        while let Ok(data) = MPU_CHANNEL.try_receive() {
            let res = mpu_proc.update(data.accel, data.gyro, data.mag);
            mpu_delta = res.0;
            mag_heading = res.1;
            loop_dt = res.2;
            accel_x = res.3;
            accel_y = res.4;
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
            odom_delta = odom_proc.update();
            updated = true;
        } else {
            // Update even if no new ticks, based on 0-delta
            odom_delta = odom_proc.update();
        }

        if updated {
            let state = fusion_proc.update(loop_dt, odom_delta, mpu_delta, accel_x, accel_y, mag_heading);
            info!("Position -> X: {}, Y: {}, Theta: {}", state.x, state.y, state.theta);
        }

        // Processing loop rate
        embassy_time::Timer::after_millis(10).await;
    }
}
