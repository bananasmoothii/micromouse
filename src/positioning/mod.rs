pub mod fusion;
pub mod mpu;
pub mod odometry;
pub mod types;

use core::cell::Cell;
use crate::devices::mpu9250::Mpu9250Sensor;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use crate::positioning::types::PositionState;
use crate::utils::{CellMutexUtils, DurationUtils};
use defmt::{debug, error, info};
use crate::positioning::odometry::{get_odom_delta, left_wheel_velocity, right_wheel_velocity};
use embassy_time::Instant;

/// `v_forward` is derived from wheel tick timestamps, not accelerometer integration.
/// In curves, skid-steering lateral slip makes it slightly overestimate actual forward velocity,
/// but corners are short enough that the EKF self-corrects before error accumulates.
pub static CURRENT_POS: Mutex<CriticalSectionRawMutex, Cell<PositionState>> = Mutex::new(Cell::new(PositionState {
    x: 0.0,
    y: 0.0,
    theta: 0.0,
    v_forward: 0.0,
}));

#[embassy_executor::task]
pub async fn positioning_task(mpu: &'static mut Mpu9250Sensor) {
    let mut mpu_proc = mpu::MpuProcessor::new();
    let mut fusion_proc = fusion::SensorFusion::new();

    loop {
        let odom_delta = get_odom_delta();

        let mpu_result = mpu.read()
            .map(|data| mpu_proc.update(data.gyro, data.mag));
        let mpu_result = match mpu_result {
            Some(result) => result,
            None => {
                error!("MPU read error");
                20.ms_timer().await;
                continue;
            }
        };

        let state = fusion_proc.update(odom_delta, mpu_result);
        // let now_us = Instant::now().as_micros() as u32;
        // debug!(
        //     "speed — left: {} m/s, right: {} m/s, combined: {} m/s",
        //     left_wheel_velocity(now_us),
        //     right_wheel_velocity(now_us),
        //     state.v_forward,
        // );
        CURRENT_POS.set(state);

        20.ms_timer().await;
    }
}
