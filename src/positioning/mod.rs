pub mod fusion;
pub mod mpu;
pub mod odometry;
pub mod types;

use core::cell::Cell;
use self::types::MovementDelta;
use defmt::info;
use crate::devices::mpu9250::LATEST_MPU;
use crate::devices::hall_sensor_3144::{LEFT_TICKS_TOTAL, RIGHT_TICKS_TOTAL};
use core::sync::atomic::Ordering;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use crate::positioning::types::Position2D;

pub static CURRENT_STATE: Mutex<CriticalSectionRawMutex, Cell<Position2D>> = Mutex::new(Cell::new(Position2D {
    x: 0.0,
    y: 0.0,
    theta: 0.0,
    v_x: 0.0,
    v_y: 0.0,
}));

#[embassy_executor::task]
pub async fn positioning_task() {
    let mut mpu_proc = mpu::MpuProcessor::new();
    let mut odom_proc = odometry::OdometryProcessor::new();
    let mut fusion_proc = fusion::SensorFusion::new();

    let mut last_left_ticks = LEFT_TICKS_TOTAL.load(Ordering::Relaxed);
    let mut last_right_ticks = RIGHT_TICKS_TOTAL.load(Ordering::Relaxed);

    loop {
        // Collect data from sensors
        let mut mpu_result = mpu::MpuResult {
            delta: MovementDelta { dx: 0.0, dy: 0.0, d_theta: 0.0 },
            relative_mag: 0.0,
            dt: 0.02,
            accel_x: 0.0,
            accel_y: 0.0,
        };

        if let Some(data) = LATEST_MPU.lock(|cell| cell.get()) {
            mpu_result = mpu_proc.update(data.accel, data.gyro, data.mag);
        }

        let current_left_ticks = LEFT_TICKS_TOTAL.load(Ordering::Relaxed);
        let current_right_ticks = RIGHT_TICKS_TOTAL.load(Ordering::Relaxed);

        let delta_left = current_left_ticks - last_left_ticks;
        let delta_right = current_right_ticks - last_right_ticks;

        last_left_ticks = current_left_ticks;
        last_right_ticks = current_right_ticks;

        odom_proc.add_ticks(delta_left, delta_right);
        let odom_delta = odom_proc.update();

        let state = fusion_proc.update(odom_delta, mpu_result);
        info!("Position -> X: {}, Y: {}, Theta: {}", state.x, state.y, state.theta);
        CURRENT_STATE.lock(|cell| cell.set(state));

        // Processing loop rate
        embassy_time::Timer::after_millis(20).await;
    }
}
