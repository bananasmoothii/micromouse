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
use defmt::{error, info};
use crate::positioning::odometry::get_odom_delta;

/// `v_forward` is the EMA-smoothed tick-count velocity: ticks accumulated over the 20 ms fusion
/// window divided by dt, giving a true average with no aliasing artefacts (see fusion.rs for
/// details). In curves it slightly underestimates forward speed (chord vs arc), but the error
/// is negligible over the short 20 ms windows.
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
        CURRENT_POS.set(state);

        20.ms_timer().await;
    }
}
