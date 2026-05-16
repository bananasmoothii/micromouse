pub mod fusion;
pub mod mpu;
pub mod odometry;
pub mod types;

use alloc::format;
use core::cell::Cell;
use crate::devices::mpu9250::Mpu9250Sensor;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use crate::flash_log;
use crate::positioning::types::PositionState;
use crate::utils::{CellMutexUtils, DurationUtils};
use defmt::{debug, error};
use crate::positioning::odometry::get_odom_delta;
use core::convert::TryFrom;
use core::sync::atomic::Ordering::Relaxed;
use crate::trajectory::{get_current_fusion_mode};

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
    // Average 3 raw mag readings (20 ms apart) to get a stable hard iron reference at startup.
    let mut sum = (0.0f32, 0.0f32);
    let mut count = 0u8;
    while count < 3 {
        20.ms_timer().await;
        if let Some(data) = mpu.read() {
            let x = data.mag[0];
            let y = -data.mag[1]; // axis inversion same as update()
            debug!("Mag measurement for hard iron calib: x: {} y: {}", x, y);
            if x.is_finite() && y.is_finite() {
                sum.0 += x;
                sum.1 += y;
                count += 1;
            }
        }
    }
    let hard_iron = (sum.0 / count as f32, sum.1 / count as f32);

    let mut mpu_proc = mpu::MpuProcessor::new(hard_iron);
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


        let gyro_d_deg = mpu_result.d_theta.to_degrees();
        let odom_d_deg = odom_delta.d_theta.to_degrees();
        let mag_deg = mpu_result.relative_mag.to_degrees();
        let mode = get_current_fusion_mode();
        let state = fusion_proc.update(odom_delta, mpu_result, mode);
        CURRENT_POS.set(state);

        flash_log!("Fusion: {}", state);

        20.ms_timer().await;
    }
}
