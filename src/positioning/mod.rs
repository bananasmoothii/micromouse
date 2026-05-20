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
use defmt::{debug, error, warn};
use crate::devices::hall_sensor_3144::{LEFT_TICKS_CUMULATIVE, RIGHT_TICKS_CUMULATIVE};
use crate::positioning::odometry::get_odom_delta;
use core::sync::atomic::Ordering;

/// `v_forward` is the EMA-smoothed tick-count velocity: ticks accumulated over the 20 ms fusion
/// window divided by dt, giving a true average with no aliasing artefacts (see fusion.rs for
/// details). In curves it slightly underestimates forward speed (chord vs arc), but the error
/// is negligible over the short 20 ms windows.
pub static CURRENT_POS: Mutex<CriticalSectionRawMutex, Cell<PositionState>> = Mutex::new(Cell::new(PositionState {
    x: 0.0,
    y: 0.0,
    theta: 0.0,
    v_forward: 0.0,
    omega: 0.0,
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

    // --- Diagnostic state for the "v_forward decays to zero while wheels keep spinning" freeze.
    // Tracks consecutive 20 ms windows where odometry reported no movement. If this grows large
    // while the commanded speed is non-zero, the EXTI hall ISR has likely stopped firing.
    let mut zero_tick_streak: u32 = 0;
    let mut last_left_cum: i32 = LEFT_TICKS_CUMULATIVE.load(Ordering::Relaxed);
    let mut last_right_cum: i32 = RIGHT_TICKS_CUMULATIVE.load(Ordering::Relaxed);

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

        // NaN watchdog: catch the moment a non-finite value enters the fusion pipeline so we can
        // tell apart "EKF was poisoned" from "encoders went silent".
        if !mpu_result.dt.is_finite() || !mpu_result.d_theta.is_finite() || !mpu_result.relative_mag.is_finite() {
            warn!(
                "MPU produced non-finite values: dt={} d_theta={} relative_mag={}",
                mpu_result.dt, mpu_result.d_theta, mpu_result.relative_mag
            );
        }
        if !odom_delta.dx.is_finite() || !odom_delta.d_theta.is_finite() {
            warn!(
                "Odometry produced non-finite values: dx={} d_theta={}",
                odom_delta.dx, odom_delta.d_theta
            );
        }

        let state = fusion_proc.update(odom_delta, mpu_result);
        CURRENT_POS.set(state);

        if !state.x.is_finite() || !state.y.is_finite() || !state.theta.is_finite() || !state.v_forward.is_finite() {
            warn!(
                "Fusion state has non-finite values: x={} y={} theta={} v_forward={}",
                state.x, state.y, state.theta, state.v_forward
            );
        }

        // Detect hall-sensor freeze: cumulative tick counters haven't moved this window.
        // let left_cum = LEFT_TICKS_CUMULATIVE.load(Ordering::Relaxed);
        // let right_cum = RIGHT_TICKS_CUMULATIVE.load(Ordering::Relaxed);
        // let left_dt = left_cum.wrapping_sub(last_left_cum);
        // let right_dt = right_cum.wrapping_sub(last_right_cum);
        // last_left_cum = left_cum;
        // last_right_cum = right_cum;
        // if left_dt == 0 && right_dt == 0 {
        //     zero_tick_streak += 1;
        //     // 25 × 20 ms = 500 ms with zero ticks on both wheels: well past anything explainable
        //     // by quantization (one tick every ~22 ms at 1 m/s), so something is wrong.
        //     // Log once at the threshold and every ~1 s after, with the commanded-vs-actual state.
        //     if zero_tick_streak == 25 || zero_tick_streak % 50 == 0 {
        //         warn!(
        //             "No hall ticks for {} consecutive 20ms windows; fusion v_forward={}",
        //             zero_tick_streak, state.v_forward
        //         );
        //     }
        // } else if zero_tick_streak > 0 {
        //     if zero_tick_streak >= 25 {
        //         debug!(
        //             "Hall ticks resumed after {} silent windows (L+{} R+{})",
        //             zero_tick_streak, left_dt, right_dt
        //         );
        //     }
        //     zero_tick_streak = 0;
        // }

        // flash_log!("Fusion: {}", state);

        20.ms_timer().await;
    }
}
