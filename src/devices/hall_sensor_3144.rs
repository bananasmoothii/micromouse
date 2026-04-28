use crate::devices::buzzer::{BUZZER_CHANNEL, BuzzerTask};
use crate::utils::{DurationUtils, HertzUtils};
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, Ordering};
use embassy_stm32::exti::ExtiInput;
use embassy_time::Instant;

pub static LEFT_TICKS_TOTAL: AtomicI32 = AtomicI32::new(0);
pub static RIGHT_TICKS_TOTAL: AtomicI32 = AtomicI32::new(0);

/// Set by the motor driver to indicate current commanded direction.
/// true = forward, false = backward. Read by the hall sensor ISR to sign ticks.
pub static LEFT_FORWARD: AtomicBool = AtomicBool::new(true);
pub static RIGHT_FORWARD: AtomicBool = AtomicBool::new(true);

/// Timestamp of the most recent tick (µs since boot, wraps at ~71 min — fine for competition).
/// 0 means no tick has fired yet.
pub static LEFT_LAST_TICK_US: AtomicU32 = AtomicU32::new(0);
pub static RIGHT_LAST_TICK_US: AtomicU32 = AtomicU32::new(0);

/// Duration between the last two ticks, in µs. 0 until at least two ticks have fired.
pub static LEFT_TICK_INTERVAL_US: AtomicU32 = AtomicU32::new(0);
pub static RIGHT_TICK_INTERVAL_US: AtomicU32 = AtomicU32::new(0);

pub enum WheelSide {
    Left,
    Right,
}

#[embassy_executor::task(pool_size = 2)]
pub async fn hall_sensor_continuous_measuring(mut pin: ExtiInput<'static>, side: WheelSide) {
    loop {
        pin.wait_for_rising_edge().await;
        let now_us = Instant::now().as_micros() as u32;
        match side {
            WheelSide::Left => {
                let delta = if LEFT_FORWARD.load(Ordering::Relaxed) { 1 } else { -1 };
                LEFT_TICKS_TOTAL.fetch_add(delta, Ordering::Relaxed);
                let prev = LEFT_LAST_TICK_US.swap(now_us, Ordering::Relaxed);
                if prev != 0 {
                    LEFT_TICK_INTERVAL_US.store(now_us.wrapping_sub(prev), Ordering::Relaxed);
                }
                let _ = BUZZER_CHANNEL.try_send(BuzzerTask {
                    freq: 1000.hz(),
                    duration: 20.ms(),
                });
            }
            WheelSide::Right => {
                let delta = if RIGHT_FORWARD.load(Ordering::Relaxed) { 1 } else { -1 };
                RIGHT_TICKS_TOTAL.fetch_add(delta, Ordering::Relaxed);
                let prev = RIGHT_LAST_TICK_US.swap(now_us, Ordering::Relaxed);
                if prev != 0 {
                    RIGHT_TICK_INTERVAL_US.store(now_us.wrapping_sub(prev), Ordering::Relaxed);
                }
                let _ = BUZZER_CHANNEL.try_send(BuzzerTask {
                    freq: 1500.hz(),
                    duration: 20.ms(),
                });
            }
        }
    }
}
