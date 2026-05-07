use crate::devices::buzzer::{BUZZER_CHANNEL, BuzzerTask};
use crate::devices::motors::{LEFT_FORWARD, RIGHT_FORWARD, WheelSide};
use crate::utils::{DurationUtils, HertzUtils};
use core::sync::atomic::{AtomicI32, AtomicU32, Ordering};
use embassy_stm32::exti::ExtiInput;
use embassy_time::Instant;

pub static LEFT_TICKS_TOTAL: AtomicI32 = AtomicI32::new(0);
pub static RIGHT_TICKS_TOTAL: AtomicI32 = AtomicI32::new(0);

pub static LEFT_LAST_TICK_US: AtomicU32 = AtomicU32::new(0);
pub static RIGHT_LAST_TICK_US: AtomicU32 = AtomicU32::new(0);

pub static LEFT_TICK_INTERVAL_US: AtomicU32 = AtomicU32::new(0);
pub static RIGHT_TICK_INTERVAL_US: AtomicU32 = AtomicU32::new(0);

#[embassy_executor::task(pool_size = 2)]
pub async fn hall_sensor_continuous_measuring(mut pin: ExtiInput<'static>, side: WheelSide) {
    loop {
        pin.wait_for_rising_edge().await;
        let now_us = Instant::now().as_micros() as u32;
        let (forward_flag, ticks_total, last_tick_us, tick_interval_us) = match side {
            WheelSide::Left => (
                &LEFT_FORWARD,
                &LEFT_TICKS_TOTAL,
                &LEFT_LAST_TICK_US,
                &LEFT_TICK_INTERVAL_US,
            ),
            WheelSide::Right => (
                &RIGHT_FORWARD,
                &RIGHT_TICKS_TOTAL,
                &RIGHT_LAST_TICK_US,
                &RIGHT_TICK_INTERVAL_US,
            ),
        };

        let delta = if forward_flag.load(Ordering::Relaxed) { 1 } else { -1 };
        ticks_total.fetch_add(delta, Ordering::Relaxed);
        let prev = last_tick_us.swap(now_us, Ordering::Relaxed);
        if prev != 0 {
            tick_interval_us.store(now_us.wrapping_sub(prev), Ordering::Relaxed);
        }

        let _ = BUZZER_CHANNEL.try_send(BuzzerTask {
            freq: match side {
                WheelSide::Left => 1000.hz(),
                WheelSide::Right => 1500.hz()
            },
            duration: 20.ms(),
        });
    }
}
