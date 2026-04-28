use crate::devices::buzzer::{BUZZER_CHANNEL, BuzzerTask};
use crate::utils::{DurationUtils, HertzUtils};
use core::sync::atomic::{AtomicBool, AtomicI32, Ordering};
use defmt::debug;
use embassy_stm32::exti::ExtiInput;

pub static LEFT_TICKS_TOTAL: AtomicI32 = AtomicI32::new(0);
pub static RIGHT_TICKS_TOTAL: AtomicI32 = AtomicI32::new(0);

/// Set by the motor driver to indicate current commanded direction.
/// true = forward, false = backward. Read by the hall sensor ISR to sign ticks.
pub static LEFT_FORWARD: AtomicBool = AtomicBool::new(true);
pub static RIGHT_FORWARD: AtomicBool = AtomicBool::new(true);

pub enum WheelSide {
    Left,
    Right,
}

#[embassy_executor::task(pool_size = 2)]
pub async fn hall_sensor_continuous_measuring(mut pin: ExtiInput<'static>, side: WheelSide) {
    loop {
        pin.wait_for_rising_edge().await;
        match side {
            WheelSide::Left => {
                let delta = if LEFT_FORWARD.load(Ordering::Relaxed) { 1 } else { -1 };
                LEFT_TICKS_TOTAL.fetch_add(delta, Ordering::Relaxed);
                let _ = BUZZER_CHANNEL.try_send(BuzzerTask {
                    freq: 1000.hz(),
                    duration: 20.ms(),
                });
            }
            WheelSide::Right => {
                let delta = if RIGHT_FORWARD.load(Ordering::Relaxed) { 1 } else { -1 };
                RIGHT_TICKS_TOTAL.fetch_add(delta, Ordering::Relaxed);
                let _ = BUZZER_CHANNEL.try_send(BuzzerTask {
                    freq: 1500.hz(),
                    duration: 20.ms(),
                });
            }
        }
    }
}
