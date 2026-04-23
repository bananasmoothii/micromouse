use core::sync::atomic::{AtomicI32, Ordering};
use embassy_stm32::exti::ExtiInput;

pub static LEFT_TICKS_TOTAL: AtomicI32 = AtomicI32::new(0);
pub static RIGHT_TICKS_TOTAL: AtomicI32 = AtomicI32::new(0);

pub enum WheelSide {
    Left,
    Right,
}

#[embassy_executor::task]
pub async fn hall_sensor_continuous_measuring(mut pin: ExtiInput<'static>, side: WheelSide) {
    loop {
        pin.wait_for_rising_edge().await;
        match side {
            WheelSide::Left => {
                LEFT_TICKS_TOTAL.fetch_add(1, Ordering::Relaxed);
            }
            WheelSide::Right => {
                RIGHT_TICKS_TOTAL.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}
