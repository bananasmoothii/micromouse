use embassy_stm32::exti::ExtiInput;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

pub static ODOM_LEFT_CHANNEL: Channel<CriticalSectionRawMutex, i32, 4> = Channel::new();
pub static ODOM_RIGHT_CHANNEL: Channel<CriticalSectionRawMutex, i32, 4> = Channel::new();

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
                let _ = ODOM_LEFT_CHANNEL.try_send(1);
            }
            WheelSide::Right => {
                let _ = ODOM_RIGHT_CHANNEL.try_send(1);
            }
        }
    }
}