use embassy_stm32::exti::ExtiInput;

#[embassy_executor::task]
pub async fn hall_sensor_continuous_measuring(mut pin: ExtiInput<'static>, callback: &'static dyn Fn()) {
    loop {
        pin.wait_for_rising_edge().await;
        callback();
    }
}