use cortex_m::prelude::_embedded_hal_Pwm;
use defmt::debug;
use embassy_stm32::gpio::Output;
use embassy_stm32::peripherals::TIM2;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};

pub struct BuzzerTask {
    pub freq: Hertz,
    pub duration: Duration,
}

pub static BUZZER_CHANNEL: Channel<CriticalSectionRawMutex, BuzzerTask, 16> = Channel::new();

#[embassy_executor::task]
pub async fn buzzer_task(
    mut pwm: SimplePwm<'static, TIM2>,
    pwm_channel: embassy_stm32::timer::Channel,
) {
    loop {
        let task = BUZZER_CHANNEL.receive().await;
        pwm.set_frequency(task.freq);
        pwm.set_duty(pwm_channel, pwm.max_duty_cycle() / 2);
        debug!("Enabling buzzer at {}", task.freq);
        pwm.enable(pwm_channel);
        Timer::after(task.duration).await;
        debug!("Disabling buzzer");
        pwm.disable(pwm_channel);

        // Timer::after(Duration::from_millis(100)).await;

    }
}
