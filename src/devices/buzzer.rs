use cortex_m::prelude::_embedded_hal_Pwm;
use embassy_stm32::peripherals::TIM2;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use crate::utils::{DurationUtils, HertzUtils};

pub struct BuzzerTask {
    pub freq: Hertz,
    pub duration: Duration,
}

pub static BUZZER_CHANNEL: Channel<CriticalSectionRawMutex, BuzzerTask, 16> = Channel::new();

const STARTING_BIP: bool = true;

#[embassy_executor::task]
pub async fn buzzer_task(
    mut pwm: SimplePwm<'static, TIM2>,
    pwm_channel: embassy_stm32::timer::Channel,
) {
    if STARTING_BIP {
        for freq in [523, 659, 784, 1047] {
            BUZZER_CHANNEL
                .send(BuzzerTask {
                    freq: freq.hz(),
                    duration: 100.ms(),
                })
                .await;
        }
    }

    loop {
        let task = BUZZER_CHANNEL.receive().await;
        pwm.set_frequency(task.freq);
        pwm.set_duty(pwm_channel, pwm.max_duty_cycle() / 2);
        pwm.enable(pwm_channel);
        Timer::after(task.duration).await;
        pwm.disable(pwm_channel);

        10.ms_timer().await;
    }
}
