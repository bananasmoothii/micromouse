//! PWM buzzer driver on TIM1 CH4 (PA11).
//!
//! Consumers send [`BuzzerTask`] commands to [`BUZZER_CHANNEL`]; the [`buzzer_task`] Embassy task
//! dequeues them and applies the frequency + duration to the PWM peripheral.
//!
//! At startup the task plays a short ascending melody ([`INIT_MUSIC`]) so you can confirm the
//! firmware is alive without a debug probe.  Set `STARTING_BIP = false` to disable it.
//!
//! ## Usage
//! ```rust,ignore
//! BUZZER_CHANNEL.send(BuzzerTask { freq: 1047.hz(), duration: 500.ms() }).await;
//! ```

use cortex_m::prelude::_embedded_hal_Pwm;
use embassy_stm32::peripherals::TIM1;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use crate::utils::{DurationUtils, HertzUtils};

/// A single buzzer command: play `freq` Hz for `duration`, then silence for 10 ms.
pub struct BuzzerTask {
    pub freq: Hertz,
    pub duration: Duration,
}

/// Lock-free FIFO between producers (any task) and the [`buzzer_task`] consumer.
/// Capacity: 32 commands (non-blocking `try_send` drops silently when full).
pub static BUZZER_CHANNEL: Channel<CriticalSectionRawMutex, BuzzerTask, 32> = Channel::new();

const STARTING_BIP: bool = true;

#[embassy_executor::task]
pub async fn buzzer_task(
    mut pwm: SimplePwm<'static, TIM1>,
    pwm_channel: embassy_stm32::timer::Channel,
) {
    if STARTING_BIP {
        for freq in INIT_MUSIC {
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

/// C5 → E5 → G5 → C6 startup jingle (frequencies in Hz).
/// Also played in reverse on low-battery shutdown.
pub const INIT_MUSIC: [u32; 4] = [523, 659, 784, 1047];
