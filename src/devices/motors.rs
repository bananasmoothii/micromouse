use core::sync::atomic::{AtomicBool, Ordering};
use defmt::error;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::{peripherals, Peri};
use embassy_stm32::gpio::{Level, Output, Pin, Speed};
use embassy_stm32::timer::{Channel, GeneralInstance4Channel};
use embassy_stm32::timer::simple_pwm::SimplePwm;
use crate::utils::DurationUtils;

pub struct Motor<'d, T: GeneralInstance4Channel> {
    in_a: Output<'d>,
    in_b: Output<'d>,
    pwm: SimplePwm<'d, T>,
    pwm_channel: Channel,
    driving_forward: &'static AtomicBool,
}

impl<'d, T: GeneralInstance4Channel> Motor<'d, T> {
    pub fn new(
        in_a_pin: Peri<'d, impl Pin>,
        in_b_pin: Peri<'d, impl Pin>,
        pwm: SimplePwm<'d, T>,
        pwm_channel: Channel,
        driving_forward: &'static AtomicBool,
    ) -> Self {
        let in_a = Output::new(in_a_pin, Level::Low, Speed::Medium);
        let in_b = Output::new(in_b_pin, Level::Low, Speed::Medium);

        Self {
            in_a,
            in_b,
            pwm,
            pwm_channel,
            driving_forward,
        }
    }

    pub fn set_speed(&mut self, speed: f32) {
        assert!(
            -1.0 <= speed && speed <= 1.0,
            "Speed must be between -1.0 and 1.0"
        );

        if speed == 0.0 {
            self.brake();
            return;
        }

        if speed > 0.0 {
            self.driving_forward.store(true, Ordering::Relaxed);
            self.in_a.set_high();
            self.in_b.set_low();
        } else {
            self.driving_forward.store(false, Ordering::Relaxed);
            self.in_a.set_low();
            self.in_b.set_high();
        }

        let duty = (self.pwm.max_duty_cycle() as f32 * speed) as u32;

        let mut pwm_channel = self.pwm.channel(self.pwm_channel);
        pwm_channel.set_duty_cycle(duty);

        if !pwm_channel.is_enabled() {
            pwm_channel.enable();
        }
    }

    pub fn brake(&mut self) {
        self.in_a.set_high();
        self.in_b.set_high();
        let mut pwm_channel = self.pwm.channel(self.pwm_channel);
        pwm_channel.disable();
        pwm_channel.set_duty_cycle(0);
    }

    pub fn neutral(&mut self) {
        self.in_a.set_low();
        self.in_b.set_low();
        let mut pwm_channel = self.pwm.channel(self.pwm_channel);
        pwm_channel.disable();
        pwm_channel.set_duty_cycle(0);
    }
}

#[embassy_executor::task]
pub async fn overcurrent_protection_task(
    mut adc_module: Adc<'static, peripherals::ADC2>,
    mut sense_motor1: Peri<'static, peripherals::PA4>,
    mut sense_motor2: Peri<'static, peripherals::PB0>,
    enable_motor1: Peri<'static, peripherals::PA0>,
    enable_motor2: Peri<'static, peripherals::PA1>,
) -> ! {
    let mut enable1 = Output::new(enable_motor1, Level::High, Speed::Low);
    let mut enable2 = Output::new(enable_motor2, Level::High, Speed::Low);

    // Constantes basées sur la datasheet VNH2SP30
    const K: f32 = 11370.0; // Ratio typique
    const R_SENSE: f32 = 1500.0; // Résistance sur le shield en Ohms
    const ADC_MAX: f32 = 4095.0; // Résolution 12-bit
    const V_REF: f32 = 3.3; // Tension de référence Nucleo

    loop {
        // Motor 1
        let mut max_raw1 = 0;
        for _ in 0..20 {
            let raw = adc_module.blocking_read(&mut sense_motor1, SampleTime::CYCLES84);
            if raw > max_raw1 {
                max_raw1 = raw;
            }
        }

        // Motor 2
        let mut max_raw2 = 0;
        for _ in 0..20 {
            let raw = adc_module.blocking_read(&mut sense_motor2, SampleTime::CYCLES84);
            if raw > max_raw2 {
                max_raw2 = raw;
            }
        }

        let v_sense1 = (max_raw1 as f32 / ADC_MAX) * V_REF;
        let v_sense2 = (max_raw2 as f32 / ADC_MAX) * V_REF;

        let current_amps1 = (v_sense1 / R_SENSE) * K;
        let current_amps2 = (v_sense2 / R_SENSE) * K;

        // Both channels at saturation → sense pins are floating (motor driver unpowered).
        // A real overcurrent on small motors stays well below 15 A.
        // if current_amps1 < 15.0 || current_amps2 < 15.0 {
        if current_amps1 > 4.0 || current_amps2 > 4.0 {
            error!("Overcurrent ! M1: {} A, M2: {} A", current_amps1, current_amps2);
            enable1.set_low();
            enable2.set_low();

            // Wait 5 seconds before attempting recovery
            5.s_timer().await;

            // Optional auto-recovery:
            // OVERCURRENT_FAULT.store(false, Ordering::Relaxed);
        }

        20.ms_timer().await;
    }
}
