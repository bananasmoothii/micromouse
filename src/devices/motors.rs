use defmt::{error, trace, warn};
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::gpio::{Level, Output, Pin, Speed};
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_stm32::timer::{Channel, GeneralInstance4Channel};
use embassy_stm32::{Peri, peripherals, PeripheralType};

pub struct Motor<'d, T: GeneralInstance4Channel> {
    in_a: Output<'d>,
    in_b: Output<'d>,
    pwm: SimplePwm<'d, T>,
    pwm_channel: Channel,
}

#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum MotorDirection {
    Neutral = 0,
    Forward = 1,
    Reverse = 2,
    Break = 3,
}

impl<'d, T: GeneralInstance4Channel> Motor<'d, T> {
    pub fn new(
        spawner: &Spawner,
        in_a_pin: Peri<'d, impl Pin>,
        in_b_pin: Peri<'d, impl Pin>,
        pwm: SimplePwm<'d, T>,
        pwm_channel: Channel,
        adc_module: Adc<'static, peripherals::ADC2>,
        current_sense_pin: Peri<'static, peripherals::PA4>,
        enable_pin: Peri<'static, impl Pin>,
    ) -> Self {
        let in_a = Output::new(in_a_pin, Level::Low, Speed::Medium);
        let in_b = Output::new(in_b_pin, Level::Low, Speed::Medium);
        let enable_output = Output::new(enable_pin, Level::Low, Speed::Medium);

        spawner
            .spawn(overcurrent_protection_task(
                adc_module,
                current_sense_pin,
                enable_output,
            ))
            .unwrap();

        Self {
            in_a,
            in_b,
            pwm,
            pwm_channel,
        }
    }

    pub fn set_direction(&mut self, direction: MotorDirection) {
        match direction {
            MotorDirection::Neutral => {
                self.in_a.set_low();
                self.in_b.set_low();
            }
            MotorDirection::Forward => {
                self.in_a.set_high();
                self.in_b.set_low();
            }
            MotorDirection::Reverse => {
                self.in_a.set_low();
                self.in_b.set_high();
            }
            MotorDirection::Break => {
                self.in_a.set_high();
                self.in_b.set_high();
            }
        }
    }

    pub fn set_speed(&mut self, speed: f32) {
        assert!(
            0.0 <= speed && speed <= 1.0,
            "Speed must be between 0.0 and 1.0"
        );

        let duty = (self.pwm.max_duty_cycle() as f32 * speed) as u32;

        let mut channel = self.pwm.channel(self.pwm_channel);
        channel.set_duty_cycle(duty);

        if speed == 0.0 {
            channel.disable();
        } else {
            channel.enable();
        }
    }
}

#[embassy_executor::task]
pub async fn overcurrent_protection_task(
    mut adc_module: Adc<'static, peripherals::ADC2>,
    mut current_sense_pin: Peri<'static, peripherals::PA4>,
    mut enable_pin: Output<'static>,
) -> ! {
    enable_pin.set_high();

    // Constantes basées sur la datasheet VNH2SP30
    const K: f32 = 11370.0; // Ratio typique
    const R_SENSE: f32 = 1500.0; // Résistance sur le shield en Ohms
    const ADC_MAX: f32 = 4095.0; // Résolution 12-bit
    const V_REF: f32 = 3.3; // Tension de référence Nucleo

    loop {
        // Read the raw 12-bit value from the ADC
        // Take a burst of 10 samples to ensure we catch the PWM "ON" phase
        let mut max_raw = 0;
        for _ in 0..20 {
            let raw = adc_module.blocking_read(&mut current_sense_pin, SampleTime::CYCLES144);
            if raw > max_raw {
                max_raw = raw;
            }
        }

        // 1. Convert raw ADC value to Voltage
        // $V_{SENSE} = (raw / 4095) * 3.3V$
        let v_sense = (max_raw as f32 / ADC_MAX) * V_REF;

        // 2. Convert Voltage to Sense Current (using Ohm's Law: I = V/R)
        // $I_{SENSE} = V_{SENSE} / 1500\Omega$
        let i_sense = v_sense / R_SENSE;

        // 3. Convert Sense Current to actual Motor Current
        // $I_{OUT} = I_{SENSE} * K$
        let current_amps = i_sense * K;

        if current_amps > 4.0 {
            error!("Overcurrent ! {} A (raw: {})", current_amps, max_raw);
            enable_pin.set_low();
            panic!("Overcurrent detected");
        }

        // trace!("current: {} A (raw: {})", current_amps, max_raw);

        embassy_time::Timer::after(embassy_time::Duration::from_millis(20)).await;
    }
}
