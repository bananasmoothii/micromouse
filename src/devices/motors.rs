use crate::utils::DurationUtils;
use core::sync::atomic::{AtomicBool, Ordering};
use core::sync::atomic::Ordering::Relaxed;
use defmt::{error, trace};
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::gpio::{Level, Output, Pin, Speed};
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_stm32::timer::{Channel, GeneralInstance4Channel};
use embassy_stm32::{Peri, peripherals};
use crate::devices::battery::BATTERY_VOLTAGE_MV;

pub static LEFT_FORWARD: AtomicBool = AtomicBool::new(true);
pub static RIGHT_FORWARD: AtomicBool = AtomicBool::new(true);

pub enum WheelSide {
    Left,
    Right,
}

const LEFT_WHEEL_SPEED_FACTOR: f32 = 0.97;
const RIGHT_WHEEL_SPEED_FACTOR: f32 = 1.03;

/// PWM duty cycle → m/s, measured at CALIBRATION_VOLTAGE.
const PWM_TO_SPEED_FACTOR: f32 = 4.15;
/// Battery voltage (V) at which PWM_TO_SPEED_FACTOR was measured.
const CALIBRATION_VOLTAGE: f32 = 8.3;

const MIN_USABLE_PWM: f32 = 0.095;

/// Motors might not turn below this due to too low regime
pub const MIN_USABLE_SPEED: f32 = MIN_USABLE_PWM * PWM_TO_SPEED_FACTOR;

pub struct Motor<'d, T: GeneralInstance4Channel> {
    in_a: Output<'d>,
    in_b: Output<'d>,
    pwm: SimplePwm<'d, T>,
    pwm_channel: Channel,
    side: WheelSide,
}

impl<'d, T: GeneralInstance4Channel> Motor<'d, T> {
    pub fn new(
        in_a_pin: Peri<'d, impl Pin>,
        in_b_pin: Peri<'d, impl Pin>,
        pwm: SimplePwm<'d, T>,
        pwm_channel: Channel,
        side: WheelSide,
    ) -> Self {
        let in_a = Output::new(in_a_pin, Level::Low, Speed::Medium);
        let in_b = Output::new(in_b_pin, Level::Low, Speed::Medium);
        Self {
            in_a,
            in_b,
            pwm,
            pwm_channel,
            side,
        }
    }

    /// duty_cycle should be [-1.0, 1.0], -1.0 being max speed backwards and 1.0 being max speed
    /// forwards. 0.0 will call [Motor::brake].
    pub fn set_pwm(&mut self, mut duty_cycle: f32) {
        assert!(
            -1.0 <= duty_cycle && duty_cycle <= 1.0,
            "Speed must be between -1.0 and 1.0"
        );

        if duty_cycle == 0.0 {
            self.brake();
            return;
        }
        if duty_cycle.abs() < MIN_USABLE_PWM {
            duty_cycle = if duty_cycle > 0.0 { MIN_USABLE_PWM } else { -MIN_USABLE_PWM };
        }

        let (actual_speed, forward_flag) = match self.side {
            WheelSide::Left => (duty_cycle * LEFT_WHEEL_SPEED_FACTOR, &LEFT_FORWARD),
            WheelSide::Right => (duty_cycle * RIGHT_WHEEL_SPEED_FACTOR, &RIGHT_FORWARD),
        };

        if actual_speed >= 0.0 {
            forward_flag.store(true, Relaxed);
            self.in_a.set_high();
            self.in_b.set_low();
        } else {
            forward_flag.store(false, Relaxed);
            self.in_a.set_low();
            self.in_b.set_high();
        }

        let duty = (self.pwm.max_duty_cycle() as f32 * actual_speed) as u32;
        let mut pwm_channel = self.pwm.channel(self.pwm_channel);
        pwm_channel.set_duty_cycle(duty);
        if !pwm_channel.is_enabled() {
            pwm_channel.enable();
        }
    }

    /// Set speed in m/s. Can be negative or 0.0
    pub fn set_speed(&mut self, speed_m_s: f32) {
        let voltage = BATTERY_VOLTAGE_MV.load(Relaxed) as f32 / 1000.0;
        let factor = PWM_TO_SPEED_FACTOR * voltage / CALIBRATION_VOLTAGE;
        self.set_pwm(speed_m_s / factor)
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

    const K: f32 = 11370.0;
    const R_SENSE: f32 = 1500.0;
    const ADC_MAX: f32 = 4095.0;
    const V_REF: f32 = 3.3;

    loop {
        let mut max_raw1 = 0;
        for _ in 0..20 {
            let raw = adc_module.blocking_read(&mut sense_motor1, SampleTime::CYCLES84);
            if raw > max_raw1 {
                max_raw1 = raw;
            }
        }

        let mut max_raw2 = 0;
        for _ in 0..20 {
            let raw = adc_module.blocking_read(&mut sense_motor2, SampleTime::CYCLES84);
            if raw > max_raw2 {
                max_raw2 = raw;
            }
        }

        let current_amps1 = (max_raw1 as f32 / ADC_MAX) * V_REF / R_SENSE * K;
        let current_amps2 = (max_raw2 as f32 / ADC_MAX) * V_REF / R_SENSE * K;

        if current_amps1 > 4.0 || current_amps2 > 4.0 {
            error!(
                "Overcurrent ! M1: {} A, M2: {} A",
                current_amps1, current_amps2
            );
            enable1.set_low();
            enable2.set_low();
            5.s_timer().await;
        }

        20.ms_timer().await;
    }
}
