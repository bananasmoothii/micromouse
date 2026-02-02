use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::i2c::{I2c, Master};

pub mod vl53l1x;

/// Configuration for the VL53LXX distance sensors
pub struct Config {
    pub timing_config: TimingConfig,
    pub xshut_pin: Output<'static>,
    pub gpio_interrupt: ExtiInput<'static>,
}

pub struct TimingConfig {
    /// Measurement timing budget in microseconds (for example: 66000 for 15Hz)
    pub timing_budget_us: u32,
    /// Inter-measurement period in milliseconds (minimum: 69ms from testing for VL53L1X)
    pub inter_measurement_period_ms: u32,
}

impl Default for TimingConfig {
    fn default() -> Self {
        Self {
            timing_budget_us: 66_000,
            inter_measurement_period_ms: 69,
        }
    }
}

trait MeasurementData<S> {
    fn get_distance_mm(&self) -> i16;

    fn get_sigma_mm(&self) -> f64;

    fn get_status(&self) -> S;
}
