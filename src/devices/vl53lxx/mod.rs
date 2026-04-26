use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Output;

pub mod vl53l1x;
pub mod vl53l0x;

/// Configuration for the VL53LXX distance sensors
pub struct Config {
    pub timing_config: TimingConfig,
    pub xshut_pin: Output<'static>,
    pub gpio_interrupt: ExtiInput<'static>,
}

pub struct TimingConfig {
    /// Measurement timing budget in microseconds (for example: 66000 for 15Hz)
    pub timing_budget_us: u32,
    /// Inter-measurement period in milliseconds (must be >= timing_budget_ms + 4, where 4 is
    /// `TIMED_MODE_TIMING_GUARD_MILLISECONDS` in `vl53l1/lib/vl53l1/src/lib.rs::start_measurement`)
    pub inter_measurement_period_ms: u32,
}

impl Default for TimingConfig {
    fn default() -> Self {
        Self {
            timing_budget_us: 66_000,
            inter_measurement_period_ms: 70,
        }
    }
}

trait MeasurementData<S> {
    fn get_distance_mm(&self) -> i16;

    fn get_sigma_mm(&self) -> f64;

    fn get_status(&self) -> S;
}
