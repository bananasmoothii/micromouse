//! Shared types for the VL53L0X and VL53L1X ToF distance sensor drivers.
//!
//! Both sensors share the same I2C bus.  Each has an XSHUT pin (active-low reset) and a
//! GPIO interrupt pin (active-low data-ready).  The [`Config`] struct groups these pins with
//! the measurement timing parameters so the init routine can drive them sequentially.
//!
//! For the actual measurement loop see [`vl53l1x::distance_sensor_task`].

use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Output;

pub mod vl53l0x;
pub mod vl53l1x;

/// Per-sensor hardware configuration consumed by the sensor's init function.
pub struct Config {
    pub timing_config: TimingConfig,
    /// Active-low reset pin.  Held low to keep the sensor in reset; driven high to boot it.
    pub xshut_pin: Output<'static>,
    /// Active-low data-ready interrupt from the sensor's GPIO1 pin.
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

/// Common interface for ranging measurement results, shared between VL53L0X and VL53L1X.
pub trait MeasurementData<S> {
    /// Measured distance in millimetres (after affine calibration and EMA filtering).
    fn get_distance_mm(&self) -> i16;

    /// Measurement uncertainty in mm (sigma from the sensor).
    fn get_sigma_mm(&self) -> f64;

    /// Range status (e.g. `RANGE_VALID`, `SIGNAL_FAIL`).
    fn get_status(&self) -> S;
}
