use crate::sensor::vl53lxx::Config;
use defmt::Format;
use embassy_executor::Spawner;
use embedded_hal::i2c::I2c;

pub mod vl53lxx;

pub trait Sensor<'a, I: I2c, M: Format, E, StartError: Format>: Sized {
    async fn init_new(config: Config, i2c: I) -> Result<Self, E>;

    /// Starts continuous measurement mode, where the sensor will automatically take measurements at
    /// a fixed interval and call the provided callback with the new measurement data.
    async fn start_continuous_measurement(
        &'static mut self,
        spawner: &mut Spawner,
        callable: &'static dyn Fn(&M),
    ) -> Result<(), StartError>;

    fn get_latest_measurement(&self) -> &M;
}
