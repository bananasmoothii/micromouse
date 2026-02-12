use crate::sensor::vl53lxx::Config;
use defmt::Format;
use embassy_executor::Spawner;
use embedded_hal::i2c::I2c;

pub mod vl53lxx;

pub trait Sensor<'a, I: I2c, M: Format, E, StartError: Format>: Sized {
    async fn init_new(config: Config, i2c: I) -> Result<Self, E>;

    async fn start_continuous_measurement(
        &'static mut self,
        spawner: &mut Spawner,
    ) -> Result<(), StartError>;

    fn get_latest_measurement(&self) -> Result<&M, E>;
}
