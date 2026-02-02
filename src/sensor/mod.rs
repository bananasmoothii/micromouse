use defmt::Format;
use embassy_executor::{SpawnError, Spawner};
use crate::sensor::vl53lxx::Config;
use embassy_stm32::i2c::{I2c, Master};
use embassy_stm32::mode::Async;

pub mod vl53lxx;

pub trait Sensor<M: Format, E: Format>: Sized {
    async fn init_new(
        config: Config,
        i2c: &'static mut I2c<'static, Async, Master>,
    ) -> Result<Self, E>;

    fn start_continuous_measurement(&'static mut self, spawner: &mut Spawner) -> Result<(), SpawnError>;

    fn get_latest_measurement(&self) -> Result<&M, E>;
}
