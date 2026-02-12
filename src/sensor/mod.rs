use crate::sensor::vl53lxx::Config;
use defmt::Format;
use embassy_executor::Spawner;
use embassy_stm32::i2c::{I2c, Master};
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::mutex::Mutex;

pub mod vl53lxx;

pub trait Sensor<'a, M: Format, E, StartError: Format>: Sized {
    async fn init_new(
        config: Config,
        i2c: &'a mut Mutex<NoopRawMutex, I2c<'static, Async, Master>>,
    ) -> Result<Self, E>;

    async fn start_continuous_measurement(&'static mut self, spawner: &mut Spawner) -> Result<(), StartError>;

    fn get_latest_measurement(&self) -> Result<&M, E>;
}
