use defmt::Format;
use embassy_executor::Spawner;

pub mod vl53lxx;
pub mod mpu9250;

pub trait Sensor<M, StartError: Format>: Sized {
    /// Starts continuous measurement mode, where the sensor will automatically take measurements at
    /// a fixed interval and call the provided callback with the new measurement data.
    async fn start_continuous_measurement(
        &'static mut self,
        spawner: &mut Spawner,
        callable: &'static dyn Fn(&M),
    ) -> Result<(), StartError>;

    fn get_latest_measurement(&self) -> &M;
}
