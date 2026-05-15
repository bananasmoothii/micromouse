use core::convert::Infallible;
use defmt::{error, info};
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::spi;
use embassy_stm32::spi::Spi;
use embassy_stm32::spi::mode::Master;
use embassy_time::Delay;
use mpu9250::{Error, GyroScale, Marg, MargMeasurements, Mpu9250, MpuConfig, SpiDevice, SpiError};
use crate::utils::HertzUtils;

// Note: interrupts don't seem to work with the MPU9250. However, each new read gives a new
// measurement, even if reading as fast as possible.

pub struct Mpu9250Sensor {
    device: Mpu9250<SpiDevice<Spi<'static, Async, Master>, Output<'static>>, Marg>,
}

impl Mpu9250Sensor {
    pub(crate) fn init_new(
        com: Spi<'static, Async, Master>,
        ncs: Output<'static>,
    ) -> Result<Self, Error<SpiError<spi::Error, Infallible>>> {
        info!("Initializing MPU9250 via SPI...");
        let device = Mpu9250::marg_with_reinit(
            com,
            ncs,
            &mut Delay,
            &mut MpuConfig::marg().gyro_scale(GyroScale::_2000DPS),
            |mut spi, ncs| {
                let mut new_spi_config = spi::Config::default();
                new_spi_config.frequency = 16.mhz();
                spi.set_config(&new_spi_config).ok().map(|_| (spi, ncs))
            },
        )?;
        info!("MPU9250 initialized successfully");
        Ok(Self { device })
    }

    pub fn read(&mut self) -> Option<MargMeasurements<[f32; 3]>> {
        match self.device.all() {
            Ok(data) => Some(data),
            Err(e) => {
                error!("Failed to read MPU9250: {}", e);
                None
            }
        }
    }
}
