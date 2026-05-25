//! MPU-9250 9-DOF IMU driver (SPI, mode 3).
//!
//! ## Initialization
//! The SPI bus is configured at 1 MHz for init (MPU9250 requirement) and bumped to 16 MHz
//! after the `marg_with_reinit` callback completes.  The gyro is set to ±2000 DPS full-scale.
//! The magnetometer (AK8963) is enabled as part of the MARG (Magnetic, Angular Rate, Gravity)
//! configuration.
//!
//! ## Reading
//! [`Mpu9250Sensor::read`] returns accelerometer, gyro, and magnetometer data in one SPI burst.
//! Errors (SPI timeouts, bus errors) are logged and return `None`; the caller should skip the
//! fusion step for that cycle rather than propagating stale data.
//!
//! ## Note on interrupts
//! The data-ready interrupt line (INT pin) does not function reliably in practice — possibly
//! due to the custom Embassy SPI driver.  Reading as fast as possible still yields fresh data
//! on every call, so the driver polls without waiting for the interrupt.

use crate::utils::HertzUtils;
use core::convert::Infallible;
use defmt::{error, info};
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::spi;
use embassy_stm32::spi::Spi;
use embassy_stm32::spi::mode::Master;
use embassy_time::Delay;
use mpu9250::{Error, GyroScale, Marg, MargMeasurements, Mpu9250, MpuConfig, SpiDevice, SpiError};

// Note: interrupts don't seem to work with the MPU9250. However, each new read gives a new
// measurement, even if reading as fast as possible.

/// Wrapper around the `mpu9250` crate's MARG device, providing simple `init` / `read` API.
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
                // MPU9250 library requires Mode 3 (CPOL=1, CPHA=1)
                // This matches mpu9250::MODE constant: IdleHigh, CaptureOnSecondTransition
                new_spi_config.mode = spi::Mode {
                    polarity: spi::Polarity::IdleHigh,
                    phase: spi::Phase::CaptureOnSecondTransition,
                };
                // Can't go high than 16MHz if the MCU is at default settings (16MHz clock)
                new_spi_config.frequency = 16.mhz();
                spi.set_config(&new_spi_config).ok().map(|_| (spi, ncs))
            },
        )?;
        info!("MPU9250 initialized successfully");
        Ok(Self { device })
    }

    /// Reads accel, gyro, and magnetometer in one SPI transaction.
    /// Returns `None` and logs an error on any SPI failure.
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
