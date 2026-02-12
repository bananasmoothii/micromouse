use crate::sensor::Sensor;
use crate::sensor::vl53lxx::Config;
use alloc::format;
use core::fmt::Debug;
use defmt::{Format, debug, warn};
use embassy_executor::{SpawnError, Spawner};
use embassy_stm32::i2c::Master;
use embassy_stm32::mode::Async;
use embassy_time::{Duration, Timer};
use embedded_hal::i2c::I2c;
use embedded_hal_bus::i2c::RefCellDevice;
use vl53l0x::*;

/// VL53L0X Time-of-Flight distance sensor implementation
///
/// This sensor uses a shared I2C bus through a mutex, allowing multiple sensors
/// to share the same I2C peripheral safely.
pub struct VL53L0XSensor {
    device: VL53L0x<I>,
    gpio_interrupt: embassy_stm32::exti::ExtiInput<'static>,
    last_data: u16,
}

#[derive(Debug, Format)]
pub enum StartError {
    I2cError(E),
    SpawnError(SpawnError),
}

// I hate not being able to use generics due to the embassy task
type I = RefCellDevice<'static, embassy_stm32::i2c::I2c<'static, Async, Master>>;
type E = embassy_stm32::i2c::Error;

impl<'a> Sensor<'a, I, u16, Error<E>, StartError> for VL53L0XSensor {
    async fn init_new(mut config: Config, i2c: I) -> Result<Self, Error<E>> {
        // Toggle XSHUT pin to reset the device
        debug!("  Toggling XSHUT pin...");
        config.xshut_pin.set_low();
        Timer::after(Duration::from_millis(10)).await;

        // Wait for device boot
        config.xshut_pin.set_high();
        Timer::after(Duration::from_millis(10)).await;
        debug!("  XSHUT toggled");

        let mut device = VL53L0x::new(i2c)?;
        device.set_address(0x30)?;

        device.set_measurement_timing_budget(config.timing_config.timing_budget_us)?;

        Ok(Self {
            device,
            gpio_interrupt: config.gpio_interrupt,
            last_data: 0,
        })
    }

    async fn start_continuous_measurement(
        &'static mut self,
        spawner: &mut Spawner,
    ) -> Result<(), StartError> {
        self.device
            .start_continuous(0)
            .map_err(|e| StartError::I2cError(e))?;
        spawner
            .spawn(distance_sensor_task(self))
            .map_err(|e| StartError::SpawnError(e))?;
        Ok(())
    }

    fn get_latest_measurement(&self) -> Result<&u16, Error<E>> {
        Ok(&self.last_data)
    }
}

#[embassy_executor::task]
async fn distance_sensor_task(self_: &'static mut VL53L0XSensor) -> ! {
    debug!("Distance sensor task running");

    loop {
        self_.gpio_interrupt.wait_for_falling_edge().await;

        match self_.device.read_range_mm() {
            Ok(distance) => {
                self_.last_data = distance;
                debug!("VL53L0X Distance: {} mm", distance);
            }
            Err(nb::Error::WouldBlock) => {}
            Err(nb::Error::Other(e)) => {
                let s = format!("{:?}", e);
                let s: &str = s.as_ref();
                warn!("VL53L0X read error: {}", s);
            }
        }
    }
}
