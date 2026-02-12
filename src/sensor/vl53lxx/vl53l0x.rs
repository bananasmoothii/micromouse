use crate::sensor::Sensor;
use crate::sensor::vl53lxx::Config;
use alloc::format;
use defmt::{Format, debug, warn};
use embassy_executor::{SpawnError, Spawner};
use embassy_stm32::i2c;
use embassy_stm32::i2c::{I2c, Master};
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use vl53l0x::*;

/// VL53L0X Time-of-Flight distance sensor implementation
///
/// This sensor uses a shared I2C bus through a mutex, allowing multiple sensors
/// to share the same I2C peripheral safely.
pub struct VL53L0XSensor<'a> {
    device: VL53L0x<'a, I2c<'static, Async, Master>, NoopRawMutex>,
    gpio_interrupt: embassy_stm32::exti::ExtiInput<'static, Async>,
    last_data: u16,
}

#[derive(Debug, Format)]
pub enum StartError {
    I2cError(i2c::Error),
    SpawnError(SpawnError),
}

impl<'a> Sensor<'a, u16, Error<i2c::Error>, StartError> for VL53L0XSensor<'a>
where
    Self: Sized,
{
    async fn init_new(
        mut config: Config,
        i2c: &'a mut Mutex<NoopRawMutex, I2c<'static, Async, Master>>,
    ) -> Result<Self, Error<i2c::Error>> {
        // Toggle XSHUT pin to reset the device
        debug!("  Toggling XSHUT pin...");
        config.xshut_pin.set_low();
        Timer::after(Duration::from_millis(10)).await;

        // Wait for device boot
        config.xshut_pin.set_high();
        Timer::after(Duration::from_millis(10)).await;
        debug!("  XSHUT toggled");

        let mut device = VL53L0x::with_address_set(i2c, 0x29, true).await?;

        device
            .set_measurement_timing_budget(config.timing_config.timing_budget_us)
            .await?;

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
            .await
            .map_err(|e| StartError::I2cError(e))?;
        spawner.spawn(distance_sensor_task(self).map_err(|e| StartError::SpawnError(e))?);
        Ok(())
    }

    fn get_latest_measurement(&self) -> Result<&u16, Error<i2c::Error>> {
        Ok(&self.last_data)
    }
}

#[embassy_executor::task]
async fn distance_sensor_task(self_: &'static mut VL53L0XSensor<'static>) -> ! {
    debug!("Distance sensor task running");

    loop {
        self_.gpio_interrupt.wait_for_falling_edge().await;

        match self_.device.read_range_mm().await {
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
