use crate::sensor::Sensor;
use crate::sensor::vl53lxx::Config;
use alloc::format;
use defmt::{Format, debug, error, info, trace, warn};
use embassy_executor::{SpawnError, Spawner};
use embassy_stm32::i2c;
use embassy_stm32::i2c::{I2c, Master};
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Timer};
use embedded_hal::i2c::{ErrorType, I2c as I2cTrait};
use vl53l0x::*;

/// Wrapper around a shared I2C mutex that implements embedded-hal 1.0 I2c trait
pub struct I2cWrapper<'a> {
    i2c: &'a mut Mutex<CriticalSectionRawMutex, I2c<'static, Async, Master>>,
}

impl<'a> I2cWrapper<'a> {
    pub fn new(i2c: &'a mut Mutex<CriticalSectionRawMutex, I2c<'static, Async, Master>>) -> Self {
        Self { i2c }
    }
}

impl<'a> ErrorType for I2cWrapper<'a> {
    type Error = i2c::Error;
}

impl<'a> I2cTrait for I2cWrapper<'a> {
    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        embassy_futures::block_on(async {
            let mut i2c = self.i2c.lock().await;
            i2c.blocking_read(address, buffer)
        })
    }

    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        embassy_futures::block_on(async {
            let mut i2c = self.i2c.lock().await;
            i2c.blocking_write(address, bytes)
        })
    }

    fn write_read(
        &mut self,
        address: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        embassy_futures::block_on(async {
            let mut i2c = self.i2c.lock().await;
            i2c.blocking_write_read(address, bytes, buffer)
        })
    }

    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        embassy_futures::block_on(async {
            let mut i2c = self.i2c.lock().await;
            i2c.blocking_transaction(address, operations)
        })
    }
}

/// VL53L0X Time-of-Flight distance sensor implementation
///
/// This sensor uses a shared I2C bus through a mutex, allowing multiple sensors
/// to share the same I2C peripheral safely.
pub struct VL53L0XSensor<'a> {
    device: VL53L0x<I2cWrapper<'a>>,
    gpio_interrupt: embassy_stm32::exti::ExtiInput<'static>,
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
        i2c: &'a mut Mutex<CriticalSectionRawMutex, I2c<'static, Async, Master>>,
    ) -> Result<Self, Error<i2c::Error>> {
        // Toggle XSHUT pin to reset the device
        debug!("  Toggling XSHUT pin...");
        config.xshut_pin.set_low();
        Timer::after(Duration::from_millis(10)).await;
        let a = i2c;

        // Lock I2C for the critical address change operation
        let mut device = {
            let mut i2c_guard = i2c.lock().await;

            // Wait for device boot
            config.xshut_pin.set_high();
            Timer::after(Duration::from_millis(10)).await;
            debug!("  XSHUT toggled");

            // Change the I2C address
            i2c_guard.blocking_write(0x29, &[0x8A, 0x60])?;
            debug!("  Address changed to 0x30");
            VL53L0x::with_address(I2cWrapper::new(i2c), 0x30)?
        };

        device.set_measurement_timing_budget(config.timing_config.timing_budget_us)?;

        Ok(Self {
            device,
            gpio_interrupt: config.gpio_interrupt,
            last_data: 0,
        })
    }

    fn start_continuous_measurement(
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

    fn get_latest_measurement(&self) -> Result<&u16, Error<i2c::Error>> {
        Ok(&self.last_data)
    }
}

#[embassy_executor::task]
async fn distance_sensor_task(self_: &'static mut VL53L0XSensor<'static>) -> ! {
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
