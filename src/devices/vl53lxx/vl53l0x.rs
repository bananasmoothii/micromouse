use crate::devices::vl53lxx::Config;
use core::fmt::Debug;
use defmt::{Format, debug, warn};
use embassy_executor::{SpawnError, Spawner};
use embassy_stm32::i2c;
use embassy_stm32::i2c::{I2c, Master};
use embassy_stm32::mode::Async;
use embassy_time::{Duration, Timer};
use embedded_hal::i2c::I2c as _;
use embedded_hal_bus::i2c::RefCellDevice;
use vl53l0x::RangeStatus::{PhaseFail, SignalFail};
use vl53l0x::*;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use crate::utils::DurationUtils;

pub static VL53L0X_CHANNEL: Channel<CriticalSectionRawMutex, MeasurementData, 4> = Channel::new();

/// VL53L0X Time-of-Flight distance sensor implementation
///
/// This sensor uses a shared I2C bus through a mutex, allowing multiple sensors
/// to share the same I2C peripheral safely.
pub struct VL53L0XSensor {
    device: VL53L0x<I>,
    gpio_interrupt: embassy_stm32::exti::ExtiInput<'static>,
    last_data: MeasurementData,
}

#[derive(Debug, Format)]
pub enum StartError {
    I2cError(E),
    SpawnError(SpawnError),
}

#[derive(Debug, Format, Clone, Copy)]
pub struct MeasurementData {
    pub distance_mm: u16,
    pub status: RangeStatus,
}

impl Default for MeasurementData {
    fn default() -> Self {
        Self {
            distance_mm: 0,
            status: RangeStatus::None,
        }
    }
}

// I hate not being able to use generics due to the embassy task
type I = RefCellDevice<'static, I2c<'static, Async, Master>>;
type E = i2c::Error;

impl VL53L0XSensor {
    pub(crate) async fn init_new(mut config: Config, i2c: I) -> Result<Self, Error<E>> {
        // Toggle XSHUT pin to reset the device
        debug!("  Toggling XSHUT pin...");
        config.xshut_pin.set_low();
        10.ms_timer().await;

        // Wait for device boot
        config.xshut_pin.set_high();
        10.ms_timer().await;
        debug!("  XSHUT toggled");

        let mut device = VL53L0x::new(i2c)?;
        device.set_address(0x30)?;

        device.set_measurement_timing_budget(config.timing_config.timing_budget_us)?;

        Ok(Self {
            device,
            gpio_interrupt: config.gpio_interrupt,
            last_data: MeasurementData::default(),
        })
    }

    pub async fn start_continuous_measurement(
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

    pub fn get_latest_measurement(&self) -> &MeasurementData {
        &self.last_data
    }
}

#[embassy_executor::task]
async fn distance_sensor_task(self_: &'static mut VL53L0XSensor) -> ! {
    debug!("Distance sensor task running");

    loop {
        self_.gpio_interrupt.wait_for_falling_edge().await;
        // trace!("GPIO interrupt received, reading distance...");

        match self_.device.get_range_with_status_blocking() {
            Ok((distance_mm, status)) => {
                self_.last_data = MeasurementData {
                    distance_mm,
                    status,
                };
                if status != SignalFail && status != PhaseFail {
                    debug!("VL53L0X Distance: {} mm", distance_mm);
                    let _ = VL53L0X_CHANNEL.try_send(self_.last_data);
                }
            }
            Err(e) => {
                warn!("VL53L0X read error: {}", e);
                if let Err(e) = self_.device.clear_interrupt_status() {
                    warn!("Failed to clear VL53L0X interrupt status: {}", e);
                    if let i2c::Error::Timeout = e {
                        warn!("I2C timeout detected, attempting recovery...");
                        // I2C specification chapter 3.1.16 Bus Clear
                        // If the data line (SDA) is stuck LOW, the controller should send nine clock
                        // pulses. The device that held the bus LOW should release it sometime within
                        // those nine clocks. If not, then use the HW reset or cycle power to clear the
                        // bus
                        self_.device.com.write(0, &[0]).unwrap();
                        100.ms_timer().await;
                    }
                }
            }
        }
    }
}
