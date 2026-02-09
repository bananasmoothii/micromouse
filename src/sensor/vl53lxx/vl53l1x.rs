use crate::sensor::Sensor;
use crate::sensor::vl53lxx::{Config, MeasurementData};
use alloc::format;
use alloc::string::String;
use defmt::{debug, error, info, warn};
use embassy_executor::{SpawnError, Spawner};
use embassy_stm32::i2c;
use embassy_stm32::i2c::{I2c, Master};
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Timer};
use vl53l1::RangeStatus::SIGNAL_FAIL;
use vl53l1::*;

pub struct VL53L1XSensor<'a> {
    device: Device,
    gpio_interrupt: embassy_stm32::exti::ExtiInput<'static>,
    i2c: &'a mut Mutex<CriticalSectionRawMutex, I2c<'static, Async, Master>>,
    last_data: RangingMeasurementData,
    recovery_mode: bool,
}

// Step 1: Implement the base Sensor trait
impl<'a> Sensor<'a, RangingMeasurementData, Error<i2c::Error>, SpawnError> for VL53L1XSensor<'a>
where
    Self: Sized,
{
    async fn init_new(
        mut config: Config,
        i2c: &'a mut Mutex<CriticalSectionRawMutex, I2c<'static, Async, Master>>,
    ) -> Result<Self, Error<i2c::Error>> {
        info!("Initializing VL53L1X distance sensor");

        // Toggle XSHUT pin to reset the device
        info!("  Toggling XSHUT pin...");
        config.xshut_pin.set_low();
        Timer::after(Duration::from_millis(10)).await;
        config.xshut_pin.set_high();
        Timer::after(Duration::from_millis(10)).await;

        let mut device = Device::default();

        // Initialize the sensor
        info!("  Data init...");
        data_init(&mut device, &mut *i2c.lock().await)?;

        info!("  Static init...");
        static_init(&mut device)?;

        info!("  Setting preset mode...");
        set_preset_mode(&mut device, PresetMode::Autonomous)?;

        // Set full field of view
        info!("  Setting ROI...");
        set_user_roi(
            &mut device,
            UserRoi {
                top_left_x: 0,
                top_left_y: 15,
                bot_right_x: 15,
                bot_right_y: 0,
            },
        )?;

        info!("  Setting timing budget and inter-measurement period...");
        set_measurement_timing_budget_micro_seconds(
            &mut device,
            config.timing_config.timing_budget_us,
        )?;
        set_inter_measurement_period_milli_seconds(
            &mut device,
            config.timing_config.inter_measurement_period_ms,
        )?;

        info!("  Starting measurement...");
        start_measurement(&mut device, &mut *i2c.lock().await)?;

        info!("VL53L1X initialization complete");
        Ok(Self {
            device,
            gpio_interrupt: config.gpio_interrupt,
            i2c,
            last_data: RangingMeasurementData::default(),
            recovery_mode: false,
        })
    }

    async fn start_continuous_measurement<>(
        &'static mut self,
        spawner: &mut Spawner,
    ) -> Result<(), SpawnError> {
        spawner.spawn(distance_sensor_task(self))
    }

    fn get_latest_measurement(&self) -> Result<&RangingMeasurementData, Error<i2c::Error>> {
        Ok(&self.last_data)
    }
}

#[embassy_executor::task]
async fn distance_sensor_task(self_: &'static mut VL53L1XSensor<'static>) -> ! {
    debug!("Distance sensor task running");

    loop {
        if !self_.recovery_mode {
            self_.gpio_interrupt.wait_for_falling_edge().await;
        } else {
            while let Err(e) = {
                wait_measurement_data_ready(
                    &mut self_.device,
                    &mut *self_.i2c.lock().await,
                    &mut Delay,
                )
            } {
                let str = match e {
                    nb::Error::Other(e) => format!("other error: {:?}", e),
                    nb::Error::WouldBlock => String::from("Operation would block"),
                };
                warn!(
                    "Waiting for measurement data ready failed ({}), retrying...",
                    str.as_str()
                );
                Timer::after(Duration::from_millis(10)).await;
            }
            info!("Measurement data ready after recovery");
            self_.recovery_mode = false;
        }

        // Get the ranging measurement data
        match { get_ranging_measurement_data(&mut self_.device, &mut *self_.i2c.lock().await) } {
            Err(e) => {
                warn!("Error getting ranging data: {:?}", e);
                if self_.recover_sensor().await.is_err() {
                    error!("Failed to recover sensor, waiting before retry...");
                    self_.recovery_mode = true;
                    Timer::after(Duration::from_millis(500)).await;
                }
                continue;
            }
            Ok(rmd) => {
                if rmd.range_status != SIGNAL_FAIL {
                    debug!(
                        "Distance: {} mm, Sigma: {} mm, Status: {:?}",
                        rmd.range_milli_meter,
                        rmd.sigma_milli_meter as f64 / 65536.0,
                        rmd.range_status
                    );
                    self_.last_data = rmd;
                }
            }
        }

        // Clear interrupt and start next measurement
        if let Err(e) = {
            clear_interrupt_and_start_measurement(
                &mut self_.device,
                &mut *self_.i2c.lock().await,
                &mut Delay,
            )
        } {
            warn!("Error clearing interrupt: {:?}", e);
            if self_.recover_sensor().await.is_err() {
                error!("Failed to recover sensor, waiting before retry...");
                Timer::after(Duration::from_millis(500)).await;
                self_.recovery_mode = true;
            }
        }
    }
}

impl VL53L1XSensor<'_> {
    /// Attempt to recover from a sensor error by stopping and restarting measurements
    async fn recover_sensor(&mut self) -> Result<(), Error<i2c::Error>> {
        info!("  Attempting sensor recovery...");
        let _ = { stop_measurement(&mut self.device, &mut *self.i2c.lock().await) };
        Timer::after(Duration::from_millis(100)).await;
        { start_measurement(&mut self.device, &mut *self.i2c.lock().await) }?;
        info!("  Sensor recovered");
        Ok(())
    }
}

impl MeasurementData<RangeStatus> for RangingMeasurementData {
    fn get_distance_mm(&self) -> i16 {
        self.range_milli_meter
    }

    fn get_sigma_mm(&self) -> f64 {
        self.sigma_milli_meter as f64 / 65536.0
    }

    fn get_status(&self) -> RangeStatus {
        self.range_status
    }
}
