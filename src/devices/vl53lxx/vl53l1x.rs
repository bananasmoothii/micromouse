use crate::devices::vl53lxx::{Config, MeasurementData};
use crate::utils::DurationUtils;
use defmt::{Format, debug, info, trace, warn};
use embassy_executor::{SpawnError, Spawner};
use embassy_futures::select::{Either3, select3};
use embassy_stm32::gpio::Output;
use embassy_stm32::i2c;
use embassy_stm32::i2c::{I2c, Master};
use embassy_stm32::mode::Async;
use embassy_stm32::pac::interrupt;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, TimeoutError, Timer};
use embedded_hal_bus::i2c::RefCellDevice;
use futures_util::future::select_all;
use vl53l1::*;

pub struct VL53L1XSensor {
    device: Device,
    i2c: I,
    address: u8,
    last_data: RangingMeasurementData,
    gpio_interrupt: embassy_stm32::exti::ExtiInput<'static>,
    xshut_pin: Output<'static>,
}

// I hate not being able to use generics due to the embassy task
type I = RefCellDevice<'static, I2c<'static, Async, Master>>;
type E = i2c::Error;
type Channel1 = Channel<CriticalSectionRawMutex, RangingMeasurementData, 4>;

/// right sensor looking 45° to the left
pub static VL53L1X_45D_LEFT_CHANNEL: Channel1 = Channel::new();
/// Sensor looking straight forward
pub static VL53L1X_MIDDLE_CHANNEL: Channel1 = Channel::new();
/// left sensor looking 45° to the right
pub static VL53L1X_45D_RIGHT_CHANNEL: Channel1 = Channel::new();

impl VL53L1XSensor {
    pub(crate) async fn init_new(
        mut config: Config,
        mut i2c: I,
        address: u8,
    ) -> Result<Self, Error<i2c::Error>> {
        info!(
            "Initializing VL53L1X distance sensor at address 0x{:02x}",
            address
        );

        // Toggle XSHUT pin to reset the device
        info!("  Toggling XSHUT pin...");
        config.xshut_pin.set_low();
        10.ms_timer().await;
        config.xshut_pin.set_high();
        10.ms_timer().await;

        let mut self_ = Self {
            device: Device::default(),
            i2c,
            address,
            last_data: RangingMeasurementData::default(),
            gpio_interrupt: config.gpio_interrupt,
            xshut_pin: config.xshut_pin,
        };

        self_.reinit_no_xshut(false)?;

        Ok(self_)
    }

    /// Attempt to recover from a sensor error by stopping and restarting measurements
    async fn recover_sensor(&mut self) -> Result<(), Error<i2c::Error>> {
        warn!(
            "Hardware recovery for VL53L1X {:#x} ({:#x})",
            self.address,
            self.device.address()
        );

        // XSHUT high — sensor boots at default address 0x29
        self.xshut_pin.set_high();
        10.ms_timer().await;

        self.reinit_no_xshut(true)?;

        info!("Recovery complete for {:#x}", self.address);
        Ok(())
    }

    fn reinit_no_xshut(&mut self, new_device: bool) -> Result<(), Error<i2c::Error>> {
        if new_device {
            self.device = Device::default(); // resets address to 0x29
        }

        info!("  Data init...");
        data_init(&mut self.device, &mut self.i2c)?;

        if self.address != 0x29 {
            info!("  Changing I2C address to {:#x}...", self.address);
            set_device_address(&mut self.device, &mut self.i2c, self.address)?;
        }

        info!("  Static init...");
        static_init(&mut self.device)?;

        info!("  Setting preset mode...");
        set_preset_mode(&mut self.device, PresetMode::Autonomous)?;

        // Set full field of view
        info!("  Setting ROI...");
        set_user_roi(
            &mut self.device,
            UserRoi {
                top_left_x: 0,
                top_left_y: 15,
                bot_right_x: 15,
                bot_right_y: 0,
            },
        )?;

        info!("  Setting timing budget and inter-measurement period...");
        set_measurement_timing_budget_micro_seconds(&mut self.device, 66_000)?;
        set_inter_measurement_period_milli_seconds(&mut self.device, 70)?;

        info!("  Starting measurement...");
        start_measurement(&mut self.device, &mut self.i2c)?;
        info!("VL53L1X {:#x} initialization complete", self.address);

        Ok(())
    }

    pub fn get_latest_measurement(&self) -> &RangingMeasurementData {
        &self.last_data
    }
}

#[embassy_executor::task(pool_size = 1)]
pub async fn distance_sensor_task(
    mut sensor_45d_right: VL53L1XSensor,
    mut sensor_middle: VL53L1XSensor,
    mut sensor_45d_left: VL53L1XSensor,
) -> ! {
    debug!("Distance sensor task running");
    loop {
        // Wait for data-ready interrupt, or poll at the sensor period as fallback.
        let either3 = select3(
            embassy_time::with_timeout(
                Duration::from_millis(80),
                sensor_45d_right.gpio_interrupt.wait_for_low(),
            ),
            embassy_time::with_timeout(
                Duration::from_millis(80),
                sensor_middle.gpio_interrupt.wait_for_low(),
            ),
            embassy_time::with_timeout(
                Duration::from_millis(80),
                sensor_45d_left.gpio_interrupt.wait_for_low(),
            ),
        )
            .await;

        let sensor: &mut VL53L1XSensor;
        let channel: &Channel1;
        let timeout: bool;
        match either3 {
            Either3::First(timeout_result) => {
                sensor = &mut sensor_45d_right;
                channel = &VL53L1X_45D_RIGHT_CHANNEL;
                timeout = timeout_result.is_err();
            }
            Either3::Second(timeout_result) => {
                sensor = &mut sensor_middle;
                channel = &VL53L1X_MIDDLE_CHANNEL;
                timeout = timeout_result.is_err();
            }
            Either3::Third(timeout_result) => {
                sensor = &mut sensor_45d_left;
                channel = &VL53L1X_45D_LEFT_CHANNEL;
                timeout = timeout_result.is_err();
            }
        }

        if timeout {
            warn!("GPIO timeout for VL53L1X {:#x}", sensor.address);
        }

        // Get the ranging measurement data
        match get_ranging_measurement_data(&mut sensor.device, &mut sensor.i2c) {
            Ok(data) => {
                let wrapped_data = RangingMeasurementData(data);
                sensor.last_data = wrapped_data.clone();
                trace!(
                    "VL53L1X {:#x} read: {} mm, σ={} mm, {}",
                    sensor.device.address(),
                    wrapped_data.get_distance_mm(),
                    wrapped_data.get_sigma_mm(),
                    wrapped_data.get_status(),
                );
                let _ = channel.try_send(wrapped_data);
                if let Err(e) = vl53l1::clear_interrupt(&mut sensor.device, &mut sensor.i2c) {
                    warn!(
                        "Error clearing interrupt for {:#x}: {:?}",
                        sensor.device.address(),
                        e
                    );
                }
            }
            Err(e) => {
                warn!(
                    "Error getting ranging data for {:#x} ({:#x}) {:?}",
                    sensor.address,
                    sensor.device.address(),
                    e
                );
                sensor_45d_left.xshut_pin.set_low();
                sensor_middle.xshut_pin.set_low();
                sensor_45d_right.xshut_pin.set_low();

                // SWRST the I2C peripheral to clear the BSY stuck state
                use embassy_stm32::pac;
                pac::I2C1.cr1().modify(|w| w.set_swrst(true));
                pac::I2C1.cr1().modify(|w| w.set_swrst(false));
                pac::I2C1.cr1().modify(|w| w.set_pe(true));

                10.ms_timer().await;

                info!("Recovering sensor 1");
                sensor_45d_right.xshut_pin.set_high();
                10.ms_timer().await;
                if let Err(e2) = sensor_45d_right.recover_sensor().await {
                    warn!(
                        "Recovery failed for {:#x}: {:?}",
                        sensor_45d_right.address, e2
                    );
                    500.ms_timer().await;
                    continue;
                }

                info!("Recovering sensor 2");
                sensor_middle.xshut_pin.set_high();
                10.ms_timer().await;
                if let Err(e2) = sensor_middle.recover_sensor().await {
                    warn!("Recovery failed for {:#x}: {:?}", sensor_middle.address, e2);
                    500.ms_timer().await;
                    continue;
                }

                info!("Recovering sensor 3");
                sensor_45d_left.xshut_pin.set_high();
                10.ms_timer().await;
                if let Err(e2) = sensor_45d_left.recover_sensor().await {
                    warn!(
                        "Recovery failed for {:#x}: {:?}",
                        sensor_45d_left.address, e2
                    );
                    500.ms_timer().await;
                    continue;
                }

                info!("RECOVERY COMPLETE HURRAY");
            }
        }

        // try to make i2c not crash
        // 5.ms_timer().await;
    }
}

#[derive(Clone, Default, Format)]
pub struct RangingMeasurementData(pub vl53l1::RangingMeasurementData);

impl MeasurementData<RangeStatus> for RangingMeasurementData {
    fn get_distance_mm(&self) -> i16 {
        self.0.range_milli_meter
    }

    fn get_sigma_mm(&self) -> f64 {
        self.0.sigma_milli_meter as f64 / 65536.0
    }

    fn get_status(&self) -> RangeStatus {
        self.0.range_status
    }
}
