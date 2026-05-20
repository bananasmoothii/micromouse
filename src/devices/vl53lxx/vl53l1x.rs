use crate::devices::buzzer::{BUZZER_CHANNEL, BuzzerTask};
use crate::devices::vl53lxx::{Config, MeasurementData};
use crate::utils::FairSelect3;
use crate::utils::{DurationUtils, HertzUtils, WithInstant};
use defmt::{Format, debug, info, trace, warn};
use embassy_futures::select::Either3;
use embassy_stm32::gpio::Output;
use embassy_stm32::i2c;
use embassy_stm32::i2c::{I2c, Master};
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::{Receiver, Watch};
use embassy_time::{Duration, Instant};
use embedded_hal_bus::i2c::RefCellDevice;
use vl53l1::RangeStatus::RANGE_VALID;
use vl53l1::*;

/// Exponential moving average factor applied to the affine-corrected distance before publish:
/// `filt = α · new + (1-α) · prev`. α=0.5 gives ~30% noise reduction and ~1-sample (~70 ms)
/// lag — roughly 2 cm at 0.3 m/s, acceptable for wall-follow. Reset on invalid readings so a
/// recovery doesn't smear stale data into fresh samples.
const EMA_ALPHA: f32 = 0.5;

pub struct VL53L1XSensor {
    device: Device,
    i2c: I,
    address: u8,
    last_data: RangingMeasurementData,
    gpio_interrupt: embassy_stm32::exti::ExtiInput<'static>,
    xshut_pin: Output<'static>,
    /// Per-sensor affine correction applied to `range_milli_meter` before the value is published
    /// or stored in `last_data`. `corrected = (raw as f32) * slope + offset_mm`, rounded.
    /// Defaults to (1.0, 0.0) = no-op. Tune from bench measurements at 2+ known distances.
    pub slope: f32,
    pub offset_mm: f32,
    /// EMA filter state in mm (post affine correction). `None` = no prior sample; next valid
    /// reading seeds the filter directly without blending.
    filtered_mm: Option<f32>,
}

// I hate not being able to use generics due to the embassy task
type I = RefCellDevice<'static, I2c<'static, Async, Master>>;
type E = i2c::Error;

/// Latest distance reading plus the time it was captured. Consumers should treat readings as
/// stale once their `at` timestamp is more than a few sensor periods old (~200 ms).
#[derive(Clone)]
pub struct DistanceSnapshot {
    pub data: RangingMeasurementData,
    pub at: Instant,
}

/// `N` = max number of concurrent receivers per watch. Today: straight_line wall-follow on all
/// three watches; main maze-runner also reads MIDDLE; future wall-mapper / fusion lateral
/// correction will add more. 4 leaves headroom without spending much RAM.
pub type DistWatch = Watch<CriticalSectionRawMutex, WithInstant<RangingMeasurementData>, 4>;
pub type DistReceiver<'a> =
Receiver<'a, CriticalSectionRawMutex, WithInstant<RangingMeasurementData>, 4>;

/// Sensor looking 45° to the left (mounted on the right side of the robot, crossed).
pub static VL53L1X_45D_LEFT_WATCH: DistWatch = Watch::new();
/// Sensor looking straight forward (mounted on the centerline of the robot).
pub static VL53L1X_MIDDLE_WATCH: DistWatch = Watch::new();
/// Sensor looking 45° to the right (mounted on the left side of the robot, crossed).
pub static VL53L1X_45D_RIGHT_WATCH: DistWatch = Watch::new();

impl VL53L1XSensor {
    pub(crate) async fn init_new(
        mut config: Config,
        i2c: I,
        address: u8,
    ) -> Result<Self, Error<E>> {
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
            slope: 1.0,
            offset_mm: 0.0,
            filtered_mm: None,
        };

        self_.reinit_no_xshut(false)?;

        Ok(self_)
    }

    /// Attempt to recover from a sensor error by stopping and restarting measurements
    async fn recover_sensor(&mut self) -> Result<(), Error<E>> {
        warn!("Hardware recovery for VL53L1X {:#x}", self.address);

        // XSHUT high — sensor boots at default address 0x29
        self.xshut_pin.set_high();
        10.ms_timer().await;

        self.filtered_mm = None;
        self.reinit_no_xshut(true)?;

        info!("Recovery complete for {:#x}", self.address);
        Ok(())
    }

    /// Saves I2C timing config, applies SWRST to force-clear BSY, then restores config.
    /// Must be called AFTER all XSHUT pins are LOW (sensors release SDA before SWRST).
    pub fn i2c_swrst_recovery() {
        use embassy_stm32::pac;

        // Save timing registers — SWRST resets them to 0
        let cr2_freq = pac::I2C1.cr2().read().freq();
        let ccr_r = pac::I2C1.ccr().read();
        let ccr_fs = ccr_r.f_s();
        let ccr_duty = ccr_r.duty();
        let ccr_val = ccr_r.ccr();
        let trise_val = pac::I2C1.trise().read().trise();

        // SWRST forces BSY=0 and releases SCL/SDA from the peripheral side
        pac::I2C1.cr1().modify(|w| w.set_swrst(true));
        cortex_m::asm::delay(420_000); // ~5 ms at 84 MHz — bus settles
        pac::I2C1.cr1().modify(|w| w.set_swrst(false));

        // CCR and TRISE require PE=0 to write; SWRST leaves PE=0
        pac::I2C1.cr2().modify(|w| w.set_freq(cr2_freq));
        pac::I2C1.ccr().write(|w| {
            w.set_f_s(ccr_fs);
            w.set_duty(ccr_duty);
            w.set_ccr(ccr_val);
        });
        pac::I2C1.trise().write(|w| w.set_trise(trise_val));
        pac::I2C1.cr1().modify(|w| w.set_pe(true));
    }

    fn reinit_no_xshut(&mut self, new_device: bool) -> Result<(), Error<E>> {
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
                // 8x8 centered region
                top_left_x: 4,
                bot_right_y: 4,
                bot_right_x: 11,
                top_left_y: 11,
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
    let mut priority = 0u8;
    loop {
        // Wait for data-ready interrupt, or poll at the sensor period as fallback.
        // FairSelect3 rotates which future is polled first to avoid starvation under CPU load.
        let either3 = FairSelect3::new(
            priority,
            embassy_time::with_timeout(
                Duration::from_millis(150),
                sensor_45d_right.gpio_interrupt.wait_for_low(),
            ),
            embassy_time::with_timeout(
                Duration::from_millis(150),
                sensor_middle.gpio_interrupt.wait_for_low(),
            ),
            embassy_time::with_timeout(
                Duration::from_millis(150),
                sensor_45d_left.gpio_interrupt.wait_for_low(),
            ),
        )
        .await;
        priority = priority.wrapping_add(1) % 3;

        let sensor: &mut VL53L1XSensor;
        let watch: &DistWatch;
        let timeout: bool;
        match either3 {
            Either3::First(timeout_result) => {
                sensor = &mut sensor_45d_right;
                watch = &VL53L1X_45D_RIGHT_WATCH;
                timeout = timeout_result.is_err();
            }
            Either3::Second(timeout_result) => {
                sensor = &mut sensor_middle;
                watch = &VL53L1X_MIDDLE_WATCH;
                timeout = timeout_result.is_err();
            }
            Either3::Third(timeout_result) => {
                sensor = &mut sensor_45d_left;
                watch = &VL53L1X_45D_LEFT_WATCH;
                timeout = timeout_result.is_err();
            }
        }

        if timeout {
            warn!("GPIO timeout for VL53L1X {:#x}", sensor.address);
        }

        // Get the ranging measurement data
        match get_ranging_measurement_data(&mut sensor.device, &mut sensor.i2c) {
            Ok(mut data) => {
                let raw = data.range_milli_meter;
                let status = data.range_status;

                // Drop invalid readings before touching the filter — a SIGNAL_FAIL of e.g. 0 mm
                // would otherwise drag the EMA down hard.
                if status != RANGE_VALID {
                    sensor.filtered_mm = None;
                } else {
                    // Affine correction first, then EMA in mm-space.
                    let corrected = raw as f32 * sensor.slope + sensor.offset_mm;
                    let filt = match sensor.filtered_mm {
                        Some(prev) => EMA_ALPHA * corrected + (1.0 - EMA_ALPHA) * prev,
                        None => corrected,
                    };
                    sensor.filtered_mm = Some(filt);
                    data.range_milli_meter = filt as i16;

                    let wrapped_data = RangingMeasurementData(data);
                    sensor.last_data = wrapped_data.clone();

                    if sensor.device.address() == 0x30 {
                        trace!(
                            "VL53L1X {:#x} read: {} mm (raw {}), σ={} mm, {}",
                            sensor.device.address(),
                            wrapped_data.get_distance_mm(),
                            raw,
                            wrapped_data.get_sigma_mm(),
                            wrapped_data.get_status(),
                        );
                    }

                    watch.sender().send(WithInstant::new(wrapped_data));
                }
                if let Err(e) = clear_interrupt(&mut sensor.device, &mut sensor.i2c) {
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
                // Hold all sensors in reset so they release SDA before SWRST
                sensor_45d_right.xshut_pin.set_low();
                sensor_middle.xshut_pin.set_low();
                sensor_45d_left.xshut_pin.set_low();

                let _ = BUZZER_CHANNEL.try_send(BuzzerTask {
                    freq: 1500.hz(),
                    duration: 200.ms(),
                });

                // SWRST: saves CCR/TRISE/CR2, clears BSY, restores config, re-enables PE
                VL53L1XSensor::i2c_swrst_recovery();

                // Wait for sensors to complete their internal reset
                10.ms_timer().await;

                if let Err(e2) = sensor_45d_right.recover_sensor().await {
                    warn!(
                        "Recovery failed for {:#x}: {:?}",
                        sensor_45d_right.address, e2
                    );
                    500.ms_timer().await;
                    continue;
                }
                if let Err(e2) = sensor_middle.recover_sensor().await {
                    warn!("Recovery failed for {:#x}: {:?}", sensor_middle.address, e2);
                    500.ms_timer().await;
                    continue;
                }
                if let Err(e2) = sensor_45d_left.recover_sensor().await {
                    warn!(
                        "Recovery failed for {:#x}: {:?}",
                        sensor_45d_left.address, e2
                    );
                    500.ms_timer().await;
                    continue;
                }
                info!("All sensors recovered");
            }
        }

        // try to make i2c not crash
        5.ms_timer().await;
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
