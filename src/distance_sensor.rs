use alloc::format;
use alloc::string::String;
use defmt::*;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Output};
use embassy_stm32::i2c;
use embassy_stm32::i2c::{I2c, Master};
use embassy_stm32::mode::Async;
use embassy_time::{Delay, Duration, Timer};
use vl53l1::RangeStatus::SIGNAL_FAIL;

/// Configuration for the VL53L1X distance sensor
pub struct DistanceSensorConfig {
    /// Measurement timing budget in microseconds (recommended: 66000 for 15Hz)
    pub timing_budget_us: u32,
    /// Inter-measurement period in milliseconds (minimum: 69ms from testing)
    pub inter_measurement_period_ms: u32,
}

impl Default for DistanceSensorConfig {
    fn default() -> Self {
        Self {
            timing_budget_us: 66_000,
            inter_measurement_period_ms: 69,
        }
    }
}

/// Initialize the VL53L1X distance sensor
pub async fn init_sensor(
    i2c: &mut I2c<'static, Async, Master>,
    xshut_pin: &mut Output<'static>,
    config: DistanceSensorConfig,
) -> Result<vl53l1::Device, vl53l1::Error<i2c::Error>> {
    info!("Initializing VL53L1X distance sensor");

    // Toggle XSHUT pin to reset the device
    info!("  Toggling XSHUT pin...");
    xshut_pin.set_low();
    Timer::after(Duration::from_millis(10)).await;
    xshut_pin.set_high();
    Timer::after(Duration::from_millis(10)).await;

    let mut dev = vl53l1::Device::default();

    // Initialize the sensor
    info!("  Data init...");
    vl53l1::data_init(&mut dev, i2c)?;

    info!("  Static init...");
    vl53l1::static_init(&mut dev)?;

    info!("  Setting preset mode...");
    vl53l1::set_preset_mode(&mut dev, vl53l1::PresetMode::Autonomous)?;

    // Set full field of view
    info!("  Setting ROI...");
    vl53l1::set_user_roi(
        &mut dev,
        vl53l1::UserRoi {
            top_left_x: 0,
            top_left_y: 15,
            bot_right_x: 15,
            bot_right_y: 0,
        },
    )?;

    info!("  Setting timing budget and inter-measurement period...");
    vl53l1::set_measurement_timing_budget_micro_seconds(&mut dev, config.timing_budget_us)?;
    vl53l1::set_inter_measurement_period_milli_seconds(
        &mut dev,
        config.inter_measurement_period_ms,
    )?;

    info!("  Starting measurement...");
    vl53l1::start_measurement(&mut dev, i2c)?;

    info!("VL53L1X initialization complete");
    Ok(dev)
}

/// Attempt to recover from a sensor error by stopping and restarting measurements
async fn recover_sensor(
    dev: &mut vl53l1::Device,
    i2c: &mut I2c<'static, Async, Master>,
) -> Result<(), vl53l1::Error<i2c::Error>> {
    info!("  Attempting sensor recovery...");
    let _ = vl53l1::stop_measurement(dev, i2c);
    Timer::after(Duration::from_millis(100)).await;
    vl53l1::start_measurement(dev, i2c)?;
    info!("  Sensor recovered");
    Ok(())
}

/// Embassy task for the VL53L1X distance sensor
///
/// This task continuously reads distance measurements and logs them.
/// It uses the GPIO interrupt pin to detect when new measurements are ready.
#[embassy_executor::task]
pub async fn distance_sensor_task(
    mut i2c: I2c<'static, Async, Master>,
    mut gpio_interrupt: ExtiInput<'static>,
    mut xshut_pin: Output<'static>,
) {
    let mut dev = match init_sensor(&mut i2c, &mut xshut_pin, DistanceSensorConfig::default()).await
    {
        Ok(dev) => dev,
        Err(e) => {
            error!("Failed to initialize VL53L1X sensor: {:?}", e);
            return;
        }
    };

    info!("Distance sensor task running");

    let mut recorver = false;

    loop {
        if !recorver {
            gpio_interrupt.wait_for_falling_edge().await;
        } else {
            while let Err(e) = vl53l1::wait_measurement_data_ready(&mut dev, &mut i2c, &mut Delay) {
                let str = match e {
                    nb::Error::Other(e) => format!("other error: {:?}", e),
                    nb::Error::WouldBlock => String::from("Operation would block"),
                };
                warn!("Waiting for measurement data ready failed ({}), retrying...", str.as_str());
                Timer::after(Duration::from_millis(10)).await;
            }
            info!("Measurement data ready after recovery");
            recorver = false;
        }

        // Get the ranging measurement data
        match vl53l1::get_ranging_measurement_data(&mut dev, &mut i2c) {
            Err(e) => {
                warn!("Error getting ranging data: {:?}", e);
                if recover_sensor(&mut dev, &mut i2c).await.is_err() {
                    error!("Failed to recover sensor, waiting before retry...");
                    recorver = true;
                    Timer::after(Duration::from_millis(500)).await;
                }
                continue;
            }
            Ok(rmd) => {
                // Check if data looks valid
                if rmd.range_status != SIGNAL_FAIL {
                    info!(
                        "Distance: {} mm (σ: {} mm, status: {:?})",
                        rmd.range_milli_meter,
                        rmd.sigma_milli_meter as f64 / 65536.0,
                        rmd.range_status
                    );
                }
            }
        }

        // Clear interrupt and start next measurement
        if let Err(e) =
            vl53l1::clear_interrupt_and_start_measurement(&mut dev, &mut i2c, &mut Delay)
        {
            warn!("Error clearing interrupt: {:?}", e);
            if recover_sensor(&mut dev, &mut i2c).await.is_err() {
                error!("Failed to recover sensor, waiting before retry...");
                Timer::after(Duration::from_millis(500)).await;
                recorver = true;
            }
        }
    }
}
