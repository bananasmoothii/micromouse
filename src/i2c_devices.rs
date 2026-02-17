use crate::sensor::Sensor;
use crate::sensor::vl53lxx::TimingConfig;
use crate::sensor::vl53lxx::vl53l0x::VL53L0XSensor;
use crate::sensor::vl53lxx::vl53l1x::VL53L1XSensor;
use crate::{Irqs, sensor};
use alloc::boxed::Box;
use alloc::vec::Vec;
use core::cell::RefCell;
use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_stm32::Peri;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Output, Speed};
use embassy_stm32::i2c::{Config, I2c};
use embassy_stm32::peripherals::{DMA1_CH0, DMA1_CH6, I2C1, PB8, PB9};
use embassy_stm32::time::Hertz;
use embedded_hal_bus::i2c::RefCellDevice;

pub async fn init_i2c_devices(
    mut spawner: &mut Spawner,
    i2c_peri: Peri<'static, I2C1>,
    scl: Peri<'static, PB8>,
    sda: Peri<'static, PB9>,
    tx_dma: Peri<'static, DMA1_CH6>,
    rx_dma: Peri<'static, DMA1_CH0>,
    irqs: Irqs,
    mut xshuts: Vec<Output<'static>>,
    mut interrupts: Vec<ExtiInput<'static>>,
) {
    let mut i2c_config = Config::default();
    // Use 100kHz for more reliable communication
    i2c_config.frequency = Hertz::khz(200);
    i2c_config.gpio_speed = Speed::High;

    let i2c = I2c::new(i2c_peri, scl, sda, irqs, tx_dma, rx_dma, i2c_config);

    // Leak i2c_rc to get a 'static reference, required for the sensor
    let i2c_rc = Box::leak(Box::new(RefCell::new(i2c)));

    // Initialize the distance sensor using the trait-based API
    info!("Initializing distance sensors...");

    let sensor0 = match VL53L0XSensor::init_new(
        sensor::vl53lxx::Config {
            timing_config: TimingConfig::default(),
            xshut_pin: xshuts.remove(0),
            gpio_interrupt: interrupts.remove(0),
        },
        RefCellDevice::new(i2c_rc),
    )
        .await
    {
        Ok(s) => {
            info!("Distance sensor 0 initialized successfully");
            Box::leak(Box::new(s))
        }
        Err(e) => {
            error!("Failed to initialize distance 0 sensor: {}", e);
            core::panic!("Sensor initialization failed");
        }
    };

    let sensor1 = match VL53L1XSensor::init_new(
        sensor::vl53lxx::Config {
            timing_config: TimingConfig::default(),
            xshut_pin: xshuts.remove(0),
            gpio_interrupt: interrupts.remove(0),
        },
        RefCellDevice::new(i2c_rc),
    )
        .await
    {
        Ok(s) => {
            info!("Distance sensor 1 initialized successfully");
            Box::leak(Box::new(s))
        }
        Err(e) => {
            error!("Failed to initialize distance 1 sensor: {}", e);
            core::panic!("Sensor initialization failed");
        }
    };

    info!("Starting continuous measurement");
    sensor0
        .start_continuous_measurement(&mut spawner, &|data| {
            info!("New measurement: {} mm {}", data.distance_mm, data.status);
        })
        .await
        .unwrap();

    sensor1
        .start_continuous_measurement(&mut spawner, &|data| {
            info!(
                "New measurement: {} mm {} σ={}",
                data.range_milli_meter,
                data.range_status,
                data.sigma_milli_meter as f32 / 65536.0
            );
        })
        .await
        .unwrap();
}
