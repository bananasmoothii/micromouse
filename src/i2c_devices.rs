use crate::devices::vl53lxx::vl53l1x::VL53L1XSensor;
use crate::{Irqs, devices};
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
use embassy_time::{Duration, Timer};
use embedded_hal_bus::i2c::RefCellDevice;

pub async fn init_i2c_devices<const N: usize>(
    spawner: &mut Spawner,
    i2c_peri: Peri<'static, I2C1>,
    scl: Peri<'static, PB8>,
    sda: Peri<'static, PB9>,
    tx_dma: Peri<'static, DMA1_CH6>,
    rx_dma: Peri<'static, DMA1_CH0>,
    irqs: Irqs,
    mut xshuts: [Output<'static>; N],
    interrupts: [ExtiInput<'static>; N],
) {
    let mut i2c_config = Config::default();
    // Use 200kHz for reliable communication
    i2c_config.frequency = Hertz::khz(200);
    i2c_config.gpio_speed = Speed::High;
    i2c_config.timeout = Duration::from_millis(50);

    // 1. Initial State: Pull all XSHUT low to keep all sensors in reset
    info!("Resetting all distance sensors...");
    for xshut in &mut xshuts {
        xshut.set_low();
    }
    Timer::after(Duration::from_millis(50)).await;

    let i2c = I2c::new(i2c_peri, scl, sda, irqs, tx_dma, rx_dma, i2c_config);

    // Leak i2c_rc to get a 'static reference, required for the sensor
    let i2c_rc = Box::leak(Box::new(RefCell::new(i2c)));

    // 2. Sequential Initialization
    info!("Initializing distance sensors...");
    let mut initialized_sensors = Vec::with_capacity(N);
    let base_address = 0x30;

    for (i, (xshut, interrupt)) in xshuts.into_iter().zip(interrupts.into_iter()).enumerate() {
        let new_address = base_address + (i as u8);

        let i2c_dev = RefCellDevice::new(i2c_rc);

        match VL53L1XSensor::init_new(
            devices::vl53lxx::Config {
                timing_config: devices::vl53lxx::TimingConfig::default(),
                xshut_pin: xshut,
                gpio_interrupt: interrupt,
            },
            i2c_dev,
            new_address,
        ).await {
            Ok(s) => {
                info!("Distance sensor {} initialized at 0x{:02x}", i, new_address);
                initialized_sensors.push(Box::leak(Box::new(s)));
            }
            Err(e) => {
                error!("Failed to initialize distance sensor {}: {}", i, e);
                core::panic!("Sensor initialization failed");
            }
        }
        if i == 1 {
            break;
        }
    }

    info!("Starting continuous measurement for {} sensors", N);

    for sensor in initialized_sensors {
        sensor.start_continuous_measurement(spawner).await.unwrap();
    }
}
