#![no_std]
#![no_main]
extern crate alloc;

mod sensor;

use crate::sensor::vl53lxx::vl53l0x::VL53L0XSensor;
use alloc::boxed::Box;
use alloc::vec::Vec;
use core::cell::RefCell;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::i2c;
use embassy_stm32::i2c::I2c;
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, interrupt};
use embedded_alloc::LlffHeap as Heap;
use embedded_hal_bus::i2c::RefCellDevice;
use panic_probe as _;
use sensor::Sensor;
use sensor::vl53lxx::vl53l1x::VL53L1XSensor;
use sensor::vl53lxx::{Config, TimingConfig};

#[global_allocator]
static HEAP: Heap = Heap::empty();
const HEAP_SIZE: usize = size_of::<VL53L0XSensor>() + size_of::<VL53L1XSensor>() + 500;

bind_interrupts!(
    pub struct Irqs {
        // used for button input
        EXTI15_10 => exti::InterruptHandler<interrupt::typelevel::EXTI15_10>;
        // used for gpio input (VL53LXX interrupt)
        EXTI0 => exti::InterruptHandler<interrupt::typelevel::EXTI0>;
        EXTI1 => exti::InterruptHandler<interrupt::typelevel::EXTI1>;
        // I2C1 interrupts
        I2C1_EV => i2c::EventInterruptHandler<embassy_stm32::peripherals::I2C1>;
        I2C1_ER => i2c::ErrorInterruptHandler<embassy_stm32::peripherals::I2C1>;
    }
);

#[embassy_executor::main]
async fn main(mut spawner: Spawner) {
    println!("Allocating heap, size: {} bytes", HEAP_SIZE);
    unsafe {
        embedded_alloc::init!(HEAP, HEAP_SIZE);
    }

    let p = embassy_stm32::init(Default::default());

    let mut i2c_config = i2c::Config::default();
    // Use 100kHz for more reliable communication
    i2c_config.frequency = Hertz::khz(200);
    i2c_config.gpio_speed = Speed::High;

    // I2C needs to be leaked to get a 'static reference for the sensor
    let i2c = I2c::new(
        p.I2C1, p.PB8, // SCL
        p.PB9, // SDA
        Irqs, p.DMA1_CH6, // TX DMA
        p.DMA1_CH0, // RX DMA
        i2c_config,
    );
    // Leak i2c_rc to get a 'static reference, required for the sensor
    let i2c_rc = Box::leak(Box::new(RefCell::new(i2c)));

    // Initialize the distance sensor using the trait-based API
    info!("Initializing distance sensors...");

    let xshut0 = Output::new(p.PC9, Level::Low, Speed::Low);
    let xshut1 = Output::new(p.PC8, Level::Low, Speed::Low);

    let sensor0 = VL53L0XSensor::init_new(
        Config {
            timing_config: TimingConfig::default(),
            xshut_pin: xshut0,
            gpio_interrupt: ExtiInput::new(p.PA0, p.EXTI0, Pull::None, Irqs),
        },
        RefCellDevice::new(i2c_rc),
    )
        .await;

    let sensor1 = VL53L1XSensor::init_new(
        Config {
            timing_config: TimingConfig::default(),
            xshut_pin: xshut1,
            gpio_interrupt: ExtiInput::new(p.PA1, p.EXTI1, Pull::None, Irqs),
        },
        RefCellDevice::new(i2c_rc),
    )
        .await;

    let sensor0 = match sensor0 {
        Ok(s) => {
            info!("Distance sensor 0 initialized successfully");
            Box::leak(Box::new(s))
        }
        Err(e) => {
            error!("Failed to initialize distance 0 sensor: {}", e);
            core::panic!("Sensor initialization failed");
        }
    };

    let sensor1 = match sensor1 {
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
            info!("New measurement: {} mm {} σ={}", data.range_milli_meter, data.range_status, data.sigma_milli_meter);
        })
        .await
        .unwrap();

    info!("Main task ready");
    let mut button = ExtiInput::new(p.PC13, p.EXTI13, Pull::None, Irqs);
    let mut led = Output::new(p.PA5, Level::Low, Speed::Medium);

    let mut toggle_led = || {
        led.toggle();
    };

    let mut button_actions: Vec<&mut dyn FnMut()> = Vec::new();
    button_actions.push(&mut toggle_led);

    loop {
        button.wait_for_any_edge().await;
        for action in button_actions.iter_mut() {
            action()
        }
    }
}
