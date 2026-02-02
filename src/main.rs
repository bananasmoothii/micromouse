#![no_std]
#![no_main]
extern crate alloc;

mod distance_sensor;
mod sensor;

use alloc::boxed::Box;
use alloc::vec::Vec;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::i2c;
use embassy_stm32::i2c::{I2c, Master};
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, interrupt};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::mutex::Mutex;
use embedded_alloc::LlffHeap as Heap;
use panic_probe as _;
use sensor::Sensor;
use sensor::vl53lxx::{Config, TimingConfig};

#[global_allocator]
static HEAP: Heap = Heap::empty();

bind_interrupts!(
    pub struct Irqs {
        // used for button input
        EXTI15_10 => exti::InterruptHandler<interrupt::typelevel::EXTI15_10>;
        // used for gpio input (VL53L1X interrupt)
        EXTI0 => exti::InterruptHandler<interrupt::typelevel::EXTI0>;
        // I2C1 interrupts
        I2C1_EV => i2c::EventInterruptHandler<embassy_stm32::peripherals::I2C1>;
        I2C1_ER => i2c::ErrorInterruptHandler<embassy_stm32::peripherals::I2C1>;
    }
);

#[embassy_executor::main]
async fn main(mut spawner: Spawner) {
    unsafe {
        embedded_alloc::init!(HEAP, 1024);
    }

    let p = embassy_stm32::init(Default::default());

    info!("Setting up I2C for VL53L1X sensor");
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
    let i2c_mutex: Mutex<CriticalSectionRawMutex, _> = Mutex::new(i2c);
    let i2c_mutex = Box::leak(Box::new(i2c_mutex));

    // GPIO interrupt pin for VL53L1X (active low when measurement ready)
    let gpio_interrupt = ExtiInput::new(p.PA0, p.EXTI0, Pull::None, Irqs);

    // XSHUT pin for VL53L1X (active low to disable sensor)
    let xshut_pin = Output::new(p.PA4, Level::Low, Speed::Low);

    // Initialize the distance sensor using the trait-based API
    info!("Initializing distance sensor");

    let sensor_config = Config {
        timing_config: TimingConfig::default(),
        xshut_pin,
        gpio_interrupt,
    };

    let sensor =
        match sensor::vl53lxx::vl53l1x::VL53L1XSensor::init_new(sensor_config, i2c_mutex).await {
            Ok(s) => {
                info!("Distance sensor initialized successfully");
                Box::leak(Box::new(s))
            }
            Err(e) => {
                error!("Failed to initialize distance sensor: {:?}", e);
                core::panic!("Sensor initialization failed");
            }
        };

    // Start continuous measurement in the background
    info!("Starting continuous measurement");
    sensor.start_continuous_measurement(&mut spawner).unwrap();

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
