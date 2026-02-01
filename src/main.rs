#![no_std]
#![no_main]
extern crate alloc;

mod distance_sensor;
mod vec_extension;

use panic_probe as _;
use defmt_rtt as _;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::{bind_interrupts, interrupt};
use embassy_stm32::i2c::{I2c, Master};
use embassy_stm32::i2c;
use embassy_stm32::mode::Async;
use embassy_stm32::time::Hertz;
use embedded_alloc::LlffHeap as Heap;

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
async fn main(spawner: Spawner) {
    unsafe {
        embedded_alloc::init!(HEAP, 1024);
    }

    let p = embassy_stm32::init(Default::default());

    info!("Setting up I2C for VL53L1X sensor");
    let mut i2c_config = i2c::Config::default();
    // Use 100kHz for more reliable communication
    i2c_config.frequency = Hertz::khz(100);
    i2c_config.gpio_speed = Speed::High;

    let i2c = I2c::new(
        p.I2C1,
        p.PB8,  // SCL
        p.PB9,  // SDA
        Irqs,
        p.DMA1_CH6,  // TX DMA
        p.DMA1_CH0,  // RX DMA
        i2c_config,
    );

    // GPIO interrupt pin for VL53L1X (active low when measurement ready)
    let gpio_interrupt = ExtiInput::new(p.PA0, p.EXTI0, embassy_stm32::gpio::Pull::None, Irqs);

    // XSHUT pin for VL53L1X (active low to disable sensor)
    let xshut_pin = Output::new(p.PA4, Level::Low, Speed::Low);

    // Spawn the distance sensor task
    info!("Spawning distance sensor task");
    spawner.spawn(distance_sensor::distance_sensor_task(
        i2c,
        gpio_interrupt,
        xshut_pin,
    )).unwrap();

    // Main task can do other things here
    info!("Main task ready");
    loop {
        // Main task loop - can add other functionality here
        embassy_time::Timer::after(embassy_time::Duration::from_secs(1)).await;
    }
}