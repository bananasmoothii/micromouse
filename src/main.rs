#![no_std]
#![no_main]
extern crate alloc;

mod vec_extension;

use panic_probe as _;
use defmt_rtt as _;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::{bind_interrupts, interrupt};
use embassy_stm32::i2c::I2c;
use embassy_stm32::i2c;
use embassy_stm32::time::Hertz;
use embassy_time::{Delay, Duration, Timer};
use embedded_alloc::LlffHeap as Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

bind_interrupts!(
    pub struct Irqs {
        EXTI15_10 => exti::InterruptHandler<interrupt::typelevel::EXTI15_10>;
        I2C1_EV => i2c::EventInterruptHandler<embassy_stm32::peripherals::I2C1>;
        I2C1_ER => i2c::ErrorInterruptHandler<embassy_stm32::peripherals::I2C1>;
    }
);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    unsafe {
        embedded_alloc::init!(HEAP, 1024);
    }

    let p = embassy_stm32::init(Default::default());

    info!("Initialising I2C");
    let mut config = i2c::Config::default();
    // Use 100kHz for more reliable communication
    config.frequency = Hertz::khz(100);
    config.gpio_speed = Speed::High;

    let mut i2c = I2c::new(
        p.I2C1,
        p.PB8,  // SCL
        p.PB9,  // SDA
        Irqs,
        p.DMA1_CH6,  // TX DMA
        p.DMA1_CH0,  // RX DMA
        config,
    );

    let mut vl53l1_dev = vl53l1::Device::default();

    info!("Toggling XSHUT pin...");
    // XSHUT is active LOW - start with device disabled, then enable it
    let mut xshut_pin = Output::new(p.PA4, Level::Low, Speed::Low);
    Timer::after(Duration::from_millis(10)).await;
    xshut_pin.set_high();
    // Wait for device to power up and stabilize (recommended: at least 2ms)
    Timer::after(Duration::from_millis(100)).await;

    info!("Software reset...");
    while let Err(_e) = vl53l1::software_reset(&mut vl53l1_dev, &mut i2c, &mut Delay) {
        info!("  Error during software reset");
        Timer::after(Duration::from_millis(100)).await;
    }
    info!("  Complete");

    info!("Data init...");
    while let Err(e) = vl53l1::data_init(&mut vl53l1_dev, &mut i2c) {
        info!("  Error during data init: {:?}", e);
        Timer::after(Duration::from_millis(100)).await;
    }
    info!("  Complete");

    info!("Static init...");
    while let Err(e) = vl53l1::static_init(&mut vl53l1_dev) {
        info!("  Error during static init: {:?}", e);
        Timer::after(Duration::from_millis(100)).await;
    }
    info!("  Complete");

    info!("Setting region of interest...");
    let roi = vl53l1::UserRoi {
        bot_right_x: 10,
        bot_right_y: 6,
        top_left_x: 6,
        top_left_y: 10,
    };
    while let Err(e) = vl53l1::set_user_roi(&mut vl53l1_dev, roi.clone()) {
        info!("  Error setting ROI: {:?}", e);
        Timer::after(Duration::from_millis(100)).await;
    }
    info!("  Complete");

    info!("Setting timing budget and inter-measurement period...");
    while let Err(e) = vl53l1::set_measurement_timing_budget_micro_seconds(&mut vl53l1_dev, 100_000) {
        info!("  Error setting timing budget: {:?}", e);
        Timer::after(Duration::from_millis(100)).await;
    }
    while let Err(e) = vl53l1::set_inter_measurement_period_milli_seconds(&mut vl53l1_dev, 200) {
        info!("  Error setting inter-measurement period: {:?}", e);
        Timer::after(Duration::from_millis(100)).await;
    }

    info!("Start measurement...");
    while let Err(e) = vl53l1::start_measurement(&mut vl53l1_dev, &mut i2c) {
        info!("  Error starting measurement: {:?}", e);
        Timer::after(Duration::from_millis(100)).await;
    }
    info!("  Complete");

    loop {
        info!("Wait measurement data ready...");
        if vl53l1::wait_measurement_data_ready(&mut vl53l1_dev, &mut i2c, &mut Delay).is_err() {
            Timer::after(Duration::from_millis(100)).await;
            continue;
        }
        info!("  Ready");

        info!("Get ranging measurement data...");
        match vl53l1::get_ranging_measurement_data(&mut vl53l1_dev, &mut i2c) {
            Err(_e) => {
                info!("  Error getting ranging data");
                Timer::after(Duration::from_millis(70)).await;
            }
            Ok(rmd) => {
                info!("  {:#?} mm", rmd.range_milli_meter);
                continue;
            }
        }

        while let Err(_e) =
            vl53l1::clear_interrupt_and_start_measurement(&mut vl53l1_dev, &mut i2c, &mut Delay)
        {
            info!("  Error clearing interrupt");
            Timer::after(Duration::from_millis(70)).await;
        }
    }
}