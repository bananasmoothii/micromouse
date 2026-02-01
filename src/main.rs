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
        // used for button input
        EXTI15_10 => exti::InterruptHandler<interrupt::typelevel::EXTI15_10>;
        // used for gpio input
        EXTI0 => exti::InterruptHandler<interrupt::typelevel::EXTI0>;
        // I2C1 interrupts
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
    config.frequency = Hertz::khz(100); // doesn't seem to work above 250kHz even though spec says 400kHz is supported
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

    let mut gpio_input = ExtiInput::new(p.PA0, p.EXTI0, Pull::None, Irqs);

    info!("Toggling XSHUT pin...");
    // XSHUT is active LOW - start with device disabled, then enable it
    let mut xshut_pin = Output::new(p.PA4, Level::Low, Speed::Low);
    Timer::after(Duration::from_millis(10)).await;
    xshut_pin.set_high();
    // Wait for device to power up and stabilize (recommended: at least 2ms)
    Timer::after(Duration::from_millis(10)).await;

    // info!("Software reset...");
    // while let Err(_e) = vl53l1::software_reset(&mut vl53l1_dev, &mut i2c, &mut Delay) {
    //     info!("  Error during software reset");
    //     Timer::after(Duration::from_millis(100)).await;
    // }
    // info!("  Complete");

    vl53l1::data_init(&mut vl53l1_dev, &mut i2c).unwrap();
    vl53l1::static_init(&mut vl53l1_dev).unwrap();
    vl53l1::set_preset_mode(&mut vl53l1_dev, vl53l1::PresetMode::Autonomous).unwrap();

    vl53l1::set_user_roi(&mut vl53l1_dev, vl53l1::UserRoi {
        top_left_x: 0,
        top_left_y: 15,
        bot_right_x: 15,
        bot_right_y: 0,
    }).unwrap();

    info!("Setting timing budget and inter-measurement period...");
    // 66ms -> 15 measurements per second
    vl53l1::set_measurement_timing_budget_micro_seconds(&mut vl53l1_dev, 66_000).unwrap();
    // it doesn't work under 69ms for some reason
    vl53l1::set_inter_measurement_period_milli_seconds(&mut vl53l1_dev, 69).unwrap();
    // vl53l1::set_distance_mode(&mut vl53l1_dev, vl53l1::DistanceMode::Long).unwrap();

    info!("Start measurement...");
    vl53l1::start_measurement(&mut vl53l1_dev, &mut i2c).unwrap();
    info!("  Complete");

    loop {
        // Wait for measurement ready interrupt
        gpio_input.wait_for_falling_edge().await;

        match vl53l1::get_ranging_measurement_data(&mut vl53l1_dev, &mut i2c) {
            Err(e) => {
                info!("  Error getting ranging data - attempting recovery ({:?})", e);
                // Try to recover by stopping and restarting measurements
                let _ = vl53l1::stop_measurement(&mut vl53l1_dev, &mut i2c);
                Timer::after(Duration::from_millis(100)).await;
                if vl53l1::start_measurement(&mut vl53l1_dev, &mut i2c).is_ok() {
                    info!("  Measurement restarted");
                }
                continue;
            }
            Ok(rmd) => {
                // Check if data looks valid
                if rmd.range_quality_level == 0 {
                    info!("  Warning: Invalid data (quality=0)");
                } else {
                    info!("  {} mm (σ:{} mm, {:?})", rmd.range_milli_meter, rmd.sigma_milli_meter, rmd.range_status);
                }
            }
        }

        // Always clear interrupt after successfully getting measurement data
        if let Err(e) = vl53l1::clear_interrupt_and_start_measurement(&mut vl53l1_dev, &mut i2c, &mut Delay) {
            info!("  Error clearing interrupt - attempting recovery ({:?})", e);
            // Try to recover
            let _ = vl53l1::stop_measurement(&mut vl53l1_dev, &mut i2c);
            Timer::after(Duration::from_millis(100)).await;
            if vl53l1::start_measurement(&mut vl53l1_dev, &mut i2c).is_ok() {
                info!("  Measurement restarted");
            }
        }
    }
}