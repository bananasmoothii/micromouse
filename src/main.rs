#![no_std]
#![no_main]
extern crate alloc;

mod devices;
pub mod utils;

use crate::utils::{DurationUtils, HertzUtils};
use alloc::boxed::Box;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::exti::{self};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::{I2C1};
use embassy_stm32::spi::Spi;
use embassy_stm32::{bind_interrupts, interrupt};
use embassy_stm32::{i2c, spi};
use embedded_alloc::LlffHeap as Heap;
use crate::devices::mpu9250::Mpu9250Sensor;
use panic_probe as _;

#[global_allocator]
static HEAP: Heap = Heap::empty();
const HEAP_SIZE: usize = 10000;

#[embassy_executor::main]
async fn main(mut spawner: Spawner) {
    println!("Allocating heap, size: {} bytes", HEAP_SIZE);
    unsafe {
        embedded_alloc::init!(HEAP, HEAP_SIZE);
    }

    let p = embassy_stm32::init(Default::default());


    info!("Configuring SPI...");
    let mut spi_config = spi::Config::default();
    // MPU9250 library requires Mode 3 (CPOL=1, CPHA=1)
    // This matches mpu9250::MODE constant: IdleHigh, CaptureOnSecondTransition
    spi_config.mode = spi::Mode {
        polarity: spi::Polarity::IdleHigh,
        phase: spi::Phase::CaptureOnSecondTransition,
    };
    // initialization frequency should not exceed 1MHz
    spi_config.frequency = 1.mhz();

    info!("Creating SPI with MISO pull-down...");
    let spi = Spi::new(
        p.SPI1,     //
        p.PB3,      // SCK
        p.PA7,      // MOSI / SDA
        p.PA6,      // MISO
        p.DMA2_CH3, //
        p.DMA2_CH2, //
        spi_config,
    );

    info!("Setting up chip select (CS)...");
    let mut chip_select = Output::new(p.PC12, Level::High, Speed::Low);
    // let interrupt = ExtiInput::new(p.PA2, p.EXTI2, Pull::None, Irqs);

    // MPU9250 requires CS to be high during power-on to enable SPI mode
    // Pulse CS to ensure the chip recognizes SPI mode
    info!("Pulsing CS to enable SPI mode...");
    chip_select.set_low();
    10.ms_timer().await;
    chip_select.set_high();
    10.ms_timer().await;

    info!("Initializing MPU9250 IMU...");

    let imu = match Mpu9250Sensor::init_new(spi, chip_select) {
        Ok(s) => {
            info!("IMU initialized successfully");
            Box::leak(Box::new(s))
        }
        Err(e) => {
            error!("Failed to initialize IMU: {}", e);
            core::panic!("Sensor initialization failed");
        }
    };

    let data = imu.read().unwrap();
    info!("data: {}", data);
}
