#![no_std]
#![no_main]
extern crate alloc;

mod i2c_devices;
mod sensor;

use crate::i2c_devices::init_i2c_devices;
use crate::sensor::mpu9250::Mpu9250Sensor;
use crate::sensor::vl53lxx::vl53l0x::VL53L0XSensor;
use alloc::vec;
use alloc::vec::Vec;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::peripherals::I2C1;
use embassy_stm32::{bind_interrupts, interrupt};
use embassy_stm32::{i2c, spi};
use embedded_alloc::LlffHeap as Heap;
use panic_probe as _;
use sensor::vl53lxx::vl53l1x::VL53L1XSensor;

#[global_allocator]
static HEAP: Heap = Heap::empty();
const HEAP_SIZE: usize = // Add all big structs here !
    size_of::<VL53L0XSensor>() + size_of::<VL53L1XSensor>() + size_of::<Mpu9250Sensor>() + 500;

bind_interrupts!(
    struct Irqs {
        EXTI15_10 => exti::InterruptHandler<interrupt::typelevel::EXTI15_10>;
        EXTI0 => exti::InterruptHandler<interrupt::typelevel::EXTI0>;
        EXTI1 => exti::InterruptHandler<interrupt::typelevel::EXTI1>;
        EXTI2 => exti::InterruptHandler<interrupt::typelevel::EXTI2>;
        I2C1_EV => i2c::EventInterruptHandler<I2C1>;
        I2C1_ER => i2c::ErrorInterruptHandler<I2C1>;
    }
);

#[embassy_executor::main]
async fn main(mut spawner: Spawner) {
    println!("Allocating heap, size: {} bytes", HEAP_SIZE);
    unsafe {
        embedded_alloc::init!(HEAP, HEAP_SIZE);
    }

    let p = embassy_stm32::init(Default::default());

    init_i2c_devices(
        &mut spawner,
        p.I2C1,
        p.PB8,
        p.PB9,
        p.DMA1_CH6,
        p.DMA1_CH0,
        Irqs,
        vec![
            Output::new(p.PC9, Level::Low, Speed::Low),
            Output::new(p.PC8, Level::Low, Speed::Low),
        ],
        vec![
            ExtiInput::new(p.PA0, p.EXTI0, Pull::None, Irqs),
            ExtiInput::new(p.PA1, p.EXTI1, Pull::None, Irqs),
        ],
    )
        .await;

    /*
    info!("Configuring SPI...");
    let mut spi_config = spi::Config::default();
    spi_config.frequency = Hertz::khz(100); // Start with 100kHz for maximum reliability
    // MPU9250 library requires Mode 3 (CPOL=1, CPHA=1)
    // This matches mpu9250::MODE constant: IdleHigh, CaptureOnSecondTransition
    spi_config.mode = spi::Mode {
        polarity: spi::Polarity::IdleHigh,
        phase: spi::Phase::CaptureOnSecondTransition,
    };

    info!("Creating SPI with MISO pull-down...");
    let spi = Spi::new(
        p.SPI1,     //
        p.PB3,      // SCK
        p.PB5,      // MOSI / SDA
        p.PB4,      // MISO (with internal pull-down to prevent floating)
        p.DMA2_CH3, //
        p.DMA2_CH2, //
        spi_config,
    );

    info!("Setting up chip select (CS)...");
    let mut chip_select = Output::new(p.PC6, Level::High, Speed::Medium);
    let interrupt = ExtiInput::new(p.PA2, p.EXTI2, Pull::None, Irqs);

    // MPU9250 requires CS to be high during power-on to enable SPI mode
    // Pulse CS to ensure the chip recognizes SPI mode
    info!("Pulsing CS to enable SPI mode...");
    chip_select.set_high();
    embassy_time::Timer::after(embassy_time::Duration::from_millis(50)).await;
    chip_select.set_low();
    embassy_time::Timer::after(embassy_time::Duration::from_micros(100)).await;
    chip_select.set_high();
    embassy_time::Timer::after(embassy_time::Duration::from_millis(10)).await;

    info!("Initializing MPU9250 IMU...");

    let imu =
        Mpu9250Sensor::init_new(spi, chip_select, interrupt);
    let imu = match imu {
        Ok(s) => {
            info!("IMU initialized successfully");
            Box::leak(Box::new(s))
        }
        Err(e) => {
            error!("Failed to initialize IMU: {}", e);
            core::panic!("Sensor initialization failed");
        }
    };
    */

    // imu.start_continuous_measurement(&mut spawner, &|data| {
    //     info!(
    //         "New IMU data: Accel: {:?}, Gyro: {:?}, Mag: {:?}, Temp: {}",
    //         data.accel, data.gyro, data.mag, data.temp
    //     );
    // })
    // .await
    // .unwrap();

    let user_button = ExtiInput::new(p.PC13, p.EXTI13, Pull::None, Irqs);
    let led = Output::new(p.PA5, Level::Low, Speed::Medium);

    button_task(user_button, led).await;
}

async fn button_task(mut button: ExtiInput<'_>, mut led: Output<'_>) {
    info!("Main task ready");
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
