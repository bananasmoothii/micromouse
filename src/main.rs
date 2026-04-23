#![no_std]
#![no_main]
extern crate alloc;

mod devices;
mod i2c_devices;
pub mod positioning;
mod trajectory;
pub mod dimensions;
mod labyrinth;

use crate::devices::battery::battery_monitoring_task;
use crate::devices::hall_sensor_3144::{hall_sensor_continuous_measuring, WheelSide};
use crate::devices::motors::Motor;
use crate::devices::mpu9250::Mpu9250Sensor;
use crate::devices::vl53lxx::vl53l0x::VL53L0XSensor;
use alloc::boxed::Box;
use alloc::vec::Vec;
use defmt::*;
use defmt_rtt as _;
use devices::vl53lxx::vl53l1x::VL53L1XSensor;
use embassy_executor::Spawner;
use embassy_stm32::adc::Adc;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::peripherals::{I2C1, TIM3};
use embassy_stm32::spi::Spi;
use embassy_stm32::{bind_interrupts, interrupt};
use embassy_stm32::{i2c, spi};
use embassy_time::{Duration, Timer};
use embedded_alloc::LlffHeap as Heap;
use panic_probe as _;
use crate::positioning::positioning_task;

#[global_allocator]
static HEAP: Heap = Heap::empty();
const HEAP_SIZE: usize = // Add all big structs here !
    size_of::<VL53L0XSensor>() + size_of::<VL53L1XSensor>() + size_of::<Mpu9250Sensor>() + 10000;

bind_interrupts!(
    struct Irqs {
        EXTI0 => exti::InterruptHandler<interrupt::typelevel::EXTI0>;
        EXTI1 => exti::InterruptHandler<interrupt::typelevel::EXTI1>;
        EXTI2 => exti::InterruptHandler<interrupt::typelevel::EXTI2>;
        EXTI3 => exti::InterruptHandler<interrupt::typelevel::EXTI3>;
        EXTI4 => exti::InterruptHandler<interrupt::typelevel::EXTI4>;
        EXTI9_5 => exti::InterruptHandler<interrupt::typelevel::EXTI9_5>;
        EXTI15_10 => exti::InterruptHandler<interrupt::typelevel::EXTI15_10>;
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

    // poc_trajectory_svg();

    let p = embassy_stm32::init(Default::default());

    spawner
        .spawn(battery_monitoring_task(Adc::new(p.ADC1), p.PC1, p.PC0))
        .unwrap();

    // Start Positioning task
    spawner.spawn(positioning_task()).unwrap();

    /*
    init_i2c_devices(
        &mut spawner,
        p.I2C1,
        p.PB8,
        p.PB9,
        p.DMA1_CH6,
        p.DMA1_CH0,
        Irqs,
        vec![
            Output::new(p.PB13, Level::Low, Speed::Low),
            Output::new(p.PB14, Level::Low, Speed::Low),
            Output::new(p.PB15, Level::Low, Speed::Low),
        ],
        vec![
            ExtiInput::new(p.PC5, p.EXTI5, Pull::None, Irqs),
            ExtiInput::new(p.PC6, p.EXTI6, Pull::None, Irqs),
            ExtiInput::new(p.PC8, p.EXTI8, Pull::None, Irqs),
        ],
    )
        .await;
     */

    info!("Configuring SPI...");
    let mut spi_config = spi::Config::default();
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
        p.PA7,      // MOSI / SDA
        p.PA6,      // MISO (with internal pull-down to prevent floating)
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
    Timer::after(Duration::from_millis(10)).await;
    chip_select.set_high();
    Timer::after(Duration::from_millis(10)).await;

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

    imu.start_continuous_measurement(&mut spawner)
        .await
        .unwrap();


    spawner.spawn(hall_sensor_continuous_measuring(ExtiInput::new(p.PC2, p.EXTI2, Pull::None, Irqs), WheelSide::Left)).unwrap();

    spawner.spawn(hall_sensor_continuous_measuring(ExtiInput::new(p.PC3, p.EXTI3, Pull::None, Irqs), WheelSide::Right)).unwrap();

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

#[embassy_executor::task]
async fn motor_task(mut motor1: Motor<'static, TIM3>) {

    // minimum speed percentage seems to be 9%
    for i in 9..=100 {
        motor1.set_speed(i as f32 * 0.01);
        info!("speed: {}%", i);
        Timer::after(Duration::from_millis(100)).await;
    }
    info!("reached max speed");

    // if this function ends, pins are dropped and the motor halts
    Timer::after(Duration::from_secs(5000)).await;

    // motor1.set_speed(0.5);
    // loop {
    //     motor1.set_direction(MotorDirection::Forward);
    //     Timer::after(Duration::from_secs(1)).await;
    //     motor1.set_direction(MotorDirection::Reverse);
    //     Timer::after(Duration::from_secs(1)).await;
    // }
}

fn poc_trajectory_svg() {
    use crate::trajectory::{Trajectory, TrajectoryOptimizer, SmoothCornerOptimizer, VelocityProfileOptimizer};
    use crate::labyrinth::Labyrinth;

    info!("--- SVG POC START ---");
    let mut lab = Labyrinth::new();

    // Create a simple zig-zag obstacle course
    lab.ray_east_wall(4, 0, true);
    lab.ray_east_wall(4, 1, true);
    lab.ray_east_wall(4, 2, true);
    lab.ray_east_wall(4, 3, true);

    lab.ray_south_wall(5, 4, true);
    lab.ray_south_wall(6, 4, true);
    lab.ray_south_wall(7, 4, true);

    lab.update_cell_distance(0, 0); // Recompute distances based on walls

    let raw_traj = Trajectory::build_from_labyrinth(&lab, 0, 0);

    // 1. Smooth corners
    let smooth_opt = SmoothCornerOptimizer {
        radius: trajectory::config::CORNER_RADIUS,
        corner_speed: trajectory::config::CORNER_SPEED,
    };
    let traj_smooth = smooth_opt.optimize(raw_traj);

    // 2. Trapezoidal velocity profiling
    let vel_opt = VelocityProfileOptimizer {
        max_acceleration: trajectory::config::MAX_ACCELERATION
    };
    let traj_opt = vel_opt.optimize(traj_smooth);

    // Simulate to get points
    let points = traj_opt.simulate(0.0, 0.0, 0.0);

    // Print raw points as SVG polyline.
    // println outputs raw text without timestamp or level.
    println!("<svg viewBox=\"-0.1 -0.1 3.0 3.0\" xmlns=\"http://www.w3.org/2000/svg\">");
    println!("<polyline points=\"");
    for p in points {
        println!("{},{}", p.0, p.1);
    }
    println!("\" fill=\"none\" stroke=\"red\" stroke-width=\"0.02\"/>");
    println!("</svg>");
    info!("--- SVG POC END ---");

    // Hang forever so the rest of the firmware doesn't interfere
    loop {
        cortex_m::asm::wfi();
    }
}

