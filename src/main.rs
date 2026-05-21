#![no_std]
#![no_main]
extern crate alloc;

mod devices;
mod dimensions;
mod flash_log;
mod i2c_devices;
mod labyrinth;
mod panic_handler;
pub mod positioning;
mod trajectory;
pub mod utils;

use crate::devices::battery::start_battery_monitoring;
use crate::devices::buzzer::{BUZZER_CHANNEL, BuzzerTask, buzzer_task};
use crate::devices::hall_sensor_3144;
use crate::devices::motors::{Motor, WheelSide};
use crate::devices::mpu9250::Mpu9250Sensor;
use crate::positioning::positioning_task;
use crate::labyrinth::Labyrinth;
use crate::trajectory::{CardinalHeading, LabyrinthPlan};
use crate::utils::{DurationUtils, HertzUtils};
use alloc::boxed::Box;
use alloc::vec::Vec;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::adc::Adc;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::flash::Flash;
use embassy_stm32::gpio::{Level, Output, OutputType, Pull, Speed};
use embassy_stm32::peripherals::{I2C1, TIM2, TIM3};
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::{bind_interrupts, interrupt};
use embassy_stm32::{i2c, spi, rcc};
use embedded_alloc::LlffHeap as Heap;
use crate::i2c_devices::init_i2c_devices;

#[global_allocator]
static HEAP: Heap = Heap::empty();
const HEAP_SIZE: usize = 10000;

bind_interrupts!(
    struct Irqs {
        EXTI0 => exti::InterruptHandler<interrupt::typelevel::EXTI0>;
        EXTI1 => exti::InterruptHandler<interrupt::typelevel::EXTI1>;
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

    // SYSCLK = 84 MHz from HSI 16 MHz via PLL (VCO=168 MHz, /2 = 84 MHz).
    // APB1 = 42 MHz (max 45), APB2 = 84 MHz (max 90). VOS=SCALE1 + over-drive
    // are set automatically by embassy when a PLL is configured.
    // If CYCLES_PER_US in devices/hall_sensor_3144.rs is ever recomputed by hand,
    // it must match SYSCLK / 1_000_000.
    let mut rcc_config = rcc::Config::default();
    rcc_config.hsi = true;
    rcc_config.pll_src = rcc::PllSource::HSI;
    rcc_config.pll = Some(rcc::Pll {
        prediv: rcc::PllPreDiv::DIV16,
        mul: rcc::PllMul::MUL168,
        divp: Some(rcc::PllPDiv::DIV2),
        divq: Some(rcc::PllQDiv::DIV7),
        divr: None,
    });
    rcc_config.sys = rcc::Sysclk::PLL1_P;
    rcc_config.ahb_pre = rcc::AHBPrescaler::DIV1;
    rcc_config.apb1_pre = rcc::APBPrescaler::DIV2;
    rcc_config.apb2_pre = rcc::APBPrescaler::DIV1;
    let mut stm_config = embassy_stm32::Config::default();
    stm_config.rcc = rcc_config;
    let p = embassy_stm32::init(stm_config);

    start_battery_monitoring(
        &spawner,
        Adc::new(p.ADC1),
        None,
        p.PC1,
        p.PC4,
        [
            Output::new(p.PB1, Level::High, Speed::Low),
            Output::new(p.PB15, Level::High, Speed::Low),
            Output::new(p.PB14, Level::High, Speed::Low),
            Output::new(p.PB13, Level::High, Speed::Low),
        ],
    )
        .await;

    let buzzer_channel = PwmPin::new(p.PA11, OutputType::PushPull);

    spawner
        .spawn(buzzer_task(
            SimplePwm::new(
                p.TIM1,
                None,
                None,
                None,
                Some(buzzer_channel),
                1000.hz(),
                CountingMode::EdgeAlignedUp,
            ),
            Channel::Ch4,
        ))
        .unwrap();

    // Replay any log saved during the previous run, then erase the sector.
    let mut flash = Flash::new_blocking(p.FLASH);
    flash_log::startup_dump(&mut flash);

    // TODO: add pull-up resistors to SDA and SCL
    init_i2c_devices(
        &mut spawner,
        p.I2C1,
        p.PB8,
        p.PB9,
        p.DMA1_CH6,
        p.DMA1_CH0,
        Irqs,
        [
            Output::new(p.PB7, Level::Low, Speed::Low),
            Output::new(p.PB2, Level::Low, Speed::Low),
            Output::new(p.PB12, Level::Low, Speed::Low),
        ],
        [
            ExtiInput::new(p.PC5, p.EXTI5, Pull::Up, Irqs),
            ExtiInput::new(p.PC6, p.EXTI6, Pull::Up, Irqs),
            ExtiInput::new(p.PC8, p.EXTI8, Pull::Up, Irqs),
        ],
    )
        .await;

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

    spawner.spawn(positioning_task(imu)).unwrap();

    hall_sensor_3144::init(p.PC2, p.EXTI2, p.PC3, p.EXTI3);

    let pwm1_pin = PwmPin::new(p.PB4, OutputType::PushPull);
    let pwm1 = SimplePwm::new(
        p.TIM3,
        Some(pwm1_pin),
        None,
        None,
        None,
        Hertz::khz(15),
        CountingMode::EdgeAlignedUp,
    );

    let pwm2_pin = PwmPin::new(p.PB10, OutputType::PushPull);
    let pwm2 = SimplePwm::new(
        p.TIM2,
        None,
        None,
        Some(pwm2_pin),
        None,
        Hertz::khz(15),
        CountingMode::EdgeAlignedUp,
    );

    let motor_left = Motor::new(p.PA8, p.PA9, pwm1, Channel::Ch1, WheelSide::Left);
    let motor_right = Motor::new(p.PB5, p.PC7, pwm2, Channel::Ch3, WheelSide::Right);

    // Start overcurrent protection
    spawner
        .spawn(devices::motors::overcurrent_protection_task(
            Adc::new(p.ADC2),
            p.PA4,
            p.PB0,
            p.PA0,
            p.PA1,
        ))
        .unwrap();

    spawner
        .spawn(maze_runner_task(motor_left, motor_right, flash))
        .unwrap();

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

// ── Maze runner ─────────────────────────────────────────────────────────────

// Start cell and initial heading.  Edit these to change the launch position.
const START_X: usize = 0;
const START_Y: usize = 0;
const START_HEADING: CardinalHeading = CardinalHeading::East;
// Distance from the front wall at which the robot stops, centred in the cell:
//   (LAB_CELL - ROBOT_LENGTH) / 2  =  (0.18 - 0.13) / 2  =  0.025 m
const STOP_OFFSET: f32 = 0.025;

#[embassy_executor::task]
async fn maze_runner_task(
    mut motor_left: Motor<'static, TIM3>,
    mut motor_right: Motor<'static, TIM2>,
    mut flash: Flash<'static, embassy_stm32::flash::Blocking>,
) {
    2.s_timer().await;

    let lab = build_known_maze();
    let plan = LabyrinthPlan::from_labyrinth(&lab, START_X, START_Y, START_HEADING, STOP_OFFSET);
    plan.execute(&mut motor_left, &mut motor_right, 200).await;

    BUZZER_CHANNEL
        .send(BuzzerTask { freq: 1047.hz(), duration: 500.ms() })
        .await;
    1.s_timer().await;
    flash_log::flush(&mut flash);
}

/// Known 8×8 test maze.  Exit at (5, 5).
///
/// ```text
///      x:  0    1    2    3    4    5    6    7
///        ┌────┬────┬────┬────┬────┬────┬────┬────┐
///   y=0  │ S  →    →    →    →    ↓   │         │
///        ╞════╪════╪════╪════╪════╝   ╞════╪════╡  ← south walls (x=0..4)
///   y=1  │                        ↓   ‖         │
///   y=2  │                        ↓   ‖         │  ← east walls (x=5, y=1..4)
///   y=3  │                        ↓   ‖         │
///   y=4  │                        ↓   ‖         │
///        │                        ╞═══╡         │  ← south wall (x=5, y=5 = exit)
///   y=5  │                        E             │
///        └────┴────┴────┴────┴────┴────┴────┴────┘
///
///  →↓ robot path   S start (0,0)   E exit (5,5)
///  ╞═╡ south wall   ‖ east wall
/// ```
///
/// Robot path: East 5 cells → turn right → South 5 cells → stop at exit.
///
/// Wall definitions
/// ─────────────────
/// south walls at y=0, x∈{0,1,2,3,4}  — floor of east corridor
/// east wall at x=5, y∈{0,1,2,3,4}    — right wall of south corridor (also acts as
///                                        "dead end to the east" forcing the turn)
/// south wall at (x=5, y=5)            — exit stop wall
fn build_known_maze() -> Labyrinth {
    let mut lab = Labyrinth::new();

    // Bottom wall of the east corridor (row 0)
    for x in 0..5 {
        lab.ray_south_wall(x, 0, true);
    }

    // Right wall of the south corridor (column 5) + dead end forcing east→south turn
    for y in 0..5 {
        lab.ray_east_wall(5, y, true);
    }

    // Stop wall at the exit cell
    lab.ray_south_wall(5, 5, true);

    lab
}

// ── Commented-out single-segment motor test (kept for reference) ─────────────
/*
#[embassy_executor::task]
async fn motor_tests(
    mut motor_left: Motor<'static, TIM3>,
    mut motor_right: Motor<'static, TIM2>,
    mut flash: Flash<'static, embassy_stm32::flash::Blocking>,
) {
    use crate::trajectory::straight_line::{StraightLine, StraightLineGoal};
    use crate::trajectory::in_place_turn::InPlaceTurn;

    2.s_timer().await;
    let segments: alloc::vec::Vec<Box<dyn TrajectorySegment>> = alloc::vec![
        Box::new(StraightLine { goal: StraightLineGoal::DistanceToFrontWall(0.02), out_speed: 0.0 }),
        Box::new(InPlaceTurn::from_degrees(90.0)),
        Box::new(StraightLine { goal: StraightLineGoal::DistanceToFrontWall(0.02), out_speed: 0.0 }),
        Box::new(InPlaceTurn::from_degrees(90.0)),
        Box::new(StraightLine { goal: StraightLineGoal::DistanceToFrontWall(0.02), out_speed: 0.0 }),
        Box::new(InPlaceTurn::from_degrees(90.0)),
        Box::new(StraightLine { goal: StraightLineGoal::DistanceToFrontWall(0.02), out_speed: 0.0 }),
        Box::new(InPlaceTurn::from_degrees(90.0)),
    ];
    for segment in &segments {
        segment.execute(&mut motor_left, &mut motor_right).await;
        motor_left.set_speed(0.0);
        motor_right.set_speed(0.0);
        200.ms_timer().await;
    }
    BUZZER_CHANNEL.send(BuzzerTask { freq: 1047.hz(), duration: 500.ms() }).await;
    1.s_timer().await;
    flash_log::flush(&mut flash);
}
*/
