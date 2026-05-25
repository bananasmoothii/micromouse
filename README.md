# [Bananasmoothii's Micromouse](https://github.com/bananasmoothii/micromouse)

![robot](./docs/timeline/media/PXL_20260517_130954581.jpg)

A school project aiming to make a Micromouse robot and make it solve a maze.

This repo contains almost everything related to this project except the
[**CAD models**](
https://cad.onshape.com/documents/aaf7a6983651c7702bceaa13/w/41b76918edb6f00955c96fe0/e/62dd90809362918a445464ef)

## Hardware used

* Nucleo board (with ST-Link): MCU is STM32F446RE
* Monster Moto shield
* Two IGARASHI N 2738-45 motors
* One LiPo 2S battery (6,6V - 8.4V)
* Two 3144 Hall sensors (magnet → `DOUT = HIGH`) for odometry (12 magnets per wheel)
* One MPU 9250 (accelerometer + gyroscope + magnetometer + temperature sensor)
* Three VL53L1X distance sensors

## Flash and run code

Install [probe-rs](https://probe.rs/) with one of the methods in their docs, for example with Cargo:

```sh
cargo binstall probe-rs-tools
```

Then flash and run with:

```sh
cargo run
```

(configured in `./.cargo`)

The robot has a power mechanism that cuts power when battery isn't measured to be enough, but this requires you to
maintain the button (on the back custom PCB) pressed when flashing the MCU. I recommend having an IDE keyboard
shortcut for this command. Flashing often fails on the first try, if it does just try again.

## Repository structure

```
micromouse/
├── src/
│   ├── main.rs                   — Embassy entry point: heap init, peripheral setup, task spawning
│   ├── dimensions.rs             — Physical constants (cell size, wall thickness, wheel geometry)
│   ├── i2c_devices.rs            — Sequential XSHUT-based I2C init for the 3 ToF sensors
│   ├── flash_log.rs              — For storing logs in flash memory
│   ├── labyrinth.rs              — 16×16 maze grid, probabilistic wall detection, pathfinding (flood-fill)
│   ├── utils.rs                  — Shared utilities
│   ├── panic_handler.rs          — Custom panic handler (logs then halts)
│   │
│   ├── devices/                  — Hardware drivers
│   │   ├── motors.rs             — DC motor PI feedback loop, reads waypoints from PATH_CHANNEL
│   │   ├── hall_sensor_3144.rs   — Interrupt-driven wheel encoders (AtomicI32 tick counters)
│   │   ├── mpu9250.rs            — 9-DOF IMU over SPI (accel, gyro, magnetometer)
│   │   ├── battery.rs            — 2-cell LiPo voltage monitoring via ADC
│   │   ├── buzzer.rs             — PWM audio (TIM2), command channel-driven
│   │   └── vl53lxx/              — ToF distance sensors
│   │       ├── vl53l0x.rs        — VL53L0X driver (unused, kept for reference)
│   │       └── vl53l1x.rs        — VL53L1X driver (distance sensors, runtime I2C address change)
│   │
│   ├── positioning/              — Sensor fusion and position tracking
│   │   ├── mod.rs                — positioning_task: 20 ms sensor-fusion loop
│   │   ├── odometry.rs           — Skid-steering wheel kinematics
│   │   ├── mpu.rs                — Gyro → yaw, magnetometer → absolute heading
│   │   ├── fusion.rs             — Kalman Filter (odometry + gyro + magnetometer)
│   │   └── types.rs              — Position2D, MovementDelta
│   │
│   └── trajectory/               — Path planning and motion profiling
│       ├── mod.rs                — Converts cell path → smooth velocity-profiled segments
│       ├── straight_line.rs      — Straight segment with trapezoidal velocity profile
│       └── in_place_turn.rs      — In-place rotation segment
│
├── docs/
│   └── timeline/                 — Project timeline web app (photos, videos, notes)
│
├── issues_and_knowledge/         — Notes on hardware quirks and bugs encountered
│   ├── architecture.md           — High-level architecture overview
│   ├── embassy_exti_missed_ticks_fix.md
│   ├── i2c_registers_and_pullups_explained.md
│   ├── monster moto shield ressources.md
│   └── vl53l1x_i2c_bsy_lockup_recovery.md
│
├── build.rs                      — Build script (linker script selection)
├── Cargo.toml                    — Dependencies and patch section for forked sensor crates
├── memory.x                      — Linker memory map for STM32F446RE
└── .cargo/config.toml            — Default target (thumbv7em-none-eabi), runner (probe-rs), log level
```
