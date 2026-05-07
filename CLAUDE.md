# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Embedded firmware for an autonomous **Micromouse** competition robot, targeting the **STM32F446RE** (Cortex-M4F, 180
MHz). The robot solves a 16×16 maze using ToF distance sensors, a 9-DOF IMU, and wheel odometry, with an Embassy-based
async runtime.

## Build & Flash

```sh
# Build (release is the default target for embedded)
cargo build --release

# Flash & run via probe-rs (STLink over SWD)
cargo run --release

# Check without building binary
cargo check
```

There are no tests — this is a bare-metal firmware project.

**Toolchain requirements**: `rustup target add thumbv7em-none-eabi`, `probe-rs` installed and on PATH.

The `.cargo/config.toml` sets the default target to `thumbv7em-none-eabi` and the runner to
`probe-rs run --chip STM32F446RE`. `DEFMT_LOG=trace` is set there too, enabling all log levels.

RTT log output is written to `./logs` and displayed in the terminal. GDB server starts on `127.0.0.1:1337`.

## Architecture

```
src/
  main.rs              — Embassy entry point; heap init, peripheral setup, task spawning
  dimensions.rs        — Physical constants (cell size 0.18 m, wall thickness 0.01 m)
  i2c_devices.rs       — Sequential XSHUT-based I2C init for the 3 ToF sensors
  devices/             — Hardware drivers
    motors.rs          — DC motor PI feedback loop, reads from PATH_CHANNEL
    hall_sensor_3144.rs— Interrupt-driven wheel encoders (AtomicI32 tick counters)
    vl53lxx/           — ToF distance sensors (VL53L0X + VL53L1X), I2C, 66 ms budget
    mpu9250.rs         — 9-DOF IMU over SPI (accel, gyro, mag)
    battery.rs         — 2-cell LiPo voltage monitoring via ADC
    buzzer.rs          — PWM audio (TIM2), command channel-driven
  positioning/
    mod.rs             — positioning_task: 20 ms sensor-fusion loop
    odometry.rs        — Skid-steering wheel kinematics (r=0.02 m, L=0.078 m, 12 ticks/rev); wheels slip laterally in curves so odometry heading is less reliable than gyro during turns
    mpu.rs             — Gyro → yaw, magnetometer → absolute heading
    fusion.rs          — Extended Kalman Filter (odometry + gyro + mag)
    types.rs           — Position2D, MovementDelta
  trajectory/
    mod.rs             — Converts cell path → smooth, velocity-profiled segments
                         (SmoothCornerOptimizer, VelocityProfileOptimizer)
  labyrinth/
    mod.rs             — 16×16 maze grid, probabilistic wall detection, pathfinding
```

### Data flow

1. **Sensors** (hall sensors, ToF, MPU9250) feed raw measurements via Embassy tasks.
2. **`positioning_task`** runs every 20 ms: fuses odometry + IMU through an EKF → `Position2D`.
3. **`labyrinth`** accumulates wall hits/misses from ToF readings and finds the shortest path to the exit (10, 10).
4. **`trajectory`** converts the discrete cell path into smooth arc+straight segments with trapezoidal velocity
   profiles (max 0.8 m/s, max accel 1.0 m/s², corner radius 0.05 m, corner speed 0.2 m/s).
5. **`motors`** tracks trajectory waypoints via `PATH_CHANNEL` using a PI loop.

### Concurrency model

- **Embassy async tasks** for all subsystems (sensor reading, motor control, buzzer).
- **`AtomicI32`** for lock-free hall-sensor tick counters shared across interrupt and task contexts.
- **`embassy_sync::Channel`** for trajectory waypoints (`PATH_CHANNEL`, buffered FIFO).
- **`Mutex<CriticalSectionRawMutex, _>`** for fusion filter output shared between positioning and motor tasks.

### I2C bus init

Three ToF sensors share one I2C bus. `i2c_devices.rs` holds all XSHUT pins low, then brings them up one by one assigning
addresses `0x30`, `0x31`, `0x32` at 200 kHz with DMA (TX DMA1_CH6, RX DMA1_CH0).

### Heap

`embedded-alloc` is used. Heap size is computed at runtime:
`size_of(VL53L0X) + size_of(VL53L1X) + size_of(MPU9250) + 10 KB`.

### Sensor driver forks

The sensor crates (`vl53l1`, `vl53l0x`, `mpu9250`) are patched git forks under the `bananasmoothii` GitHub account with
custom fixes (e.g., runtime I2C address changing for the VL53L1X).
