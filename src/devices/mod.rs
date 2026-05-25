//! Hardware driver modules.
//!
//! Each sub-module wraps one physical component or subsystem:
//!
//! | Module | Component | Notes |
//! |---|---|---|
//! | [`battery`] | 2S LiPo monitor | ADC voltage divider, 4-LED bar, overcurrent-based shutdown |
//! | [`buzzer`] | PWM buzzer on TIM1 CH4 | Channel-driven; play notes via [`buzzer::BUZZER_CHANNEL`] |
//! | [`hall_sensor_3144`] | Wheel encoders (Hall A3144) | Raw EXTI ISRs on PC2/PC3, retriggerable debounce |
//! | [`motors`] | DC motors via H-bridge | Battery-compensated PWM, overcurrent protection task |
//! | [`mpu9250`] | MPU-9250 IMU (SPI) | Accel + gyro + magnetometer; 2000 DPS gyro scale |
//! | [`vl53lxx`] | VL53L0X / VL53L1X ToF | 3 × VL53L1X on shared I2C bus, EMA filtering, recovery |

pub mod vl53lxx;
pub mod mpu9250;
pub mod battery;
pub mod hall_sensor_3144;
pub mod buzzer;
pub mod motors;
