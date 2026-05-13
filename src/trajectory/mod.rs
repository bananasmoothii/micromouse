pub mod straight_line;

use embassy_stm32::peripherals::{TIM1, TIM2, TIM3};
use crate::devices::motors::Motor;

const UPDATE_INTERVAL_MS: u64 = 20;


pub trait TrajectorySegment {
    async fn execute<'a>(&self, motor_left: &mut Motor<'a, TIM3>, motor_right: &mut Motor<'a, TIM2>, override_start_speed: Option<f32>);
}