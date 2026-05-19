pub mod straight_line;
pub mod in_place_turn;

use crate::Box;
use embassy_stm32::peripherals::{TIM2, TIM3};
use crate::devices::motors::Motor;

pub const UPDATE_INTERVAL_MS: u64 = 20;

#[async_trait::async_trait]
pub trait TrajectorySegment {
    async fn execute<'a>(&self, motor_left: &mut Motor<'a, TIM3>, motor_right: &mut Motor<'a, TIM2>);
}
