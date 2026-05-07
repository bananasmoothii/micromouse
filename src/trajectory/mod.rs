use embassy_stm32::peripherals::{TIM1, TIM3};
use crate::devices::motors::Motor;

pub trait Trajectory {
    async fn execute(&self, motor_left: &mut Motor<TIM3>, motor_right: &mut Motor<TIM1>);
}

pub struct StraightLine {
    distance: f32,
    in_speed: f32,
    out_speed: f32,
}

impl Trajectory for StraightLine {
    async fn execute(&self, motor_left: &mut Motor<'_, TIM3>, motor_right: &mut Motor<'_, TIM1>) {}
}