pub mod straight_line;
pub mod in_place_turn;

use alloc::boxed::Box;
use core::sync::atomic::AtomicU8;
use core::sync::atomic::Ordering::Relaxed;
use defmt::Format;
use embassy_stm32::peripherals::{TIM2, TIM3};
use crate::devices::motors::Motor;

pub const UPDATE_INTERVAL_MS: u64 = 20;

#[repr(u8)]
#[derive(Clone, Copy, Format)]
pub enum FusionMode {
    Idle = 0,
    Straight = 1,
    Pivot = 2,
}

impl From<u8> for FusionMode {
    fn from(value: u8) -> Self {
        match value {
            1 => FusionMode::Straight,
            2 => FusionMode::Pivot,
            _ => FusionMode::Idle,
        }
    }
}

static FUSION_MODE: AtomicU8 = AtomicU8::new(0);

pub fn get_current_fusion_mode() -> FusionMode {
    FusionMode::from(FUSION_MODE.load(Relaxed))
}

pub fn set_current_fusion_mode(mode: FusionMode) {
    FUSION_MODE.store(mode as u8, Relaxed);
}

#[async_trait::async_trait]
pub trait TrajectorySegment {
    fn fusion_mode(&self) -> FusionMode;

    async fn execute<'a>(&self, motor_left: &mut Motor<'a, TIM3>, motor_right: &mut Motor<'a, TIM2>);
}
