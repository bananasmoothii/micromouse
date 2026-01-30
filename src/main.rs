#![no_std]
#![no_main]

mod vec_extension;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::{bind_interrupts, interrupt};
use heapless::Vec;
use {defmt_rtt as _, panic_probe as _};
use crate::vec_extension::VecExt;
// #[global_allocator]
// static HEAP: Heap = Heap::empty();

bind_interrupts!(
    pub struct Irqs {
        EXTI15_10 => exti::InterruptHandler<interrupt::typelevel::EXTI15_10>;
    }
);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World!");

    let mut button = ExtiInput::new(p.PC13, p.EXTI13, Pull::None, Irqs);
    let mut led = Output::new(p.PA5, Level::Low, Speed::Medium);

    info!("Press the USER button...");

    let mut toggle_led = || {
        led.toggle();
    };


    let mut button_actions: Vec<&mut dyn FnMut(), 8> = Vec::new();
    button_actions.push_or_panic(&mut toggle_led);


    loop {
        button.wait_for_any_edge().await;
        for action in button_actions.iter_mut() {
            action()
        }
    }
}