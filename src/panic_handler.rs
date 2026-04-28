use defmt::{error, info};

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    cortex_m::interrupt::disable();
    error!("{}", defmt::Display2Format(info));
    // GPIOC BSRR: writing 1 to bit 20 (16+4) resets PC4 (EN pin → motor driver off)
    info!("Setting PC4 (enable pin) low");
    unsafe {
        core::ptr::write_volatile(0x4002_0818 as *mut u32, 1 << 20);
    }
    cortex_m::asm::udf()
}

#[defmt::panic_handler]
fn defmt_panic() -> ! {
    cortex_m::asm::udf()
}