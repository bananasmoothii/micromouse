use crate::devices::motors::{LEFT_FORWARD, RIGHT_FORWARD};
use core::sync::atomic::{AtomicI32, AtomicU32, Ordering};
use Ordering::Relaxed;
use embassy_stm32::peripherals;
use embassy_stm32::{pac, Peri};
use embassy_stm32::pac::interrupt;
use embassy_time::Instant;

pub static LEFT_TICKS_TOTAL: AtomicI32 = AtomicI32::new(0);
pub static RIGHT_TICKS_TOTAL: AtomicI32 = AtomicI32::new(0);

/// Cumulative tick counters — never drained, use for delta measurements over a maneuver.
pub static LEFT_TICKS_CUMULATIVE: AtomicI32 = AtomicI32::new(0);
pub static RIGHT_TICKS_CUMULATIVE: AtomicI32 = AtomicI32::new(0);

pub static LEFT_LAST_TICK_US: AtomicU32 = AtomicU32::new(0);
pub static RIGHT_LAST_TICK_US: AtomicU32 = AtomicU32::new(0);

pub static LEFT_TICK_INTERVAL_US: AtomicU32 = AtomicU32::new(0);
pub static RIGHT_TICK_INTERVAL_US: AtomicU32 = AtomicU32::new(0);

/// Initialize hall sensors as raw EXTI interrupts on PC2 (left) and PC3 (right).
///
/// Uses falling-edge EXTI interrupts directly instead of Embassy's async ExtiInput, which
/// disarms the interrupt after each edge and re-arms when the async task resumes — causing
/// missed ticks whenever a pulse arrives during that window.
///
/// Call once after embassy_stm32::init(). Parameters are consumed to prevent reuse.
pub fn init(
    _left_pin: Peri<'static, peripherals::PC2>,
    _left_exti: Peri<'static, peripherals::EXTI2>,
    _right_pin: Peri<'static, peripherals::PC3>,
    _right_exti: Peri<'static, peripherals::EXTI3>,
) {
    unsafe {
        // Enable GPIOC and SYSCFG clocks (safe to set even if already enabled)
        pac::RCC.ahb1enr().modify(|w| w.set_gpiocen(true));
        pac::RCC.apb2enr().modify(|w| w.set_syscfgen(true));

        // Configure PC2 and PC3 as floating inputs (breakout board has pull-up)
        pac::GPIOC.moder().modify(|w| {
            w.set_moder(2, pac::gpio::vals::Moder::INPUT);
            w.set_moder(3, pac::gpio::vals::Moder::INPUT);
        });
        pac::GPIOC.pupdr().modify(|w| {
            w.set_pupdr(2, pac::gpio::vals::Pupdr::FLOATING);
            w.set_pupdr(3, pac::gpio::vals::Pupdr::FLOATING);
        });

        // Route port C to EXTI2 and EXTI3 (EXTICR1 = index 0, covers EXTI[3:0])
        pac::SYSCFG.exticr(0).modify(|w| {
            w.set_exti(2, 2); // 2 = port C
            w.set_exti(3, 2);
        });

        // Falling edge only (sensor output goes LOW when magnet is detected)
        pac::EXTI.ftsr(0).modify(|w| {
            w.set_line(2, true);
            w.set_line(3, true);
        });
        pac::EXTI.rtsr(0).modify(|w| {
            w.set_line(2, false);
            w.set_line(3, false);
        });

        // Clear any stale pending bits before enabling
        pac::EXTI.pr(0).write(|w| {
            w.set_line(2, true);
            w.set_line(3, true);
        });

        // Unmask EXTI lines
        pac::EXTI.imr(0).modify(|w| {
            w.set_line(2, true);
            w.set_line(3, true);
        });

        // Enable in NVIC at priority 0x80 (mid-level)
        let mut cp = cortex_m::peripheral::Peripherals::steal();
        cp.NVIC.set_priority(pac::Interrupt::EXTI2, 0x80);
        cp.NVIC.set_priority(pac::Interrupt::EXTI3, 0x80);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::EXTI2);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::EXTI3);
    }
}

#[cortex_m_rt::interrupt]
unsafe fn EXTI2() {
    pac::EXTI.pr(0).write(|w| w.set_line(2, true)); // clear pending — must be first
    let now_us = Instant::now().as_micros() as u32;
    let delta: i32 = if LEFT_FORWARD.load(Relaxed) { 1 } else { -1 };
    LEFT_TICKS_TOTAL.fetch_add(delta, Relaxed);
    LEFT_TICKS_CUMULATIVE.fetch_add(delta, Relaxed);
    let prev = LEFT_LAST_TICK_US.swap(now_us, Relaxed);
    if prev != 0 {
        LEFT_TICK_INTERVAL_US.store(now_us.wrapping_sub(prev), Relaxed);
    }
}

#[cortex_m_rt::interrupt]
unsafe fn EXTI3() {
    pac::EXTI.pr(0).write(|w| w.set_line(3, true)); // clear pending — must be first
    let now_us = Instant::now().as_micros() as u32;
    let delta: i32 = if RIGHT_FORWARD.load(Relaxed) { 1 } else { -1 };
    RIGHT_TICKS_TOTAL.fetch_add(delta, Relaxed);
    RIGHT_TICKS_CUMULATIVE.fetch_add(delta, Relaxed);
    let prev = RIGHT_LAST_TICK_US.swap(now_us, Relaxed);
    if prev != 0 {
        RIGHT_TICK_INTERVAL_US.store(now_us.wrapping_sub(prev), Relaxed);
    }
}
