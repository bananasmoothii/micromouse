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

/// Time of the most recent falling edge (sensor went LOW = magnet entering).
/// Reset to 0 after the matching rising edge is processed.
static LEFT_FALLING_EDGE_US: AtomicU32 = AtomicU32::new(0);
static RIGHT_FALLING_EDGE_US: AtomicU32 = AtomicU32::new(0);

/// Minimum LOW duration to accept a detection as real.
/// Observed chatter peaks at ~150 µs (multiples of a ~61 µs timer granularity artifact).
/// Real pulses scale with speed: ~1556 µs at 1.1 m/s → ~856 µs at 2 m/s.
/// 300 µs gives 2× margin above chatter and ~2.8× below the 2 m/s floor.
const MIN_PULSE_WIDTH_US: u32 = 0;

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

        // Both edges: falling when magnet enters (sensor goes LOW), rising when it leaves
        pac::EXTI.ftsr(0).modify(|w| {
            w.set_line(2, true);
            w.set_line(3, true);
        });
        pac::EXTI.rtsr(0).modify(|w| {
            w.set_line(2, true);
            w.set_line(3, true);
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
    if pac::GPIOC.idr().read().idr(2) == pac::gpio::vals::Idr::HIGH {
        // Rising edge: magnet leaving — validate pulse width
        let fall_us = LEFT_FALLING_EDGE_US.load(Relaxed);
        if fall_us == 0 { return; }
        LEFT_FALLING_EDGE_US.store(0, Relaxed);
        let pulse_us = now_us.wrapping_sub(fall_us);
        if pulse_us < MIN_PULSE_WIDTH_US {
            // println!("L skip {}us", pulse_us);
            return;
        }
        // println!("L ok {}us", pulse_us);
        let delta: i32 = if LEFT_FORWARD.load(Relaxed) { 1 } else { -1 };
        LEFT_TICKS_TOTAL.fetch_add(delta, Relaxed);
        LEFT_TICKS_CUMULATIVE.fetch_add(delta, Relaxed);
        let prev = LEFT_LAST_TICK_US.swap(now_us, Relaxed);
        if prev != 0 {
            LEFT_TICK_INTERVAL_US.store(now_us.wrapping_sub(prev), Relaxed);
        }
    } else {
        // Falling edge: magnet detected — record time, await rising edge
        LEFT_FALLING_EDGE_US.store(now_us, Relaxed);
    }
}

#[cortex_m_rt::interrupt]
unsafe fn EXTI3() {
    pac::EXTI.pr(0).write(|w| w.set_line(3, true)); // clear pending — must be first
    let now_us = Instant::now().as_micros() as u32;
    if pac::GPIOC.idr().read().idr(3) == pac::gpio::vals::Idr::HIGH {
        // Rising edge: magnet leaving — validate pulse width
        let fall_us = RIGHT_FALLING_EDGE_US.load(Relaxed);
        if fall_us == 0 { return; }
        RIGHT_FALLING_EDGE_US.store(0, Relaxed);
        let pulse_us = now_us.wrapping_sub(fall_us);
        if pulse_us < MIN_PULSE_WIDTH_US {
            // println!("R skip {}us", pulse_us);
            return;
        }
        // println!("R ok {}us", pulse_us);
        let delta: i32 = if RIGHT_FORWARD.load(Relaxed) { 1 } else { -1 };
        RIGHT_TICKS_TOTAL.fetch_add(delta, Relaxed);
        RIGHT_TICKS_CUMULATIVE.fetch_add(delta, Relaxed);
        let prev = RIGHT_LAST_TICK_US.swap(now_us, Relaxed);
        if prev != 0 {
            RIGHT_TICK_INTERVAL_US.store(now_us.wrapping_sub(prev), Relaxed);
        }
    } else {
        // Falling edge: magnet detected — record time, await rising edge
        RIGHT_FALLING_EDGE_US.store(now_us, Relaxed);
    }
}
