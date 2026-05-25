//! Hall-effect wheel encoder driver (A3144 sensors on PC2 / PC3).
//!
//! ## Design rationale
//! Embassy's `ExtiInput::wait_for_rising_edge` disarms the interrupt after each edge and
//! re-arms it only when the async task polls again.  At high wheel speed (>2 m/s) the time
//! between edges can be shorter than one Embassy executor round-trip, causing missed ticks.
//! This module bypasses Embassy and installs raw Cortex-M EXTI ISRs that run with no
//! async overhead.
//!
//! ## Retriggerable debounce
//! The A3144 output chatters on magnet edges — typically 3–8 bounces at 5–30 µs intervals
//! over a ~340 µs window.  A fixed dead-time after accepting a tick would miss a second real
//! tick that arrives before the dead-time expires.  Instead this driver uses *retriggerable*
//! debounce: every edge (accepted or rejected) resets the 200 µs dead zone.  The first edge
//! of each chatter burst is counted; all subsequent bounces keep extending the dead zone
//! until the burst settles naturally.
//!
//! ## Tick sign
//! The ISR reads [`LEFT_FORWARD`] / [`RIGHT_FORWARD`] to determine the sign of the tick
//! increment (`+1` forward, `−1` reverse).  These flags are written by
//! [`crate::devices::motors::Motor::set_pwm`] whenever the direction changes.
//!
//! ## Cumulative vs. total counters
//! - `LEFT_TICKS_TOTAL` / `RIGHT_TICKS_TOTAL`: drained by [`crate::positioning::odometry`]
//!   every 20 ms (swapped to 0 after reading).
//! - `LEFT_TICKS_CUMULATIVE` / `RIGHT_TICKS_CUMULATIVE`: never reset; used by
//!   `straight_line.rs` as absolute references for distance-based stop conditions.

use crate::devices::motors::{LEFT_FORWARD, RIGHT_FORWARD};
use core::sync::atomic::{AtomicI32, AtomicU32, Ordering};
use Ordering::Relaxed;
use cortex_m::peripheral::DWT;
use embassy_stm32::peripherals;
use embassy_stm32::{pac, Peri};
use embassy_stm32::pac::interrupt;

pub static LEFT_TICKS_TOTAL: AtomicI32 = AtomicI32::new(0);
pub static RIGHT_TICKS_TOTAL: AtomicI32 = AtomicI32::new(0);

/// Cumulative tick counters — never drained, use for delta measurements over a maneuver.
pub static LEFT_TICKS_CUMULATIVE: AtomicI32 = AtomicI32::new(0);
pub static RIGHT_TICKS_CUMULATIVE: AtomicI32 = AtomicI32::new(0);

/// DWT cycle count of the most recent rising edge (accepted OR rejected) per wheel.
/// Updated on every edge so the retriggerable debounce extends through chatter bursts.
/// Stored as `cycles + 1` so a value of 0 unambiguously means "no edge yet".
pub static LEFT_LAST_TICK_CYCLES: AtomicU32 = AtomicU32::new(0);
pub static RIGHT_LAST_TICK_CYCLES: AtomicU32 = AtomicU32::new(0);

/// Interval (cycles) between the two most recent accepted rising edges per wheel.
/// Used by fusion.rs as a "wheel is alive" presence check (`> 0`).
pub static LEFT_TICK_INTERVAL_CYCLES: AtomicU32 = AtomicU32::new(0);
pub static RIGHT_TICK_INTERVAL_CYCLES: AtomicU32 = AtomicU32::new(0);

/// CPU cycles per µs. Must match SYSCLK configured in `main.rs` (84 MHz HSI×PLL → 84 cycles/µs).
/// **Not currently read** — `MIN_TICK_INTERVAL_CYCLES` is set directly in raw cycles.
/// Exposed `pub` so a caller can recompute `MIN_TICK_INTERVAL_CYCLES` without digging into this file.
pub const CYCLES_PER_US: u32 = 84;

/// Minimum interval between two accepted rising edges, in DWT cycles.
/// Retriggerable: any edge — accepted or rejected — resets the dead zone, so a
/// continuous chatter burst lasting many times this duration still produces
/// only one tick (the first edge of the burst).
///
/// 200 µs sits well above the longest chatter burst we've observed (~340 µs of
/// jittering LOW segments, with individual bounces at 5–30 µs intervals) — so
/// the *first* edge of a real magnet passage is accepted, and every subsequent
/// bounce keeps resetting the dead zone until the burst settles. Real tick
/// periods are several ms at typical speeds, far above 200 µs.
const MIN_TICK_INTERVAL_CYCLES: u32 = 10_000;

/// DIAGNOSTIC: when true, emit a `defmt::trace!` per rising edge with the
/// interval since the previous edge ("L ok 1234" or "L rj 56"). Pipe RTT to
/// `tools/plot_pulse_histogram.py` to characterize chatter vs real ticks.
/// Safe-ish to leave on (RTT writes are O(few bytes), no flash buffering),
/// but keep it OFF for normal driving — at high RPM the trace rate is heavy.
const TRACE_EDGES: bool = false;

/// Initialize hall sensors as raw EXTI interrupts on PC2 (left) and PC3 (right).
///
/// Uses rising-edge EXTI interrupts directly instead of Embassy's async ExtiInput, which
/// disarms the interrupt after each edge and re-arms when the async task resumes — causing
/// missed ticks whenever a pulse arrives during that window.
///
/// Only the rising edge fires the ISR (magnet entering the sensor's active zone — sensor
/// goes HIGH). Chatter is filtered by retriggerable interval debouncing.
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

        // Rising edge only (magnet entering — sensor goes HIGH).
        pac::EXTI.ftsr(0).modify(|w| {
            w.set_line(2, false);
            w.set_line(3, false);
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

        // Enable DWT cycle counter for sub-µs ISR timestamps.
        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();
    }
}

#[cortex_m_rt::interrupt]
unsafe fn EXTI2() {
    pac::EXTI.pr(0).write(|w| w.set_line(2, true));
    // +1 so a stored value of 0 unambiguously means "no edge yet".
    let now = DWT::cycle_count().wrapping_add(1);
    // Swap unconditionally — every edge resets the dead zone (retriggerable debounce).
    let prev = LEFT_LAST_TICK_CYCLES.swap(now, Relaxed);
    let dt = if prev == 0 { u32::MAX } else { now.wrapping_sub(prev) };
    if dt < MIN_TICK_INTERVAL_CYCLES {
        if TRACE_EDGES { defmt::trace!("L rj {}", dt); }
        return;
    }
    if prev != 0 {
        LEFT_TICK_INTERVAL_CYCLES.store(dt, Relaxed);
    }
    if TRACE_EDGES { defmt::trace!("L ok {}", dt); }
    let delta: i32 = if LEFT_FORWARD.load(Relaxed) { 1 } else { -1 };
    LEFT_TICKS_TOTAL.fetch_add(delta, Relaxed);
    LEFT_TICKS_CUMULATIVE.fetch_add(delta, Relaxed);
}

#[cortex_m_rt::interrupt]
unsafe fn EXTI3() {
    pac::EXTI.pr(0).write(|w| w.set_line(3, true));
    let now = DWT::cycle_count().wrapping_add(1);
    let prev = RIGHT_LAST_TICK_CYCLES.swap(now, Relaxed);
    let dt = if prev == 0 { u32::MAX } else { now.wrapping_sub(prev) };
    if dt < MIN_TICK_INTERVAL_CYCLES {
        if TRACE_EDGES { defmt::trace!("R rj {}", dt); }
        return;
    }
    if prev != 0 {
        RIGHT_TICK_INTERVAL_CYCLES.store(dt, Relaxed);
    }
    if TRACE_EDGES { defmt::trace!("R ok {}", dt); }
    let delta: i32 = if RIGHT_FORWARD.load(Relaxed) { 1 } else { -1 };
    RIGHT_TICKS_TOTAL.fetch_add(delta, Relaxed);
    RIGHT_TICKS_CUMULATIVE.fetch_add(delta, Relaxed);
}
