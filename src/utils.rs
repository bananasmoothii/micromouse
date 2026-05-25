//! Utility traits and small async helpers used throughout the firmware.

use core::cell::Cell;
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};
use embassy_futures::select::Either3;
use embassy_stm32::time::Hertz;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_time::{Duration, Instant, Timer};

/// Extension trait on `u64` for convenient [`embassy_time::Duration`] and [`embassy_time::Timer`] construction.
pub trait DurationUtils {
    fn us(self) -> Duration;
    fn ms(self) -> Duration;
    fn s(self) -> Duration;
    fn ns(self) -> Duration;

    fn us_timer(self) -> Timer;
    fn ms_timer(self) -> Timer;
    fn s_timer(self) -> Timer;
    fn ns_timer(self) -> Timer;
}

impl DurationUtils for u64 {
    #[inline]
    fn ns(self) -> Duration {
        Duration::from_nanos(self)
    }

    #[inline]
    fn us(self) -> Duration {
        Duration::from_micros(self)
    }

    #[inline]
    fn ms(self) -> Duration {
        Duration::from_millis(self)
    }

    #[inline]
    fn s(self) -> Duration {
        Duration::from_secs(self)
    }

    #[inline]
    fn ns_timer(self) -> Timer {
        Timer::after(self.ns())
    }

    #[inline]
    fn us_timer(self) -> Timer {
        Timer::after(self.us())
    }

    #[inline]
    fn ms_timer(self) -> Timer {
        Timer::after(self.ms())
    }

    #[inline]
    fn s_timer(self) -> Timer {
        Timer::after(self.s())
    }
}

/// Extension trait on `u32` for convenient [`embassy_stm32::time::Hertz`] construction.
pub trait HertzUtils {
    fn hz(self) -> Hertz;
    fn khz(self) -> Hertz;
    fn mhz(self) -> Hertz;
}

impl HertzUtils for u32 {
    #[inline]
    fn hz(self) -> Hertz {
        Hertz::hz(self)
    }

    #[inline]
    fn khz(self) -> Hertz {
        Hertz::khz(self)
    }

    #[inline]
    fn mhz(self) -> Hertz {
        Hertz::mhz(self)
    }
}

/// Ergonomic `.get()` / `.set()` accessors for a `Mutex<_, Cell<T>>`.
pub trait CellMutexUtils<T> {
    /// Reads the current value (takes a brief critical section).
    fn get(&self) -> T;
    /// Writes a new value (takes a brief critical section).
    fn set(&self, value: T);
}

impl<T: Copy, RM: RawMutex> CellMutexUtils<T> for Mutex<RM, Cell<T>> {
    #[inline]
    fn get(&self) -> T {
        self.lock(Cell::get)
    }

    #[inline]
    fn set(&self, value: T) {
        self.lock(|cell| cell.set(value));
    }
}

/// Small floating-point helpers.
pub trait MathUtils {
    fn square(&self) -> Self;
}

impl MathUtils for f32 {
    #[inline]
    fn square(&self) -> f32 {
        self * self
    }
}

/// `select3` with rotating priority to avoid always starving B and C when A is frequently ready.
///
/// Pass a `priority` counter (0–2) that you increment each loop iteration; this future starts
/// polling from that index so each future gets a turn at the front.
pub struct FairSelect3<A, B, C> {
    pub a: A,
    pub b: B,
    pub c: C,
    /// Index (0=A, 1=B, 2=C) of the future that gets polled first.
    pub priority: u8,
}

/// Same as Select3 but uses round-robin
impl<A, B, C> FairSelect3<A, B, C> {
    pub fn new(priority: u8, a: A, b: B, c: C) -> Self {
        Self { a, b, c, priority }
    }
}

impl<A: Future, B: Future, C: Future> Future for FairSelect3<A, B, C> {
    type Output = Either3<A::Output, B::Output, C::Output>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let this = unsafe { self.get_unchecked_mut() };
        for i in 0..3u8 {
            match (this.priority + i) % 3 {
                0 => {
                    let a = unsafe { Pin::new_unchecked(&mut this.a) };
                    if let Poll::Ready(x) = a.poll(cx) {
                        return Poll::Ready(Either3::First(x));
                    }
                }
                1 => {
                    let b = unsafe { Pin::new_unchecked(&mut this.b) };
                    if let Poll::Ready(x) = b.poll(cx) {
                        return Poll::Ready(Either3::Second(x));
                    }
                }
                _ => {
                    let c = unsafe { Pin::new_unchecked(&mut this.c) };
                    if let Poll::Ready(x) = c.poll(cx) {
                        return Poll::Ready(Either3::Third(x));
                    }
                }
            }
        }
        Poll::Pending
    }
}

/// A value together with the [`Instant`] at which it was produced.
///
/// Use [`WithInstant::is_newer_than`] to discard stale sensor readings before acting on them.
#[derive(Clone)]
pub struct WithInstant<T> {
    pub data: T,
    pub at: Instant,
}

impl<T> WithInstant<T> {
    /// Wraps `data`, stamping it with the current instant.
    pub fn new(data: T) -> Self {
        Self {
            data,
            at: Instant::now(),
        }
    }

    /// Returns `true` if less than `duration` has elapsed since this value was stamped.
    pub fn is_newer_than(&self, duration: Duration) -> bool {
        Instant::now()
            .checked_duration_since(self.at)
            .map(|d| d < duration)
            .unwrap_or(false)
    }
}
