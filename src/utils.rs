use core::cell::Cell;
use embassy_stm32::time::Hertz;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_time::{Duration, Instant, Timer};

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

pub trait CellMutexUtils<T> {
    fn get(&self) -> T;
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

pub trait MathUtils {
    fn square(&self) -> Self;
}

impl MathUtils for f32 {
    #[inline]
    fn square(&self) -> f32 {
        self * self
    }
}

#[derive(Clone)]
pub struct WithInstant<T> {
    pub data: T,
    pub at: Instant,
}

impl<T> WithInstant<T> {
    pub fn new(data: T) -> Self {
        Self {
            data,
            at: Instant::now(),
        }
    }

    pub fn is_newer_than(&self, duration: Duration) -> bool {
        Instant::now()
            .checked_duration_since(self.at)
            .map(|d| d < duration)
            .unwrap_or(false)
    }
}
