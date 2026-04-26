use embassy_stm32::time::Hertz;
use embassy_time::{Duration, Timer};

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