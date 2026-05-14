# Embassy async EXTI causes missed encoder ticks

## Problem

Using Embassy's `ExtiInput::wait_for_falling_edge()` in a loop to count hall sensor ticks
causes roughly 40% of ticks to be silently dropped at moderate wheel speeds (~0.5 m/s).

### Root cause

Embassy's async EXTI design **disarms the interrupt after each edge** and only re-arms it
when the async task calls `wait_for_falling_edge()` again. The sequence is:

```
edge fires → ISR clears + disarms interrupt → async task wakes → task does work → task re-arms
```

Any edge that arrives during the "task does work" window is invisible to the hardware — the
EXTI line is masked. At 0.5 m/s with 12 ticks/rev and 2 cm radius, ticks arrive every ~11 ms.
The Embassy async wake + re-arm cycle is long enough to consistently miss the next pulse.

This is by design in Embassy: it prevents spurious re-fires when a single edge wakes multiple
waiters. It is the correct behavior for button debouncing or one-shot events, but wrong for
continuous high-frequency counting.

## Fix

Replace the async Embassy task with a raw `#[cortex_m_rt::interrupt]` handler that is
**always armed**. The EXTI hardware fires the ISR directly on every edge without any
software re-arming step.

### What changed

`src/devices/hall_sensor_3144.rs` was rewritten from:

```rust
// OLD: async task, misses ~40% of ticks
#[embassy_executor::task]
async fn hall_sensor_continuous_measuring(mut pin: ExtiInput<'static>, side: WheelSide) -> ! {
    loop {
        pin.wait_for_falling_edge().await;  // disarms EXTI, re-arms after await resumes
        // ... count tick
    }
}
```

To raw interrupt handlers registered via an `init()` function:

```rust
// NEW: always-armed ISR, never misses a tick
pub fn init(_left_pin: Peri<'static, PC2>, _left_exti: Peri<'static, EXTI2>, ...) {
    unsafe {
        // configure GPIO, SYSCFG, EXTI edge select, unmask lines, enable NVIC
    }
}

#[cortex_m_rt::interrupt]
unsafe fn EXTI2() {
    pac::EXTI.pr(0).write(|w| w.set_line(2, true)); // clear pending first
    // ... count tick atomically
}
```

`src/main.rs` was updated to:

- Remove `EXTI2` and `EXTI3` from `bind_interrupts!` (they are no longer Embassy-managed)
- Replace the two `spawner.spawn(hall_sensor_continuous_measuring(...))` calls with a single
  `hall_sensor_3144::init(p.PC2, p.EXTI2, p.PC3, p.EXTI3);`

### PAC API quirks encountered (STM32F4 / stm32-metapac)

The EXTI register accessors in stm32-metapac for STM32F4 require an index argument and use
`set_line` (not the named-field setters used on other families):

```rust
// WRONG — compiles on some families, not STM32F4:
pac::EXTI.ftsr().modify( | w| w.set_tr(2, true));

// CORRECT for STM32F4:
pac::EXTI.ftsr(0).modify( | w| w.set_line(2, true));
```

The same pattern applies to `rtsr(0)`, `pr(0)`, and `imr(0)`.

### `#[cortex_m_rt::interrupt]` needs the PAC interrupt enum in scope

The `cortex_m_rt::interrupt` proc-macro expands to code that references a bare `interrupt`
module to look up the vector number by name. Without the import below, the build fails with
"use of unresolved module `interrupt`":

```rust
use embassy_stm32::pac::interrupt;  // required for #[cortex_m_rt::interrupt] to resolve names
```

This is not mentioned in the cortex-m-rt docs; it comes up specifically when using Embassy's
PAC re-export rather than a standalone device crate.
