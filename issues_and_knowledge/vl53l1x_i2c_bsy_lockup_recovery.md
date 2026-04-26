# VL53L1X: I2C BSY Lockup and Recovery

## Symptom

All three VL53L1X sensors read successfully for a short time, then `I2c(Timeout)` errors appear
permanently every ~50 ms (the configured I2C timeout). Once the first timeout fires, all subsequent
reads fail — even after a recovery attempt.

---

## Root-Cause Chain

### 1. Missing pull-up on GPIO1 (interrupt pin)

The VL53L1X signals measurement-complete by driving **GPIO1 LOW** (open-drain). It releases the pin
(high-impedance) after the interrupt is cleared by the host. Without a pull-up resistor or
`Pull::Up` on the MCU pin, the line never returns HIGH after `clear_interrupt()`. The sensor
interprets "interrupt not yet acknowledged" and stalls — it will not start the next autonomous
measurement until GPIO1 goes HIGH.

**Fix:** `ExtiInput::new(p.PC5, p.EXTI5, Pull::Up, Irqs)` for every sensor interrupt pin.

After this fix the failures moved from ~2 s to ~100 s of runtime (occasional glitches rather than
a guaranteed stall).

### 2. STM32 I2C v1 BSY lockup with no built-in recovery

When a read times out mid-transaction (the sensor is holding SDA low for a data bit), the
embassy-stm32 I2C driver returns `Error::Timeout` and **leaves the peripheral in its stuck state**.
`SR2.BUSY = 1`, and every subsequent attempt to generate a START condition also times out
immediately.

The embassy-stm32 I2C v1 driver has a documented gap (comment in `v1.rs`):
> "It COULD be possible to apply this workaround at runtime, however this would require detecting
> the timeout or BUSY lockup condition, and re-configuring the peripheral after reset."

The only hardware mechanism to force-clear BSY is **SWRST** in `I2C_CR1`.

---

## What Does Not Work (Failed Attempts)

### SWRST + `PE=1` only

```rust
pac::I2C1.cr1().modify( | w| w.set_swrst(true));
pac::I2C1.cr1().modify( | w| w.set_swrst(false));
pac::I2C1.cr1().modify( | w| w.set_pe(true));  // ← WRONG: config is gone
```

**Why it fails:** `SWRST` resets **all** I2C registers including `CCR` (clock control), `TRISE`
(rise time), and `CR2.FREQ` (peripheral clock). Re-enabling with `PE=1` leaves these at 0, so the
peripheral has no valid clock configuration. Every transaction immediately fails again — first with
`Timeout`, then with `Arbitration` as the peripheral gets into further bad states.

### GPIO 9-clock bit-bang with Embassy `Timer` delays

```rust
Timer::after(Duration::from_micros(5)).await;  // ← ZERO actual delay
```

**Why it fails:** The Embassy time driver on this board runs at 1 kHz (1 ms per tick).
`Duration::from_micros(5)` rounds to **0 ticks**. `Timer::after(0 ticks).await` yields once to
the scheduler and returns almost immediately. The "9 clock pulses" execute at CPU speed (~180 MHz),
producing nanosecond-wide pulses the sensor cannot detect, so SDA is never released.

---

## Correct Recovery Sequence

### Step 1 — Sensor side: drive XSHUT LOW on all sensors

When `XSHUT` is held LOW the VL53L1X enters hardware standby and releases its I2C pins. This frees
SDA from the sensor side. Per the STM32 reference manual, the bus must be free **before** SWRST is
cleared.

### Step 2 — MCU side: SWRST with register save/restore

```rust
fn i2c_swrst_recovery() {
    use embassy_stm32::pac;

    // Save timing registers before SWRST zeros them
    let cr2_freq = pac::I2C1.cr2().read().freq();
    let ccr_r = pac::I2C1.ccr().read();
    let ccr_fs = ccr_r.f_s();
    let ccr_duty = ccr_r.duty();
    let ccr_val = ccr_r.ccr();
    let trise = pac::I2C1.trise().read().trise();

    // SWRST forces BSY=0 and releases SCL/SDA from the peripheral side
    pac::I2C1.cr1().modify(|w| w.set_swrst(true));
    cortex_m::asm::delay(900_000); // ~5 ms at 180 MHz (blocking spin-wait, not Embassy timer)
    pac::I2C1.cr1().modify(|w| w.set_swrst(false));

    // CCR and TRISE must be written while PE=0 — SWRST leaves PE=0
    pac::I2C1.cr2().modify(|w| w.set_freq(cr2_freq));
    pac::I2C1.ccr().write(|w| {
        w.set_f_s(ccr_fs);
        w.set_duty(ccr_duty);
        w.set_ccr(ccr_val);
    });
    pac::I2C1.trise().write(|w| w.set_trise(trise));
    pac::I2C1.cr1().modify(|w| w.set_pe(true));
}
```

`cortex_m::asm::delay` is a genuine busy-wait in CPU cycles, immune to the Embassy tick-rate
limitation.

### Step 3 — Re-initialize sensors sequentially

Each sensor is brought out of reset (`XSHUT` HIGH), runs `data_init`, gets its I2C address
reassigned, and restarts autonomous measurement.

---

## Full Recovery Flow (in `distance_sensor_task`)

```
I2c error detected
  ├─ All XSHUT pins LOW        (sensors release SDA)
  ├─ i2c_swrst_recovery()      (save CCR/TRISE → SWRST → restore → PE=1)
  ├─ 10 ms wait                (sensors complete internal reset)
  ├─ recover_sensor(right)     (XSHUT HIGH + full reinit at 0x30)
  ├─ recover_sensor(middle)    (XSHUT HIGH + full reinit at 0x31)
  └─ recover_sensor(left)      (XSHUT HIGH + full reinit at 0x32)
```

If any `recover_sensor` call fails the task waits 500 ms and retries from the top of the loop.

---

## Why `cortex_m::asm::delay` Instead of `Timer::after`

Embassy's async timer is backed by a 1 kHz tick source (1 ms per tick). Any duration shorter than
1 ms rounds down to **0 ticks**, so `Timer::after(Duration::from_micros(5)).await` returns
immediately without waiting. This silently broke the earlier 9-clock bit-bang attempt.

`cortex_m::asm::delay(N)` burns exactly N CPU cycles regardless of the tick rate. At 180 MHz:
`900_000` cycles ≈ 5 ms. It also works inside plain `fn` (non-async), and prevents the scheduler
from yielding between `SWRST=1` and `SWRST=0` — which matters because the peripheral is in a
partial reset state during that window.

---

## Key Lessons

| Fact                                              | Detail                                                                            |
|---------------------------------------------------|-----------------------------------------------------------------------------------|
| `SWRST` resets `CCR`, `TRISE`, `CR2.FREQ`         | Must save and restore before/after                                                |
| Embassy timer resolution is 1 ms                  | `from_micros(N)` rounds to 0 ticks — use `cortex_m::asm::delay` for sub-ms waits  |
| XSHUT must go LOW before SWRST is cleared         | STM32 RM0390 requirement: bus must be free                                        |
| `Pull::Up` on GPIO1 is mandatory                  | Open-drain interrupt pin — without pull-up, sensor stalls after first measurement |
| embassy-stm32 I2C v1 has no built-in bus recovery | Must implement SWRST sequence manually via PAC                                    |
