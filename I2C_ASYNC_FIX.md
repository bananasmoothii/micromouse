# I2C Async Bug Fix - STOP Condition Not Completing

## The Real Problem (Finally!)

Your trace revealed the true issue:

```
After START request: CR1.START=false, CR1.PE=true, SR2.BUSY=true, SR2.MSL=true  <-- WORKS
...
After START request: CR1.START=true, CR1.PE=true, SR2.BUSY=true, SR2.MSL=false  <-- FAILS
```

And crucially: **Blocking I2C works fine, but async I2C fails after 2-3 transactions.**

## Root Cause

After completing an async I2C transaction that sends a STOP condition, the code was returning immediately without
waiting for the STOP to actually complete on the bus.

The STM32 I2C peripheral:

1. Software sets CR1.STOP=1
2. Hardware generates STOP condition on the bus (takes ~10µs)
3. Hardware clears CR1.STOP=0 when done

The async code was doing step 1, then **immediately returning**, allowing the next transaction to start while STOP was
still being generated. This left the bus in a BUSY state, preventing the next START from working properly.

## Why Blocking I2C Worked

Blocking I2C code doesn't have this issue because:

- Blocking transactions are synchronous - each completes before the next begins
- The delay between transactions is long enough for STOP to complete naturally
- No tight async task scheduling causing rapid back-to-back transactions

## The Fix

Added `yield_now().await` loops after setting STOP bit to wait for hardware completion:

```rust
if frame.send_stop() {
self.info.regs.cr1().modify( | w | {
w.set_stop(true);
});

// Wait for STOP condition to be cleared by hardware
while self.info.regs.cr1().read().stop() {
// STOP bit is cleared by hardware when complete
yield_now().await;  // <-- KEY FIX
}
}
```

This ensures:

- STOP completes before function returns
- Bus is fully released
- Next transaction sees clean bus state
- No BUSY lockup between transactions

## Changes Made

### 1. Added `yield_now` import

```rust
use embassy_futures::yield_now;
```

### 2. Fixed `write_frame()` STOP wait

- Waits for CR1.STOP to be cleared by hardware
- Uses `yield_now()` to avoid blocking executor

### 3. Fixed `read_frame()` STOP wait (two locations)

- Single-byte read case (early STOP)
- Multi-byte read case (STOP after DMA)

## Why This Happened

The async I2C implementation was ported from blocking code but missed this critical detail:

- Blocking code: Natural delays between transactions hide the issue
- Async code: Tasks can be scheduled back-to-back, exposing the race
- The interrupt-driven async model allows the next transaction to start microseconds after the previous one

## Expected Behavior Now

With the fix:

1. First transaction: START → DATA → **STOP (waits for complete)** → Return
2. **Bus is now IDLE**
3. Second transaction: START (succeeds) → DATA → **STOP (waits)** → Return
4. Third transaction: START (succeeds) → ...

No more "Bus is BUSY before START" warnings!

## Testing

Rebuild and run:

```powershell
cd C:\Users\bezar\Programmation\Rust\micromouse
cargo build --release
```

You should see:

- No BUSY warnings
- No software resets
- All I2C transactions succeed
- Sensor initializes properly

## Technical Details

### The Busy Loop Issue

Your trace also showed:

```
I2C: enabling interrupts
I2C interrupt triggered  <-- Fires immediately!
```

This was a **symptom** of the STOP issue:

- Bus still BUSY from incomplete STOP
- Peripheral has pending flags (TXE, BTF, etc.)
- Enabling interrupts triggers immediate interrupt
- Task wakes prematurely
- Creates busy-loop appearance

The STOP fix resolves this by ensuring clean bus state.

### Why yield_now()?

Instead of busy-waiting (blocking the executor), we use `yield_now()`:

- Allows other tasks to run while waiting
- Typical STOP completion: 5-10 microseconds
- Usually exits loop after 1-2 yields
- Much better for async cooperative multitasking

## Summary

**Problem:** Async I2C transactions returned before STOP completed, causing next transaction to find bus BUSY

**Fix:** Wait for CR1.STOP bit to clear (hardware completion signal) using `yield_now()` loops

**Result:** Bus is properly released between transactions, no more BUSY lockups

This was a classic async programming bug - missing synchronization point that only manifests under specific timing
conditions (rapid back-to-back transactions).
