# The ACTUAL Fix: Interrupt Busy-Loop

## The Real Problem

Your trace showed the smoking gun:

```
[TRACE] I2C: enabling interrupts
[TRACE] I2C interrupt triggered  <-- Fires IMMEDIATELY!
[TRACE] Done enabling interrupts
[TRACE] I2C: enabling interrupts  <-- Enables again immediately!
[TRACE] I2C interrupt triggered  <-- Fires again!
```

This is a **busy-loop** caused by interrupts firing spuriously on every poll.

## Root Cause

The previous fix enabled interrupts **BEFORE** checking the condition. Here's what was happening:

1. `poll_fn` is called
2. **Enable interrupts** (even if condition is already met!)
3. Check condition
4. If pending, return `Poll::Pending`
5. **Interrupt fires immediately** (from pending TXE, BTF, or other flags)
6. Waker wakes the task
7. Executor polls again → back to step 1
8. **Infinite busy-loop!**

The interrupt was firing because:

- STM32 I2C has many event flags (TXE, RXNE, BTF, ADDR, etc.)
- Enabling `ITEVTEN` triggers interrupt if **any** of these flags is set
- We were enabling interrupts even when we weren't waiting for anything
- This created immediate spurious wakeups

## The Correct Pattern

The fix is to **only enable interrupts when actually waiting**:

```rust
poll_fn( | cx| {
self.state.waker.register(cx.waker());

// CHECK FIRST
match check_condition() {
Ready => Poll::Ready(Ok(())),
NotReady => {
// ONLY enable interrupts when we're about to wait
Self::enable_interrupts( self.info);
Poll::Pending
}
}
})
```

This ensures:

1. If condition is already met, return immediately (no interrupt enable)
2. Only if we need to wait, enable interrupts
3. Interrupt fires when the event we're waiting for occurs
4. No spurious wakeups from unrelated flags

## Why The Previous Fix Was Wrong

I initially tried to fix a "race condition" by enabling interrupts before checking:

```rust
// WRONG: Enables interrupts on every poll!
Self::enable_interrupts( self .info);
match check_condition() { ... }
```

This actually **created** the busy-loop because:

- Every poll would enable interrupts
- If any unrelated flag was set (TXE, BTF, etc.), interrupt fires
- Task wakes and polls again
- Repeat forever

The correct approach:

- Check condition first
- Only enable interrupts if condition is not met
- Interrupt only fires for the event we're waiting for

## Changes Made

Fixed **all** `poll_fn` instances in the file (14 total):

### Master Mode

1. ✅ `write_frame()` START polling
2. ✅ `write_frame()` START retry polling
3. ✅ `write_frame()` address ACK polling
4. ✅ `write_frame()` DMA error polling
5. ✅ `write_frame()` BTF polling
6. ✅ `read_frame()` START polling
7. ✅ `read_frame()` START retry polling
8. ✅ `read_frame()` address ACK polling
9. ✅ `read_frame()` DMA error polling

### Slave Mode

10. ✅ `listen()` address match polling
11. ✅ `execute_slave_receive_transfer()` DMA/event polling
12. ✅ `execute_slave_transmit_transfer()` DMA/event polling
13. ✅ `handle_excess_bytes()` RXNE polling
14. ✅ `handle_padding_bytes()` TXE polling

## Expected Behavior Now

With this fix:

### Before (Busy-Loop):

```
[TRACE] I2C: enabling interrupts
[TRACE] I2C interrupt triggered  <-- Spurious!
[TRACE] Done enabling interrupts
[TRACE] I2C: enabling interrupts  <-- Again!
[TRACE] I2C interrupt triggered  <-- Spurious!
(repeats forever, task never yields)
```

### After (Correct):

```
[TRACE] I2C: enabling interrupts
(waits for actual event...)
[TRACE] I2C interrupt triggered  <-- Real event!
(proceeds with transaction)
```

## Why This Fixes Your Issue

Your sensor initialization was hanging because:

1. I2C transaction starts
2. Busy-loop begins on spurious interrupts
3. Task never actually completes the transaction
4. Never progresses to next step
5. Eventually times out

With the fix:

1. I2C transaction starts
2. Polls once, condition not met
3. Enables interrupts and yields
4. Real event occurs, interrupt fires
5. Transaction completes
6. Progresses normally

## Testing

Rebuild and run:

```powershell
cd C:\Users\bezar\Programmation\Rust\micromouse
cargo build --release
```

You should see:

- ✅ No busy-loop traces
- ✅ Normal I2C transaction flow
- ✅ Sensor initializes successfully
- ✅ Continuous measurements work

## Technical Note

This is a classic async polling anti-pattern:

**❌ Wrong:**

```rust
poll_fn( | cx| {
enable_notifications();  // Always enables!
if ready { Poll::Ready } else { Poll::Pending }
})
```

**✅ Correct:**

```rust
poll_fn( | cx| {
if ready {
Poll::Ready
} else {
enable_notifications();  // Only when waiting
Poll::Pending
}
})
```

The rule: **Only set up notifications (enable interrupts, register wakers, etc.) when actually returning `Poll::Pending`
**.

## Summary

- **Problem**: Enabling interrupts before checking condition caused busy-loop
- **Symptom**: Interrupts fire immediately on every poll, task never yields
- **Fix**: Only enable interrupts when returning `Poll::Pending`
- **Result**: Proper async behavior, transactions complete normally

This should finally fix your I2C issues for good! 🎉
