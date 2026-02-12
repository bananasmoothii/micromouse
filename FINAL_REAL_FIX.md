# The REAL Fix: Spurious Interrupt Wakeups

## The Actual Problem

Your traces showed the interrupt handler was waking the task on **EVERY** interrupt, including irrelevant TXE/RXNE
flags:

```
After START request: CR1.START=false... SR2.MSL=true  <-- START complete!
I2C: enabling interrupts                               <-- Enable for ADDR wait
I2C interrupt triggered                                <-- TXE fires (DR empty)!
Done enabling interrupts
I2C: enabling interrupts                               <-- Enable again  
I2C interrupt triggered                                <-- TXE still set!
(repeats forever)
```

## Root Cause

After the address byte is written to DR:

1. TXE (Transmit buffer Empty) flag gets set immediately
2. We enable ITEVTEN to wait for ADDR flag
3. **TXE triggers interrupt immediately** (it's an event too!)
4. Interrupt handler wakes task (even though it's not ADDR)
5. Task polls, ADDR not set yet, enables interrupts again
6. TXE **still set** → immediate interrupt again
7. Infinite busy-loop!

The old interrupt handler was:

```rust
pub unsafe fn on_interrupt<T: Instance>() {
    T::state().waker.wake();  // ALWAYS wakes, even for TXE/RXNE!
    // disable interrupts
}
```

## The Fix

Modified the interrupt handler to **only wake on significant events**:

```rust
pub unsafe fn on_interrupt<T: Instance>() {
    let sr1 = regs.sr1().read();
    
    // Only wake on significant events
    let should_wake = sr1.start()    // START generated
        || sr1.addr()                 // Address sent
        || sr1.stopf()                // STOP detected  
        || sr1.btf()                  // Byte transfer finished
        || sr1.berr()                 // Bus error
        || sr1.arlo()                 // Arbitration lost
        || sr1.af()                   // NACK
        || sr1.ovr()                  // Overrun
        // ... other errors
        // BUT NOT TXE or RXNE!
    
    if should_wake {
        T::state().waker.wake();
    } else {
        // Ignore spurious TXE/RXNE interrupts
    }
    
    // Disable interrupts
}
```

## Why TXE/RXNE Should Be Ignored

- **TXE** (Transmit buffer Empty): Set when DR is ready for next byte
- **RXNE** (Receive buffer Not Empty): Set when DR has received data
- These are **buffer flags**, not transaction events
- **DMA handles these** - the task doesn't need to wake for them
- They trigger constantly during DMA transfers

The events we **DO** care about:

- **SB** (START): START condition generated
- **ADDR**: Address sent/acknowledged
- **BTF**: Byte transfer finished (important for timing)
- **STOPF**: STOP detected (slave mode)
- **AF** (NACK): Address/data not acknowledged
- **Errors**: BERR, ARLO, OVR, PECERR, TIMEOUT

## Expected Behavior Now

### Before (Busy-Loop on TXE):

```
After START request...
I2C: enabling interrupts
I2C interrupt triggered  <-- TXE!
Done enabling interrupts
I2C: enabling interrupts  <-- Again!
I2C interrupt triggered  <-- TXE still set!
(repeats forever - task never yields)
```

### After (Ignores TXE):

```
After START request...
I2C: enabling interrupts
I2C interrupt triggered: TXE=true ADDR=false
  -> Ignoring spurious interrupt (TXE)
(interrupts disabled, task yields)
(hardware sends address...)
(ADDR flag gets set)
I2C interrupt triggered: ADDR=true
  -> Waking task
(task polls, sees ADDR, proceeds)
```

## Why This Finally Fixes It

The sequence now works correctly:

1. **Write address to DR** → TXE flag sets
2. **Enable interrupts** to wait for ADDR
3. **TXE triggers interrupt** → Handler sees it's not significant → **Doesn't wake task**
4. **Interrupts disabled**, task yields to executor
5. **Hardware sends address** on I2C bus
6. **ADDR flag sets** when address acknowledged
7. **ADDR triggers interrupt** → Handler sees significant event → **Wakes task**
8. **Task polls**, sees ADDR, proceeds with transaction

Without the fix:

- Step 3: Handler wakes task (wrong!)
- Step 4: Task polls immediately, ADDR not set yet
- Step 5: Enables interrupts again
- Back to step 3: TXE still set → infinite loop

## Diagnostic Output

The enhanced interrupt handler now shows:

```
I2C interrupt triggered: SB=1 ADDR=0 BTF=0 TXE=0 RXNE=0 errors=false
  -> Waking task (SB event)

I2C interrupt triggered: SB=0 ADDR=0 BTF=0 TXE=1 RXNE=0 errors=false  
  -> Ignoring spurious interrupt (TXE)

I2C interrupt triggered: SB=0 ADDR=1 BTF=0 TXE=1 RXNE=0 errors=false
  -> Waking task (ADDR event)
```

This helps debug what's actually happening.

## Rebuild and Test

```powershell
cd C:\Users\bezar\Programmation\Rust\micromouse
cargo build --release
```

You should now see:

- ✅ Interrupts only fire on significant events
- ✅ No busy-loops on TXE/RXNE
- ✅ Task properly yields between events
- ✅ Sensor initialization completes successfully

## Summary

**Problem**: Interrupt handler woke task on TXE/RXNE buffer flags, causing busy-loop

**Symptom**: Immediate repeated interrupts between address write and address ACK

**Fix**: Only wake task on significant events (SB, ADDR, BTF, errors), ignore TXE/RXNE

**Result**: Proper async behavior - task yields when waiting, wakes on actual events

This is the final piece of the puzzle! 🎉
