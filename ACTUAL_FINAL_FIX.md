# The ACTUAL Final Fix: ADDR Flag Not Cleared

## The Real Problem (This Time For Real!)

Your trace showed `ADDR=true` on BOTH interrupts:

```
I2C: enabling interrupts
I2C interrupt triggered: ADDR=true BTF=false TXE=true ...
  -> Waking task
Done enabling interrupts
I2C: enabling interrupts                               <-- Enable again!
I2C interrupt triggered: ADDR=true BTF=false TXE=false ... <-- ADDR STILL SET!
  -> Waking task
```

The ADDR flag was **never being cleared**, so it kept triggering interrupts!

## Why ADDR Wasn't Cleared

The STM32 I2C requires a specific sequence to clear ADDR:

1. Read SR1 (Status Register 1)
2. Read SR2 (Status Register 2)

The code flow was:

1. ADDR flag sets → interrupt fires
2. Interrupt handler reads SR1, wakes task, **disables interrupts**
3. Task polls: `check_and_clear_error_flags` reads SR1 again (step 1 ✓)
4. Sees ADDR=true, returns `Poll::Ready`
5. poll_fn completes
6. Line 611: `self.info.regs.sr2().read()` - **This should clear ADDR** (step 2 ✓)
7. **BUT**: Next transaction starts immediately
8. Enables interrupts
9. **ADDR still set** (why??) → Immediate interrupt!

The problem: The interrupt handler **already read SR1**, but never read SR2 to complete the clearing sequence. So even
though the main code reads SR2, the flag isn't cleared because the **interrupt handler's SR1 read** wasn't followed by
an SR2 read!

## The Fix

Clear ADDR flag **in the interrupt handler** immediately after reading SR1:

```rust
pub unsafe fn on_interrupt<T: Instance>() {
    let regs = T::info().regs;
    let sr1 = regs.sr1().read();  // Step 1: Read SR1
    
    // Clear ADDR flag by reading SR2 (completing the sequence)
    if sr1.addr() {
        let _ = regs.sr2().read();  // Step 2: Read SR2
        trace!("  -> Cleared ADDR flag");
    }
    
    // ... rest of handler
}
```

This ensures the ADDR flag is cleared immediately and won't trigger a second interrupt.

## Why This Was So Hard To Find

The sequence requirement is subtle:

- **Any** read of SR1 starts a "clear ADDR" sequence
- That read MUST be followed by a read of SR2
- If you read SR1 in the interrupt handler, you must also read SR2 there
- You can't read SR1 in the handler and SR2 in the main code - doesn't work!

The old flow split the sequence:

```
Interrupt Handler: Read SR1 (starts sequence)
Interrupt Handler: Disables interrupts
Main Code: Read SR1 again (starts NEW sequence)
Main Code: Read SR2 (completes second sequence, but first is abandoned)
Result: ADDR not actually cleared from the first read!
```

## Expected Behavior Now

```
I2C: enabling interrupts
I2C interrupt triggered: ADDR=true ...
  -> Cleared ADDR flag          <-- NEW!
  -> Waking task
Done enabling interrupts
(Next transaction starts)
I2C: enabling interrupts
(Waits for actual event, no immediate interrupt)
```

## Rebuild and Test

```powershell
cd C:\Users\bezar\Programmation\Rust\micromouse
cargo build --release
```

You should now see:

- ✅ "Cleared ADDR flag" trace after ADDR interrupt
- ✅ No repeated ADDR interrupts
- ✅ Transactions complete properly
- ✅ Sensor initializes successfully

## Summary

**Problem**: ADDR flag not cleared because SR1 read in interrupt handler wasn't followed by SR2 read

**Symptom**: ADDR=true on repeated interrupts, causing busy-loop

**Fix**: Clear ADDR in interrupt handler by reading SR2 immediately after reading SR1

**Result**: ADDR properly cleared, no repeated interrupts, normal operation

This HAS to be it! 🤞
