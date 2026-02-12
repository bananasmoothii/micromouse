# Current Status: ADDR Fixed, Now Investigating DMA

## Progress

✅ **FIXED**: ADDR flag clearing - no more repeated ADDR interrupts
✅ **FIXED**: Spurious TXE/RXNE interrupts - handler now filters them
✅ **FIXED**: Interrupt enable/disable race conditions

## Current Issue

The code now gets stuck **after** address acknowledgment:

```
I2C: enabling interrupts
I2C interrupt triggered: ADDR=true ...
  -> Cleared ADDR flag  ✅
  -> Waking task
Done enabling interrupts
I2C: enabling interrupts  <-- This is the DMA error poll
Done enabling interrupts
(stuck here - waiting forever)
```

## What's Happening

After ADDR is cleared:

1. ✅ Code starts DMA transfer
2. ✅ Creates error polling future
3. ✅ `select()` waits for either DMA complete OR error
4. ❌ Neither future completes - task stuck

## Possible Causes

1. **DMA not starting**: The DMA peripheral might not be configured correctly
2. **DMA stuck**: DMA started but waiting for TXE events that aren't happening
3. **DMA complete but not waking**: DMA finished but the future isn't being notified

## Added Diagnostics

Added traces to see:

- When DMA transfer starts
- Which future (DMA or error) completes the select

## Next Steps

Rebuild and check the new traces to see:

```powershell
cd C:\Users\bezar\Programmation\Rust\micromouse
cargo build --release
```

Look for:

- `"Starting DMA transfer for X bytes"` - confirms DMA starts
- `"DMA/Error select completed: ..."` - shows which completes

If you don't see the DMA start trace, it means the code is stuck before that point (but the trace shows it should have
passed that).

If DMA starts but never completes, we have a DMA configuration or I2C peripheral issue.
