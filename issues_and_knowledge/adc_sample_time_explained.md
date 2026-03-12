# ADC Sample Time - Why Choose CYCLES?

## Question: Why do I have to choose a cycles value?

### The Simple Answer

The **sample time** determines **how long the ADC waits** to accurately measure the voltage on a pin. It's a trade-off
between:

- ⚡ **Speed** (fewer cycles = faster reads)
- 🎯 **Accuracy** (more cycles = more accurate reads)
- 🔌 **Source impedance** (high resistance sources need more time)

### How ADC Sampling Works

When the ADC reads a pin, it doesn't instantly know the voltage. Here's what happens:

```
1. ADC connects to pin
2. Internal capacitor (C_ADC) starts charging from the pin
3. Waits for "sample time" (e.g., 112 cycles)
4. Disconnects and measures the capacitor voltage
5. Converts to digital value (0-4095)
```

```
Pin Voltage ────┐
                │    ┌─────────────  Fully charged ✓
                │   ╱
                │  ╱  
                │ ╱   CYCLES112 (enough time)
                │╱
                └────────────────────────> Time
                ↑                      ↑
              Start               Measure
```

If you don't wait long enough, the capacitor doesn't fully charge, and you get an **inaccurate reading**!

### Available Sample Times (STM32F446RE)

Embassy provides these options:

| Sample Time | CPU Cycles | Time @ 84MHz | When to Use                           |
|-------------|------------|--------------|---------------------------------------|
| `CYCLES3`   | 3          | 35.7 ns      | ⚠️ Very fast, very low impedance only |
| `CYCLES15`  | 15         | 178.6 ns     | Low impedance sources                 |
| `CYCLES28`  | 28         | 333.3 ns     | Medium impedance                      |
| `CYCLES56`  | 56         | 666.7 ns     | Good general purpose                  |
| `CYCLES84`  | 84         | 1.0 µs       | Safe default                          |
| `CYCLES112` | 112        | 1.33 µs      | ✅ Recommended default                 |
| `CYCLES144` | 144        | 1.71 µs      | High impedance sources                |
| `CYCLES480` | 480        | 5.71 µs      | Very high impedance (>50kΩ)           |

### How to Choose the Right Value?

#### Rule of Thumb Formula:

The sample time must be long enough to charge the ADC's internal capacitor through the source impedance:

```
t_sample ≥ 5 × (R_source + R_internal) × C_ADC
```

For STM32F446RE:

- **C_ADC** ≈ 5 pF (internal ADC capacitor)
- **R_internal** ≈ 1 kΩ (internal switch resistance)
- **R_source** = Your sensor/circuit resistance

#### Quick Guide by Source Impedance:

| Your Source Resistance          | Recommended Sample Time    |
|---------------------------------|----------------------------|
| < 1 kΩ (direct voltage divider) | `CYCLES56` - `CYCLES84`    |
| 1-10 kΩ (typical sensors)       | `CYCLES84` - `CYCLES112` ✅ |
| 10-50 kΩ (high resistance)      | `CYCLES112` - `CYCLES144`  |
| > 50 kΩ (very high resistance)  | `CYCLES480`                |

### For Your Micromouse:

**Typical use cases:**

1. **Battery voltage monitoring** (with voltage divider):
    - Usually low impedance (few kΩ)
    - ✅ Use `CYCLES84` or `CYCLES112`

2. **Analog sensors** (distance, light, etc.):
    - Medium impedance (1-10 kΩ)
    - ✅ Use `CYCLES112` (safe default)

3. **Potentiometers**:
    - Can be high impedance
    - ✅ Use `CYCLES112` or `CYCLES144`

### Impact on Performance

**Total ADC conversion time** = Sample Time + 12 cycles (for conversion)

| Sample Time | Total Time @ 84MHz | Max Sampling Rate |
|-------------|--------------------|-------------------|
| `CYCLES3`   | ~179 ns            | ~5.6 MHz          |
| `CYCLES112` | ~1.48 µs           | ~676 kHz          |
| `CYCLES480` | ~5.86 µs           | ~171 kHz          |

**For a micromouse reading 6 sensors:**

- With `CYCLES112`: 6 × 1.48 µs = **8.88 µs total**
- That's **112,000 complete reads per second** - way more than you need!

### Practical Example

```rust
use embassy_stm32::adc::{Adc, SampleTime};

let mut adc = Adc::new(p.ADC1);

// Reading battery voltage (low impedance voltage divider)
let battery = adc.blocking_read( & mut p.PC0, SampleTime::CYCLES84);

// Reading infrared sensor (medium impedance)
let ir_sensor = adc.blocking_read( & mut p.PC1, SampleTime::CYCLES112);

// Reading high-impedance source
let high_z = adc.blocking_read( & mut p.PC2, SampleTime::CYCLES480);
```

### What Happens If You Choose Wrong?

#### ❌ Too Short (e.g., CYCLES3 for high impedance):

```
Expected: 3.3V
Measured: 2.1V  ← Capacitor didn't fully charge!
Result: Inaccurate readings
```

#### ✅ Too Long (e.g., CYCLES480 for low impedance):

```
Expected: 3.3V
Measured: 3.3V  ✓ Accurate!
Result: Slower but still works fine
```

**Conclusion:** If in doubt, use a **longer sample time**. It's safer to be slow and accurate than fast and wrong!

### Recommended Default for Your Project

```rust
// Safe default for most sensors and circuits
let value = adc.blocking_read( & mut pin, SampleTime::CYCLES112);
```

This gives you:

- ✅ Good accuracy for most sources
- ✅ Still very fast (1.48 µs)
- ✅ Works with source impedances up to ~10 kΩ
- ✅ Safe for typical micromouse sensors

### When to Optimize

Only reduce sample time if:

1. You **know** your source impedance is very low (< 1 kΩ)
2. You need **maximum speed** (thousands of reads per second)
3. You've **tested** and verified accuracy is maintained

For a micromouse, `CYCLES112` is perfect - you'll never notice the ~1.5 µs read time, and your readings will be
accurate!

## Summary

- **Sample time** = How long ADC waits to charge its capacitor
- **Longer** = More accurate, especially for high-impedance sources
- **Shorter** = Faster, but needs low-impedance sources
- **Default recommendation**: `SampleTime::CYCLES112` for most uses
- **Your micromouse**: CYCLES112 is perfect - accurate and still super fast!


