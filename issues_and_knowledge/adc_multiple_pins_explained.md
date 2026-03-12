# STM32 ADC - Reading Multiple Analog Pins Explained

## Question: What does ADC1 refer to? How to read 6 analog pins?

### The Answer

**ADC1, ADC2, and ADC3 are ADC peripherals, NOT individual pins!**

Think of it like this:

- **ADC1** = The analog-to-digital converter **hardware module**
- **Channels/Pins** = The actual GPIO pins that carry analog signals (PA0, PA1, PB0, PC0, etc.)

### How It Works

Each ADC peripheral can be connected to **multiple analog input pins** through a multiplexer. You use **one ADC
peripheral** (like ADC1) to read from **many different pins** by switching between channels.

```
         ┌─────────────┐
  PA0 ───┤             │
  PA1 ───┤             │
  PA4 ───┤ Multiplexer ├──── ADC1 (12-bit converter) ───► Digital Value
  PB0 ───┤             │
  PC0 ───┤             │
  PC1 ───┤             │
         └─────────────┘
```

### STM32F446RE ADC Specifications

- **3 ADC peripherals**: ADC1, ADC2, ADC3
- **16 external channels** per ADC
- **12-bit resolution**: Values from 0 to 4095 (4096 steps)
- **Voltage range**: 0V to 3.3V (on your Nucleo board)
- **Many shared channels**: Most pins can be used by multiple ADCs

### ADC1 Pin Mapping (Common Pins)

Here are the most accessible analog pins on the STM32F446RE:

| Pin | ADC1 Channel | Arduino Header | Notes               |
|-----|--------------|----------------|---------------------|
| PA0 | IN0          | A0             | User button (EXTI0) |
| PA1 | IN1          | A1             | Available           |
| PA4 | IN4          | A2             | Available           |
| PB0 | IN8          | A3             | Available           |
| PC0 | IN10         | A5             | Available           |
| PC1 | IN11         | A4             | Available           |
| PA5 | IN5          | D13            | LED (may conflict)  |
| PA6 | IN6          | D12            | Available           |
| PA7 | IN7          | D11            | Available           |
| PB1 | IN9          | -              | Available           |
| PC2 | IN12         | -              | Available           |
| PC3 | IN13         | -              | Available           |
| PC4 | IN14         | -              | Available           |
| PC5 | IN15         | -              | Available           |

### Code Example: Reading 6 Pins with ADC1

```rust
use embassy_stm32::adc::{Adc, SampleTime};

// Create the ADC1 peripheral
let mut adc = Adc::new(p.ADC1);

// Create references to the analog pins (these are just GPIO pins in analog mode)
let mut adc_pin1 = p.PA0;  // ADC1_IN0
let mut adc_pin2 = p.PA1;  // ADC1_IN1
let mut adc_pin3 = p.PA4;  // ADC1_IN4
let mut adc_pin4 = p.PB0;  // ADC1_IN8
let mut adc_pin5 = p.PC0;  // ADC1_IN10
let mut adc_pin6 = p.PC1;  // ADC1_IN11

// Read from each pin sequentially
// SampleTime::CYCLES112 is recommended for most sensors (accurate and fast)
let value1 = adc.blocking_read( & mut adc_pin1, SampleTime::CYCLES112);  // Returns 0-4095
let value2 = adc.blocking_read( & mut adc_pin2, SampleTime::CYCLES112);
let value3 = adc.blocking_read( & mut adc_pin3, SampleTime::CYCLES112);
let value4 = adc.blocking_read( & mut adc_pin4, SampleTime::CYCLES112);
let value5 = adc.blocking_read( & mut adc_pin5, SampleTime::CYCLES112);
let value6 = adc.blocking_read( & mut adc_pin6, SampleTime::CYCLES112);
```

**Note:** The `SampleTime` parameter determines how long the ADC waits to charge its internal capacitor before
measuring. `CYCLES112` (~1.3 µs) is a good default for most sensors and circuits. See `adc_sample_time_explained.md` for
details on choosing the right sample time.

### Converting to Voltage

The ADC returns a raw value from 0 to 4095. To convert to voltage:

```rust
// For STM32F446RE with 3.3V reference voltage
fn adc_to_voltage(raw_value: u16) -> f32 {
    (raw_value as f32 / 4095.0) * 3.3
}

let voltage = adc_to_voltage(value1);
info!("Pin PA0 voltage: {:.3}V", voltage);
```

### When to Use Multiple ADC Peripherals?

You typically only need ADC2 or ADC3 if:

1. **Simultaneous sampling**: You need to read multiple pins at the **exact same time**
    - ADC1, ADC2, and ADC3 can run simultaneously in dual/triple mode
    - Useful for synchronized measurements (e.g., 3-phase motor control)

2. **Higher throughput**: You need to sample many pins very quickly
    - Using multiple ADCs in parallel increases overall sampling rate

3. **DMA channels**: Each ADC can use a different DMA channel to avoid conflicts

For most applications (including a micromouse), **reading 6 pins sequentially with ADC1 is perfectly fine**!

### Blocking vs Async Reading

Embassy provides two ways to read ADC values:

```rust
use embassy_stm32::adc::SampleTime;

// Blocking read (waits until conversion is done)
let value = adc.blocking_read( & mut adc_pin1, SampleTime::CYCLES112);

// Async read (doesn't block the executor)
let value = adc.read( & mut adc_pin1, SampleTime::CYCLES112).await;
```

Use async reads if you're reading in a task and don't want to block other tasks.

**About SampleTime:** This parameter controls the ADC sampling duration. Use `CYCLES112` as a safe default for most
applications - it provides good accuracy (works with source impedances up to ~10 kΩ) while still being very fast (~1.3
µs per read). See `adc_sample_time_explained.md` for more details.

### Example: Continuous Reading Task

```rust
#[embassy_executor::task]
async fn adc_reading_task(mut adc: Adc<'static, ADC1>, mut pins: [SomePin; 6]) -> ! {
    loop {
        for (i, pin) in pins.iter_mut().enumerate() {
            let value = adc.read(pin).await;
            let voltage = (value as f32 / 4095.0) * 3.3;
            info!("Pin {} voltage: {:.3}V", i, voltage);
        }
        embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;
    }
}
```

### Important Notes

⚠️ **Pin Conflicts**: Make sure the pins you choose aren't already used for other purposes:

- PA0: Used for EXTI0 in your I2C device code (commented out)
- PA1: Used for EXTI1 in your I2C device code (commented out)
- PA5: Connected to the onboard LED

✅ **Safe pins for analog reading** (not used in your current code):

- PA4, PA6, PA7
- PB0, PB1
- PC0, PC1, PC2, PC3, PC4, PC5

### Summary

- **ADC1 is a peripheral, not a pin** - it can read from many pins
- **You can read 6 pins** (or more) using just ADC1
- **Each pin is called a "channel"** (e.g., PA0 = Channel 0)
- **Reading is sequential** - the ADC switches between channels
- **Use ADC2/ADC3** only if you need simultaneous sampling or higher throughput
- **Raw values** range from 0 to 4095 (12-bit resolution)
- **Convert to voltage**: `voltage = (raw / 4095.0) * 3.3`





