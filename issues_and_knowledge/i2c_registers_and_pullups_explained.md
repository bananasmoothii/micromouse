# STM32 I2C Registers & Pull-up Resistors Explained

## Question 1: What are the registers?

Yes, these are **hardware registers inside the STM32F446RE microcontroller**. They control the I2C peripheral hardware.

### The I2C Registers in the STM32F446RE

Your chip uses **I2C version 1** (for STM32F4 family). The main registers involved are:

#### **CR1 (Control Register 1)**

- Memory address: Base + 0x00
- Controls: START/STOP generation, peripheral enable (PE), software reset (SWRST)
- Example from the code:
  ```rust
  self.info.regs.cr1().modify(|reg| {
      reg.set_start(true);  // Tell hardware: "Generate a START condition"
  });
  ```

#### **CR2 (Control Register 2)**

- Memory address: Base + 0x04
- Controls: Interrupts, DMA, peripheral clock frequency
- Sets the APB1 bus frequency the I2C peripheral runs on

#### **SR1 (Status Register 1)**

- Memory address: Base + 0x14
- **This is what times out!** Contains status bits like:
    - `SB` (START bit) - Set when START condition is generated
    - `ADDR` - Address sent
    - `BTF` - Byte transfer finished
    - `AF` - Acknowledge failure (NACK)
    - `ARLO` - Arbitration lost
    - `BERR` - Bus error
    - `TIMEOUT` - Timeout error

#### **SR2 (Status Register 2)**

- Memory address: Base + 0x18
- Contains: Master/Slave mode, bus busy status
- Reading SR2 clears certain flags in SR1

#### **DR (Data Register)**

- Memory address: Base + 0x10
- Holds the byte to send or received byte

### What's Happening in Your Timeout

```rust
// Line 167: Software sets START bit in CR1
self .info.regs.cr1().modify( | reg| {
reg.set_start(true);  // Write to CR1 register
});

// Line 171: Wait for hardware to set SB bit in SR1
while ! Self::check_and_clear_error_flags( self .info) ?.start() {
timeout.check() ?;  // Line 172: TIMEOUT! SB never appeared in SR1
}
```

**The problem:** CR1.START tells hardware "please generate START", but SR1.SB never confirms it happened.

### Register Memory Map (STM32F446RE)

```
I2C1 Base Address: 0x4000_5400
├─ CR1    @ 0x4000_5400
├─ CR2    @ 0x4000_5404
├─ OAR1   @ 0x4000_5408
├─ OAR2   @ 0x4000_540C
├─ DR     @ 0x4000_5410
├─ SR1    @ 0x4000_5414  ← This is what we're checking!
├─ SR2    @ 0x4000_5418
├─ CCR    @ 0x4000_541C  (Clock control)
└─ TRISE  @ 0x4000_5420  (Rise time)
```

---

## Question 2: Pull-up Resistors - Do You Need External Ones?

### Short Answer: **YES, you MUST add external pull-up resistors!** ⚠️

The STM32F446RE **does NOT have strong enough internal pull-ups for I2C**.

### Why External Pull-ups Are Required

#### What the STM32 datasheet says:

- STM32F446RE has **weak internal pull-ups** (~30-50 kΩ)
- I2C requires **strong pull-ups** (1.5-10 kΩ depending on bus capacitance)
- The internal pull-ups are **NOT sufficient for reliable I2C operation**

#### I2C is an Open-Drain Protocol

```
       VDD (3.3V)
         │
         ├─ Rp (Pull-up resistor) ← YOU MUST PROVIDE THIS
         │
    ─────┴──── SDA/SCL line
         │
    ┌────┴────┐
    │ STM32   │  Can only pull LOW
    │  or     │  Cannot drive HIGH
    │ Sensor  │  (open-drain)
    └─────────┘
         │
        GND
```

When no device is pulling the line LOW, the pull-up resistor pulls it HIGH. Without proper pull-ups:

- Signal rise times are too slow
- Bus is unreliable
- **You get timeouts and communication failures!** ← Your exact problem!

---

## Question 3: Resistor Values and Placement

### Both SDA AND SCL Need Pull-ups

You need one resistor on **each** line:

- One on **SDA** (PB9)
- One on **SCL** (PB8)

### Recommended Values for Your Setup

#### Standard Calculation

The I2C specification says:

```
Rise time must be < 1000 ns (for 100 kHz)
Rise time = 0.8473 × Rp × Cbus

Where:
- Rp = pull-up resistance
- Cbus = total bus capacitance
```

#### For Your Setup (200 kHz I2C):

- **Typical starting point: 4.7 kΩ**
- **Range: 2.2 kΩ to 10 kΩ**

#### Specific Recommendations:

| Bus Speed | Typical Capacitance | Recommended Resistor |
|-----------|---------------------|----------------------|
| 100 kHz   | < 200 pF            | 4.7 kΩ               |
| 100 kHz   | 200-400 pF          | 2.2 kΩ               |
| 400 kHz   | < 100 pF            | 2.2 kΩ               |
| 400 kHz   | 100-200 pF          | 1.0 kΩ               |

**For your 200 kHz with 2 sensors (moderate capacitance):**

- **Start with 4.7 kΩ** (most common, very reliable)
- If you have signal integrity issues, try **2.2 kΩ**

### What Happens with Different Values

| Resistance              | Effect                                  |
|-------------------------|-----------------------------------------|
| Too high (>10 kΩ)       | Slow rise times → timeouts, errors      |
| Just right (2.2-4.7 kΩ) | Clean signals, reliable operation       |
| Too low (<1 kΩ)         | High current draw, possible GPIO damage |

---

## Nucleo Board Considerations

### Does the Nucleo-F446RE have I2C pull-ups?

**NO** - The Nucleo-F446RE board does **NOT** have pull-up resistors on the Arduino headers (where PB8/PB9 are).

#### What the Nucleo board has:

- ✅ Internal weak pull-ups (~40 kΩ) - **NOT sufficient for I2C**
- ❌ No external pull-up resistors on Arduino connector pins
- ❌ No I2C pull-ups on morpho connector pins

Some sensor breakout boards (like Adafruit, SparkFun) **do** include pull-ups, but you need to verify this for your
specific VL53L0X and VL53L1X boards.

---

## How to Add Pull-up Resistors

### Physical Wiring

```
STM32 Nucleo-F446RE          VL53L0X/VL53L1X Sensors
                             
3.3V ──┬─────────┬──────────── VDD (sensor 1)
       │         │
     [4.7k]   [4.7k] ← ADD THESE RESISTORS
       │         │
PB8 ───┴─────────┴──────────── SCL (all sensors)
       │         
     [4.7k]   [4.7k] ← ADD THESE RESISTORS
       │         │
PB9 ───┴─────────┴──────────── SDA (all sensors)

GND ────────────────────────── GND (all sensors)
```

### Where to Place Them

**Option 1: On a breadboard** (easiest for testing)

```
3.3V rail ──[4.7kΩ]── SDA line
         └──[4.7kΩ]── SCL line
```

**Option 2: Direct soldering**

- Solder between 3.3V pin and SDA wire
- Solder between 3.3V pin and SCL wire

**Option 3: If your sensor boards have built-in pull-ups**

- Check the sensor board schematic
- If they already have 4.7 kΩ or similar, you're good!
- Many VL53L0X breakout boards include them

---

## Important Notes

### 1. Only ONE set of pull-ups for the whole bus

Even with multiple sensors, you only need one pair of pull-ups total (not per sensor), because I2C is a bus where all
devices share the same SDA and SCL lines.

### 2. Check your sensor breakout boards

Run a multimeter in resistance mode:

- Measure between VDD and SDA pins on sensor board (power off)
- Measure between VDD and SCL pins on sensor board (power off)
- If you read 2-10 kΩ, the board has pull-ups already! ✅
- If you read >100 kΩ or open circuit, you need external pull-ups ⚠️

### 3. Lower I2C speed first

Before buying resistors, try reducing to 100 kHz (as I showed earlier). If that alone fixes it, the problem might be
signal integrity at higher speeds, and proper pull-ups will help.

---

## Action Items for You

1. **Immediate (no hardware needed):**
   ```rust
   // In i2c_devices.rs line 32, change to:
   i2c_config.frequency = Hertz::khz(100);
   ```

2. **Check your sensor boards:**
    - Use multimeter to check if VL53L0X/VL53L1X breakouts have pull-ups
    - Look at board photos/schematics

3. **If no pull-ups found, add them:**
    - Get two 4.7 kΩ resistors (1/4 watt is fine)
    - Connect one between 3.3V and SDA (PB9)
    - Connect one between 3.3V and SCL (PB8)

4. **Test and adjust:**
    - Start with 4.7 kΩ
    - If still unreliable, try 2.2 kΩ
    - Use oscilloscope if available to check signal quality

---

## Summary

- **Registers:** Yes, they're hardware registers in the STM32F446RE chip that control the I2C peripheral
- **Pull-ups:** You MUST have external pull-up resistors (STM32 internal ones are too weak)
- **Where:** Both SDA **and** SCL need pull-ups
- **Value:** 4.7 kΩ is the standard starting point for 100-400 kHz I2C
- **Nucleo board:** Does NOT include I2C pull-ups on Arduino headers
- **Check sensors:** Your VL53L0X/VL53L1X breakout boards might already have them!
