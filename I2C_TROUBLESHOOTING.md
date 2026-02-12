# I2C Troubleshooting Guide

## Current Issue: START Condition Never Completes

Your logs show:

```
After START request: CR1.START=true, CR1.PE=true, SR2.BUSY=true, SR2.MSL=true
```

This means:

- ✅ I2C peripheral is enabled (CR1.PE=true)
- ✅ START bit was set in control register (CR1.START=true)
- ✅ Master mode is active (SR2.MSL=true)
- ✅ Bus state shows activity started (SR2.BUSY=true)
- ❌ **BUT: The SB (Start Bit) flag in SR1 never gets set**

## Why Does This Happen?

The START condition requires the I2C peripheral to pull SDA low while SCL is high. If this cannot complete physically on
the hardware, the SB flag never sets.

### Common Causes (in order of likelihood):

### 1. **Missing or Weak Pull-up Resistors** (MOST COMMON)

**Symptoms:** Exactly what you're seeing - START requested but never completes.

**Solution:**

- Add 4.7kΩ pull-up resistors from SCL (PB8) to VCC (3.3V)
- Add 4.7kΩ pull-up resistors from SDA (PB9) to VCC (3.3V)
- Some breakout boards have built-in pull-ups, but they're often 10kΩ (too weak)
- If using a breadboard, verify resistor values and connections

**Test:** Use a multimeter to measure voltage on SCL and SDA when idle:

- Should read 3.3V on both lines
- If reading 0V or floating, pull-ups are missing
- If reading ~1-2V, pull-ups might be too weak

### 2. **Device Not Powered or Not Responding**

**Symptoms:** Same as above - bus appears stuck because no device ACKs.

**Solution:**

- Verify VL53L0X sensor has 3.3V on VDD pin
- Check GND connection
- Verify XSHUT pin is HIGH (3.3V) - sensor is disabled if XSHUT is LOW
- Wait at least 10ms after bringing XSHUT high before attempting I2C

**Your code:** The sensor init does this correctly:

```rust
config.xshut_pin.set_low();   // Disable
Timer::after(Duration::from_millis(10)).await;
config.xshut_pin.set_high();  // Enable
Timer::after(Duration::from_millis(10)).await;  // Boot time
```

### 3. **Wrong GPIO Configuration**

**Symptoms:** Lines don't behave correctly, may be stuck high or low.

**Solution:**

- I2C pins MUST be configured as open-drain (not push-pull)
- Embassy STM32 should handle this automatically when you use `I2c::new()`
- Verify you're not accidentally reconfiguring PB8/PB9 elsewhere as regular GPIO

### 4. **Another Device Holding the Bus**

**Symptoms:** SDA or SCL stuck low.

**Solution:**

- Disconnect all other I2C devices
- Try with only one sensor connected
- Power cycle everything

### 5. **Wiring Issues**

**Symptoms:** Intermittent or permanent failure.

**Solution:**

- Verify connections:
    - STM32 PB8 → VL53L0X SCL
    - STM32 PB9 → VL53L0X SDA
    - STM32 GND → VL53L0X GND
    - STM32 3.3V → VL53L0X VDD
    - STM32 PA4 → VL53L0X XSHUT
- Check for:
    - Loose connections
    - Crossed wires (SCL/SDA swapped)
    - Broken breadboard holes
    - Too-long wires (should be < 20cm for reliable 100kHz operation)

### 6. **I2C Speed Too High**

**Symptoms:** Works sometimes, fails other times.

**Solution:**

- Current setting: 100kHz (good starting point)
- If still having issues, try 50kHz:
  ```rust
  i2c_config.frequency = Hertz::khz(50);
  ```
- For debugging, slower is better

## Debugging Steps

### Step 1: Visual Inspection

- [ ] Verify pull-up resistors are installed (4.7kΩ on SCL and SDA)
- [ ] Check all wiring connections
- [ ] Verify VL53L0X sensor is powered (3.3V on VDD)
- [ ] Ensure XSHUT pin is connected to PA4

### Step 2: Multimeter Checks (Power Off)

- [ ] Check continuity: STM32 PB8 to VL53L0X SCL
- [ ] Check continuity: STM32 PB9 to VL53L0X SDA
- [ ] Check continuity: GND to GND
- [ ] Verify no shorts between SCL and SDA
- [ ] Verify no shorts to VCC or GND

### Step 3: Multimeter Checks (Power On, Before I2C Init)

- [ ] SCL voltage: Should be 3.3V (pull-up working)
- [ ] SDA voltage: Should be 3.3V (pull-up working)
- [ ] VL53L0X VDD: Should be 3.3V
- [ ] XSHUT: Should start LOW, then go HIGH during init

### Step 4: Scope/Logic Analyzer (If Available)

- [ ] Monitor SCL and SDA during START attempt
- [ ] Should see SDA go low while SCL stays high
- [ ] If SCL or SDA stuck low, trace back to cause

## Quick Test: Loopback

If you have another I2C-compatible device (even an Arduino), try:

1. Connect it to the same I2C bus
2. Try scanning for I2C devices
3. This confirms if the problem is your STM32 setup or the VL53L0X

## Hardware Checklist

```
Hardware Setup Checklist:
[ ] 4.7kΩ resistor from PB8 (SCL) to 3.3V
[ ] 4.7kΩ resistor from PB9 (SDA) to 3.3V
[ ] VL53L0X VDD connected to 3.3V
[ ] VL53L0X GND connected to GND
[ ] VL53L0X SCL connected to PB8
[ ] VL53L0X SDA connected to PB9
[ ] VL53L0X XSHUT connected to PA4
[ ] VL53L0X GPIO1 connected to PA0 (for interrupt)
[ ] All connections verified with continuity test
[ ] No shorts between adjacent pins
[ ] Wire length < 20cm
```

## Expected Behavior After Fix

Once hardware is correct, you should see:

```
[INFO ] === I2C Configuration ===
[INFO ] I2C1: SCL=PB8, SDA=PB9, Speed=100kHz
[INFO ] Initializing distance sensor
[DEBUG] Toggling XSHUT pin...
[DEBUG] XSHUT toggled
[INFO ] Distance sensor initialized successfully
```

No timeout errors, no software resets needed!

## Still Not Working?

If you've checked everything above and it still fails:

1. **Try a different VL53L0X sensor** - the sensor might be damaged
2. **Try different GPIO pins** - test with I2C2 or I2C3 if available
3. **Verify STM32 is functioning** - test with a simpler peripheral (LED, UART)
4. **Check for conflicting peripherals** - ensure no DMA or timer conflicts
5. **Review schematic** - if using a custom PCB, verify the design

## Technical Details

The error sequence you're seeing:

1. Software requests START condition (sets CR1.START=1)
2. Hardware begins START sequence (SR2.BUSY=1, SR2.MSL=1)
3. **Peripheral waits for SDA to actually go low**
4. **This never happens due to hardware issue**
5. SB flag never sets in SR1
6. Timeout occurs after 10ms
7. Software reset attempted
8. Same issue repeats

The root cause is almost certainly **missing or insufficient pull-up resistors**.
