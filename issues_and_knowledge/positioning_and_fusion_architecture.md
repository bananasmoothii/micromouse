# Micromouse Positioning System

## Overview
Initially, the positioning logic was organized with a deep use of closures/callbacks, forcing the main module to pass around closures that could lead to tricky borrow checker limitations and lifetime constraints when executing on a single-core embedded environment without allocations. 

To resolve this, the architecture was re-engineered using `embassy_sync::channel::Channel`s. Now:
1. **Sensors** (Hardware interrupts, DMA spi polling) rapidly output their raw readings into non-blocking queues (Channels).
2. A single main **Positioning Task** reads from these queues safely, calculates physical displacements, and updates the core State.

This separation of concerns treats the devices as pure "Producers" and keeps the calculations free of IO latency. Processor files inside `src/positioning/` are now simply clean state machines without closures.

## How the Modules Work Together

### Odometry
Hall effect wheel encoders throw EXTI interrupts dynamically, pushing ticks onto `ODOM_LEFT_CHANNEL` and `ODOM_RIGHT_CHANNEL`.
The **`OdometryProcessor`** reads these counts, considers wheel configurations (like radius, axle base distance, and `ticks_per_rev`), and translates them into:
- Center distance moved forward ($\Delta D_c$)
- Angle turned based solely on wheels ($\Delta \theta_{odom}$)

### MPU System (Inverted)
The STM32 periodically receives `MargMeasurements`.
Because the sensor is upside down, the Y and Z axes are inverted. 
The **`MpuProcessor`** integrates the Z-axis gyro reading combined with hardware timer elapsed times to generate a rotational prediction ($\Delta \theta_{mpu}$).
Additionally, the processor captures the X and Y axes of the magnetometer (compass logic) to establish an absolute heading in relation to where the device turned on (`mag_heading_relative`).

### Sensor Fusion (Kalman Filter)
The system uses an intrinsic **1D Kalman Filter** to reliably merge all this incoming sensor data into one absolute orientation (heading) which in turn powers the calculation of the absolute X/Y Cartesian coordinates.
There is a supplementary **2D Phase Kalman Filter** executing simultaneously to calculate global X/Y Positioning. The architecture uses kinematic *Constant-Acceleration* mechanics over elapsed time.

**The Position Filter handles Three Aspects:**
1. **Prediction** leverages the *MPU9250 Accelerometer*. Because checking wheels with only 2 ticks/revolution provides incredibly staggered, "chunky" data, the accelerometer solves the "in-between" periods. Using precise kinematic integration ($p_{new} = p_{old} + v \cdot dt + \frac{1}{2} a \cdot dt^2$), and standard Euler left rectangles for instantaneous velocity ($V_{new} = V_{old} + a \cdot dt$), the system *smoothly predicts* rapid lateral and forward motion between those long gaps.
2. **Short-Term Correction** uses *Wheel Odometry* deltas to reel the prediction back into reality every time a physical wheel pulse registers, killing accelerometer integration drift permanently.
3. **Long-Term Correction** (NEW): Incorporates the *MPU Magnetometer (Compass)*. A compass reading inside a robot filled with moving motors, changing magnetic fields, and battery noise is generally very loud and jittery. It is therefore assigned a huge `r_theta_mag` static noise constant in the code. Because of this, it barely alters the heading step-by-step; however, it *prevents the robot's heading from drifting indefinitely to infinity* over several minutes, steadily pulling the robot's understanding of "forward" back down toward physical magnetic reality.

By tracking `p` (covariance uncertainty), the system relies purely on variance probabilities rather than manual tuning ratios, resulting in fluid positioning at extreme speeds.

## Physical Units and Tuning the Filter

All properties involving rotation inside the `SensorFusion` structure are grounded in mathematical physics:

- **`theta`**: The robot's actual heading in 2D space. Unit: **Radians ($rad$)**.
- **`p_theta` (Error Covariance - $P$)**: Our statistical uncertainty in `theta` right now. Unit: **Radians Squared ($rad^2$)**.
- **`q_theta` (Process Noise Covariance - $Q$)**: The expected variance accumulating off the internal gyro calculations *per time step*. Unit: **Radians Squared ($rad^2$)**.
- **`r_theta_odom` / `r_theta_mag` (Measurement Noise - $R$)**: The expected variance/jitter within the Odometry wheels or the Magnetometer logic. Unit: **Radians Squared ($rad^2$)**.

Because they are statistical variances ($rad^2$), they are natively positive floats. Here is a practical guide on predicting what happens when you adjust them for future needs:

### 1) Adjusting `q_theta` (Gyro Drift)
If you execute long straight runs and notice your heading drifting away when you mathematically know you are going exactly straight, your MPU internal gyro prediction is generating too much noise or poor integration. 
👉 *Action: INCREASE `q_theta`.* The filter will become more pessimistic about the gyroscope and weigh the odometry measurements (which expect 0 straight rotation) more heavily.

### 2) Adjusting `r_theta_odom` (Wheel Slippage)
If your robot turns into a tight corner at maximum velocity, the wheels might skid or lose contact with the mat. In this state, odometry is lying (the counts drop).
👉 *Action: INCREASE `r_theta_odom`.* By admitting higher expected noise from the wheels, the filter will lean harder on the gyro during fast movements, ignoring moments where the encoder ticks "freeze" during a skid. (If you improve wheel grip later, DECREASE this variable).

### 3) Adjusting `r_theta_mag` (Compass Interference)
Motors intrinsically distort magnetic fields when pumping PWM. Because the Micromouse pushes amps inside a tight chassis, the compass will swing wildly while moving. This is why `r_theta_mag` is initialized at a massive value (e.g. `10.0`). 
👉 *Action:* If you somehow manage to shield the magnetometer or run tests without motors, you can DECREASE this (to e.g., `0.5`). This would radically improve global positional permanence. Otherwise, leaving it high ensures the compass only steps in to gradually fight endless "long-walk gyro drifting" that happens across multiple minutes.
