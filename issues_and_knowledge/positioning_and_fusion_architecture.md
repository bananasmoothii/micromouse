# Micromouse Positioning System

## Overview

Initially, the positioning logic was organized with a deep use of closures/callbacks, forcing the main module to pass
around closures that could lead to tricky borrow checker limitations and lifetime constraints when executing on a
single-core embedded environment without allocations.

To resolve this, the architecture was re-engineered using `embassy_sync::channel::Channel`s. Now:

1. **Sensors** (Hardware interrupts, DMA spi polling) rapidly output their raw readings into non-blocking queues (
   Channels).
2. A single main **Positioning Task** reads from these queues safely, calculates physical displacements, and updates the
   core State.

This separation of concerns treats the devices as pure "Producers" and keeps the calculations free of IO latency.
Processor files inside `src/positioning/` are now simply clean state machines without closures.

## How the Modules Work Together

### Odometry

Hall effect wheel encoders throw EXTI interrupts dynamically, pushing ticks onto `ODOM_LEFT_CHANNEL` and
`ODOM_RIGHT_CHANNEL`.
The **`OdometryProcessor`** reads these counts, considers wheel configurations (like radius, axle base distance, and
`ticks_per_rev`), and translates them into:

- Center distance moved forward ($\Delta D_c$)
- Angle turned based solely on wheels ($\Delta \theta_{odom}$)

### MPU System (Inverted)

The STM32 periodically receives `MargMeasurements`.
Because the sensor is upside down, the Y and Z axes are inverted.
The **`MpuProcessor`** integrates the Z-axis gyro reading combined with hardware timer elapsed times to generate a
rotational prediction ($\Delta \theta_{mpu}$).
Additionally, the processor captures the X and Y axes of the magnetometer (compass logic) to establish an absolute
heading in relation to where the device turned on (`mag_heading_relative`).

### Sensor Fusion (Linear Kalman Filter)

The system uses an intrinsic **1D Linear Kalman Filter** to reliably merge all this incoming sensor data into one
absolute
orientation (heading) which in turn powers the calculation of the absolute X/Y Cartesian coordinates.
There is a supplementary **2D Linear Kalman Filter** executing simultaneously to calculate global X/Y Positioning.

*Note on Filter Type:* This implementation is strictly a **Linear Kalman Filter (LKF)**. While robot kinematics are
inherently non-linear (due to trigonometric rotation), the architecture deliberately uses a "decoupled" approach to save
CPU cycles on the Cortex-M microcontroller. Instead of computing complex Jacobian matrices for an Extended Kalman
Filter (EKF) or tracking sigma points for an Unscented Kalman Filter (UKF), it statically resolves the non-linear
trigonometry (`sin`/`cos`) first, converts the sensor inputs into global linear coordinate frames, and *then* feeds
those projections into the fast Linear Kalman Filter matrices.

The architecture uses kinematic *Constant-Acceleration* mechanics over elapsed time.

**The Position Filter handles Three Aspects:**

1. **Prediction** leverages the *MPU9250 Accelerometer*. Because checking wheels with only 2 ticks/revolution provides
   incredibly staggered, "chunky" data, the accelerometer solves the "in-between" periods. Using precise kinematic
   integration ($p_{new} = p_{old} + v \cdot dt + \frac{1}{2} a \cdot dt^2$), and standard Euler left rectangles for
   instantaneous velocity ($V_{new} = V_{old} + a \cdot dt$), the system *smoothly predicts* rapid lateral and forward
   motion between those long gaps.
2. **Short-Term Correction** uses *Wheel Odometry* deltas to reel the prediction back into reality every time a physical
   wheel pulse registers, killing accelerometer integration drift permanently.
3. **Long-Term Correction** (NEW): Incorporates the *MPU Magnetometer (Compass)*. A compass reading inside a robot
   filled with moving motors, changing magnetic fields, and battery noise is generally very loud and jittery. It is
   therefore assigned a huge `r_theta_mag` static noise constant in the code. Because of this, it barely alters the
   heading step-by-step; however, it *prevents the robot's heading from drifting indefinitely to infinity* over several
   minutes, steadily pulling the robot's understanding of "forward" back down toward physical magnetic reality.

By tracking `p` (covariance uncertainty), the system relies purely on variance probabilities rather than manual tuning
ratios, resulting in fluid positioning at extreme speeds.

## Physical Units and Tuning the Filter

All properties involving rotation inside the `SensorFusion` structure are grounded in mathematical physics:

- **`theta`**: The robot's actual heading in 2D space. Unit: **Radians ($rad$)**.
- **`p_theta` (Error Covariance - $P$)**: Our statistical uncertainty in `theta` right now. Unit: **Radians
  Squared ($rad^2$)**.
- **`q_theta` (Process Noise Covariance - $Q$)**: The expected variance accumulating off the internal gyro calculations
  *per time step*. Unit: **Radians Squared ($rad^2$)**.
- **`r_theta_odom` / `r_theta_mag` (Measurement Noise - $R$)**: The expected variance/jitter within the Odometry wheels
  or the Magnetometer logic. Unit: **Radians Squared ($rad^2$)**.

Because they are statistical variances ($rad^2$), they are natively positive floats. Here is a practical guide on
predicting what happens when you adjust them for future needs:

### 1) Adjusting `q_theta` (Gyro Drift)

If you execute long straight runs and notice the heading drifting away when you mathematically know you are going
exactly straight, the MPU internal gyro prediction is generating too much noise or poor integration.

*Action: INCREASE `q_theta`.* The filter will become more pessimistic about the gyroscope and weigh the odometry
measurements (which expect 0 straight rotation) more heavily.

### 2) Adjusting `r_theta_odom` (Wheel Slippage)

If the robot turns into a tight corner at maximum velocity, the wheels might skid or lose contact with the mat. In this
state, odometry is lying (the counts drop).

*Action: INCREASE `r_theta_odom`.* By admitting higher expected noise from the wheels, the filter will lean harder on
the gyro during fast movements, ignoring moments where the encoder ticks "freeze" during a skid. (If you improve wheel
grip later, DECREASE this variable).

### 3) Adjusting `r_theta_mag` (Compass Interference)

Motors intrinsically distort magnetic fields when pumping PWM. Because the Micromouse pushes amps inside a tight
chassis, the compass will swing wildly while moving. This is why `r_theta_mag` is initialized at a massive value (e.g.
`10.0`).

*Action:* If you somehow manage to shield the magnetometer or run tests without motors, you can DECREASE this (to e.g.,
`0.5`). This would radically improve global positional permanence. Otherwise, leaving it high ensures the compass only
steps in to gradually fight endless "long-walk gyro drifting" that happens across multiple minutes.

## Demystifying the Kalman Filter Math

If you look at the Wikipedia page for the Kalman filter, you will see a lot of intimidating matrix math with variables
like $\mathbf{P}$, $\mathbf{Q}$, and $\mathbf{R}$. Because our implementation is a simplified *1D Scalar (Linear)*
filter, those matrices collapse into standard float variables (`p`, `q`, and `r`).

### Where did the tuning values come from?

- **`p` (from $\mathbf{P}_{k|k}$)**: The *Estimate Covariance matrix*. It represents our current uncertainty. It is
  initialized at `1.0` (a wild guess), but it self-corrects and converges to its true optimal value almost immediately
  after running a few loops, so the starting value doesn't really matter!
- **`q` (from $\mathbf{Q}_k$)**: The *Process Noise covariance matrix*. We initialized it arbitrarily small (e.g.,
  `0.001`) because gyroscopes and kinematic equations are mathematically very accurate over a tiny time step (10ms).
- **`r` (from $\mathbf{R}_k$)**: The *Measurement Noise covariance matrix*. We initialized it higher (e.g., `0.05` for
  odometry, `10.0` for compass) based on educated guesses of their physical reliability, because raw sensors easily
  jitter.

### The Formulas Used

Because we don't use matrices, the scary vector equations from Wikipedia simplify dramatically into basic algebra:

**1. Predict Step:**

- *State Prediction*: $x = x + \Delta u$ (Code: `theta += dtheta`)
- *Covariance Prediction*: $P = P + Q$

**2. Update (Measurement) Step:**

- *Innovation (Residual - $\mathbf{y}_k$)*: $y = z - x$ (Where $z$ is the measurement from the sensor. How far off was
  our prediction?)
- *Innovation Covariance ($\mathbf{S}_k$)*: $S = P + R$ (Total uncertainty of the system + the sensor)
- *Kalman Gain ($\mathbf{K}_k$)*: $K = P / S$ (A ratio from 0.0 to 1.0 representing how much we should trust this
  measurement)
- *State Update*: $x = x + K \cdot y$
- *Covariance Update*: $P = (1 - K) \cdot P$ (Our uncertainty shrinks because a measurement gave us more information)

### Multiple Inputs to One Output (Sequential Updating)

Wikipedia often explains multi-sensor fusion using a massive measurement matrix ($\mathbf{H}$). In a microcontroller,
doing multi-dimensional matrix inversion is very slow and complex.

Instead, we use a perfectly mathematically equivalent technique called **Sequential Updating**.
When we have multiple measurements (Odometry and Magnetometer) for the same state (`theta`), we simply run the
**Update** step sequentially!

1. **Predict** using the Gyroscope.
2. **Update** using Odometry. This gives us a slightly better `theta` and a smaller `P`.
3. **Update** *again* immediately using the Magnetometer, feeding in the new `theta` and the smaller `P` from step 2.

This cascades the corrections cleanly, fusing all three sensors into a single dimension without needing any matrix
algebra.
