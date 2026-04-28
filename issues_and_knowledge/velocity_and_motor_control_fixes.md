# Velocity Estimation & Motor Control: Fixes Log

This document records the chain of bugs found and fixed across commits `fix odometry` → `fix velocity
calculations` and the subsequent uncommitted session. Each section describes what was wrong, why it was wrong,
and how it was fixed.

---

## 1. Backward Ticks Not Counted

### Symptom

`LEFT_TICKS_TOTAL` / `RIGHT_TICKS_TOTAL` only ever increased. Reversing a wheel produced the same tick
increments as going forward, so the robot had no way to know it was moving backward.

### Root Cause

The hall sensor ISR (`hall_sensor_continuous_measuring`) always called `fetch_add(1, …)` unconditionally.
Direction information was never read.

### Fix

`Motor::set_speed()` now writes a `LEFT_FORWARD` / `RIGHT_FORWARD` `AtomicBool` **before** changing the H-bridge
GPIO pins. The ISR reads that flag and signs the tick accordingly:

```rust
let delta = if LEFT_FORWARD.load(Ordering::Relaxed) { 1 } else { - 1 };
LEFT_TICKS_TOTAL.fetch_add(delta, Ordering::Relaxed);
```

**Important ordering:** The flag is stored before the GPIO state changes, so when the very next tick fires the
direction is already correct.

---

## 2. Velocity Non-Zero at Rest (Accelerometer Drift)

### Symptom

Log at standstill showed `v_x: 0.34  v_y: 0.52` (roughly 0.6 m/s) when the robot was not moving.

### Root Cause

The position Kalman filter predicted velocity by integrating the MPU9250 accelerometer (`a * dt`) and decaying it
with `v *= 0.95`. Two problems:

1. Every real accelerometer has a DC bias (a few mg). Integrating a small constant bias indefinitely produces a
   linearly growing velocity, and the 0.95 decay only stabilises it at a non-zero steady state.
2. There was a "velocity bump" after each odometry correction: when the Kalman update moved the position estimate,
   the code added `(k_xy * z) * 0.5 / dt` to velocity. With `dt = 0.02 s` and any non-zero correction, this
   injected large velocity spikes into every tick.

### Fix

Removed the accelerometer from the velocity path entirely. Velocity is now derived from inter-tick timestamps
(see §3). The velocity bump was also deleted.

---

## 3. Velocity Spike per Tick / Zero Between Ticks

### Symptom

With 2 ticks per revolution and `v = ticks_in_frame * distance_per_tick / DT`, every frame where a tick fired
showed `v_x ≈ 3.14 m/s` (one full circumference per 20 ms) and every other frame showed `v_x = 0.00`.

### Root Cause

`TICKS_PER_REVOLUTION = 2.0` and `WHEEL_RADIUS = 0.020 m` give
`DISTANCE_PER_TICK = 2π × 0.02 / 2 ≈ 0.063 m`. Dividing by `DT = 0.02 s` gives `≈ 3.14 m/s` for a single
tick. The raw `delta_ticks / DT` formula has no memory between frames: when no tick fires the result is zero.

### Fix

#### Inter-tick timestamp velocity

Each time the ISR fires it records `now_us` and updates two atomics:

- `*_LAST_TICK_US` — when the last tick happened
- `*_TICK_INTERVAL_US` — how long it took between the last two ticks

Velocity is then computed as:

```
effective_period = max(elapsed_since_last_tick, last_tick_interval)
velocity = DISTANCE_PER_TICK / effective_period
```

The `max()` is the key insight:

- **Constant speed**: `elapsed ≈ interval` → period is stable → velocity is stable.
- **Decelerating**: `elapsed > interval` → the denominator grows → velocity smoothly decays toward zero
  *without needing another tick to arrive*.
- **Stopped** (`elapsed > MAX_TICK_GAP_US = 700 ms`): returns 0.
- **Fewer than two ticks yet** (`interval == 0`): returns 0.

This adaptive formula works identically at low and high speeds without any manual window-size tuning.

The helper `smooth_wheel_velocity()` lives in `positioning/odometry.rs` and is called from both `fusion.rs`
(for position state) and `motors.rs` (for the PI feedback loop) via `left_wheel_velocity(now_us)` /
`right_wheel_velocity(now_us)`.

---

## 4. Motors Too Fast at Start / PI Integral Windup

### Symptom

At the start of a trajectory the wheels commanded near-maximum speed immediately. During deceleration at the end,
the PI integral had wound up so much that it commanded reverse while the wheels were still rolling forward, causing
backward ticks during a forward-only run.

### Root Causes

1. **No slew limit**: the PI output could jump from 0 to full PWM in a single 20 ms frame.
2. **No direction lock**: when the robot was still rolling forward but `target_v = 0`, `actual_v > 0` gave a large
   negative error. The PI integral kept accumulating and eventually produced a negative output, reversing the motor
   command.

### Fix

#### PWM slew-rate limiter

```rust
const MAX_PWM_SLEW: f32 = 0.10;
let left_out = left_raw.clamp(prev_left_out - MAX_PWM_SLEW, prev_left_out + MAX_PWM_SLEW);
```

Maximum PWM change is 0.10 per frame → full ramp-up takes at least 10 frames (200 ms).

#### Direction lock

After the slew clamp, a second clamp enforces that the output sign matches the target sign:

```rust
.clamp(
if target_left_v < 0.0 { - 1.0 } else { 0.0 },
if target_left_v > 0.0 { 1.0 } else { 0.0 },
)
```

If the target is forward (≥ 0), the output is clamped to `[0, 1]`, making a PI-induced reverse command
physically impossible.

---

## 5. Trajectory Racing Ahead of Reality

### Symptom

The motor controller consumed waypoints from `PATH_CHANNEL` as fast as the control loop ran, so the desired
position raced many cells ahead of where the robot actually was. When the robot finally caught up it had already
consumed the braking waypoints and overshot.

### Root Cause

The old code called `PATH_CHANNEL.try_receive()` every loop unconditionally, regardless of whether the robot
was keeping up.

### Fix

#### Lag guard

```rust
const MAX_LAG_M: f32 = 0.12;

let should_advance = match & last_waypoint {
None => true,
Some(wp) => {
let lag = (wp.x - state.x) * cos_t + (wp.y - state.y) * sin_t;
lag < MAX_LAG_M
}
};
```

If the robot is more than 12 cm behind its current waypoint in the forward direction, new waypoints are not
consumed. The position-correction gains (§6) then drive the robot toward the waypoint at a controlled rate.

#### Soft position correction

When a waypoint is active, positional error is projected into forward/lateral/heading components and added on top
of the trajectory's commanded velocity:

```rust
const KP_FWD: f32 = 0.6;  // (m/s) per meter behind
const KP_LAT: f32 = 2.0;  // (rad/s) per meter lateral offset
const KP_HDG: f32 = 1.0;  // (rad/s) per radian heading error

target_lin = (target_lin + KP_FWD * err_fwd).clamp(0.0, 0.8);
target_ang = (target_ang + KP_LAT * err_lat + KP_HDG * err_hdg).clamp(- 3.0, 3.0);
```

This replaces the old hard-tracking approach: corrections are bounded and additive, not overriding.

---

## 6. Zig-Zag Due to Sparse Tick Theta Perturbation

### Symptom

The robot zig-zagged visibly during straight runs. Each solo wheel tick (left fires, right not yet) caused the
EKF to estimate a small turn, which the motor controller then over-corrected, causing a physical turn the other
way.

### Root Cause

With only 2 ticks/rev each tick represents `DISTANCE_PER_TICK ≈ 0.063 m`. A solo left tick with no right tick
is processed as `d_theta = -0.063 / 0.078 ≈ -0.81 rad (46°)` of apparent rotation in the odometry delta. With
the old `r_theta_odom = 0.05` (very low noise assumed), the Kalman gain was high and the filter applied ~40% of
that apparent rotation to `theta`. Even at the corrected value of 5.7° per tick this was enough to steer the
robot noticeably.

### Fix (two-part)

#### Part A: Kalman noise retuning

`r_theta_odom` was raised from `0.05` to `1.0` (20×). The Kalman gain for the odometry update is
`K = P / (P + R)`. With `P ≈ 0.001` (converged) and `R = 1.0`, `K ≈ 0.001` — the filter now
ignores solo-tick apparent rotations almost entirely and trusts the gyro instead.
`r_theta_mag` was simultaneously lowered from `10.0` to `5.0` to partially compensate by letting the
magnetometer do slightly more long-term drift correction.

#### Part B: Gyro angular rate feedback in the motor controller

Even with the EKF fixed, physical wheel imbalance (one motor slightly faster than the other) causes real yaw.
A gyro feedback loop was added directly to `motor_controller_task`:

```rust
const K_ANG: f32 = 0.3;

let actual_omega = LATEST_MPU.lock( | c| c.get()).map( | d| - d.gyro[2]).unwrap_or(0.0);
let corrected_ang = target_ang + K_ANG * (target_ang - actual_omega);
```

`gyro[2]` is negated because the IMU is mounted upside-down (same sign convention as `fusion.rs`).

This is a **cascade angular loop**: the outer loop (waypoints + position correction) sets a desired yaw rate;
the inner loop (gyro) corrects for physical wheel imbalance within the same 20 ms frame — far faster than
waiting for an EKF position update to reflect the drift.

**Tuning `K_ANG`:** start at `0.3`. Increase toward `0.6` if zig-zag persists; decrease toward `0.1` if the
straight-line control becomes oscillatory.

---

## Summary Table

| Bug                                   | Fix                                                                 | Files                                   |
|---------------------------------------|---------------------------------------------------------------------|-----------------------------------------|
| Backward ticks not signed             | Direction flag written by motor driver, read by ISR                 | `hall_sensor_3144.rs`, `motors.rs`      |
| Velocity non-zero at rest             | Removed accelerometer integration; use inter-tick timestamps        | `fusion.rs`                             |
| Velocity spike / zero between ticks   | Adaptive `max(elapsed, interval)` period formula                    | `odometry.rs`, `fusion.rs`, `motors.rs` |
| PI reverses motor during decel        | Direction-lock clamp on PWM output                                  | `motors.rs`                             |
| Wheels too fast at start              | PWM slew-rate limiter (0.10/frame)                                  | `motors.rs`                             |
| Trajectory races ahead                | Lag guard (MAX_LAG_M = 0.12 m) pauses waypoint consumption          | `motors.rs`                             |
| No position correction                | Soft forward/lateral/heading PD correction added to target velocity | `motors.rs`                             |
| Theta oscillation from sparse ticks   | `r_theta_odom` raised 20× (0.05 → 1.0); gyro dominates              | `fusion.rs`                             |
| Physical zig-zag from wheel imbalance | Gyro angular rate cascade loop in motor controller                  | `motors.rs`                             |
