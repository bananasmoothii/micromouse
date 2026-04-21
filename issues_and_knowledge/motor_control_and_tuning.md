# Controlling Motors & PID Tuning for Micromouse

## Distances and Trajectories

Following a maze efficiently depends heavily on executing exact curves and splines.
Because "distance tracking" and stop-and-go behavior wastes momentum, the Motor Controller now consumes mapped Spline
points directly.
The control loop processes a new `PathPoint { x, y, theta }` from an `embassy_sync::channel::Channel` sequentially
exactly once every `DT = 20ms`.

Because points are fed into a standard Channel buffer with a fixed known `DT`, the motor controller can automatically
differentiate the difference between the next point and the current point to derive the required real-time
`target_velocity` matrix natively using basic geometry (`v = sqrt(dx^2 + dy^2) / dt` and `w = d_theta / dt`).

If the upper-level algorithm plans out a 5-second spline path around a corner, it simply pipes 250 `PathPoint` fragments
into `PATH_CHANNEL` and allows the PID controller to physical-track the line.
If the trajectory buffer empties, the robot decelerates or breaks natively.

## Concurrency Note: Why Atomics and Mutexes?

We use `embassy_sync::channel::Channel` when we need a pure FIFO *Buffer Queue* (like consuming sequential points
exactly 1 per loop).
However, for global metrics (like sharing Odometry ticks and MPU IMU data backwards to sensors):

* `AtomicI32` is lock-free, O(1), and completely eliminates dropped packets.
* `Mutex<Cell<T>>` provides a completely single-value, lock-free way to grab the "Last Data", safely avoiding Borrow
  Checker issues.

*Wait, doesn't `Mutex` alone provide interior mutability? What's the point of a Mutex if it doesn't give mutability?*

In computer science, a "Mutex" (Mutual Exclusion) only has one actual job: guaranteeing that *only one execution
context can access the data at a time*. In an embedded system, this prevents a hardware interrupt from firing and
reading the data exactly while we are halfway through writing to it (which would cause corrupted garbage data).

`std::sync::Mutex` in desktop Rust bundles *both* Mutual Exclusion and Interior Mutability together for convenience.

However, embedded Rust (`embassy`) handles Mutexes in two totally different flavors depending on what you are trying to
protect:

1. **`embassy_sync::mutex::Mutex`**: This is an *async* Mutex. Just like the desktop `std::sync::Mutex`, it *does*
   bundle Interior Mutability. When you `.await` its lock, it yields a `MutexGuard` that implements `DerefMut`, allowing
   you to get `&mut T`. This works perfectly for sharing an I2C/SPI bus across tasks where tasks can afford to sleep (
   `await`) while another task is talking to the hardware.
2. **`embassy_sync::blocking_mutex::Mutex`**: This is a *synchronous* lock used inside interrupt handlers or atomic
   variable tracking (like our odometry and MPU ticks). Because you cannot `.await` inside a hardware interrupt (the
   processor has to finish the math instantly), you use a `blocking_mutex`. To ensure maximum zero-cost safety and
   prevent deadlocks, `blocking_mutex::Mutex` strictly separates mutual exclusion from interior mutability. It only
   yields an immutable `&T`.

Because we need instant, synchronous, lock-free access to our `PATH_CHANNEL` endpoints and `LATEST_MPU` (you can't
`await` inside a synchronous math pipeline easily without messing up the 20ms timing delta), we use `blocking_mutex`.

By wrapping our data in a `Cell<T>`, we explicitly tell Rust's borrow checker "I want to mutate this safely," allowing
us to use `.set()` and `.get()` instantly inside the synchronous blocking lock! Because `Cell` is mathematically
zero-cost memory-wise for `Copy` types (like our tuples), we get the safest, fastest possible shared global state
without async overhead or data races!

## PID Tuning (Kp, Ki)

These determine how rigorously the Motor Controller tries to achieve the real-world requested speed.

- `KP` *(Proportional Factor)*: Pushes current directly against the immediate speed error gap. If the robot gets pushed,
  or slowed down dynamically by a turn, Kp ramps PWM.
- `KI` *(Integral Factor)*: Sums up long-standing error over time. Very helpful for ensuring the micromouse achieves the
  commanded velocity eventually rather than having a constant 5% slowdown error gap. Fixes battery-drain induced
  performance dips.

*Note on Fusioned Velocities:*
Currently, we derive velocity by evaluating absolute Odometry Ticks (`raw ticks`). With 2.0 Ticks per Revolution, this
might jitter.
*Proposition:* A future step could read `dx`/`dy`/`d_theta` from the fusion matrix if the Kp/Ki struggles accurately
reading the `AtomicI32` ticks jitter inside the 20ms delta window.

