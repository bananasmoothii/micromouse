use crate::devices::hall_sensor_3144::{LEFT_TICKS_TOTAL, RIGHT_TICKS_TOTAL};
use crate::positioning::types::MovementDelta;
use core::f32::consts::PI;
use core::sync::atomic::Ordering;
use micromath::F32Ext;

/// Physical radius of the wheels in meters (tire included).
/// Empirically derived: robot traveled 2.15 m for a 191-tick stop → r = 2.15×12/(2π×191) = 21.5 mm.
pub const WHEEL_RADIUS: f32 = 0.0215;

/// Distance between the two differential wheels in meters (track width).
pub const WHEEL_BASE: f32 = 0.078;

/// Hall sensor ticks per full wheel revolution (12 magnets, one tick each).
pub const TICKS_PER_REVOLUTION: u32 = 12;

/// Arc length per tick: circumference / ticks_per_rev.
pub const DISTANCE_PER_TICK: f32 = 2.0 * PI * WHEEL_RADIUS / (TICKS_PER_REVOLUTION as f32);

// --- Velocity architecture note ---
// Forward velocity (v_forward) is derived from tick *count* accumulated over each 20 ms
// fusion window: v = odom_delta.dx / dt. This is computed in fusion::SensorFusion::update().
//
// An earlier tick-*interval* approach (DISTANCE_PER_TICK / inter_tick_time) was tried but
// discarded: with only 12 ticks/rev the inter-tick period (~11 ms at 0.5 m/s) aliases badly
// against the 20 ms sample rate, producing large periodic velocity spikes that corrupt the
// PI speed controller. The tick-count method has bounded quantisation noise (0–2 ticks per
// window) and no aliasing spikes, making it far more suitable as a controller input.

/// Drains accumulated ticks since last call and returns the movement delta.
pub fn get_odom_delta() -> MovementDelta {
    let d_left = LEFT_TICKS_TOTAL.swap(0, Ordering::Relaxed) as f32 * DISTANCE_PER_TICK;
    let d_right = RIGHT_TICKS_TOTAL.swap(0, Ordering::Relaxed) as f32 * DISTANCE_PER_TICK;

    let d_center = (d_right + d_left) / 2.0;
    let d_theta = (d_right - d_left) / WHEEL_BASE;

    let dx;
    let dy;

    // Handle straight line case to avoid division by zero (and infinite radius)
    if d_theta.abs() < 1e-6 {
        dx = d_center;
        dy = 0.0;
    } else {
        // Note about the Orthornomal Basis vs Trigonometry:
        // You might expect dx = d_center * cos(d_theta) and dy = d_center * sin(d_theta).
        // However, that formula only applies if the robot moved in a *straight line* at an angle d_theta.
        // But the robot is turning continuously, meaning it drives along the arc of a circle.
        //
        // If the robot starts at (0,0) facing along the +X axis (theta = 0):
        // - The center of the turning circle is located on the Y axis at (0, R).
        // - Using circle parametric equations around center (0, R), the robot's X position is:
        //   X = 0 + R * cos(-90deg + d_theta) -> R * sin(d_theta)
        // - The robot's Y position is:
        //   Y = R + R * sin(-90deg + d_theta) -> R - R * cos(d_theta) -> R * (1 - cos(d_theta))
        //
        // What if we turn right?
        // If turning right, d_theta is negative, which makes the calculated `radius` negative.
        // Let d_theta = -a, radius = -R.
        // dx = -R * sin(-a) = -R * (-sin(a)) = R * sin(a) -> dx remains positive (moving forward).
        // dy = -R * (1 - cos(-a)) = -R * (1 - cos(a))     -> dy becomes negative (drifting right/-Y).
        // The math perfectly self-corrects!
        //
        // This geometrically perfect arc calculation gives us the local frame movement.
        // These local limits are then mapped back to the global orthonormal plane later in the fusion/Kalman logic!
        let radius = d_center / d_theta;

        let (d_theta_sin, d_theta_cos) = d_theta.sin_cos();

        dx = radius * d_theta_sin;
        dy = radius * (1.0 - d_theta_cos);
    }

    MovementDelta { dx, dy, d_theta }
}
