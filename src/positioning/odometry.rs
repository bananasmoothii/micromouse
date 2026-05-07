use crate::devices::hall_sensor_3144::{
    LEFT_LAST_TICK_US, LEFT_TICK_INTERVAL_US, LEFT_TICKS_TOTAL,
    RIGHT_LAST_TICK_US, RIGHT_TICK_INTERVAL_US, RIGHT_TICKS_TOTAL,
};
use crate::devices::motors::{LEFT_FORWARD, RIGHT_FORWARD};
use crate::positioning::types::MovementDelta;
use core::f32::consts::PI;
use core::sync::atomic::Ordering;
use micromath::F32Ext;

/// Physical radius of the wheels in meters. Matches the actual tire size.
pub const WHEEL_RADIUS: f32 = 0.020;

/// Distance between the two differential wheels in meters (track width).
pub const WHEEL_BASE: f32 = 0.078;

/// Raw odometry resolution: Number of hall effect ticks generated per full wheel revolution.
pub const TICKS_PER_REVOLUTION: f32 = 12.0;

pub const DISTANCE_PER_TICK: f32 = 2.0 * PI * WHEEL_RADIUS / TICKS_PER_REVOLUTION;

/// Speed below which the wheel is considered stopped.
const STALL_SPEED_M_S: f32 = 0.02;
/// Tick gap corresponding to STALL_SPEED_M_S — no tick within this window means stopped.
const MAX_TICK_GAP_US: u32 = (DISTANCE_PER_TICK / STALL_SPEED_M_S * 1_000_000.0) as u32;

/// Smooth signed wheel velocity (m/s) derived from inter-tick timestamps.
///
/// Uses `max(elapsed_since_last_tick, last_tick_interval)` as the effective period:
/// - Constant speed: elapsed ≈ interval → stable velocity.
/// - Decelerating: elapsed > interval → period grows → velocity decreases smoothly without waiting for the next tick.
/// - Stopped: elapsed exceeds MAX_TICK_GAP_US → returns 0.
/// - Fewer than two ticks fired (interval == 0): returns 0.
pub fn smooth_wheel_velocity(last_us: u32, interval_us: u32, now_us: u32, forward: bool) -> f32 {
    if interval_us == 0 {
        return 0.0;
    }
    let elapsed_us = now_us.wrapping_sub(last_us);
    if elapsed_us > MAX_TICK_GAP_US {
        return 0.0;
    }
    let period_us = elapsed_us.max(interval_us);
    let v = DISTANCE_PER_TICK / (period_us as f32 * 1e-6);
    if forward { v } else { -v }
}

pub fn left_wheel_velocity(now_us: u32) -> f32 {
    smooth_wheel_velocity(
        LEFT_LAST_TICK_US.load(Ordering::Relaxed),
        LEFT_TICK_INTERVAL_US.load(Ordering::Relaxed),
        now_us,
        LEFT_FORWARD.load(Ordering::Relaxed),
    )
}

pub fn right_wheel_velocity(now_us: u32) -> f32 {
    smooth_wheel_velocity(
        RIGHT_LAST_TICK_US.load(Ordering::Relaxed),
        RIGHT_TICK_INTERVAL_US.load(Ordering::Relaxed),
        now_us,
        RIGHT_FORWARD.load(Ordering::Relaxed),
    )
}

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
