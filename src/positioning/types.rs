//! Data structures shared across the positioning and trajectory subsystems.

use micromath::F32Ext;
use alloc::format;
use core::fmt::{Display, Formatter};
use defmt::Format;

/// Fused robot state published by [`crate::positioning::CURRENT_POS`].
///
/// All values are in SI units. The coordinate frame is:
/// - **x** increases eastward (maze `+x`).
/// - **y** increases southward (maze `+y`).
/// - **θ** (`theta`) is positive counter-clockwise (right-hand rule around `+z` pointing up).
///   `theta = 0` → facing east; `theta = -π/2` → facing south.
#[derive(Clone, Copy, Default, Debug)]
pub struct PositionState {
    /// Global x position in metres (east positive).
    pub x: f32,
    /// Global y position in metres (south positive).
    pub y: f32,
    /// Heading in radians, counter-clockwise positive. 0 = east, −π/2 = south.
    pub theta: f32,
    /// EMA-smoothed forward speed in m/s (average of both wheels, positive = forward).
    pub v_forward: f32,
    /// Angular rate in rad/s from gyro, positive = counter-clockwise.
    pub omega: f32,
}

impl PositionState {
    pub fn distance_from(&self, other: &PositionState) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }
}

impl Format for PositionState {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "x: {} y: {} theta: {}° v: {} omega: {}rad/s",
            format!("{:.2}", self.x).as_str(),
            format!("{:.2}", self.y).as_str(),
            format!("{:.1}", self.theta.to_degrees()).as_str(),
            format!("{:.2}", self.v_forward).as_str(),
            format!("{:.2}", self.omega).as_str()
        );
    }
}

impl Display for PositionState {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "x: {} y: {} theta: {}° v: {} omega: {}rad/s",
            format!("{:.2}", self.x).as_str(),
            format!("{:.2}", self.y).as_str(),
            format!("{:.1}", self.theta.to_degrees()).as_str(),
            format!("{:.2}", self.v_forward).as_str(),
            format!("{:.2}", self.omega).as_str()
        )
    }
}

/// Incremental movement in the robot-local frame, produced by odometry each 20 ms window.
#[derive(Clone, Copy, Default, Debug, Format)]
pub struct MovementDelta {
    /// Forward displacement in metres (positive = forward, along robot's current heading).
    pub dx: f32,
    /// Lateral displacement in metres (positive = left in a right-hand system).
    pub dy: f32,
    /// Heading change in radians (positive = counter-clockwise).
    pub d_theta: f32,
}
