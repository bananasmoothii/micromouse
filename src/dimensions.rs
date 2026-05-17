//! Physical dimensions of the robot and the maze. Single source of truth for any module that
//! needs to translate between sensor readings and global geometry. All values in metres unless
//! the constant name says otherwise.

use core::f32::consts::PI;

// ── Maze ────────────────────────────────────────────────────────────────────
/// Cell interior side length (wall-to-wall).
pub const LAB_CELL: f32 = 0.18;
/// Wall thickness.
pub const LAB_WALL: f32 = 0.01;

// ── Robot chassis ──────────────────────────────────────────────────────────
pub const ROBOT_LENGTH: f32 = 0.13;
pub const ROBOT_WIDTH: f32 = 0.10;

// ── Distance sensors ───────────────────────────────────────────────────────
// The two diagonal ToF sensors are mounted in a "U" pointing inward. Each sensor's reported
// "0 mm" point is the centre of the U, which sits on the robot's front edge on the centreline.
// In robot-local coords, treat both diagonal rays as starting at (0, +ROBOT_LENGTH/2) and going
// outward at ±DIAGONAL_ANGLE from the forward axis.
pub const DIAGONAL_ANGLE: f32 = PI / 4.0; // 45° outward from forward
/// Each U-tip is this far from the robot centreline (informational; not needed in math because
/// the U-centre absorbs the offset).
pub const SENSOR_TIP_OFFSET: f32 = 0.06;

// ── Derived ────────────────────────────────────────────────────────────────
pub const LAB_CELL_HALF: f32 = LAB_CELL / 2.0;
/// Front-edge-to-front-wall distance when the robot is centred in a cell with a front wall.
pub const FRONT_CLEARANCE_CENTERED: f32 = (LAB_CELL - ROBOT_LENGTH) / 2.0;
/// Max lateral drift before the chassis touches a side wall.
pub const LATERAL_CLEARANCE: f32 = (LAB_CELL - ROBOT_WIDTH) / 2.0;
