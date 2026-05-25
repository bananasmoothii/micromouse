//! Physical dimensions of the robot and the maze. Single source of truth for any module that
//! needs to translate between sensor readings and global geometry. All values in metres unless
//! the constant name says otherwise.

use core::f32::consts::PI;

// ── Maze ────────────────────────────────────────────────────────────────────
/// Cell interior side length (wall-to-wall).
pub const LAB_CELL: f32 = 0.18;
/// Wall thickness. Not used in calculations currently (sensors measure to the wall face, not its centre).
pub const LAB_WALL: f32 = 0.01;

// ── Robot chassis ──────────────────────────────────────────────────────────
/// Robot body length (nose to tail). Used in `STOP_OFFSET` calculation in `main.rs`.
pub const ROBOT_LENGTH: f32 = 0.13;
pub const ROBOT_WIDTH: f32 = 0.10;

// ── Distance sensors ───────────────────────────────────────────────────────
// The two diagonal ToF sensors are mounted in a "U" pointing inward. Each sensor's reported
// "0 mm" point is the centre of the U, which sits on the robot's front edge on the centreline.
// In robot-local coords, treat both diagonal rays as starting at (0, +ROBOT_LENGTH/2) and going
// outward at ±DIAGONAL_ANGLE from the forward axis.
/// Diagonal sensor angle. Informational — not currently used in math (wall distances are
/// projected to lateral axis with a simple ÷√2 in `straight_line.rs`).
pub const DIAGONAL_ANGLE: f32 = PI / 4.0;
/// Each U-tip is this far from the robot centreline. Not used in calculations; the U-centre
/// geometry absorbs the offset. Kept for documentation of the physical assembly.
pub const SENSOR_TIP_OFFSET: f32 = 0.06;

// ── Derived ────────────────────────────────────────────────────────────────
/// Half cell width. Not currently referenced in code; kept for convenience.
pub const LAB_CELL_HALF: f32 = LAB_CELL / 2.0;
/// Front-edge-to-front-wall distance when the robot is centred in a cell with a front wall.
/// Equals `STOP_OFFSET` in `main.rs`. Not directly referenced elsewhere; kept for documentation.
pub const FRONT_CLEARANCE_CENTERED: f32 = (LAB_CELL - ROBOT_LENGTH) / 2.0;
/// Max lateral drift before the chassis touches a side wall.
pub const LATERAL_CLEARANCE: f32 = (LAB_CELL - ROBOT_WIDTH) / 2.0;
