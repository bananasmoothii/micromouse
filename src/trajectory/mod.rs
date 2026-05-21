pub mod straight_line;
pub mod in_place_turn;

use alloc::boxed::Box;
use alloc::vec::Vec;
use core::f32::consts::PI;
use embassy_stm32::peripherals::{TIM2, TIM3};
use crate::devices::motors::Motor;
use crate::labyrinth::Labyrinth;
use crate::positioning::CURRENT_POS;
use crate::trajectory::in_place_turn::InPlaceTurn;
use crate::trajectory::straight_line::{StraightLine, StraightLineGoal};
use crate::utils::{CellMutexUtils, DurationUtils};

pub const UPDATE_INTERVAL_MS: u64 = 20;

#[async_trait::async_trait]
pub trait TrajectorySegment {
    async fn execute<'a>(&self, motor_left: &mut Motor<'a, TIM3>, motor_right: &mut Motor<'a, TIM2>);
}

/// Cardinal heading in maze/screen coordinates: East = x+, South = y+ (down), West = x-, North = y-
#[derive(Copy, Clone, Eq, PartialEq)]
pub enum CardinalHeading {
    East = 0,
    South = 1,
    West = 2,
    North = 3,
}

impl CardinalHeading {
    /// Absolute robot-world theta (rad) for this heading.
    /// Convention: theta=0 = East, positive = CCW.  South = CW from East = -π/2.
    pub fn to_robot_theta(self) -> f32 {
        -(self as i32 as f32) * PI / 2.0
    }
}

/// One step in the abstract route.
pub enum PlanStep {
    /// Turn until the robot faces this absolute cardinal heading, correcting for drift.
    Turn(CardinalHeading),
    /// Drive forward until the front wall is `stop_offset` metres away, then stop.
    /// The `CardinalHeading` is the expected corridor direction: used to compute
    /// `initial_heading_error` from the live theta at execution time.
    Straight(f32, CardinalHeading),
}

/// Abstract route derived from a `Labyrinth`.
///
/// Each `Turn` step stores the **absolute target heading** rather than a relative angle.
/// At execution time the robot's live theta is read from `CURRENT_POS` and subtracted,
/// so any angular drift accumulated in previous segments is automatically corrected:
/// "planned 90° CW, already drifted 5° CW → turn only 85°".
pub struct LabyrinthPlan {
    pub steps: Vec<PlanStep>,
}

impl LabyrinthPlan {
    /// Builds the plan by following `next_cell()` from `(start_x, start_y)` to the exit.
    ///
    /// The maze **must** have a front wall at every turning point and at the exit cell so
    /// that `DistanceToFrontWall` can trigger the stop.
    pub fn from_labyrinth(
        lab: &Labyrinth,
        start_x: usize,
        start_y: usize,
        start_heading: CardinalHeading,
        stop_offset: f32,
    ) -> Self {
        let mut steps = Vec::new();
        let mut cur_x = start_x;
        let mut cur_y = start_y;
        let mut cur_heading = start_heading;

        loop {
            if lab.exit_distance(cur_x, cur_y) == 0 {
                break;
            }
            let (nx, ny) = lab.next_cell(cur_x, cur_y);
            if nx == cur_x && ny == cur_y {
                break; // no path to exit
            }

            let target = heading_from_delta(nx as i32 - cur_x as i32, ny as i32 - cur_y as i32);
            if target != cur_heading {
                steps.push(PlanStep::Turn(target));
                cur_heading = target;
            }

            // Walk ahead until the path direction changes or the exit is reached.
            let mut run_x = cur_x;
            let mut run_y = cur_y;
            loop {
                if lab.exit_distance(run_x, run_y) == 0 {
                    break;
                }
                let (nnx, nny) = lab.next_cell(run_x, run_y);
                if nnx == run_x && nny == run_y {
                    break;
                }
                if heading_from_delta(nnx as i32 - run_x as i32, nny as i32 - run_y as i32)
                    != cur_heading
                {
                    break;
                }
                run_x = nnx;
                run_y = nny;
            }
            steps.push(PlanStep::Straight(stop_offset, cur_heading));
            cur_x = run_x;
            cur_y = run_y;
        }

        LabyrinthPlan { steps }
    }

    /// Executes the plan.
    ///
    /// Before each `Turn`, the robot's live heading is read from `CURRENT_POS` so any
    /// residual angular error from the previous segment is corrected on the fly.
    /// After every step motors are zeroed and the robot waits `settle_ms` milliseconds
    /// for wheel slip to dissipate before the next heading sample or segment start.
    pub async fn execute<'a>(
        &self,
        motor_left: &mut Motor<'a, TIM3>,
        motor_right: &mut Motor<'a, TIM2>,
        settle_ms: u64,
    ) {
        for step in &self.steps {
            match step {
                PlanStep::Turn(target_heading) => {
                    let current_theta = CURRENT_POS.get().theta;
                    // Remaining angle = target − actual, normalised to (−π, π].
                    let mut angle = target_heading.to_robot_theta() - current_theta;
                    while angle > PI {
                        angle -= 2.0 * PI;
                    }
                    while angle < -PI {
                        angle += 2.0 * PI;
                    }
                    if angle.abs() > 2.0 * PI / 180.0 {
                        InPlaceTurn { angle }.execute(motor_left, motor_right).await;
                    }
                }
                PlanStep::Straight(stop_offset, target_heading) => {
                    // Compute heading residual from live theta after settle.
                    // Captures both InPlaceTurn undershoot and any coast overshoot
                    // during the settle wait, so StraightLine corrects from tick one.
                    let actual_theta = CURRENT_POS.get().theta;
                    let mut initial_heading_error =
                        actual_theta - target_heading.to_robot_theta();
                    while initial_heading_error > PI { initial_heading_error -= 2.0 * PI; }
                    while initial_heading_error < -PI { initial_heading_error += 2.0 * PI; }
                    StraightLine {
                        goal: StraightLineGoal::DistanceToFrontWall(*stop_offset),
                        out_speed: 0.0,
                        initial_heading_error,
                    }
                        .execute(motor_left, motor_right)
                        .await;
                }
            }
            motor_left.set_speed(0.0);
            motor_right.set_speed(0.0);
            settle_ms.ms_timer().await;
        }
    }
}

fn heading_from_delta(dx: i32, dy: i32) -> CardinalHeading {
    match (dx, dy) {
        (1, 0) => CardinalHeading::East,
        (-1, 0) => CardinalHeading::West,
        (0, 1) => CardinalHeading::South,
        _ => CardinalHeading::North,
    }
}
