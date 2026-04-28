use crate::devices::motors::{DT, PATH_CHANNEL, PathPoint};
use crate::dimensions::LAB_CELL;
use crate::labyrinth::Labyrinth;
use crate::positioning::CURRENT_STATE;
use alloc::vec::Vec;
use core::cell::Cell;
use micromath::F32Ext;

pub mod config {
    /// Maximum linear velocity in m/s
    pub const MAX_SPEED: f32 = 0.8;
    /// Maximum linear acceleration in m/s^2
    pub const MAX_ACCELERATION: f32 = 1.0;
    /// Radius of the smooth arcs at corners in meters
    pub const CORNER_RADIUS: f32 = 0.05;
    /// The constant speed at which corners are taken in m/s
    pub const CORNER_SPEED: f32 = 0.2;
}

#[derive(Clone, Debug)]
pub enum Segment {
    Straight {
        distance: f32,
        max_speed: f32,
        entry_speed: f32,
        exit_speed: f32,
        acceleration: f32,
    },
    TurnInPlace {
        angle: f32,
        speed: f32,
    },
    SmoothTurn {
        angle: f32,
        radius: f32,
        speed: f32,
    },
}

pub struct Trajectory {
    pub segments: Vec<Segment>,
}

/// A trait for future trajectory optimizers that can convert basic cell paths
/// into smoother, faster paths.
pub trait TrajectoryOptimizer {
    fn optimize(&self, raw_trajectory: Trajectory) -> Trajectory;
}

pub struct SmoothCornerOptimizer {
    pub radius: f32,
    pub corner_speed: f32,
}

impl TrajectoryOptimizer for SmoothCornerOptimizer {
    fn optimize(&self, raw_trajectory: Trajectory) -> Trajectory {
        let mut optimized = Vec::new();
        let segments = raw_trajectory.segments;

        let mut i = 0;
        let mut previous_straight_remainder: Option<f32> = None;

        while i < segments.len() {
            if i + 2 <= segments.len() {
                if let (
                    Segment::Straight {
                        distance: d1,
                        max_speed: ms1,
                        entry_speed: en1,
                        exit_speed: _ex1,
                        acceleration: a1,
                    },
                    Segment::TurnInPlace { angle, speed: _ts },
                    Segment::Straight {
                        distance: d2,
                        max_speed: _ms2,
                        entry_speed: _en2,
                        exit_speed: _ex2,
                        acceleration: _a2,
                    },
                ) = (&segments[i], &segments[i + 1], &segments[i + 2])
                {
                    let actual_d1 = previous_straight_remainder.unwrap_or(*d1);

                    // The distance to offset is offset = radius * tan(|angle| / 2)
                    let offset = self.radius * (angle.abs() / 2.0).tan();

                    if actual_d1 >= offset && *d2 >= offset {
                        if actual_d1 > offset {
                            optimized.push(Segment::Straight {
                                distance: actual_d1 - offset,
                                max_speed: *ms1,
                                entry_speed: *en1,
                                exit_speed: self.corner_speed,
                                acceleration: *a1,
                            });
                        }
                        optimized.push(Segment::SmoothTurn {
                            angle: *angle,
                            radius: self.radius,
                            speed: self.corner_speed,
                        });
                        previous_straight_remainder = Some(*d2 - offset);

                        i += 2; // We processed S1 and T1. Next loop starts at S2.
                        continue;
                    }
                }
            }

            // If it doesn't match the pattern or we couldn't optimize
            if let Segment::Straight {
                distance,
                max_speed,
                entry_speed,
                exit_speed,
                acceleration,
            } = &segments[i]
            {
                let actual_d = previous_straight_remainder.unwrap_or(*distance);
                if actual_d > 0.0 {
                    optimized.push(Segment::Straight {
                        distance: actual_d,
                        max_speed: *max_speed,
                        entry_speed: if previous_straight_remainder.is_some() {
                            self.corner_speed
                        } else {
                            *entry_speed
                        },
                        exit_speed: *exit_speed,
                        acceleration: *acceleration,
                    });
                }
                previous_straight_remainder = None;
            } else {
                optimized.push(segments[i].clone());
            }

            i += 1;
        }

        Trajectory {
            segments: optimized,
        }
    }
}

pub struct VelocityProfileOptimizer {
    pub max_acceleration: f32,
}

impl TrajectoryOptimizer for VelocityProfileOptimizer {
    fn optimize(&self, raw_trajectory: Trajectory) -> Trajectory {
        let mut segments = raw_trajectory.segments;

        // Backward pass
        let mut next_entry_speed = 0.0;
        for segment in segments.iter_mut().rev() {
            match segment {
                Segment::Straight {
                    distance,
                    exit_speed,
                    entry_speed,
                    max_speed,
                    ..
                } => {
                    *exit_speed = next_entry_speed;
                    // Max possible entry speed given the exit speed and distance
                    let max_possible_entry =
                        (exit_speed.powi(2) + 2.0 * self.max_acceleration * *distance).sqrt();
                    *entry_speed = max_speed.min(max_possible_entry);
                    next_entry_speed = *entry_speed;
                }
                Segment::SmoothTurn { speed, .. } => {
                    next_entry_speed = *speed;
                }
                Segment::TurnInPlace { .. } => {
                    next_entry_speed = 0.0;
                }
            }
        }

        // Forward pass
        let mut prev_exit_speed: f32 = 0.0;
        for segment in segments.iter_mut() {
            match segment {
                Segment::Straight {
                    distance,
                    exit_speed,
                    entry_speed,
                    ..
                } => {
                    *entry_speed = prev_exit_speed.min(*entry_speed);
                    let max_possible_exit =
                        (entry_speed.powi(2) + 2.0 * self.max_acceleration * *distance).sqrt();
                    *exit_speed = exit_speed.min(max_possible_exit);
                    prev_exit_speed = *exit_speed;
                }
                Segment::SmoothTurn { speed, .. } => {
                    prev_exit_speed = *speed;
                }
                Segment::TurnInPlace { .. } => {
                    prev_exit_speed = 0.0;
                }
            }
        }

        Trajectory { segments }
    }
}

macro_rules! iterate_trajectory {
    ($self:expr, $start_x:expr, $start_y:expr, $start_theta:expr, |$x:ident, $y:ident, $theta:ident| $action:expr) => {{
        let mut curr_x = $start_x;
        let mut curr_y = $start_y;
        let mut curr_theta = $start_theta;

        for segment in &$self.segments {
            match segment {
                Segment::Straight {
                    distance,
                    max_speed,
                    entry_speed,
                    exit_speed,
                    acceleration,
                } => {
                    let mut current_v = *entry_speed;
                    let mut dist_covered = 0.0;
                    while dist_covered < *distance {
                        let decel_dist = (current_v * current_v - exit_speed.powi(2)).max(0.0)
                            / (2.0 * acceleration);
                        let target_v = if *distance - dist_covered <= decel_dist {
                            (current_v - acceleration * DT).max(*exit_speed)
                        } else {
                            (current_v + acceleration * DT).min(*max_speed)
                        };
                        let v = target_v.max(0.01);
                        let ds = v * DT;
                        dist_covered += ds;
                        current_v = v;
                        curr_x += curr_theta.cos() * ds;
                        curr_y += curr_theta.sin() * ds;

                        let $x = curr_x;
                        let $y = curr_y;
                        let $theta = curr_theta;
                        $action
                    }
                }
                Segment::TurnInPlace { angle, speed } => {
                    let time = angle.abs() / speed;
                    let nb_points = (time / DT) as u32;
                    let d_theta = angle;
                    if nb_points > 0 {
                        for _ in 0..nb_points {
                            curr_theta += d_theta / (nb_points as f32);
                            let $x = curr_x;
                            let $y = curr_y;
                            let $theta = curr_theta;
                            $action
                        }
                    }
                }
                Segment::SmoothTurn {
                    angle,
                    radius,
                    speed,
                } => {
                    let omega = speed / radius;
                    let time = angle.abs() / omega;
                    let nb_points = (time / DT) as u32;
                    let d_theta = angle;
                    if nb_points > 0 {
                        for _ in 0..nb_points {
                            curr_theta += d_theta / (nb_points as f32);
                            curr_x += curr_theta.cos() * speed * DT;
                            curr_y += curr_theta.sin() * speed * DT;
                            let $x = curr_x;
                            let $y = curr_y;
                            let $theta = curr_theta;
                            $action
                        }
                    }
                }
            }
        }
    }};
}

impl Trajectory {
    /// Builds a basic trajectory strictly following cell centers
    pub fn from_cell_path(path: &[(usize, usize)]) -> Self {
        let mut segments = Vec::new();
        if path.len() < 2 {
            return Self { segments };
        }

        let mut current_dir = (
            path[1].0 as isize - path[0].0 as isize,
            path[1].1 as isize - path[0].1 as isize,
        );
        let mut straight_count = 1;

        for i in 1..path.len() - 1 {
            let next_dir = (
                path[i + 1].0 as isize - path[i].0 as isize,
                path[i + 1].1 as isize - path[i].1 as isize,
            );

            if next_dir == current_dir {
                straight_count += 1;
            } else {
                // Add accumulated straight segment
                segments.push(Segment::Straight {
                    distance: straight_count as f32 * LAB_CELL,
                    max_speed: config::MAX_SPEED,
                    entry_speed: config::MAX_SPEED,
                    exit_speed: config::MAX_SPEED,
                    acceleration: config::MAX_ACCELERATION,
                });

                // Calculate turn angle
                let current_theta = (current_dir.1 as f32).atan2(current_dir.0 as f32);
                let next_theta = (next_dir.1 as f32).atan2(next_dir.0 as f32);

                let mut angle = next_theta - current_theta;
                // normalize between -PI and PI
                let pi = core::f32::consts::PI;
                let tau = 2.0 * pi;
                while angle > pi {
                    angle -= tau;
                }
                while angle < -pi {
                    angle += tau;
                }

                segments.push(Segment::TurnInPlace {
                    angle,
                    speed: config::CORNER_SPEED,
                });

                current_dir = next_dir;
                straight_count = 1;
            }
        }

        // Add final straight
        if straight_count > 0 {
            segments.push(Segment::Straight {
                distance: straight_count as f32 * LAB_CELL,
                max_speed: config::MAX_SPEED,
                entry_speed: config::MAX_SPEED,
                exit_speed: 0.0,
                acceleration: config::MAX_ACCELERATION,
            });
        }

        Self { segments }
    }

    /// Convenience function to build a trajectory from a labyrinth state
    pub fn build_from_labyrinth(lab: &Labyrinth, start_x: usize, start_y: usize) -> Self {
        let mut path = Vec::new();
        let mut curr_x = start_x;
        let mut curr_y = start_y;

        path.push((curr_x, curr_y));

        let mut current_dir = None;

        while lab.exit_distance(curr_x, curr_y) > 0 {
            let current_dist = lab.exit_distance(curr_x, curr_y);
            let mut best_next = None;

            for (nx, ny) in lab.neighbors(curr_x, curr_y) {
                if !lab.wall_btwn(curr_x, curr_y, nx, ny).is_present()
                    && lab.exit_distance(nx, ny) < current_dist
                {
                    let dir = (nx as isize - curr_x as isize, ny as isize - curr_y as isize);
                    if Some(dir) == current_dir {
                        best_next = Some((nx, ny, dir));
                        break; // straight is the best
                    } else if best_next.is_none() {
                        best_next = Some((nx, ny, dir));
                    }
                }
            }

            if let Some((nx, ny, dir)) = best_next {
                curr_x = nx;
                curr_y = ny;
                current_dir = Some(dir);
                path.push((curr_x, curr_y));
            } else {
                // Should not happen if maze is solvable, fallback to next_cell
                let (nx, ny) = lab.next_cell(curr_x, curr_y);
                if nx == curr_x && ny == curr_y {
                    break;
                }
                curr_x = nx;
                curr_y = ny;
                path.push((curr_x, curr_y));
            }
        }

        Self::from_cell_path(&path)
    }

    /// Feeds the trajectory to the motors via PATH_CHANNEL
    pub async fn execute(&self) {
        let state = CURRENT_STATE.lock(Cell::get);
        iterate_trajectory!(self, state.x, state.y, state.theta, |x, y, theta| {
            PATH_CHANNEL.send(PathPoint { x, y, theta }).await;
        });
    }

    pub fn simulate(&self, current_x: f32, current_y: f32, current_theta: f32) -> Vec<(f32, f32)> {
        let mut points = Vec::new();
        points.push((current_x, current_y));
        iterate_trajectory!(self, current_x, current_y, current_theta, |x, y, _theta| {
            points.push((x, y));
        });
        points
    }
}
