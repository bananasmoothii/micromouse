use alloc::vec::Vec;
use micromath::F32Ext;
use crate::devices::motors::{PathPoint, PATH_CHANNEL, DT};
use crate::dimensions::LAB_CELL;
use crate::labyrinth::Labyrinth;
use core::cell::Cell;
use crate::positioning::CURRENT_STATE;

#[derive(Clone, Debug)]
pub enum Segment {
    Straight { distance: f32, speed: f32 },
    TurnInPlace { angle: f32, speed: f32 },
    // A future optimization layer will add smooth curves here (e.g., Spline or Arc)
    // Curve { radius: f32, angle: f32, speed: f32 },
}

pub struct Trajectory {
    pub segments: Vec<Segment>,
}

/// A trait for future trajectory optimizers that can convert basic cell paths 
/// into smoother, faster paths.
pub trait TrajectoryOptimizer {
    fn optimize(&self, raw_trajectory: Trajectory) -> Trajectory;
}

impl Trajectory {
    /// Builds a basic trajectory strictly following cell centers
    pub fn from_cell_path(path: &[(usize, usize)]) -> Self {
        let mut segments = Vec::new();
        if path.len() < 2 {
            return Self { segments };
        }

        let mut current_dir = (path[1].0 as isize - path[0].0 as isize, path[1].1 as isize - path[0].1 as isize);
        let mut straight_count = 1;

        for i in 1..path.len() - 1 {
            let next_dir = (path[i + 1].0 as isize - path[i].0 as isize, path[i + 1].1 as isize - path[i].1 as isize);

            if next_dir == current_dir {
                straight_count += 1;
            } else {
                // Add accumulated straight segment
                segments.push(Segment::Straight {
                    distance: straight_count as f32 * LAB_CELL,
                    speed: 0.5,
                });

                // Calculate turn angle
                let current_theta = (current_dir.1 as f32).atan2(current_dir.0 as f32);
                let next_theta = (next_dir.1 as f32).atan2(next_dir.0 as f32);

                let mut angle = next_theta - current_theta;
                // normalize between -PI and PI
                let pi = core::f32::consts::PI;
                let tau = 2.0 * pi;
                while angle > pi { angle -= tau; }
                while angle < -pi { angle += tau; }

                segments.push(Segment::TurnInPlace { angle, speed: pi });

                current_dir = next_dir;
                straight_count = 1;
            }
        }

        // Add final straight
        if straight_count > 0 {
            segments.push(Segment::Straight {
                distance: straight_count as f32 * LAB_CELL,
                speed: 0.5,
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

        while lab.exit_distance(curr_x, curr_y) > 0 {
            let (nx, ny) = lab.next_cell(curr_x, curr_y);
            // Break to avoid infinite loops if maze is unsolved
            if nx == curr_x && ny == curr_y {
                break;
            }
            curr_x = nx;
            curr_y = ny;
            path.push((curr_x, curr_y));
        }

        Self::from_cell_path(&path)
    }

    /// Feeds the trajectory to the motors via PATH_CHANNEL
    pub async fn execute(&self) {
        let state = CURRENT_STATE.lock(Cell::get);
        let mut current_x = state.x;
        let mut current_y = state.y;
        let mut current_theta = state.theta;

        for segment in &self.segments {
            match segment {
                Segment::Straight { distance, speed } => {
                    let time = distance / speed;
                    let nb_points = (time / DT) as u32;
                    let dx = current_theta.cos() * *distance;
                    let dy = current_theta.sin() * *distance;

                    if nb_points > 0 {
                        for _ in 0..nb_points {
                            current_x += dx / (nb_points as f32);
                            current_y += dy / (nb_points as f32);
                            PATH_CHANNEL.send(PathPoint { x: current_x, y: current_y, theta: current_theta }).await;
                        }
                    }
                }
                Segment::TurnInPlace { angle, speed } => {
                    let time = angle.abs() / speed;
                    let nb_points = (time / DT) as u32;
                    let d_theta = angle;

                    if nb_points > 0 {
                        for _ in 0..nb_points {
                            current_theta += d_theta / (nb_points as f32);
                            PATH_CHANNEL.send(PathPoint { x: current_x, y: current_y, theta: current_theta }).await;
                        }
                    }
                }
            }
        }
    }
}
