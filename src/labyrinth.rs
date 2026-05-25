//! 8 × 8 probabilistic maze representation and live-update flood-fill pathfinder.
//!
//! ## Coordinate system
//! `x` increases eastward, `y` increases southward (matches the `CardinalHeading` enum).
//! Cell `(0, 0)` is the top-left / north-west corner; cell `(LAB_SIZE-1, LAB_SIZE-1)` is
//! the bottom-right / south-east corner.
//!
//! ## Wall model
//! Each internal wall face is represented by a [`Wall::NormalWall`] with independent
//! `hits` and `misses` counters.  The wall is considered *present* when `hits > misses`.
//! Border walls are always [`Wall::BorderWall`] (always present, never updated).
//! This probabilistic model lets the robot revise its map as new sensor data arrives.
//!
//! ## Pathfinding
//! The maze uses a lazy flood-fill stored *inside each cell*: every cell carries
//! `exit_distance` (BFS cost to the exit) and `(next_x, next_y)` (the neighbouring
//! cell on the shortest path).  When a wall observation changes the perceived topology,
//! [`Labyrinth::update_cell_distance`] propagates the change recursively to all affected
//! cells — a standard flood-fill / wavefront update.
//!
//! ## Usage
//! ```rust,ignore
//! let mut lab = Labyrinth::new();
//! lab.ray_east_wall(2, 0, true);   // sensor saw an east wall at (2,0)
//! lab.ray_south_wall(2, 3, false); // sensor saw no south wall at (2,3)
//! let (nx, ny) = lab.next_cell(0, 0); // follow shortest path
//! ```

use alloc::vec::Vec;
use crate::trajectory::CardinalHeading;

/// Side length of the maze grid in cells.
pub const LAB_SIZE: usize = 8;

/// X coordinate of the target exit cell.
const LAB_EXIT_X: usize = 5;
/// Y coordinate of the target exit cell.
const LAB_EXIT_Y: usize = 5;

/// Internal representation of one maze cell.
///
/// Walls are stored on the **east** and **south** faces only; west/north walls of a cell
/// are the east/south walls of the adjacent cell.  The `exit_distance` + `next_*` fields
/// form the embedded flood-fill graph that [`Labyrinth::next_cell`] reads at runtime.
#[derive(Copy, Clone)]
pub struct Cell {
    /// Wall on the eastern face of this cell (shared with `(x+1, y).west`).
    east_wall: Wall,
    /// Wall on the southern face of this cell (shared with `(x, y+1).north`).
    south_wall: Wall,
    /// BFS cost (number of cells) to the exit from this cell.
    exit_distance: usize,
    /// X coordinate of the next cell on the shortest path to the exit.
    next_x: usize,
    /// Y coordinate of the next cell on the shortest path to the exit.
    next_y: usize,
}

/// Probabilistic wall state.
///
/// A border wall (`x == 0`, `x == LAB_SIZE`, etc.) is always [`Wall::BorderWall`].
/// Every other wall starts as `NormalWall { hits: 0, misses: 0 }` and is considered
/// *present* once `hits > misses`.
#[derive(Copy, Clone)]
pub enum Wall {
    /// Internal wall with evidence counters accumulated from ToF sensor observations.
    NormalWall {
        /// Number of times a sensor reported this wall as present.
        hits: usize,
        /// Number of times a sensor reported this wall as absent.
        misses: usize,
    },
    /// Outer boundary — always present, never modified.
    BorderWall,
}

/// 8 × 8 maze graph with probabilistic walls and an embedded flood-fill shortest path.
pub struct Labyrinth {
    /// Row-major: `cells[y][x]`.
    cells: [[Cell; LAB_SIZE]; LAB_SIZE],
}

impl Wall {
    /// Returns `true` if this wall is considered physically present.
    /// Border walls are always present; normal walls require `hits > misses`.
    pub fn is_present(self) -> bool {
        match self {
            Wall::NormalWall { hits, misses } => hits > misses,
            Wall::BorderWall => true,
        }
    }
}

impl Labyrinth {
    /// Creates an empty maze: no walls observed, flood-fill pre-seeded with Manhattan distances
    /// to the exit so `next_cell` works before any sensor observations arrive.
    pub fn new() -> Self {
        Self {
            cells: core::array::from_fn(|y| core::array::from_fn(|x| Cell {
                east_wall: if x == LAB_SIZE - 1 {Wall::BorderWall} else {Wall::NormalWall {hits: 0, misses: 0}},
                south_wall: if y == LAB_SIZE - 1 {Wall::BorderWall} else {Wall::NormalWall {hits: 0, misses: 0}},
                exit_distance: x.abs_diff(LAB_EXIT_X) + y.abs_diff(LAB_EXIT_Y),
                next_x: if x < LAB_EXIT_X {x+1} else if x > LAB_EXIT_X {x-1} else {x},
                next_y: if LAB_EXIT_X != x || LAB_EXIT_Y == y {y} else if y < LAB_EXIT_Y {y + 1} else {y - 1},
            }))
        }
    }

    fn has_wall(self, x: usize, y: usize, dir: CardinalHeading) -> bool {
        match dir {
            CardinalHeading::South => y == LAB_SIZE || self.has_south_wall(x, y),
            CardinalHeading::North => y == 0 || self.has_south_wall(x, y - 1),
            CardinalHeading::East => x == LAB_SIZE || self.has_east_wall(x, y),
            CardinalHeading::West => x == 0 || self.has_east_wall(x - 1, y),
        }
    }

    fn ray_wall(&mut self, x: usize, y: usize, dir: CardinalHeading, seen: bool) {
        match dir {
            CardinalHeading::South => if y != LAB_SIZE { self.ray_south_wall(x, y, seen) },
            CardinalHeading::North => if y == 0 { self.ray_south_wall(x, y - 1, seen) },
            CardinalHeading::East => if x == LAB_SIZE { self.ray_east_wall(x, y, seen) },
            CardinalHeading::West => if x == 0 { self.ray_east_wall(x - 1, y, seen) },
        }
    }
    /// Returns `true` if the southern face of cell `(x, y)` currently has a wall.
    pub fn has_south_wall(&self, x: usize, y: usize) -> bool {
        self.cells[y][x].south_wall.is_present()
    }
    /// Records one sensor observation of the southern face of cell `(x, y)`.
    /// `seen = true` increments hits; `seen = false` increments misses.
    /// If the perceived wall state flips, the flood-fill is updated.
    pub fn ray_south_wall(&mut self, x: usize, y: usize, seen: bool) {
        let had_wall = self.has_south_wall(x, y);

        if let Wall::NormalWall {hits, misses} = &mut self.cells[y][x].south_wall {
            if seen {
                *hits += 1;
            } else {
                *misses += 1;
            }
        }

        if had_wall != self.has_south_wall(x, y) {
            self.update_cell_distance(x, y);
        }
    }
    /// Returns `true` if the eastern face of cell `(x, y)` currently has a wall.
    pub fn has_east_wall(&self, x: usize, y: usize) -> bool {
        self.cells[y][x].east_wall.is_present()
    }
    /// Records one sensor observation of the eastern face of cell `(x, y)`.
    /// `seen = true` increments hits; `seen = false` increments misses.
    /// If the perceived wall state flips, the flood-fill is updated.
    pub fn ray_east_wall(&mut self, x: usize, y: usize, seen: bool) {
        let had_wall = self.has_east_wall(x, y);

        if let Wall::NormalWall {hits, misses} = &mut self.cells[y][x].east_wall {
            if seen {
                *hits += 1;
            } else {
                *misses += 1;
            }
        }

        if had_wall != self.has_east_wall(x, y) {
            self.update_cell_distance(x, y);
        }
    }

    /// Returns the wall between two adjacent cells.
    /// Panics (debug) if the cells are not exactly one step apart.
    pub fn wall_btwn(&self, x1: usize, y1: usize, x2: usize, y2: usize) -> Wall {
        assert_eq!(x1.abs_diff(x2) + y1.abs_diff(y2), 1);
        if x1 < x2 {
            self.cells[y1][x1].east_wall
        }
        else if x1 > x2 {
            self.cells[y2][x2].east_wall
        }
        else if y1 < y2 {
            self.cells[y1][x1].south_wall
        }
        else {
            self.cells[y2][x2].south_wall
        }
    }

    /// Returns the grid coordinates of all in-bounds neighbours (4-connectivity).
    pub fn neighbors(&self, x: usize, y: usize) -> Vec<(usize, usize)> {
        let mut neighbors = Vec::new();
        if x > 0 {neighbors.push((x-1, y));}
        if x < LAB_SIZE - 1 {neighbors.push((x+1, y));}
        if y > 0 {neighbors.push((x, y-1));}
        if y < LAB_SIZE - 1 {neighbors.push((x, y+1));}
        neighbors
    }

    /// Propagates a flood-fill update starting at `(x, y)`.
    ///
    /// Called whenever a wall observation changes the perceived maze topology. Recomputes
    /// `exit_distance` and `next_*` for `(x, y)` then recurses to all neighbours whose
    /// shortest path previously passed through this cell or whose distance is now worse.
    pub fn update_cell_distance(&mut self, x: usize, y: usize) {
        if x == LAB_EXIT_X && y == LAB_EXIT_Y {
            return;
        }

        let mut best = LAB_SIZE * LAB_SIZE; // theorical max

        self.cells[y][x].exit_distance = best;

        for (dx, dy) in self.neighbors(x, y) {
            let dcell = self.cells[dy][dx];
            if self.wall_btwn(dx, dy, x, y).is_present() {
                // if someone points at me when there is a wall btwn us, better tell him
                if dcell.next_x == x && dcell.next_y == y {
                    self.update_cell_distance(dx, dy)
                }
            }
            else {
                if dcell.exit_distance < best {
                    best = dcell.exit_distance;

                    let cell = &mut self.cells[y][x];
                    cell.next_x = dx;
                    cell.next_y = dy;
                }
            }
        }

        if best == LAB_SIZE * LAB_SIZE {
            self.cells[y][x].exit_distance = best;
            return;
        }

        self.cells[y][x].exit_distance = best + 1;

        for (dx, dy) in self.neighbors(x, y) {
            let dcell = self.cells[dy][dx];
            if dcell.next_x == x && dcell.next_y == y || dcell.exit_distance > best + 1 {
                self.update_cell_distance(dx, dy)
            }
        }
    }

    /// Returns the next cell `(nx, ny)` to enter when following the shortest path from `(x, y)`.
    /// If no path exists (disconnected island), returns `(x, y)` itself.
    pub fn next_cell(&self, x: usize, y: usize) -> (usize, usize) {
        let cell = self.cells[y][x];
        (cell.next_x, cell.next_y)
    }

    /// BFS distance from cell `(x, y)` to the exit. `0` means this cell *is* the exit.
    pub fn exit_distance(&self, x: usize, y: usize) -> usize {
        self.cells[y][x].exit_distance
    }
}
