use alloc::vec::Vec;
use crate::trajectory::CardinalHeading;

pub const LAB_SIZE: usize = 8;

const LAB_EXIT_X: usize = 5;
const LAB_EXIT_Y: usize = 5;

#[derive(Copy, Clone)]
pub struct Cell {
    east_wall: Wall,
    south_wall: Wall,
    exit_distance: usize,
    next_x: usize,
    next_y: usize,
}

#[derive(Copy, Clone)]
pub enum Wall {
    NormalWall {
        hits: usize,
        misses: usize,
    },
    BorderWall
}

pub struct Labyrinth {
    cells: [[Cell; LAB_SIZE]; LAB_SIZE],
}

impl Wall {
    pub fn is_present(self) -> bool {
        match self {
            Wall::NormalWall { hits, misses } => hits > misses,
            Wall::BorderWall => true,
        }
    }
}

impl Labyrinth {
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
    pub fn has_south_wall(&self, x: usize, y: usize) -> bool {
        self.cells[y][x].south_wall.is_present()
    }
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
    pub fn has_east_wall(&self, x: usize, y: usize) -> bool {
        self.cells[y][x].east_wall.is_present()
    }
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

    /**
     * |x1 - x2| + |y1 - y2| = 1
     */
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

    pub fn neighbors(&self, x: usize, y: usize) -> Vec<(usize, usize)> {
        let mut neighbors = Vec::new();
        if x > 0 {neighbors.push((x-1, y));}
        if x < LAB_SIZE - 1 {neighbors.push((x+1, y));}
        if y > 0 {neighbors.push((x, y-1));}
        if y < LAB_SIZE - 1 {neighbors.push((x, y+1));}
        neighbors
    }

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

    pub fn next_cell(&self, x: usize, y: usize) -> (usize, usize) {
        let cell = self.cells[y][x];
        (cell.next_x, cell.next_y)
    }

    pub fn exit_distance(&self, x: usize, y: usize) -> usize {
        self.cells[y][x].exit_distance
    }
}
