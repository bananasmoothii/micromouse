use alloc::format;
use defmt::Format;

#[derive(Clone, Copy, Default, Debug)]
pub struct Position2D {
    pub x: f32,     // meters
    pub y: f32,     // meters
    pub theta: f32, // radians, counter-clockwise
    pub v_x: f32,   // local forward velocity
    pub v_y: f32,   // local lateral velocity
}

impl Format for Position2D {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "x: {} y: {} theta: {}° v_x: {} v_y: {}",
            format!("{:.2}", self.x).as_str(),
            format!("{:.2}", self.y).as_str(),
            format!("{:.1}", self.theta.to_degrees()).as_str(),
            format!("{:.2}", self.v_x).as_str(),
            format!("{:.2}", self.v_y).as_str()
        );
    }
}

#[derive(Clone, Copy, Default, Debug, Format)]
pub struct MovementDelta {
    pub dx: f32,
    pub dy: f32,
    pub d_theta: f32,
}
