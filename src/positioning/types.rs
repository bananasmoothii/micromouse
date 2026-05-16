use micromath::F32Ext;
use alloc::format;
use core::fmt::{Display, Formatter};
use defmt::Format;

#[derive(Clone, Copy, Default, Debug)]
pub struct PositionState {
    pub x: f32,     // meters
    pub y: f32,     // meters
    pub theta: f32, // radians, counter-clockwise
    pub v_forward: f32, // local forward velocity (m/s), average of both wheels
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
            "x: {} y: {} theta: {}° v: {}",
            format!("{:.2}", self.x).as_str(),
            format!("{:.2}", self.y).as_str(),
            format!("{:.1}", self.theta.to_degrees()).as_str(),
            format!("{:.2}", self.v_forward).as_str()
        );
    }
}

impl Display for PositionState {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "x: {} y: {} theta: {}° v: {}",
            format!("{:.2}", self.x).as_str(),
            format!("{:.2}", self.y).as_str(),
            format!("{:.1}", self.theta.to_degrees()).as_str(),
            format!("{:.2}", self.v_forward).as_str()
        )
    }
}

#[derive(Clone, Copy, Default, Debug, Format)]
pub struct MovementDelta {
    pub dx: f32,
    pub dy: f32,
    pub d_theta: f32,
}
