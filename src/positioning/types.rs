use defmt::Format;

#[derive(Clone, Copy, Default, Debug, Format)]
pub struct Position2D {
    pub x: f32, // meters
    pub y: f32, // meters
    pub theta: f32, // radians, counter-clockwise
}

#[derive(Clone, Copy, Default, Debug, Format)]
pub struct MovementDelta {
    pub dx: f32,
    pub dy: f32,
    pub dtheta: f32,
}

