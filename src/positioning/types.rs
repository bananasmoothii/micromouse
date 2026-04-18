use defmt::Format;

#[derive(Clone, Copy, Default, Debug, Format)]
pub struct Position2D {
    pub x: f32, // meters
    pub y: f32, // meters
    pub theta: f32, // radians, counter-clockwise
    pub v_x: f32, // local forward velocity
    pub v_y: f32, // local lateral velocity
}

#[derive(Clone, Copy, Default, Debug, Format)]
pub struct MovementDelta {
    pub dx: f32,
    pub dy: f32,
    pub d_theta: f32,
}
