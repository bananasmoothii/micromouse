use micromath::F32Ext;
use core::cell::Cell;
use core::ptr::null;
use critical_section::CriticalSection;
use crate::devices::motors::{PathPoint, DT, PATH_CHANNEL};
use crate::positioning::CURRENT_STATE;
use crate::positioning::types::Position2D;


pub async fn turn(theta: f32) {
    let state = CURRENT_STATE.lock(Cell::get);

    // TODO turn in multiple step cause it probabaly takes more than 2 hundredth of a sec
    PATH_CHANNEL.send(PathPoint { x: state.x, y: state.y, theta }).await;
}

pub async fn go_to(goal: PathPoint, time: f32) {
    let state = CURRENT_STATE.lock(Cell::get);

    let dx = goal.x - state.x;
    let dy = goal.y - state.y;
    let theta = dy.atan2(dx);

    let mut x = state.x;
    let mut y = state.y;

    let nb_points = time / DT;

    for i in 0..(nb_points as u32) {
        PATH_CHANNEL.send(PathPoint { x, y, theta }).await;

        x += dy / nb_points;
        y += dy / nb_points;
    }

    PATH_CHANNEL.send(goal).await;
}
