//! A ball of radius 5 about the origin bored along z by a hole of radius 3, the ball's poles at
//! (3, 0, 4) and (−3, 0, −4), on the ring's rims.
use crate::bench::{cut, cylinder_z, frame, p, v};
use crate::{Refusal, Solid};
use ngk::modeling::solids;

pub fn build() -> Result<Solid, Refusal> {
    let ball = solids::sphere_at(frame(p(0.0, 0.0, 0.0), v(3.0, 0.0, 4.0)), 5.0)?;
    cut(ball, &cylinder_z(0.0, 0.0, -10.0, 10.0, 3.0)?)
}
