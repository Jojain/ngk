//! A cylinder of radius 5 on z from z = 0 to 10 less the ball of radius 5 about (0, 0, 10): the
//! dish meets the wall tangentially at a knife edge.
use crate::bench::{ball, cut, cylinder_z, p};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    cut(cylinder_z(0.0, 0.0, 0.0, 10.0, 5.0)?, &ball(p(0.0, 0.0, 10.0), 5.0)?)
}
