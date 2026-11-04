//! A ball of radius 5 about the origin bored along z by a hole of radius 3, the drill a cylinder
//! from z = −10 to z = 10.
use crate::bench::{ball, cut, cylinder_z, p};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    cut(ball(p(0.0, 0.0, 0.0), 5.0)?, &cylinder_z(0.0, 0.0, -10.0, 10.0, 3.0)?)
}
