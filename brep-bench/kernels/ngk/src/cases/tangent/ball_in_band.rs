//! A ball of radius 5 about the origin united with the cylinder of the same radius on z from
//! z = −3 to 3, tangent to it along the equator.
use crate::bench::{ball, cylinder_z, fuse, p};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    fuse(ball(p(0.0, 0.0, 0.0), 5.0)?, &cylinder_z(0.0, 0.0, -3.0, 3.0, 5.0)?)
}
