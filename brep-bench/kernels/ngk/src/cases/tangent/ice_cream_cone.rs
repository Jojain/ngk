//! A ball of radius 15 about the origin united with the cone from its apex (0, 0, −25) to its base,
//! the disc of radius 12 at z = −9, tangent to the sphere along the base's rim.
use crate::bench::{ball, cone, fuse, p, v};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let c = cone(p(0.0, 0.0, -25.0), v(0.0, 0.0, 1.0), 0.0, 12.0, 16.0)?;
    fuse(ball(p(0.0, 0.0, 0.0), 15.0)?, &c)
}
