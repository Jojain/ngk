//! A ball of radius 10 about the origin joined to a post, the cylinder of radius 4 on the z axis
//! from z = 0 to 15. The circle where the post leaves the ball, radius 4 at z = √84, is rounded
//! to radius 2, adding material.
use crate::bench::{ball, cylinder_z, fillet_through, fuse, p};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let part = fuse(ball(p(0.0, 0.0, 0.0), 10.0)?, &cylinder_z(0.0, 0.0, 0.0, 15.0, 4.0)?)?;
    fillet_through(part, &[p(0.0, 4.0, 84f64.sqrt())], 2.0)
}
