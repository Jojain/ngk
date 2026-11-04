//! Two cylinders of radius 5 and length 30 crossing at right angles, along x and along y, each
//! from −15 to 15: their union.
use crate::bench::{cylinder, fuse, p, v};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let first = cylinder(p(-15.0, 0.0, 0.0), v(1.0, 0.0, 0.0), 5.0, 30.0)?;
    let second = cylinder(p(0.0, -15.0, 0.0), v(0.0, 1.0, 0.0), 5.0, 30.0)?;
    fuse(first, &second)
}
