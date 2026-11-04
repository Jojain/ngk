//! The common part of two cylinders of radius 5 and length 30, along x, and parallel to y on the
//! line x = 0, z = 9.99: they overlap to a depth of 0.01.
use crate::bench::{common, cylinder, p, v};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let first = cylinder(p(-15.0, 0.0, 0.0), v(1.0, 0.0, 0.0), 5.0, 30.0)?;
    let second = cylinder(p(0.0, -15.0, 9.99), v(0.0, 1.0, 0.0), 5.0, 30.0)?;
    common(first, &second)
}
