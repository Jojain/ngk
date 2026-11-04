//! A plate (−20, −20, 0)–(20, 20, 4) with a boss, the cylinder of radius 5 on z from z = 4 to 8.
//! Its rim (convex) and its root (concave) are both rounded to radius 2, meeting at z = 6.
use crate::bench::{block, cylinder_z, fillet_through, fuse, p};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let plate = block(p(-20.0, -20.0, 0.0), p(20.0, 20.0, 4.0))?;
    let part = fuse(plate, &cylinder_z(0.0, 0.0, 4.0, 8.0, 5.0)?)?;
    fillet_through(part, &[p(0.0, 5.0, 8.0), p(0.0, 5.0, 4.0)], 2.0)
}
