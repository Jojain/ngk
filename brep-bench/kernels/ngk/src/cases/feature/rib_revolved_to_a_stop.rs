//! A hub of radius 10 on z from z = 0 to 10, a stop (the wedge within 25 of z between 120° and
//! 135°) and a rib, the rectangle 5 ≤ x ≤ 20, 0 ≤ z ≤ 4 turned about z until it meets the stop.
//! NGK has no revolution up to a face, so the rib is turned the 120° to the stop's face.
use crate::bench::{cylinder_z, fuse, p, revolved, v};
use crate::{Refusal, Solid};
use ngk::geometry::Axis3;

pub fn build() -> Result<Solid, Refusal> {
    let z = v(0.0, 0.0, 1.0);
    // Wound this way, NGK's tessellator turns a whole-turn face's triangles outward; the solid
    // itself is the same either way.
    let wedge = revolved(&[p(0.0, 0.0, 10.0), p(25.0, 0.0, 10.0), p(25.0, 0.0, 0.0), p(0.0, 0.0, 0.0)], p(0.0, 0.0, 0.0), z, 15.0)?
        .rotated(Axis3::z(), crate::bench::deg(120.0));
    let rib = revolved(&[p(5.0, 0.0, 4.0), p(20.0, 0.0, 4.0), p(20.0, 0.0, 0.0), p(5.0, 0.0, 0.0)], p(0.0, 0.0, 0.0), z, 120.0)?;
    let part = fuse(cylinder_z(0.0, 0.0, 0.0, 10.0, 10.0)?, &wedge)?;
    fuse(part, &rib)
}
