//! A truncated cone on the z axis from z = 0 to z = 6, radius 5 at its base and 2 at its top:
//! NGK has no cone primitive, so its half section in the xz plane is turned a full turn about z.
use crate::bench::{axis, deg, p, v};
use crate::{Refusal, Solid};
use ngk::modeling::{faces, revolve};

pub fn build() -> Result<Solid, Refusal> {
    // Wound this way, NGK's tessellator turns a whole-turn face's triangles outward; the solid
    // itself is the same either way.
    let section = faces::polygon(&[p(0.0, 0.0, 6.0), p(2.0, 0.0, 6.0), p(5.0, 0.0, 0.0), p(0.0, 0.0, 0.0)])?;
    Ok(revolve::revolve_face(section, axis(p(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0)), deg(360.0))?)
}
