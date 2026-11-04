//! A pulley on z: a hub of radius 12 from z = −14 to 14, a rim of radius 28 from z = −9 to 9, a
//! bore of radius 6 through, and a V groove: the triangle (22, 0), (29, −5), (29, 5) in xz,
//! turned a full turn about z and taken away.
use crate::bench::{cut, cylinder_z, fuse, p, revolved, v};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let body = fuse(cylinder_z(0.0, 0.0, -14.0, 14.0, 12.0)?, &cylinder_z(0.0, 0.0, -9.0, 9.0, 28.0)?)?;
    let body = cut(body, &cylinder_z(0.0, 0.0, -15.0, 15.0, 6.0)?)?;
    // Wound this way, NGK's tessellator turns a whole-turn face's triangles outward; the solid
    // itself is the same either way.
    let groove = revolved(&[p(22.0, 0.0, 0.0), p(29.0, 0.0, 5.0), p(29.0, 0.0, -5.0)], p(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0), 360.0)?;
    cut(body, &groove)
}
