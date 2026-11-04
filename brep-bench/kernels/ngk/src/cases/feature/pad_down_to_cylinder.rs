//! A cylinder of radius 10 along x from −20 to 20 with a pad, the square |x|, |y| ≤ 3 at z = 15
//! extruded down until it meets the cylinder, joined to it. NGK has no extrusion up to a face, so
//! the pad is extruded down to z = 0, inside the cylinder, which the union then absorbs.
use crate::bench::{block, cylinder, fuse, p, v};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let drum = cylinder(p(-20.0, 0.0, 0.0), v(1.0, 0.0, 0.0), 10.0, 40.0)?;
    fuse(drum, &block(p(-3.0, -3.0, 0.0), p(3.0, 3.0, 15.0))?)
}
