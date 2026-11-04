//! A cube (0, 0, 0)–(10, 10, 10) united with the cylinder of radius 5 on the vertical through
//! (10, 5), from z = 0 to z = 10, tangent to the faces y = 0 and y = 10 along the cube's edges.
use crate::bench::{block, cylinder_z, fuse, p};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let cube = block(p(0.0, 0.0, 0.0), p(10.0, 10.0, 10.0))?;
    fuse(cube, &cylinder_z(10.0, 5.0, 0.0, 10.0, 5.0)?)
}
