//! A cube (−5, −5, 0)–(5, 5, 10) united with a copy of itself turned 0.001° about the z axis.
use crate::bench::{block, deg, fuse, p};
use crate::{Refusal, Solid};
use ngk::geometry::Axis3;

pub fn build() -> Result<Solid, Refusal> {
    let cube = block(p(-5.0, -5.0, 0.0), p(5.0, 5.0, 10.0))?;
    let turned = block(p(-5.0, -5.0, 0.0), p(5.0, 5.0, 10.0))?.rotated(Axis3::z(), deg(0.001));
    fuse(cube, &turned)
}
