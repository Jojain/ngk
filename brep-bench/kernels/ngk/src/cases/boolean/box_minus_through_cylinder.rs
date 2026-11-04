//! A cube of side 10 between the origin and (10, 10, 10), drilled through along z at x = y = 5
//! by a cylinder of radius 3 running from z = −1 to z = 11.
use crate::bench::{block, cut, cylinder_z, p};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let cube = block(p(0.0, 0.0, 0.0), p(10.0, 10.0, 10.0))?;
    cut(cube, &cylinder_z(5.0, 5.0, -1.0, 11.0, 3.0)?)
}
