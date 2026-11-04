//! A plate (0, −5, 0)–(80, 5, 40) with a pin hole of radius 8 along y on the line x = 60, z = 20.
use crate::bench::{block, cut, cylinder, p, v};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let plate = block(p(0.0, -5.0, 0.0), p(80.0, 5.0, 40.0))?;
    cut(plate, &cylinder(p(60.0, -6.0, 20.0), v(0.0, 1.0, 0.0), 8.0, 12.0)?)
}
