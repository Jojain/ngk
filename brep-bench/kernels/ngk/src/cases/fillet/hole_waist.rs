//! A plate (−10, −10, 0)–(10, 10, 2) with a hole of radius 3 on the z axis, both rims rounded to
//! radius 1, half the thickness: the rounds meet at the waist and the hole's wall is gone.
use crate::bench::{block, cut, cylinder_z, fillet_through, p};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let plate = cut(block(p(-10.0, -10.0, 0.0), p(10.0, 10.0, 2.0))?, &cylinder_z(0.0, 0.0, -1.0, 3.0, 3.0)?)?;
    fillet_through(plate, &[p(0.0, 3.0, 2.0), p(0.0, 3.0, 0.0)], 1.0)
}
