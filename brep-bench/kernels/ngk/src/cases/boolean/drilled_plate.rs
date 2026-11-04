//! A plate (0, 0, 0)–(100, 20, 10) drilled along z by nine holes of radius 2 at y = 10 and
//! x = 10, 20, … 90, each hole its own cut.
use crate::bench::{block, cut, cylinder_z, p};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let mut plate = block(p(0.0, 0.0, 0.0), p(100.0, 20.0, 10.0))?;
    for i in 1..=9 {
        plate = cut(plate, &cylinder_z(10.0 * i as f64, 10.0, -1.0, 11.0, 2.0)?)?;
    }
    Ok(plate)
}
