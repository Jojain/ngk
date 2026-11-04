//! A disc of radius 20 on z from z = −5 to 0, with six holes of radius 2 on the circle of radius
//! 12, the first at (12, 0) and the others turned by 60° steps, each its own cut.
use crate::bench::{cut, cylinder_z};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let mut disc = cylinder_z(0.0, 0.0, -5.0, 0.0, 20.0)?;
    for i in 0..6 {
        let a = (60.0 * i as f64).to_radians();
        disc = cut(disc, &cylinder_z(12.0 * a.cos(), 12.0 * a.sin(), -6.0, 1.0, 2.0)?)?;
    }
    Ok(disc)
}
