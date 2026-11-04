//! A flange on z: a plate, the disc of radius 32 from z = 0 to 6; a hub of radius 15 from z = 6
//! to 16; a bore of radius 8 through both; six bolt holes of radius 3 on the circle of radius
//! 24, one every 60° from (24, 0), each its own cut.
use crate::bench::{cut, cylinder_z, fuse};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let body = fuse(cylinder_z(0.0, 0.0, 0.0, 6.0, 32.0)?, &cylinder_z(0.0, 0.0, 6.0, 16.0, 15.0)?)?;
    let mut flange = cut(body, &cylinder_z(0.0, 0.0, -1.0, 17.0, 8.0)?)?;
    for i in 0..6 {
        let a = (60.0 * i as f64).to_radians();
        flange = cut(flange, &cylinder_z(24.0 * a.cos(), 24.0 * a.sin(), -1.0, 7.0, 3.0)?)?;
    }
    Ok(flange)
}
