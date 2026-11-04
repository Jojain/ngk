//! A block (−6, −6, −8)–(18, 18, 0) grooved by a ball of radius 3 swung a quarter turn about z
//! from centre (12, 0, 0) to (0, 12, 0). NGK has no sweep of a solid; a ball's sweep is the tube
//! about its centre's path with a ball at each end: the disc of radius 3 about (12, 0, 0) in the
//! xz plane turned 90° about z, and the balls at (12, 0, 0) and (0, 12, 0), each its own cut.
use crate::bench::{ball, block, cut_each, p, plane_xy, revolve_about_z, v};
use crate::{Refusal, Solid};
use ngk::modeling::faces;

pub fn build() -> Result<Solid, Refusal> {
    let section = faces::circle(plane_xy(p(12.0, 0.0, 0.0), v(1.0, 0.0, 0.0), v(0.0, 0.0, 1.0)), 3.0)?;
    let tube = revolve_about_z(section, 90.0)?;
    let b = block(p(-6.0, -6.0, -8.0), p(18.0, 18.0, 0.0))?;
    cut_each(b, &[tube, ball(p(12.0, 0.0, 0.0), 3.0)?, ball(p(0.0, 12.0, 0.0), 3.0)?])
}
