//! A ball of radius 10 about the origin and the cylinder of radius 5 on the line x = 5, y = 0
//! from z = −12 to 12, which touches the sphere from inside at (10, 0, 0): their cut. The
//! ball's poles are on the y axis and its seam through (−10, 0, 0); the cylinder's seam is on
//! the line x = 0, y = 0.
use crate::bench::{cut, frame_zx, p, v};
use crate::{Refusal, Solid};
use ngk::modeling::solids;

pub fn build() -> Result<Solid, Refusal> {
    let ball = solids::sphere_at(frame_zx(p(0.0, 0.0, 0.0), v(0.0, 1.0, 0.0), v(-1.0, 0.0, 0.0)), 10.0)?;
    let drum = solids::cylinder_at(frame_zx(p(5.0, 0.0, -12.0), v(0.0, 0.0, 1.0), v(-1.0, 0.0, 0.0)), 5.0, 24.0)?;
    cut(ball, &drum)
}
