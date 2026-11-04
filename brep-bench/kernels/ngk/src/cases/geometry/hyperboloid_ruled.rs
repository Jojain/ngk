//! The solid between the discs of radius 5 about z at z = −5 and z = 5 and the surface ruled from
//! the lower rim's point at θ to the upper rim's at θ + 90°: a ruled loft between the two discs,
//! the upper one's circle starting a quarter turn on.
use crate::bench::{p, plane_xy, v};
use crate::{Refusal, Solid};
use ngk::builders::loft::LoftOptions;
use ngk::modeling::{faces, loft};

pub fn build() -> Result<Solid, Refusal> {
    let lower = faces::circle(plane_xy(p(0.0, 0.0, -5.0), v(1.0, 0.0, 0.0), v(0.0, 1.0, 0.0)), 5.0)?;
    let upper = faces::circle(plane_xy(p(0.0, 0.0, 5.0), v(0.0, 1.0, 0.0), v(-1.0, 0.0, 0.0)), 5.0)?;
    Ok(loft::loft(&[&lower, &upper], LoftOptions::ruled())?)
}
