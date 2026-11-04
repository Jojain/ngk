//! A loft from the square −20 ≤ x, y ≤ 20 at z = 0 to the circle of radius 15 at z = 30: a ruled
//! loft between the two faces.
use crate::bench::{p, plane};
use crate::{Refusal, Solid};
use ngk::builders::loft::LoftOptions;
use ngk::modeling::{faces, loft};

pub fn build() -> Result<Solid, Refusal> {
    let square = faces::polygon(&[p(-20.0, -20.0, 0.0), p(20.0, -20.0, 0.0), p(20.0, 20.0, 0.0), p(-20.0, 20.0, 0.0)])?;
    let round = faces::circle(plane(p(0.0, 0.0, 30.0), nalgebra::Vector3::z()), 15.0)?;
    Ok(loft::loft(&[&square, &round], LoftOptions::ruled())?)
}
