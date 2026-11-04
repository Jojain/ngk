//! A loft from the square −20 ≤ x, y ≤ 20 at z = 0 to the octagon at z = 30, each corner of the
//! square paired with a cut side of the octagon: a ruled loft between the two faces.
use crate::bench::p;
use crate::{Refusal, Solid};
use ngk::builders::loft::LoftOptions;
use ngk::modeling::{faces, loft};

pub fn build() -> Result<Solid, Refusal> {
    let square = faces::polygon(&[p(-20.0, -20.0, 0.0), p(20.0, -20.0, 0.0), p(20.0, 20.0, 0.0), p(-20.0, 20.0, 0.0)])?;
    let octagon = faces::polygon(&[
        p(-6.0, -14.0, 30.0),
        p(6.0, -14.0, 30.0),
        p(14.0, -6.0, 30.0),
        p(14.0, 6.0, 30.0),
        p(6.0, 14.0, 30.0),
        p(-6.0, 14.0, 30.0),
        p(-14.0, 6.0, 30.0),
        p(-14.0, -6.0, 30.0),
    ])?;
    Ok(loft::loft(&[&square, &octagon], LoftOptions::ruled())?)
}
