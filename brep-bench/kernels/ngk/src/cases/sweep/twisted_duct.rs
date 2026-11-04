//! A duct along y from the square A B C D at y = 0 to the diamond A′ B′ C′ D′ at y = 40, each
//! corner running straight to its namesake: a ruled loft between the two, corners in order.
use crate::bench::p;
use crate::{Refusal, Solid};
use ngk::builders::loft::LoftOptions;
use ngk::modeling::{faces, loft};

pub fn build() -> Result<Solid, Refusal> {
    let square = faces::polygon(&[p(10.0, 0.0, -10.0), p(-10.0, 0.0, -10.0), p(-10.0, 0.0, 10.0), p(10.0, 0.0, 10.0)])?;
    let diamond = faces::polygon(&[p(0.0, 40.0, -12.0), p(-12.0, 40.0, 0.0), p(0.0, 40.0, 12.0), p(12.0, 40.0, 0.0)])?;
    Ok(loft::loft(&[&square, &diamond], LoftOptions::ruled())?)
}
