//! A hollow square tube tapering along y from y = 0 to 40: outside 24 × 24 with a hole 20 × 20 at
//! y = 0, outside 12 × 12 with a hole 8 × 8 at y = 40, every corner running straight between
//! them: a ruled loft between the two hollow squares.
use crate::bench::{p, plane_xy, v};
use crate::{Refusal, Solid};
use ngk::builders::loft::LoftOptions;
use ngk::modeling::{faces, loft};

pub fn build() -> Result<Solid, Refusal> {
    let square = |y: f64, h: f64| vec![p(-h, y, -h), p(h, y, -h), p(h, y, h), p(-h, y, h)];
    let section = |y: f64, h: f64, k: f64| {
        faces::polygon_with_holes(plane_xy(p(0.0, y, 0.0), v(1.0, 0.0, 0.0), v(0.0, 0.0, 1.0)), &square(y, h), &[&square(y, k)])
    };
    let (foot, top) = (section(0.0, 12.0, 10.0)?, section(40.0, 6.0, 4.0)?);
    Ok(loft::loft(&[&foot, &top], LoftOptions::ruled())?)
}
