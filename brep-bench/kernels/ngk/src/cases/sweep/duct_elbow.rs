//! A hollow square duct, the square 18 × 18 with a hole 14 × 14 standing in the xz plane centred
//! on (30, 0, 0), turned a quarter turn about the z axis from +x towards +y.
use crate::bench::{axis, deg, p, plane_xy, v};
use crate::{Refusal, Solid};
use ngk::modeling::{faces, revolve};

pub fn build() -> Result<Solid, Refusal> {
    let square = |h: f64| vec![p(30.0 - h, 0.0, -h), p(30.0 + h, 0.0, -h), p(30.0 + h, 0.0, h), p(30.0 - h, 0.0, h)];
    let (outer, hole) = (square(9.0), square(7.0));
    let section = faces::polygon_with_holes(plane_xy(p(30.0, 0.0, 0.0), v(1.0, 0.0, 0.0), v(0.0, 0.0, 1.0)), &outer, &[&hole])?;
    Ok(revolve::revolve_face(section, axis(p(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0)), deg(90.0))?)
}
