//! A prism from z = 0 to 10 over the periodic uniform quadratic B-spline on the square
//! (10, 10), (−10, 10), (−10, −10), (10, −10).
use crate::bench::{extrude, face_from_edges, p, periodic_bspline_edges, v};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let [a, b] = periodic_bspline_edges(2, &[p(10.0, 10.0, 0.0), p(-10.0, 10.0, 0.0), p(-10.0, -10.0, 0.0), p(10.0, -10.0, 0.0)])?;
    extrude(face_from_edges(&[&a, &b])?, v(0.0, 0.0, 10.0))
}
