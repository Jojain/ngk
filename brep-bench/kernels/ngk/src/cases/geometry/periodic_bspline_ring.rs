//! The periodic uniform quadratic B-spline on the square (13, 0, 3), (7, 0, 3), (7, 0, −3),
//! (13, 0, −3) in the xz plane, the region inside it turned a whole turn about z.
use crate::bench::{face_from_edges, p, periodic_bspline_edges, revolve_about_z};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let [a, b] = periodic_bspline_edges(2, &[p(13.0, 0.0, 3.0), p(7.0, 0.0, 3.0), p(7.0, 0.0, -3.0), p(13.0, 0.0, -3.0)])?;
    revolve_about_z(face_from_edges(&[&a, &b])?, 360.0)
}
