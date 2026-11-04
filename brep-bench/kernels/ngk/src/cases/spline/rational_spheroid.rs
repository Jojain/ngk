//! The region in the xz plane between the z axis from (0, 0, −10) to (0, 0, 10) and the rational
//! quadratic B-spline (0, 0, −10), (6, 0, −10), (6, 0, 0), (6, 0, 10), (0, 0, 10), weights
//! 1, √2/2, 1, √2/2, 1, knots (0, 0, 0, ½, ½, 1, 1, 1), turned a full turn about z.
use crate::bench::{face_from_edges, line, nurbs_edge, p, revolve_about_z};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let w = 2f64.sqrt() / 2.0;
    let shaft = line(p(0.0, 0.0, -10.0), p(0.0, 0.0, 10.0))?;
    let rim = nurbs_edge(
        2,
        &[p(0.0, 0.0, 10.0), p(6.0, 0.0, 10.0), p(6.0, 0.0, 0.0), p(6.0, 0.0, -10.0), p(0.0, 0.0, -10.0)],
        &[1.0, w, 1.0, w, 1.0],
        &[0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0],
    )?;
    revolve_about_z(face_from_edges(&[&shaft, &rim])?, 360.0)
}
