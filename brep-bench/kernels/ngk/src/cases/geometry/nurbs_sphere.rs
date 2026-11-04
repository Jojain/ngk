//! A ball of radius 5 about the origin whose surface is the biquadratic NURBS sphere. NGK makes
//! no face on a given NURBS surface, so the sphere is the net's meridian, the rational quadratic
//! semicircle (0, −5), (5, −5), (5, 0), (5, 5), (0, 5) of the xz plane with weights 1, √2/2, 1,
//! √2/2, 1 on the knots (0, 0, 0, ½, ½, 1, 1, 1), turned a whole turn about z.
use crate::bench::{face_from_edges, line, nurbs_edge, p, revolve_about_z};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let w = 2f64.sqrt() / 2.0;
    let shaft = line(p(0.0, 0.0, -5.0), p(0.0, 0.0, 5.0))?;
    let meridian = nurbs_edge(
        2,
        &[p(0.0, 0.0, 5.0), p(5.0, 0.0, 5.0), p(5.0, 0.0, 0.0), p(5.0, 0.0, -5.0), p(0.0, 0.0, -5.0)],
        &[1.0, w, 1.0, w, 1.0],
        &[0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0],
    )?;
    revolve_about_z(face_from_edges(&[&shaft, &meridian])?, 360.0)
}
