//! The spindle torus about z of radii 3 and 5, counted once: the part x ≥ 0 of the disc of radius
//! 5 about (3, 0, 0) in the xz plane, between the z axis from (0, 0, −4) to (0, 0, 4) and the
//! outer arc through (8, 0, 0), turned a whole turn about z.
use crate::bench::{deg, face_from_edges, line, p, plane_xy, revolve_about_z, v};
use crate::{Refusal, Solid};
use ngk::modeling::edges;

pub fn build() -> Result<Solid, Refusal> {
    let a = (4f64).atan2(-3.0).to_degrees();
    let xz = plane_xy(p(3.0, 0.0, 0.0), v(1.0, 0.0, 0.0), v(0.0, 0.0, 1.0));
    let arc = edges::arc(xz, 5.0, deg(-a), deg(a))?;
    let chord = line(p(0.0, 0.0, 4.0), p(0.0, 0.0, -4.0))?;
    revolve_about_z(face_from_edges(&[&chord, &arc])?, 360.0)
}
