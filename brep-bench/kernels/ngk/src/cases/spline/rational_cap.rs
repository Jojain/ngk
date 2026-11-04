//! The region in the xz plane between the z axis from (0, 0, −5) to (0, 0, 10), the segment to
//! (5√3, 0, −5) and the rational quadratic Bézier (5√3, 0, −5), (10√3, 0, 10), (0, 0, 10) with
//! weights 1, ½, 1, turned a full turn about z.
use crate::bench::{face_from_edges, line, nurbs_edge, p, revolve_about_z};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let s = 3f64.sqrt();
    // Wound this way, NGK's tessellator turns a whole-turn face's triangles outward; the solid
    // itself is the same either way.
    let shaft = line(p(0.0, 0.0, -5.0), p(0.0, 0.0, 10.0))?;
    let floor = line(p(5.0 * s, 0.0, -5.0), p(0.0, 0.0, -5.0))?;
    let arc = nurbs_edge(
        2,
        &[p(0.0, 0.0, 10.0), p(10.0 * s, 0.0, 10.0), p(5.0 * s, 0.0, -5.0)],
        &[1.0, 0.5, 1.0],
        &[0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
    )?;
    revolve_about_z(face_from_edges(&[&shaft, &floor, &arc])?, 360.0)
}
