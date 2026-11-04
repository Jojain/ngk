//! The region in the xz plane bounded by the z axis from (0, 0, 0) to (0, 0, 10), the x axis to
//! (9, 0, 0) and the cubic Bézier (9, 0, 0), (12, 0, 4), (9, 0, 10), (0, 0, 10), turned a full
//! turn about z.
use crate::bench::{bezier_edge, face_from_edges, line, p, revolve_about_z};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    // Wound this way, NGK's tessellator turns a whole-turn face's triangles outward; the solid
    // itself is the same either way.
    let shaft = line(p(0.0, 0.0, 0.0), p(0.0, 0.0, 10.0))?;
    let floor = line(p(9.0, 0.0, 0.0), p(0.0, 0.0, 0.0))?;
    let dome = bezier_edge([p(0.0, 0.0, 10.0), p(9.0, 0.0, 10.0), p(12.0, 0.0, 4.0), p(9.0, 0.0, 0.0)])?;
    revolve_about_z(face_from_edges(&[&shaft, &floor, &dome])?, 360.0)
}
