//! The parabola x² = 8z of the xz plane from its vertex to (8, 0, 8), closed by the disc at z = 8
//! and the axis, turned a whole turn about z. The parabola is the quadratic Bézier (0, 0, 0),
//! (4, 0, 0), (8, 0, 8).
use crate::bench::{conic_edge, face_from_edges, line, p, revolve_about_z};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let shaft = line(p(0.0, 0.0, 0.0), p(0.0, 0.0, 8.0))?;
    let lid = line(p(0.0, 0.0, 8.0), p(8.0, 0.0, 8.0))?;
    let side = conic_edge(p(8.0, 0.0, 8.0), p(4.0, 0.0, 0.0), 1.0, p(0.0, 0.0, 0.0))?;
    revolve_about_z(face_from_edges(&[&shaft, &lid, &side])?, 360.0)
}
