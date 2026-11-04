//! The branch z > 0 of the hyperbola z²/9 − x²/16 = 1 in the xz plane, from its vertex (0, 0, 3)
//! to x = 4√3, z = 6, closed by the disc at z = 6 and the axis, turned a whole turn about z. The
//! arc is the rational quadratic Bézier from the vertex over the tangents' meeting (4/√3, 0, 3),
//! weight √6/2, to (4√3, 0, 6).
use crate::bench::{conic_edge, face_from_edges, line, p, revolve_about_z};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let s = 3f64.sqrt();
    let shaft = line(p(0.0, 0.0, 3.0), p(0.0, 0.0, 6.0))?;
    let lid = line(p(0.0, 0.0, 6.0), p(4.0 * s, 0.0, 6.0))?;
    let side = conic_edge(p(4.0 * s, 0.0, 6.0), p(4.0 / s, 0.0, 3.0), 6f64.sqrt() / 2.0, p(0.0, 0.0, 3.0))?;
    revolve_about_z(face_from_edges(&[&shaft, &lid, &side])?, 360.0)
}
