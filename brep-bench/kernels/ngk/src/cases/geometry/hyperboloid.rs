//! The branch x > 0 of the hyperbola x²/16 − z²/36 = 1 in the xz plane between z = −6 and 6,
//! closed by the axis and the two ends, turned a whole turn about z. The arc is the rational
//! quadratic Bézier from (4√2, 0, 6) over the tangents' meeting (2√2, 0, 0), weight √2, to
//! (4√2, 0, −6).
use crate::bench::{conic_edge, face_from_edges, line, p, revolve_about_z};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let r = 4.0 * 2f64.sqrt();
    let shaft = line(p(0.0, 0.0, -6.0), p(0.0, 0.0, 6.0))?;
    let top = line(p(0.0, 0.0, 6.0), p(r, 0.0, 6.0))?;
    let side = conic_edge(p(r, 0.0, 6.0), p(r / 2.0, 0.0, 0.0), 2f64.sqrt(), p(r, 0.0, -6.0))?;
    let bottom = line(p(r, 0.0, -6.0), p(0.0, 0.0, -6.0))?;
    revolve_about_z(face_from_edges(&[&shaft, &top, &side, &bottom])?, 360.0)
}
