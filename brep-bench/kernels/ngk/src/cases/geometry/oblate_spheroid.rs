//! The half x ≥ 0 of the ellipse x²/36 + z²/9 ≤ 1 in the xz plane, turned a whole turn about z.
use crate::bench::{curve_edge, face_from_edges, line, p, v};
use crate::bench::revolve_about_z;
use crate::{Refusal, Solid};
use ngk::geometry::{Curve, Ellipse, Frame};

pub fn build() -> Result<Solid, Refusal> {
    // The ellipse in the xz plane, its major axis along x; its right half runs from (0, 0, −3)
    // through (6, 0, 0) to (0, 0, 3).
    let e = Ellipse::new(Frame::from_xy(p(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), v(0.0, 0.0, 1.0)), 6.0, 3.0);
    let rim = curve_edge(p(0.0, 0.0, -3.0), p(0.0, 0.0, 3.0), Curve::Ellipse(e))?;
    let shaft = line(p(0.0, 0.0, 3.0), p(0.0, 0.0, -3.0))?;
    revolve_about_z(face_from_edges(&[&rim, &shaft])?, 360.0)
}
