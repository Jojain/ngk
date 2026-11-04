//! A cone on the disc of radius 5 at z = 0 with apex (0, 0, 10), its side the NURBS net that is
//! linear up the axis. NGK makes no face on a given NURBS surface, so the side is its iso-u line,
//! the segment from the rim to the apex as a degree-1 NURBS, turned a whole turn about z.
use crate::bench::{face_from_edges, line, nurbs_edge, p, revolve_about_z};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let shaft = line(p(0.0, 0.0, 0.0), p(0.0, 0.0, 10.0))?;
    let side = nurbs_edge(1, &[p(0.0, 0.0, 10.0), p(5.0, 0.0, 0.0)], &[1.0, 1.0], &[0.0, 0.0, 1.0, 1.0])?;
    let floor = line(p(5.0, 0.0, 0.0), p(0.0, 0.0, 0.0))?;
    revolve_about_z(face_from_edges(&[&shaft, &side, &floor])?, 360.0)
}
