//! A section in xz, (0, 0), (20, 0), (20, 60), (16, 60), the quarter circle of radius 4 about
//! (16, 64) to (12, 64), then (12, 160), (0, 160) and down the axis, turned a full turn about z.
use crate::bench::{deg, face_from_edges, line, p, plane_xy, revolve_about_z, v};
use crate::{Refusal, Solid};
use ngk::modeling::edges;

pub fn build() -> Result<Solid, Refusal> {
    let at = |x: f64, z: f64| p(x, 0.0, z);
    let round = edges::arc(plane_xy(at(16.0, 64.0), v(1.0, 0.0, 0.0), v(0.0, 0.0, 1.0)), 4.0, deg(180.0), deg(270.0))?;
    // Wound this way, NGK's tessellator turns a whole-turn face's triangles outward; the solid
    // itself is the same either way.
    let sides = [
        line(at(0.0, 0.0), at(0.0, 160.0))?,
        line(at(0.0, 160.0), at(12.0, 160.0))?,
        line(at(12.0, 160.0), at(12.0, 64.0))?,
        line(at(16.0, 60.0), at(20.0, 60.0))?,
        line(at(20.0, 60.0), at(20.0, 0.0))?,
        line(at(20.0, 0.0), at(0.0, 0.0))?,
    ];
    let mut all: Vec<_> = sides.iter().collect();
    all.push(&round);
    revolve_about_z(face_from_edges(&all)?, 360.0)
}
