//! An L bracket: the outline (0, 0), (20, 0), (20, 92), the quarter circle of radius 8 about
//! (28, 92) to (28, 100), then (100, 100), (100, 120), (0, 120) in xz, extruded from y = −5 to 5.
use crate::bench::{deg, extrude, face_from_edges, line, p, plane_xy, v};
use crate::{Refusal, Solid};
use ngk::modeling::edges;

pub fn build() -> Result<Solid, Refusal> {
    let at = |x: f64, z: f64| p(x, -5.0, z);
    // The round runs the way the outline does, from (20, 92) to (28, 100).
    let round = edges::arc(plane_xy(at(28.0, 92.0), v(1.0, 0.0, 0.0), v(0.0, 0.0, -1.0)), 8.0, deg(180.0), deg(270.0))?;
    let sides = [
        line(at(0.0, 0.0), at(20.0, 0.0))?,
        line(at(20.0, 0.0), at(20.0, 92.0))?,
        line(at(28.0, 100.0), at(100.0, 100.0))?,
        line(at(100.0, 100.0), at(100.0, 120.0))?,
        line(at(100.0, 120.0), at(0.0, 120.0))?,
        line(at(0.0, 120.0), at(0.0, 0.0))?,
    ];
    let mut all: Vec<_> = sides.iter().collect();
    all.push(&round);
    extrude(face_from_edges(&all)?, v(0.0, 10.0, 0.0))
}
