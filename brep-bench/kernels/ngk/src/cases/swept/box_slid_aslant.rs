//! The solid the box (0, 0, 0)–(10, 6, 4) fills sliding along (8, 5, 3). NGK has no sweep of a
//! solid; the box slid along a line is the box at each end and each face slid along the line, so
//! it is the union of the two boxes and the six faces extruded along the vector.
use crate::bench::{block, extrude, fuse, p, v};
use crate::{Refusal, Solid};
use ngk::modeling::faces;

pub fn build() -> Result<Solid, Refusal> {
    let slide = v(8.0, 5.0, 3.0);
    let start = block(p(0.0, 0.0, 0.0), p(10.0, 6.0, 4.0))?;
    let end = block(p(8.0, 5.0, 3.0), p(18.0, 11.0, 7.0))?;
    let mut swept = fuse(start, &end)?;
    let corner = |x: f64, y: f64, z: f64| p(10.0 * x, 6.0 * y, 4.0 * z);
    let quads = [
        [corner(0.0, 0.0, 0.0), corner(0.0, 1.0, 0.0), corner(0.0, 1.0, 1.0), corner(0.0, 0.0, 1.0)],
        [corner(1.0, 0.0, 0.0), corner(1.0, 1.0, 0.0), corner(1.0, 1.0, 1.0), corner(1.0, 0.0, 1.0)],
        [corner(0.0, 0.0, 0.0), corner(1.0, 0.0, 0.0), corner(1.0, 0.0, 1.0), corner(0.0, 0.0, 1.0)],
        [corner(0.0, 1.0, 0.0), corner(1.0, 1.0, 0.0), corner(1.0, 1.0, 1.0), corner(0.0, 1.0, 1.0)],
        [corner(0.0, 0.0, 0.0), corner(1.0, 0.0, 0.0), corner(1.0, 1.0, 0.0), corner(0.0, 1.0, 0.0)],
        [corner(0.0, 0.0, 1.0), corner(1.0, 0.0, 1.0), corner(1.0, 1.0, 1.0), corner(0.0, 1.0, 1.0)],
    ];
    for q in quads {
        swept = fuse(swept, &extrude(faces::polygon(&q)?, slide)?)?;
    }
    Ok(swept)
}
