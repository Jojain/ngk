//! A block (−36, 0, −24)–(36, 16, 24) less the pocket (−33, 0, −21)–(33, 13, 21) opening at
//! y = 0, joined to four standoffs: tubes along y of radii 5 and 2 from y = 5 to 13, standing on
//! the floor at (x, z) = (±26, ±14). Each standoff is the annulus at y = 5 extruded 8 along +y,
//! its hole blind against the floor.
use crate::bench::{block, cut, extrude, fuse, p, plane_xy, v};
use crate::{Refusal, Solid};
use ngk::modeling::faces;

pub fn build() -> Result<Solid, Refusal> {
    let shell = block(p(-36.0, 0.0, -24.0), p(36.0, 16.0, 24.0))?;
    // The pocket's box runs 1 past the opening face.
    let mut tray = cut(shell, &block(p(-33.0, -1.0, -21.0), p(33.0, 13.0, 21.0))?)?;
    for (x, z) in [(26.0, 14.0), (-26.0, 14.0), (-26.0, -14.0), (26.0, -14.0)] {
        let ring = faces::annulus(plane_xy(p(x, 5.0, z), v(1.0, 0.0, 0.0), v(0.0, 0.0, -1.0)), 5.0, 2.0)?;
        tray = fuse(tray, &extrude(ring, v(0.0, 8.0, 0.0))?)?;
    }
    Ok(tray)
}
