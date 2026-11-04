//! A rod of radius 5 on z from z = 0 to 20 grooved by two beads on one axis parallel to y, slid
//! 40 along z from below the rod to above it: the first bead at (11.5, 0, −9) of radii 6 and 2,
//! the second at (11.5, 1.5, −9) of radii 6 and 1.5. NGK has no sweep of a solid. Each torus slid
//! square to its axis fills its shadow along z, a stadium: |x − 11.5| ≤ 6 with half discs of
//! radius 2 about y = 0, and of radius 1.5 about y = 1.5. The wheel's sweep is the two stadiums
//! extruded 40 along z, with the beads at each end, all clear of the rod; each is its own cut.
use crate::bench::{cut_each, cylinder_z, extrude, stadium, v};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let first = extrude(stadium(11.5, 0.0, -9.0, 6.0, 2.0)?, v(0.0, 0.0, 40.0))?;
    let second = extrude(stadium(11.5, 1.5, -9.0, 6.0, 1.5)?, v(0.0, 0.0, 40.0))?;
    cut_each(cylinder_z(0.0, 0.0, 0.0, 20.0, 5.0)?, &[first, second])
}
