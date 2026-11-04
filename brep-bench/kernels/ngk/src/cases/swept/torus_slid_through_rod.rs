//! A rod of radius 5 on z from z = 0 to 20 grooved by a torus, axis parallel to y, radii 6 and 2,
//! slid along z from centre (11.5, 0, −9) to (11.5, 0, 29). NGK has no sweep of a solid. A torus
//! slid along z, square to its axis, fills at each height the torus's shadow along z: the stadium
//! |x − 11.5| ≤ 6, |y| ≤ 2 with half discs of radius 2 at its ends. So its sweep is that stadium
//! extruded from z = −9 to 29, with the torus at each end; both end tori are clear of the rod.
use crate::bench::{cut, cylinder_z, extrude, stadium, v};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let swept = extrude(stadium(11.5, 0.0, -9.0, 6.0, 2.0)?, v(0.0, 0.0, 38.0))?;
    cut(cylinder_z(0.0, 0.0, 0.0, 20.0, 5.0)?, &swept)
}
