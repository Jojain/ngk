//! A rod of radius 5 on z from z = 0 to 20 grooved by a ball of radius 1 whose centre follows the
//! helix of radius 5 from (5, 0, −2), right-handed, rising 4 a turn, six turns. NGK has no sweep
//! of a solid; a ball's sweep is the tube about its centre's path, the disc swept along the
//! helix, with a ball at each end, here clear of the rod.
use crate::bench::{axis, cut, cylinder_z, deg, disc, p, v};
use crate::{Refusal, Solid};
use ngk::geometry::Fraction;
use ngk::modeling::{edges, sweep};

pub fn build() -> Result<Solid, Refusal> {
    // NGK measures a helix's angle from its axis frame's x, which for this axis is −x.
    let path = edges::helix(axis(p(0.0, 0.0, -2.0), v(0.0, 0.0, 1.0)), 5.0, 4.0, deg(180.0), deg(180.0 + 6.0 * 360.0))?;
    let t = path.edge().trimmed_curve();
    let section = disc(t.start(), t.derivative_at(Fraction::START, 1), 1.0)?;
    let tube = sweep::sweep_face(section, &path.edge(), Default::default())?;
    cut(cylinder_z(0.0, 0.0, 0.0, 20.0, 5.0)?, &tube)
}
