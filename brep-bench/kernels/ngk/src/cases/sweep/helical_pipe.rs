//! A pipe of radius 1 along the helix of radius 10 about the z axis rising 5 a turn, from
//! (10, 0, 0), right-handed, three whole turns; the section square to the path, the ends flat.
use crate::bench::{axis, deg, disc, p, v};
use ngk::geometry::Fraction;
use crate::{Refusal, Solid};
use ngk::modeling::{edges, sweep};

pub fn build() -> Result<Solid, Refusal> {
    let path = edges::helix(axis(p(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0)), 10.0, 5.0, deg(180.0), deg(180.0 + 3.0 * 360.0))?;
    // (NGK measures a helix's angle from its axis frame's x, which for this axis is −x.)
    // The section square to the helix where it starts.
    let start = path.edge().trimmed_curve();
    let section = disc(start.start(), start.derivative_at(Fraction::START, 1), 1.0)?;
    Ok(sweep::sweep_face(section, &path.edge(), Default::default())?)
}
