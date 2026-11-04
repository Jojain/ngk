//! The helical pipe of `sweep/helical-pipe`, each end closed by the half ball a ball rolled along
//! the helix leaves: the pipe joined to a ball of radius 1 at each end of the helix.
use crate::bench::{axis, ball, deg, disc, fuse, p, v};
use ngk::geometry::Fraction;
use crate::{Refusal, Solid};
use ngk::modeling::{edges, sweep};

pub fn build() -> Result<Solid, Refusal> {
    let path = edges::helix(axis(p(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0)), 10.0, 5.0, deg(180.0), deg(180.0 + 3.0 * 360.0))?;
    // (NGK measures a helix's angle from its axis frame's x, which for this axis is −x.)
    let start = path.edge().trimmed_curve();
    let section = disc(start.start(), start.derivative_at(Fraction::START, 1), 1.0)?;
    let pipe = sweep::sweep_face(section, &path.edge(), Default::default())?;
    let pipe = fuse(pipe, &ball(p(10.0, 0.0, 0.0), 1.0)?)?;
    fuse(pipe, &ball(p(10.0, 0.0, 15.0), 1.0)?)
}
