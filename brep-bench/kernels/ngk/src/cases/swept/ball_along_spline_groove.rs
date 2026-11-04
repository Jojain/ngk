//! A block (−6, −6, −8)–(36, 26, 0) grooved by a ball of radius 3 whose centre follows the cubic
//! Bézier (0, 0, 0), (10, 0, 0), (20, 20, 0), (30, 20, 0). NGK has no sweep of a solid; a ball's
//! sweep is the tube about its centre's path, the disc swept along the curve, with a ball at
//! each end, each its own cut.
use crate::bench::{ball, bezier_edge, block, cut_each, disc, p};
use crate::{Refusal, Solid};
use ngk::geometry::Fraction;
use ngk::modeling::sweep;

pub fn build() -> Result<Solid, Refusal> {
    let path = bezier_edge([p(0.0, 0.0, 0.0), p(10.0, 0.0, 0.0), p(20.0, 20.0, 0.0), p(30.0, 20.0, 0.0)])?;
    let t = path.edge().trimmed_curve();
    let section = disc(t.start(), t.derivative_at(Fraction::START, 1), 3.0)?;
    let tube = sweep::sweep_face(section, &path.edge(), Default::default())?;
    let b = block(p(-6.0, -6.0, -8.0), p(36.0, 26.0, 0.0))?;
    cut_each(b, &[tube, ball(p(0.0, 0.0, 0.0), 3.0)?, ball(p(30.0, 20.0, 0.0), 3.0)?])
}
