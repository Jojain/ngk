//! A disc of radius 8 swept along the cubic Bézier (0, 0, 0), (12, 16, 0), (26, 8, 0),
//! (24, 0, 16), square to it, its ends flat.
use crate::bench::{bezier_edge, disc, p};
use crate::{Refusal, Solid};
use ngk::geometry::Fraction;
use ngk::modeling::sweep;

pub fn build() -> Result<Solid, Refusal> {
    let path = bezier_edge([p(0.0, 0.0, 0.0), p(12.0, 16.0, 0.0), p(26.0, 8.0, 0.0), p(24.0, 0.0, 16.0)])?;
    let t = path.edge().trimmed_curve();
    let section = disc(t.start(), t.derivative_at(Fraction::START, 1), 8.0)?;
    Ok(sweep::sweep_face(section, &path.edge(), Default::default())?)
}
