//! A disc of radius 2 swept along three edges of a cube of side 30, (0, 0, 0) to (30, 0, 0) to
//! (30, 30, 0) to (30, 30, 30), mitred at both corners.
use crate::bench::{disc, p, sweep_along_polyline, v};
use crate::{Refusal, Solid};
use ngk::builders::sweep::SweepTransition;

pub fn build() -> Result<Solid, Refusal> {
    let section = disc(p(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0), 2.0)?;
    let path = [p(0.0, 0.0, 0.0), p(30.0, 0.0, 0.0), p(30.0, 30.0, 0.0), p(30.0, 30.0, 30.0)];
    sweep_along_polyline(section, &path, SweepTransition::Straight)
}
