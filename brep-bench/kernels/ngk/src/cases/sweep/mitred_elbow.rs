//! A square bar 10 × 10 swept from (0, 0, 0) along +x to (40, 0, 0), then 40 on to
//! (20, 20√3, 0), mitred at the corner in the plane that bisects it.
use crate::bench::{p, sweep_along_polyline};
use crate::{Refusal, Solid};
use ngk::builders::sweep::SweepTransition;
use ngk::modeling::faces;

pub fn build() -> Result<Solid, Refusal> {
    let section = faces::polygon(&[p(0.0, -5.0, -5.0), p(0.0, 5.0, -5.0), p(0.0, 5.0, 5.0), p(0.0, -5.0, 5.0)])?;
    let path = [p(0.0, 0.0, 0.0), p(40.0, 0.0, 0.0), p(20.0, 20.0 * 3f64.sqrt(), 0.0)];
    sweep_along_polyline(section, &path, SweepTransition::Straight)
}
