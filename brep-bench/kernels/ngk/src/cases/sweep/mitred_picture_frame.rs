//! A moulding swept round the rectangle (±30, ±20, 0), mitred at its four corners. The section,
//! at (s, z) outward from the path, is (0, 2), (3, 2), (3, 0), (8, 0), (8, 6), (0, 6); the closed
//! path starts at the middle of the side y = −20, where the section stands in the plane x = 0.
use crate::bench::{p, sweep_along_polyline};
use crate::{Refusal, Solid};
use ngk::builders::sweep::SweepTransition;
use ngk::modeling::faces;

pub fn build() -> Result<Solid, Refusal> {
    let at = |s: f64, z: f64| p(0.0, -20.0 - s, z);
    let section = faces::polygon(&[at(0.0, 2.0), at(3.0, 2.0), at(3.0, 0.0), at(8.0, 0.0), at(8.0, 6.0), at(0.0, 6.0)])?;
    let path = [
        p(0.0, -20.0, 0.0),
        p(30.0, -20.0, 0.0),
        p(30.0, 20.0, 0.0),
        p(-30.0, 20.0, 0.0),
        p(-30.0, -20.0, 0.0),
        p(0.0, -20.0, 0.0),
    ];
    sweep_along_polyline(section, &path, SweepTransition::Straight)
}
