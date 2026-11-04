//! A disc of radius 3 swept from (−30, 0, 0) along +x to the origin, then along +y to (0, 30, 0),
//! carried round the corner by turning it about the z axis.
use crate::bench::{disc, p, sweep_along_polyline, v};
use crate::{Refusal, Solid};
use ngk::builders::sweep::SweepTransition;

pub fn build() -> Result<Solid, Refusal> {
    let section = disc(p(-30.0, 0.0, 0.0), v(1.0, 0.0, 0.0), 3.0)?;
    let path = [p(-30.0, 0.0, 0.0), p(0.0, 0.0, 0.0), p(0.0, 30.0, 0.0)];
    sweep_along_polyline(section, &path, SweepTransition::Rounded)
}
