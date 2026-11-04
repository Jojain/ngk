//! A torus about z of radii 10 and 3 united with the drum of radius 13 on z from z = −1 to 1,
//! tangent to it along the outer equator.
use crate::bench::{cylinder_z, fuse};
use crate::{Refusal, Solid};
use ngk::modeling::solids;

pub fn build() -> Result<Solid, Refusal> {
    fuse(solids::torus(10.0, 3.0)?, &cylinder_z(0.0, 0.0, -1.0, 1.0, 13.0)?)
}
