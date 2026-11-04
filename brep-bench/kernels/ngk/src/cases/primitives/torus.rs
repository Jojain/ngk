//! A torus about the z axis, centred on the origin: the tube's middle circle has radius 10 and
//! the tube radius 2.
use crate::{Refusal, Solid};
use ngk::modeling::solids;

pub fn build() -> Result<Solid, Refusal> {
    Ok(solids::torus(10.0, 2.0)?)
}
