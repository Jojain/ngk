//! A torus about z, its tube's middle circle of radius 5 and its tube of radius 4.99.
use crate::{Refusal, Solid};
use ngk::modeling::solids;

pub fn build() -> Result<Solid, Refusal> {
    Ok(solids::torus(5.0, 4.99)?)
}
