//! A ball of radius 5 centred on the origin.
use crate::{Refusal, Solid};
use ngk::modeling::solids;

pub fn build() -> Result<Solid, Refusal> {
    Ok(solids::sphere(5.0)?)
}
