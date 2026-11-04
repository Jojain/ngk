//! A cylinder of radius 5 and height 10 on the z axis, from z = 0 to z = 10.
use crate::{Refusal, Solid};
use ngk::modeling::solids;

pub fn build() -> Result<Solid, Refusal> {
    Ok(solids::cylinder(5.0, 10.0)?)
}
