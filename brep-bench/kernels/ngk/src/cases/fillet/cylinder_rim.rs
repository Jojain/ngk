//! A cylinder of radius 5 on z from z = 0 to 10, the rim of its top face rounded to radius 1.
use crate::bench::{fillet_through, p};
use crate::{Refusal, Solid};
use ngk::modeling::solids;

pub fn build() -> Result<Solid, Refusal> {
    fillet_through(solids::cylinder(5.0, 10.0)?, &[p(0.0, 5.0, 10.0)], 1.0)
}
