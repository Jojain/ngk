//! The torus about z of radii 10 and 2 with everything where x < 0 cut away by a block.
use crate::bench::{block, cut, p};
use crate::{Refusal, Solid};
use ngk::modeling::solids;

pub fn build() -> Result<Solid, Refusal> {
    let ring = solids::torus(10.0, 2.0)?;
    cut(ring, &block(p(-20.0, -20.0, -5.0), p(0.0, 20.0, 5.0))?)
}
