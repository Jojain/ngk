//! A cylinder of radius 5 on z from z = 0 to 10, its top rim chamfered 5 on both faces: the top
//! shrinks to its centre and the chamfer is a cone of half-angle 45°.
use crate::bench::{chamfer_through, p};
use crate::{Refusal, Solid};
use ngk::modeling::solids;

pub fn build() -> Result<Solid, Refusal> {
    chamfer_through(solids::cylinder(5.0, 10.0)?, &[p(0.0, 5.0, 10.0)], 5.0)
}
