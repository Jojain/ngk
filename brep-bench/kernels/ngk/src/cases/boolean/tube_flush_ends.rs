//! A cylinder of radius 5 on z from z = 0 to 10, less one of radius 3 on the same axis running
//! exactly from z = 0 to 10: the tool's ends lie in the target's.
use crate::bench::{cut, cylinder_z};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    cut(cylinder_z(0.0, 0.0, 0.0, 10.0, 5.0)?, &cylinder_z(0.0, 0.0, 0.0, 10.0, 3.0)?)
}
