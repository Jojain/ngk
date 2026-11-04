//! A cylinder of radius 5 on z from z = 0 to 10, bored along its axis by a hole of radius 3 cut
//! with a cylinder from z = −1 to 11.
use crate::bench::{cut, cylinder_z};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    cut(cylinder_z(0.0, 0.0, 0.0, 10.0, 5.0)?, &cylinder_z(0.0, 0.0, -1.0, 11.0, 3.0)?)
}
