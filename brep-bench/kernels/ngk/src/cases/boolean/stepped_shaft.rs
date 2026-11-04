//! A cylinder of radius 5 from z = 0 to 10 and one of radius 3 from z = 10 to 20 on the z axis,
//! joined.
use crate::bench::{cylinder_z, fuse};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    fuse(cylinder_z(0.0, 0.0, 0.0, 10.0, 5.0)?, &cylinder_z(0.0, 0.0, 10.0, 20.0, 3.0)?)
}
