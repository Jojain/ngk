//! Two cylinders of radius 5 on z, from z = 0 to 10 and from 6 to 16, united; the second turned a
//! quarter turn about z so the seams of the coincident walls differ.
use crate::bench::{cylinder_z, frame_zx, fuse, p, v};
use crate::{Refusal, Solid};
use ngk::modeling::solids;

pub fn build() -> Result<Solid, Refusal> {
    let second = solids::cylinder_at(frame_zx(p(0.0, 0.0, 6.0), v(0.0, 0.0, 1.0), v(0.0, 1.0, 0.0)), 5.0, 10.0)?;
    fuse(cylinder_z(0.0, 0.0, 0.0, 10.0, 5.0)?, &second)
}
