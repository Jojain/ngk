//! A block (−10, −10, 0)–(10, 10, 10) united with the cone whose apex is the origin and whose
//! base is the disc of radius 5 at z = 15: the tip touches the bottom face from inside.
use crate::bench::{block, cone, fuse, p, v};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let b = block(p(-10.0, -10.0, 0.0), p(10.0, 10.0, 10.0))?;
    fuse(b, &cone(p(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0), 0.0, 5.0, 15.0)?)
}
