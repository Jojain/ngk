//! A block (0, 0, 0)–(10, 4, 6), the two edges of its top running along x chamfered 2 on both
//! faces, half the top's width: the chamfers meet in a ridge at y = 2, z = 6.
use crate::bench::{block, chamfer_through, p};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let b = block(p(0.0, 0.0, 0.0), p(10.0, 4.0, 6.0))?;
    chamfer_through(b, &[p(5.0, 0.0, 6.0), p(5.0, 4.0, 6.0)], 2.0)
}
