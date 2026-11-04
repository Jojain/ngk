//! A block (0, 0, 0)–(10, 4, 6), the two edges of its top running along x rounded to radius 2,
//! half the top's width: the top vanishes and becomes a half cylinder.
use crate::bench::{block, fillet_through, p};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let b = block(p(0.0, 0.0, 0.0), p(10.0, 4.0, 6.0))?;
    fillet_through(b, &[p(5.0, 0.0, 6.0), p(5.0, 4.0, 6.0)], 2.0)
}
