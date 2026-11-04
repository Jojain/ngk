//! A plate (0, 0, 0)–(20, 10, 2), the two edges of its face y = 10 running along x rounded to
//! radius 0.9999, leaving a strip 0.0002 tall between them.
use crate::bench::{block, fillet_through, p};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let b = block(p(0.0, 0.0, 0.0), p(20.0, 10.0, 2.0))?;
    fillet_through(b, &[p(10.0, 10.0, 0.0), p(10.0, 10.0, 2.0)], 0.9999)
}
