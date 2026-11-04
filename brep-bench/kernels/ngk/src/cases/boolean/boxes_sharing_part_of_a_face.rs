//! A cube (0, 0, 0)–(10, 10, 10) joined to a cube (5, 0, 10)–(15, 10, 20) standing on half its top.
use crate::bench::{block, fuse, p};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let first = block(p(0.0, 0.0, 0.0), p(10.0, 10.0, 10.0))?;
    let second = block(p(5.0, 0.0, 10.0), p(15.0, 10.0, 20.0))?;
    fuse(first, &second)
}
