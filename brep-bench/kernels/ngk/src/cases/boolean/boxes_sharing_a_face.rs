//! Two cubes of side 10 side by side, (0, 0, 0)–(10, 10, 10) and (10, 0, 0)–(20, 10, 10), joined:
//! the face they share disappears.
use crate::bench::{block, fuse, p};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let first = block(p(0.0, 0.0, 0.0), p(10.0, 10.0, 10.0))?;
    let second = block(p(10.0, 0.0, 0.0), p(20.0, 10.0, 10.0))?;
    fuse(first, &second)
}
