//! Two blocks 10 high on the xy plane, x and y from 0 to 10 and from 5 to 15: their common.
use crate::bench::{block, common, p};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let first = block(p(0.0, 0.0, 0.0), p(10.0, 10.0, 10.0))?;
    let second = block(p(5.0, 5.0, 0.0), p(15.0, 15.0, 10.0))?;
    common(first, &second)
}
