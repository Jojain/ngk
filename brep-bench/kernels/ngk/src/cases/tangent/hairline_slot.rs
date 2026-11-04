//! A cube of side 10 with a slot 10⁻⁶ wide, x from 5 to 5.000001, sawn from z = 5 up through the
//! top, the cutting block running past the front, back and top.
use crate::bench::{block, cut, p};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let cube = block(p(0.0, 0.0, 0.0), p(10.0, 10.0, 10.0))?;
    cut(cube, &block(p(5.0, -1.0, 5.0), p(5.000001, 11.0, 11.0))?)
}
