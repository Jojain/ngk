//! A block (−10, −6, 0)–(10, 6, 5) with its four vertical edges rounded to radius 3, then the
//! rim of its top face, eight edges running tangent into each other, rounded to radius 1.
use crate::bench::{block, fillet_through, p};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let b = block(p(-10.0, -6.0, 0.0), p(10.0, 6.0, 5.0))?;
    let b = fillet_through(
        b,
        &[p(10.0, 6.0, 2.5), p(-10.0, 6.0, 2.5), p(-10.0, -6.0, 2.5), p(10.0, -6.0, 2.5)],
        3.0,
    )?;
    // One straight edge of the top rim: the round runs on round all eight.
    fillet_through(b, &[p(0.0, 6.0, 5.0)], 1.0)
}
