//! The common part of the cube (−5, −5, −5)–(5, 5, 5) and the ball of radius 5√2 about the origin,
//! which touches the cube's edges at their midpoints.
use crate::bench::{ball, block, common, p};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    common(block(p(-5.0, -5.0, -5.0), p(5.0, 5.0, 5.0))?, &ball(p(0.0, 0.0, 0.0), 50f64.sqrt())?)
}
