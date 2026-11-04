//! The common part of a cube of side 10 and the ball of radius 5 inscribed in it.
use crate::bench::{ball, block, common, p};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    common(block(p(0.0, 0.0, 0.0), p(10.0, 10.0, 10.0))?, &ball(p(5.0, 5.0, 5.0), 5.0)?)
}
