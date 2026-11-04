//! A ball of radius 5 about the origin with everything below the xy plane cut away by a block.
use crate::bench::{ball, block, cut, p};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    cut(ball(p(0.0, 0.0, 0.0), 5.0)?, &block(p(-10.0, -10.0, -10.0), p(10.0, 10.0, 0.0))?)
}
