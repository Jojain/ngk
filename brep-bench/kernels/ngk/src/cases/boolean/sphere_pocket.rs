//! A block (−10, −10, −10)–(10, 10, 0) less the ball of radius 5 about the origin: a
//! hemispherical pocket in its top.
use crate::bench::{ball, block, cut, p};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    cut(block(p(-10.0, -10.0, -10.0), p(10.0, 10.0, 0.0))?, &ball(p(0.0, 0.0, 0.0), 5.0)?)
}
