//! A ball of radius 5 about the origin, its poles on z, less everything where x < 0: the cut runs
//! along a meridian from pole to pole.
use crate::bench::{ball, block, cut, p};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    cut(ball(p(0.0, 0.0, 0.0), 5.0)?, &block(p(-10.0, -10.0, -10.0), p(0.0, 10.0, 10.0))?)
}
