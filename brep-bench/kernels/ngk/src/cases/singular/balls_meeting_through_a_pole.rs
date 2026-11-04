//! The union of two balls of radius 5 about the origin and (4.8, 0, 3.6), each with its poles on
//! the vertical through its centre: the circle where they meet runs through a pole of each.
use crate::bench::{ball, fuse, p};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    fuse(ball(p(0.0, 0.0, 0.0), 5.0)?, &ball(p(4.8, 0.0, 3.6), 5.0)?)
}
