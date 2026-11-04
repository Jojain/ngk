//! The union of two balls of radius 5 about the origin and (6, 0, 0).
use crate::bench::{ball, fuse, p};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    fuse(ball(p(0.0, 0.0, 0.0), 5.0)?, &ball(p(6.0, 0.0, 0.0), 5.0)?)
}
