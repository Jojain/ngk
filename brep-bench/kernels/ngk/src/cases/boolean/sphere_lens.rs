//! The lens common to two balls of radius 5 about the origin and (6, 0, 0).
use crate::bench::{ball, common, p};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    common(ball(p(0.0, 0.0, 0.0), 5.0)?, &ball(p(6.0, 0.0, 0.0), 5.0)?)
}
