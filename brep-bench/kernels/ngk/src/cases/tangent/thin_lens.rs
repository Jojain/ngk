//! The lens common to two balls of radius 5 about the origin and (9.999999, 0, 0), 10⁻⁶ thick.
use crate::bench::{ball, common, p};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    common(ball(p(0.0, 0.0, 0.0), 5.0)?, &ball(p(9.999999, 0.0, 0.0), 5.0)?)
}
