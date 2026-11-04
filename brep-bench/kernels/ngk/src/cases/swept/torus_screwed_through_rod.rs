//! A solid swept by a tool moving with more than a translation, whose walls are envelopes.
use crate::{Refusal, Solid, refuse};

pub fn build() -> Result<Solid, Refusal> {
    refuse("NGK has no sweep of a solid, and this sweep's walls are envelopes it cannot build from its other operations")
}
