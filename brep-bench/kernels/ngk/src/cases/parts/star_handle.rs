//! A four-armed star outline in xz extruded 12 along y with a 2° draft.
use crate::{Refusal, Solid, refuse};

pub fn build() -> Result<Solid, Refusal> {
    refuse("NGK extrudes straight; it has no draft, nor an extrusion that tapers")
}
