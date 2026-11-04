//! A dumbbell plate shelled thicker than its neck: the inside walls cross and the neck fills.
use crate::{Refusal, Solid, refuse};

pub fn build() -> Result<Solid, Refusal> {
    refuse("NGK has no shell: it cannot hollow a solid to a wall of constant thickness")
}
