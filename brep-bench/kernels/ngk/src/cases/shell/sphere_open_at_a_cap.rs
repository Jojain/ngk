//! A ball cut by a plane and shelled open there: the rim meets the walls aslant.
use crate::{Refusal, Solid, refuse};

pub fn build() -> Result<Solid, Refusal> {
    refuse("NGK has no shell: it cannot hollow a solid to a wall of constant thickness")
}
