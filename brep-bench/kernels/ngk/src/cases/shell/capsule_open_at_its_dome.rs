//! A capsule's end shelled open at its dome: the walls run on past their face to meet it.
use crate::{Refusal, Solid, refuse};

pub fn build() -> Result<Solid, Refusal> {
    refuse("NGK has no shell: it cannot hollow a solid to a wall of constant thickness")
}
