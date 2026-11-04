//! A cone shelled open at its base, its apex moved in.
use crate::{Refusal, Solid, refuse};

pub fn build() -> Result<Solid, Refusal> {
    refuse("NGK has no shell: it cannot hollow a solid to a wall of constant thickness")
}
