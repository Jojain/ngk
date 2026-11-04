//! A box shelled nearly to half its width, leaving a cavity 0.02 wide.
use crate::{Refusal, Solid, refuse};

pub fn build() -> Result<Solid, Refusal> {
    refuse("NGK has no shell: it cannot hollow a solid to a wall of constant thickness")
}
