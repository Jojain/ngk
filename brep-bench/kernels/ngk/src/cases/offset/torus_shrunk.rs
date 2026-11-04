//! A torus shrunk by 1.5.
use crate::{Refusal, Solid, refuse};

pub fn build() -> Result<Solid, Refusal> {
    refuse("NGK has no offset of a solid")
}
