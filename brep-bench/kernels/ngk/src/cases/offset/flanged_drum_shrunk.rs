//! A flanged drum shrunk until its flange is gone.
use crate::{Refusal, Solid, refuse};

pub fn build() -> Result<Solid, Refusal> {
    refuse("NGK has no offset of a solid")
}
