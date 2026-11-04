//! A square sheet with two holes, thickened to both sides.
use crate::{Refusal, Solid, refuse};

pub fn build() -> Result<Solid, Refusal> {
    refuse("NGK has no thickening of a sheet into a solid")
}
