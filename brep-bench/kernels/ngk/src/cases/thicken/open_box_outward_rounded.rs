//! An open box's sheet thickened outward, its edges rounded.
use crate::{Refusal, Solid, refuse};

pub fn build() -> Result<Solid, Refusal> {
    refuse("NGK has no thickening of a sheet into a solid")
}
