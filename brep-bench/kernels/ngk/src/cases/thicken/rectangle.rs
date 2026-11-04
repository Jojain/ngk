//! A rectangular sheet thickened by 2.
use crate::{Refusal, Solid, refuse};

pub fn build() -> Result<Solid, Refusal> {
    refuse("NGK has no thickening of a sheet into a solid")
}
