//! A bent sheet-metal flange thickened outside its bend.
use crate::{Refusal, Solid, refuse};

pub fn build() -> Result<Solid, Refusal> {
    refuse("NGK has no thickening of a sheet into a solid")
}
