//! A cube of side 10, the edge x = 10, y = 10 rounded with a radius growing from 1 at z = 0 to
//! 2 at z = 10.
use crate::{Refusal, Solid, refuse};

pub fn build() -> Result<Solid, Refusal> {
    refuse("NGK's fillet takes one constant radius; it has no variable-radius law")
}
