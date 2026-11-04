//! A box (0, 0, 0)–(20, 10, 6), the four edges round its top chamfered 1 across the top and 2
//! down the sides.
use crate::{Refusal, Solid, refuse};

pub fn build() -> Result<Solid, Refusal> {
    refuse("NGK's chamfer sets both rails back one distance; it has no chamfer with two distances")
}
