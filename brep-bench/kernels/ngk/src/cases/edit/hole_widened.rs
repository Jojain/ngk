//! A plate's hole widened.
use crate::{Refusal, Solid, refuse};

pub fn build() -> Result<Solid, Refusal> {
    refuse("NGK has no face move: it cannot move faces of a finished solid and extend or trim the faces beside them")
}
