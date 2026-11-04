//! A solid lofted through sections whose corners run along guide curves.
use crate::{Refusal, Solid, refuse};

pub fn build() -> Result<Solid, Refusal> {
    refuse("NGK lofts through sections alone; it has no loft along guide curves")
}
