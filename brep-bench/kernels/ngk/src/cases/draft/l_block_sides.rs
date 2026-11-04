//! An L-block with its sides drafted, about a concave edge.
use crate::{Refusal, Solid, refuse};

pub fn build() -> Result<Solid, Refusal> {
    refuse("NGK has no draft: it cannot turn faces about a neutral plane and trim their neighbours")
}
