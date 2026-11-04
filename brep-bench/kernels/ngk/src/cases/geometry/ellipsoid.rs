//! The ellipsoid of semi-axes 6, 4 and 3 along x, y and z.
use crate::{Refusal, Solid, refuse};

pub fn build() -> Result<Solid, Refusal> {
    refuse("NGK has no ellipsoid surface, and its motions are rigid: no ball can be stretched into one")
}
