//! The octahedron |x| + |y| + |z| ≤ 10, all twelve edges chamfered √6 on both faces and each
//! corner cut flat by the square x = 6 and the like.
use crate::{Refusal, Solid, refuse};

pub fn build() -> Result<Solid, Refusal> {
    refuse("NGK cuts a solid corner only where three edges meet, by the triangle through the points along them; it has no flat cut of a corner where four chamfers meet")
}
