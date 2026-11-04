//! A block −12 ≤ x ≤ 12, −8 ≤ y ≤ 8, 0 ≤ z ≤ 8 bored along z by the elliptic hole of semi-axes 6
//! and 3, cut with an elliptic cylinder from z = −1 to 9.
use crate::bench::{block, cut, ellipse_face, extrude, p, v};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let drill = extrude(ellipse_face(p(0.0, 0.0, -1.0), 6.0, 3.0)?, v(0.0, 0.0, 10.0))?;
    cut(block(p(-12.0, -8.0, 0.0), p(12.0, 8.0, 8.0))?, &drill)
}
