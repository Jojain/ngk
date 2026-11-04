//! A ball of radius 20 about the origin bored parallel to y by a hole of radius 6 on the line
//! x = 5, z = 0, the drill running from y = −30 to y = 30.
use crate::bench::{ball, cut, cylinder, p, v};
use crate::{Refusal, Solid};

pub fn build() -> Result<Solid, Refusal> {
    let drill = cylinder(p(5.0, -30.0, 0.0), v(0.0, 1.0, 0.0), 6.0, 60.0)?;
    cut(ball(p(0.0, 0.0, 0.0), 20.0)?, &drill)
}
