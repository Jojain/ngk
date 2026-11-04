//! A ball of radius 5 about the origin, its poles on z, less the cap beyond the plane x + z = 5,
//! which runs through the pole (0, 0, 5). The cap is cut by a block laid on the plane.
use crate::bench::{ball, block, cut, p, v};
use crate::{Refusal, Solid};
use ngk::geometry::{Frame, Rigid};

pub fn build() -> Result<Solid, Refusal> {
    let beyond = block(p(-10.0, -10.0, 0.0), p(10.0, 10.0, 10.0))?;
    let onto = Frame::from_xz(p(2.5, 0.0, 2.5), v(1.0, 0.0, -1.0), v(1.0, 0.0, 1.0));
    let beyond = beyond.moved(Rigid::between_frames(&Frame::xyz(), &onto));
    cut(ball(p(0.0, 0.0, 0.0), 5.0)?, &beyond)
}
