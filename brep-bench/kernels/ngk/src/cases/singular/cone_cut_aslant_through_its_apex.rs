//! The cone on z with base the disc of radius 5 at z = 0 and apex (0, 0, 10), less the side
//! 4x + z > 10 of the plane through its apex, cut by a block laid on the plane.
use crate::bench::{block, cone, cut, p, v};
use crate::{Refusal, Solid};
use ngk::geometry::{Frame, Rigid};

pub fn build() -> Result<Solid, Refusal> {
    let c = cone(p(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0), 5.0, 0.0, 10.0)?;
    let beyond = block(p(-20.0, -20.0, 0.0), p(20.0, 20.0, 20.0))?;
    let onto = Frame::from_xz(p(0.0, 0.0, 10.0), v(1.0, 0.0, -4.0), v(4.0, 0.0, 1.0));
    cut(c, &beyond.moved(Rigid::between_frames(&Frame::xyz(), &onto)))
}
