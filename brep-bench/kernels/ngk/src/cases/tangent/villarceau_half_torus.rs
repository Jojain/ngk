//! A torus about z of radii 5 and 3 cut by the bitangent plane 3x − 4z = 0, keeping z ≤ 3x/4.
//! The torus's seam is put in the plane x = 0 on the side y > 0; the cutting tool is a block
//! laid in the plane, reaching past the torus.
use crate::bench::{block, cut, frame_zx, p, v};
use crate::{Refusal, Solid};
use ngk::geometry::{Frame, Rigid};
use ngk::modeling::solids;

pub fn build() -> Result<Solid, Refusal> {
    let ring = solids::torus_at(frame_zx(p(0.0, 0.0, 0.0), v(0.0, 0.0, 1.0), v(0.0, 1.0, 0.0)), 5.0, 3.0)?;
    // The block above the plane: its bottom face in the plane, its x along (4, 0, 3).
    let above = block(p(-20.0, -20.0, 0.0), p(20.0, 20.0, 20.0))?;
    let onto = Frame::from_xz(p(0.0, 0.0, 0.0), v(4.0, 0.0, 3.0), v(-3.0, 0.0, 4.0));
    let above = above.moved(Rigid::between_frames(&Frame::xyz(), &onto));
    cut(ring, &above)
}
