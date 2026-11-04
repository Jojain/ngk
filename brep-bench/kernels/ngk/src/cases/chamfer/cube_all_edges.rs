//! A cube of side 10, all twelve edges chamfered 2 on both faces and each corner cut flat by
//! the triangle through the three square corners nearest it: the edges and the corners are
//! chamfered together.
use crate::{Refusal, Solid};
use ngk::builders::blend::{BlendSelection, BlendTarget};
use ngk::modeling::{blend, solids};

pub fn build() -> Result<Solid, Refusal> {
    let cube = solids::block(10.0, 10.0, 10.0)?;
    let mut target = BlendTarget::new();
    for e in cube.solid().edges() {
        target = target.with(BlendSelection::Edge(e.key()));
    }
    for corner in cube.solid().vertices() {
        target = target.with(BlendSelection::Vertex(corner.key()));
    }
    Ok(blend::chamfered(cube, target, 2.0)?)
}
