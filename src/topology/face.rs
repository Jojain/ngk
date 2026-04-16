use super::loop_::LoopRef;
use crate::{geometry::surfaces::Surface, topology::attributes::SFaceAttr};
pub type FaceId = usize;

#[derive(Clone)]
pub struct Face<'a> {
    id: FaceId,
    attribute: SFaceAttr,
    pub loops: Vec<LoopRef<'a>>,
}

impl<'a> Face<'a> {
    pub fn new(id: FaceId, attribute: SFaceAttr, loops: Vec<LoopRef<'a>>) -> Self {
        Self {
            id,
            attribute,
            loops,
        }
    }

    pub fn outer_loop(&self) -> Option<&LoopRef<'a>> {
        self.loops.first()
    }

    pub fn surface(&self) -> &Surface {
        &self.attribute.surface
    }
    
}
