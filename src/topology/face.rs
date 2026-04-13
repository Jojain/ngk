use super::gmap::Dart;
use super::loop_::LoopRef;
pub type FaceId = usize;

#[derive(Clone)]
pub struct Face<'a> {
    id: FaceId,
    pub loops: Vec<LoopRef<'a>>,
}

impl<'a> Face<'a> {
    pub fn new(id: FaceId, loops: Vec<LoopRef<'a>>) -> Self {
        Self { id, loops }
    }

    pub fn outer_loop(&self) -> Option<&LoopRef<'a>> {
        self.loops.first()
    }
}
