use super::edge::EdgeRef;
use super::face::Face;
use super::gmap::{Dart, GMap};
use super::loop_::LoopRef;
use super::vertex::VertexRef;

#[derive(Clone)]
pub struct ShellRef<'a> {
    gmap: &'a GMap<'a>,
    pub dart: Dart,
}

impl<'a> ShellRef<'a> {
    pub fn new(gmap: &'a GMap, dart: Dart) -> Self {
        Self { gmap, dart }
    }

    pub fn vertices(&self) -> Vec<VertexRef<'a>> {
        self.gmap
            .orbit(self.dart, self.gmap.orbit_indices(0))
            .map(|dart| VertexRef::new(self.gmap, dart))
            .collect()
    }

    pub fn edges(&self) -> Vec<EdgeRef<'a>> {
        self.gmap
            .orbit(self.dart, self.gmap.orbit_indices(1))
            .map(|dart| EdgeRef::new(self.gmap, dart))
            .collect()
    }

    pub fn loops(&self) -> Vec<LoopRef<'a>> {
        self.gmap
            .orbit(self.dart, self.gmap.orbit_indices(2))
            .map(|dart| LoopRef::new(self.gmap, dart))
            .collect()
    }

    pub fn faces(&self) -> Vec<&Face<'a>> {
        self.loops().iter().map(|loop_| loop_.face()).collect()
    }
}
