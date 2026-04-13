use super::gmap::{Dart, GMap};
use super::loop_::LoopRef;
use super::vertex::VertexRef;

pub struct EdgeRef<'a> {
    gmap: &'a GMap<'a>,
    dart: Dart,
}

impl<'a> EdgeRef<'a> {
    pub fn new(gmap: &'a GMap, dart: Dart) -> Self {
        Self { gmap, dart }
    }

    pub fn darts(&self) -> impl Iterator<Item = Dart> + '_ {
        self.gmap.orbit(self.dart, self.gmap.orbit_indices(1))
    }

    pub fn start_vertex(&self) -> VertexRef<'a> {
        VertexRef::new(self.gmap, self.dart)
    }
    pub fn end_vertex(&self) -> VertexRef<'a> {
        VertexRef::new(self.gmap, self.gmap.alpha(0, self.dart))
    }
    pub fn vertices(&self) -> Vec<VertexRef<'a>> {
        self.gmap
            .incident_cells(self.dart, 1, 0)
            .map(|d| VertexRef::new(self.gmap, d))
            .collect()
    }

    pub fn loops(&self) -> Vec<LoopRef<'a>> {
        self.gmap
            .incident_cells(self.dart, 0, 2)
            .map(|d| LoopRef::new(self.gmap, d))
            .collect()
    }

    pub fn shells(&self) -> Vec<ShellRef<'a>> {
        self.gmap
            .incident_cells(self.dart, 0, 3)
            .map(|d| ShellRef::new(self.gmap, d))
            .collect()
    }
}
