use crate::geometry::utils::Point3;

use super::edge::EdgeRef;
use super::gmap::{Dart, GMap};
use super::loop_::LoopRef;
use super::shell::ShellRef;

pub struct VertexRef<'a> {
    gmap: &'a GMap<'a>,
    pub dart: Dart,
}

impl<'a> VertexRef<'a> {
    pub fn new(gmap: &'a GMap, dart: Dart) -> Self {
        Self { gmap, dart }
    }

    pub fn edges(&self) -> Vec<EdgeRef<'a>> {
        self.gmap
            .incident_cells(self.dart, 0, 1)
            .map(|d| EdgeRef::new(self.gmap, d))
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

    pub fn point(&self) -> &Point3 {
        self.gmap.get_vertex_point(self.dart)
    }
}
