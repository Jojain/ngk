use super::edge::EdgeRef;
use super::face::Face;
use super::gmap::{Dart, GMap};
use super::shell::ShellRef;
use super::vertex::VertexRef;

#[derive(Clone)]
pub struct LoopRef<'a> {
    gmap: &'a GMap<'a>,
    pub dart: Dart,
}

impl<'a> LoopRef<'a> {
    pub fn new(gmap: &'a GMap<'a>, dart: Dart) -> Self {
        Self { gmap, dart }
    }

    pub fn darts(&self) -> impl Iterator<Item = Dart> + '_ {
        self.gmap.orbit(self.dart, self.gmap.orbit_indices(2))
    }

    pub fn vertices(&self) -> Vec<VertexRef<'a>> {
        self.gmap
            .incident_cells(self.dart, 2, 0)
            .map(|d| VertexRef::new(self.gmap, d))
            .collect()
    }

    pub fn edges(&self) -> Vec<EdgeRef<'a>> {
        self.gmap
            .incident_cells(self.dart, 2, 1)
            .map(|d| EdgeRef::new(self.gmap, d))
            .collect()
    }

    pub fn face(&self) -> &'a Face<'a> {
        self.darts()
            .find_map(|d| d.face_id) // Returns the first Some(id) it finds
            .map(|id| self.gmap.get_face(id))
            .expect("A loop should always have a face dart")
    }

    pub fn shells(&self) -> Vec<ShellRef<'a>> {
        self.gmap
            .incident_cells(self.dart, 2, 3)
            .map(|d| ShellRef::new(self.gmap, d))
            .collect()
    }
}
