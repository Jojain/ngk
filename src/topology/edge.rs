use super::facet::FacetRef;
use super::gmap::{Dart, GMap};
use super::payload::{Payload, StandardPayload};
use super::sheet::SheetRef;
use super::vertex::VertexRef;

pub struct EdgeRef<'a, P: Payload = StandardPayload> {
    gmap: &'a GMap<'a, P>,
    pub dart: Dart,
}

impl<'a, P: Payload> EdgeRef<'a, P> {
    pub fn new(gmap: &'a GMap<'a, P>, dart: Dart) -> Self {
        Self { gmap, dart }
    }

    pub fn darts(&self) -> impl Iterator<Item = Dart> + '_ {
        self.gmap.orbit(self.dart, self.gmap.orbit_indices(1))
    }

    pub fn start(&self) -> VertexRef<'a, P> {
        VertexRef::new(self.gmap, self.dart)
    }

    pub fn end(&self) -> VertexRef<'a, P> {
        VertexRef::new(self.gmap, self.gmap.alpha(0, self.dart))
    }

    pub fn vertices(&self) -> Vec<VertexRef<'a, P>> {
        self.gmap
            .incident_cells(self.dart, 1, 0)
            .map(|d| VertexRef::new(self.gmap, d))
            .collect()
    }

    /// Gmap 2-cells incident to this edge.
    pub fn facets(&self) -> Vec<FacetRef<'a, P>> {
        self.gmap
            .incident_cells(self.dart, 1, 2)
            .map(|d| FacetRef::new(self.gmap, d))
            .collect()
    }

    /// 2D sheets (⟨α₀, α₁, α₂⟩) incident to this edge. Wrap in
    /// [`Closed::new`](super::closed::Closed::new) to promote to a shell.
    pub fn sheets(&self) -> Vec<SheetRef<'a, P>> {
        self.gmap
            .incident_cells(self.dart, 1, 3)
            .map(|d| SheetRef::new(self.gmap, d))
            .collect()
    }
}
