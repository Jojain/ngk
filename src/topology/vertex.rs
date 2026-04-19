use crate::geometry::utils::Point3;
use crate::topology::gmap::Cell0;

use super::edge::EdgeRef;
use super::facet::FacetRef;
use super::gmap::{Dart, GMap};
use super::payload::{Payload, StandardPayload};
use super::sheet::SheetRef;

#[derive(Clone, Copy)]
pub struct VertexRef<'a, P: Payload = StandardPayload> {
    gmap: &'a GMap<'a, P>,
    pub dart: Dart,
}

impl<'a, P: Payload> VertexRef<'a, P> {
    pub fn new(gmap: &'a GMap<'a, P>, dart: Dart) -> Self {
        Self { gmap, dart }
    }

    pub fn edges(&self) -> Vec<EdgeRef<'a, P>> {
        self.gmap
            .incident_cells(self.dart, 0, 1)
            .map(|d| EdgeRef::new(self.gmap, d))
            .collect()
    }

    /// Gmap 2-cells incident to this vertex.
    pub fn facets(&self) -> Vec<FacetRef<'a, P>> {
        self.gmap
            .incident_cells(self.dart, 0, 2)
            .map(|d| FacetRef::new(self.gmap, d))
            .collect()
    }

    /// 2D sheets (⟨α₀, α₁, α₂⟩) incident to this vertex. Wrap in
    /// [`Closed::new`](super::closed::Closed::new) to promote to a shell.
    pub fn sheets(&self) -> Vec<SheetRef<'a, P>> {
        self.gmap
            .incident_cells(self.dart, 0, 3)
            .map(|d| SheetRef::new(self.gmap, d))
            .collect()
    }

    pub fn point(&self) -> Option<&Point3> {
        self.gmap.attribute::<Cell0>(self.dart).map(|v| &v.point)
    }
}
