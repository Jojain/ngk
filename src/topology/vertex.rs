use crate::geometry::utils::Point3;
use crate::topology::gmap::{Cell0, Dim};
use crate::topology::shape_keys::VertexKey;

use super::edge::Edge;
use super::facet::Facet;
use super::gmap::{Dart, GMap};
use super::payload::{Payload, StandardPayload};
use super::sheet::Sheet;

#[derive(Clone, Copy)]
pub struct Vertex<'a, P: Payload = StandardPayload> {
    pub dart: Dart,
    gmap: &'a GMap<P>,
}

impl<'a, P: Payload> Vertex<'a, P> {
    pub fn new(gmap: &'a GMap<P>, dart: Dart) -> Self {
        Self { gmap, dart }
    }

    pub fn key(&self) -> Option<&VertexKey> {
        self.gmap.dart_to_vertex.get(&self.dart)
    }

    pub fn edges(&self) -> Vec<Edge<'a, P>> {
        self.gmap
            .incident_cells(self.dart, Dim::Zero, Dim::One)
            .map(|d| Edge::new(self.gmap, d))
            .collect()
    }

    /// Gmap 2-cells incident to this vertex.
    pub fn facets(&self) -> Vec<Facet<'a, P>> {
        self.gmap
            .incident_cells(self.dart, Dim::Zero, Dim::Two)
            .map(|d| Facet::new(self.gmap, d))
            .collect()
    }

    /// 2D sheets (⟨α₀, α₁, α₂⟩) incident to this vertex. Wrap in
    /// [`Closed::new`](super::closed::Closed::new) to promote to a shell.
    pub fn sheets(&self) -> Vec<Sheet<'a, P>> {
        self.gmap
            .incident_cells(self.dart, Dim::Zero, Dim::Three)
            .map(|d| Sheet::new(self.gmap, d))
            .collect()
    }

    pub fn point(&self) -> Option<&Point3> {
        self.gmap.attribute::<Cell0>(self.dart).map(|v| &v.point)
    }
}
