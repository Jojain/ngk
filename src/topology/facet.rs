use crate::topology::gmap::Dim;

use super::edge::Edge;
use super::face::Face;
use super::gmap::{Cell2, Dart, GMap};
use super::payload::{Payload, StandardPayload};
use super::vertex::Vertex;

/// A facet is the gmap 2-cell — the orbit ⟨α_k : k ≠ 2⟩. In a 2-Gmap this
/// coincides with a profile (`⟨α₀, α₁⟩`); in a 3-Gmap a facet has two
/// α₃-paired "sides", each side being a profile loop.
///
/// Domain-level [`Face`]s are built on top of facets: a face is (one side of)
/// a facet plus its boundary loops.
pub struct Facet<'a, P: Payload = StandardPayload> {
    gmap: &'a GMap<P>,
    pub dart: Dart,
}

impl<'a, P: Payload> Clone for Facet<'a, P> {
    fn clone(&self) -> Self {
        Self {
            gmap: self.gmap,
            dart: self.dart,
        }
    }
}

impl<'a, P: Payload> Facet<'a, P> {
    pub fn new(gmap: &'a GMap<P>, dart: Dart) -> Self {
        Self { gmap, dart }
    }

    /// Every dart of this facet, traversed via the gmap 2-cell orbit.
    pub fn darts(&self) -> impl Iterator<Item = Dart> + '_ {
        self.gmap
            .orbit(self.dart, self.gmap.orbit_indices(Dim::Two))
    }

    pub fn vertices(&self) -> Vec<Vertex<'a, P>> {
        self.gmap
            .incident_cells(self.dart, Dim::Two, Dim::Zero)
            .map(|d| Vertex::new(self.gmap, d))
            .collect()
    }

    pub fn edges(&self) -> Vec<Edge<'a, P>> {
        self.gmap
            .incident_cells(self.dart, Dim::Two, Dim::One)
            .map(|d| Edge::new(self.gmap, d))
            .collect()
    }

    pub fn face(&self) -> Option<Face<'a, P>> {
        self.gmap
            .attribute::<Cell2>(self.dart)
            .and_then(|key| self.gmap.faces.get(*key).map(|attr| attr.face(self.gmap)))
    }
}
