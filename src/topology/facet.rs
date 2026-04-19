use super::edge::EdgeRef;
use super::face::Face;
use super::gmap::{Cell2, Dart, GMap};
use super::payload::{Payload, StandardPayload};
use super::vertex::VertexRef;

/// A facet is the gmap 2-cell — the orbit ⟨α_k : k ≠ 2⟩. In a 2-Gmap this
/// coincides with a profile (`⟨α₀, α₁⟩`); in a 3-Gmap a facet has two
/// α₃-paired "sides", each side being a profile loop.
///
/// Domain-level [`Face`]s are built on top of facets: a face is (one side of)
/// a facet plus its boundary loops.
pub struct FacetRef<'a, P: Payload = StandardPayload> {
    gmap: &'a GMap<'a, P>,
    pub dart: Dart,
}

impl<'a, P: Payload> Clone for FacetRef<'a, P> {
    fn clone(&self) -> Self {
        Self {
            gmap: self.gmap,
            dart: self.dart,
        }
    }
}

impl<'a, P: Payload> FacetRef<'a, P> {
    pub fn new(gmap: &'a GMap<'a, P>, dart: Dart) -> Self {
        Self { gmap, dart }
    }

    /// Every dart of this facet, traversed via the gmap 2-cell orbit.
    pub fn darts(&self) -> impl Iterator<Item = Dart> + '_ {
        self.gmap.orbit(self.dart, self.gmap.orbit_indices(2))
    }

    pub fn vertices(&self) -> Vec<VertexRef<'a, P>> {
        self.gmap
            .incident_cells(self.dart, 2, 0)
            .map(|d| VertexRef::new(self.gmap, d))
            .collect()
    }

    pub fn edges(&self) -> Vec<EdgeRef<'a, P>> {
        self.gmap
            .incident_cells(self.dart, 2, 1)
            .map(|d| EdgeRef::new(self.gmap, d))
            .collect()
    }

    pub fn face(&self) -> Option<&Face<'a, P>> {
        if let Some(id) = self.gmap.attribute::<Cell2>(self.dart) {
            self.gmap.face(*id)
        } else {
            None
        }
    }
}
