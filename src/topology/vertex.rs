use crate::geometry::Point3;
use crate::topology::gmap::{Cell0, Dim, MergeTopology, TopologyMerge};
use crate::topology::shape_keys::VertexKey;

use super::edge::Edge;
use super::facet::Facet;
use super::gmap::{Dart, GMap};
use super::payload::{Payload, StandardPayload};
use super::sheet::Sheet;

/// A typed view over a 0-cell of a [`GMap`].
///
/// A vertex view is anchored by one dart in the vertex orbit. Geometry and
/// payload data live in the map's [`VertexAttr`](crate::topology::attributes::VertexAttr);
/// this view provides typed traversal to adjacent topology without exposing the
/// alpha-level representation at every call site.
#[derive(Clone, Copy)]
pub struct Vertex<'a, P: Payload = StandardPayload> {
    /// A dart belonging to this vertex's 0-cell orbit.
    pub dart: Dart,
    gmap: &'a GMap<P>,
}

impl<'a, P: Payload> Vertex<'a, P> {
    /// Creates a vertex view rooted at `dart`.
    ///
    /// The dart is not validated eagerly. Methods that need a stored vertex
    /// attribute or key expect `dart` to belong to a registered 0-cell.
    pub fn new(gmap: &'a GMap<P>, dart: Dart) -> Self {
        Self { gmap, dart }
    }

    /// Returns the stable key of this vertex attribute in the source map.
    ///
    /// The key is resolved through the canonical representative of the 0-cell,
    /// so equivalent darts in the same vertex orbit return the same key.
    ///
    /// # Panics
    ///
    /// Panics if this vertex orbit has no registered vertex attribute.
    pub fn key(&self) -> VertexKey {
        let dart = self.gmap.cell_representative(self.dart, Dim::Zero);
        self.gmap.dart_to_vertex[&dart]
    }

    /// Returns all edge 1-cells incident to this vertex.
    ///
    /// Each returned [`Edge`] is a view over the same source map and is rooted
    /// at a dart discovered by the incident-cell traversal.
    pub fn edges(&self) -> Vec<Edge<'a, P>> {
        self.gmap
            .incident_cells(self.dart, Dim::Zero, Dim::One)
            .map(|d| Edge::new(self.gmap, d))
            .collect()
    }

    /// Returns all gmap 2-cell facets incident to this vertex.
    ///
    /// A [`Facet`] is the topological 2-cell. Use [`Facet::face`] when you need
    /// the optional domain-level [`Face`](crate::topology::face::Face) attached
    /// to that facet.
    pub fn facets(&self) -> Vec<Facet<'a, P>> {
        self.gmap
            .incident_cells(self.dart, Dim::Zero, Dim::Two)
            .map(|d| Facet::new(self.gmap, d))
            .collect()
    }

    /// Returns all 2-dimensional sheets incident to this vertex.
    ///
    /// Sheets are dart-rooted connected components of `<alpha0, alpha1,
    /// alpha2>`. Wrap a sheet with [`Closed::new`](super::closed::Closed::new)
    /// when the caller needs the stronger shell invariant.
    pub fn sheets(&self) -> Vec<Sheet<'a, P>> {
        self.gmap
            .incident_cells(self.dart, Dim::Zero, Dim::Three)
            .map(|d| Sheet::new(self.gmap, d))
            .collect()
    }

    /// Returns this vertex's geometric point, if one is stored.
    ///
    /// `None` means the 0-cell has no registered vertex attribute in the map.
    pub fn point(&self) -> Option<&Point3> {
        self.gmap.attribute::<Cell0>(self.dart).map(|v| &v.point)
    }
}

impl<P: Payload> MergeTopology<P> for Vertex<'_, P> {
    fn merge_topology(&self) -> TopologyMerge<'_, P> {
        TopologyMerge::new(
            self.gmap,
            self.gmap
                .orbit(self.dart, self.gmap.orbit_indices(Dim::Zero))
                .collect(),
            self.dart,
        )
    }
}
