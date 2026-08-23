use std::collections::HashSet;

use crate::geometry::Point3;
use crate::topology::face::Face;
use crate::topology::gmap::{Cell0, Cell2, Dim, MergeTopology, TopologyMerge};
use crate::topology::shape_keys::VertexKey;

use super::edge::Edge;
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
    gmap: &'a GMap<P>,
    key: VertexKey,
    /// A dart belonging to this vertex's 0-cell orbit.
    pub dart: Dart,
}

impl<'a, P: Payload> Vertex<'a, P> {
    /// Creates a vertex view from its key using the attribute's reference dart.
    pub fn new(gmap: &'a GMap<P>, key: VertexKey) -> Self {
        let dart = gmap.vertex_attr_unchecked(key).dart;
        Self { gmap, key, dart }
    }

    /// Creates a vertex view from a dart in a registered vertex cell.
    pub fn from_dart(gmap: &'a GMap<P>, dart: Dart) -> Option<Self> {
        let key = gmap.cell_key::<Cell0>(dart)?;
        Some(Self { gmap, key, dart })
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
        self.key
    }

    /// Returns all edge 1-cells incident to this vertex.
    ///
    /// Each returned [`Edge`] is a view over the same source map and is rooted
    /// at a dart discovered by the incident-cell traversal.
    pub fn edges(&self) -> Vec<Edge<'a, P>> {
        self.gmap
            .incident_cells(self.dart, Dim::Zero, Dim::One)
            .filter_map(|d| Edge::from_dart(self.gmap, d))
            .collect()
    }

    /// Returns the distinct domain faces incident to this vertex.
    ///
    /// Raw 2-cells without a registered face attribute are skipped.
    pub fn faces(&self) -> Vec<Face<'a, P>> {
        let mut seen = HashSet::new();
        self.gmap
            .incident_cells(self.dart, Dim::Zero, Dim::Two)
            .filter_map(|dart| {
                let key = self.gmap.cell_key::<Cell2>(dart)?;
                seen.insert(key)
                    .then(|| Face::from_dart(self.gmap, dart))
                    .flatten()
            })
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
            .filter_map(|d| Sheet::from_dart(self.gmap, d))
            .collect()
    }

    /// Returns this vertex's geometric point, if one is stored.
    ///
    /// `None` means the 0-cell has no registered vertex attribute in the map.
    pub fn point(&self) -> Option<&Point3> {
        Some(&self.gmap.vertex_attr_unchecked(self.key).point)
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
