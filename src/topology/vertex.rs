use std::collections::HashSet;

use crate::geometry::Point3;
use crate::model::{Cell0, Cell2, MergeTopology, TopologyMerge};
use crate::topology::attributes::VertexAttr;
use crate::topology::face::Face;
use crate::topology::gmap::Dim;
use crate::topology::shape_keys::VertexKey;

use super::edge::Edge;
use super::gmap::Dart;
use super::payload::{Payload, StandardPayload};
use super::sheet::Sheet;
use crate::model::Model;

/// A typed view over a 0-cell of a [`Model`].
///
/// A vertex view is anchored by one dart in the vertex orbit. Geometry and
/// payload data live in the map's [`VertexAttr`](crate::topology::attributes::VertexAttr);
/// this view provides typed traversal to adjacent topology without exposing the
/// alpha-level representation at every call site.
#[derive(Clone, Copy)]
pub struct Vertex<'a, P: Payload = StandardPayload> {
    model: &'a Model<P>,
    key: VertexKey,
    /// A dart belonging to this vertex's 0-cell orbit.
    pub dart: Dart,
}

impl<'a, P: Payload> Vertex<'a, P> {
    /// Creates a vertex view from its key using the attribute's reference dart.
    pub fn new(model: &'a Model<P>, key: VertexKey) -> Self {
        let dart = model.vertex_attr_unchecked(key).dart;
        Self { model, key, dart }
    }

    /// Creates a vertex view from a dart in a registered vertex cell.
    pub fn from_dart(model: &'a Model<P>, dart: Dart) -> Option<Self> {
        let key = model.cell_key::<Cell0>(dart)?;
        Some(Self { model, key, dart })
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

    /// Returns the stored vertex attribute.
    pub fn attr(&self) -> &'a VertexAttr<P::V> {
        self.model.vertex_attr_unchecked(self.key)
    }

    /// Returns all edge 1-cells incident to this vertex.
    ///
    /// Each returned [`Edge`] is a view over the same source map and is rooted
    /// at a dart discovered by the incident-cell traversal.
    pub fn edges(&self) -> Vec<Edge<'a, P>> {
        self.model
            .incident_cells(self.dart, Dim::Zero, Dim::One)
            .filter_map(|d| Edge::from_dart(self.model, d))
            .collect()
    }

    /// Returns the distinct domain faces incident to this vertex.
    ///
    /// Raw 2-cells without a registered face attribute are skipped.
    pub fn faces(&self) -> Vec<Face<'a, P>> {
        let mut seen = HashSet::new();
        self.model
            .incident_cells(self.dart, Dim::Zero, Dim::Two)
            .filter_map(|dart| {
                let key = self.model.cell_key::<Cell2>(dart)?;
                seen.insert(key)
                    .then(|| Face::from_dart(self.model, dart))
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
        self.model
            .incident_cells(self.dart, Dim::Zero, Dim::Three)
            .filter_map(|d| Sheet::from_dart(self.model, d))
            .collect()
    }

    /// Returns this vertex's geometric point.
    pub fn point(&self) -> &Point3 {
        &self.model.vertex_attr_unchecked(self.key).point
    }
}

impl<P: Payload> MergeTopology<P> for Vertex<'_, P> {
    fn merge_topology(&self) -> TopologyMerge<'_, P> {
        TopologyMerge::new(
            self.model,
            self.model
                .orbit(self.dart, self.model.orbit_indices(Dim::Zero))
                .collect(),
            self.dart,
        )
    }
}
