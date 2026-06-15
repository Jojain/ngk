use crate::geometry::{Curve, LINEAR_TOLERANCE, PointCoincidence};
use crate::topology::closed::Closeable;
use crate::topology::face::Face;
use crate::topology::gmap::{Cell1, Dim, MergeTopology, TopologyMerge};
use crate::topology::shape_keys::EdgeKey;

use super::facet::Facet;
use super::gmap::{Dart, GMap};
use super::payload::{Payload, StandardPayload};
use super::sheet::Sheet;
use super::vertex::Vertex;

/// A typed view over a 1-cell of a [`GMap`].
///
/// Edges are keyed domain entities. The view is rooted at any dart in the
/// edge orbit and resolves geometry through the map's stored
/// [`EdgeAttr`](crate::topology::attributes::EdgeAttr).
pub struct Edge<'a, P: Payload = StandardPayload> {
    gmap: &'a GMap<P>,
    /// A dart belonging to this edge's 1-cell orbit.
    pub dart: Dart,
}

impl<'a, P: Payload> Edge<'a, P> {
    /// Creates an edge view rooted at `dart`.
    ///
    /// The dart is not validated eagerly. Methods that need an edge key or
    /// curve expect the 1-cell to have a registered edge attribute.
    pub fn new(gmap: &'a GMap<P>, dart: Dart) -> Self {
        Self { gmap, dart }
    }

    /// Iterates every dart in this edge's 1-cell orbit.
    pub fn darts(&self) -> impl Iterator<Item = Dart> + '_ {
        self.gmap
            .orbit(self.dart, self.gmap.orbit_indices(Dim::One))
    }

    /// Returns the stable key of this edge attribute in the source map.
    ///
    /// # Panics
    ///
    /// Panics if this edge orbit has no registered edge attribute.
    pub fn key(&self) -> EdgeKey {
        let dart = self.gmap.cell_representative(self.dart, Dim::One);
        self.gmap.dart_to_edge[&dart]
    }

    /// Returns the vertex at `self.dart`.
    ///
    /// This is the oriented start vertex for the current edge view. Creating an
    /// edge from the opposite dart swaps [`start`](Self::start) and
    /// [`end`](Self::end).
    pub fn start(&self) -> Vertex<'a, P> {
        Vertex::new(self.gmap, self.dart)
    }

    /// Returns the vertex reached by alpha0 from `self.dart`.
    ///
    /// This is the oriented end vertex for the current edge view.
    pub fn end(&self) -> Vertex<'a, P> {
        Vertex::new(self.gmap, self.gmap.alpha(Dim::Zero, self.dart))
    }

    /// Returns the distinct vertices incident to this edge.
    pub fn vertices(&self) -> Vec<Vertex<'a, P>> {
        self.gmap
            .incident_cells(self.dart, Dim::One, Dim::Zero)
            .map(|d| Vertex::new(self.gmap, d))
            .collect()
    }

    /// Returns all gmap 2-cell facets incident to this edge.
    pub fn facets(&self) -> Vec<Facet<'a, P>> {
        self.gmap
            .incident_cells(self.dart, Dim::One, Dim::Two)
            .map(|d| Facet::new(self.gmap, d))
            .collect()
    }

    /// Returns the distinct domain faces incident to this edge.
    pub fn faces(&self) -> Vec<Face<'a, P>> {
        self.facets()
            .into_iter()
            .map(|facet| facet.face().expect("a facet should have a face"))
            .collect()
    }

    /// Returns all 2-dimensional sheets incident to this edge.
    ///
    /// Wrap a returned sheet with [`Closed::new`](super::closed::Closed::new)
    /// when the caller needs the stronger shell invariant.
    pub fn sheets(&self) -> Vec<Sheet<'a, P>> {
        self.gmap
            .incident_cells(self.dart, Dim::One, Dim::Three)
            .map(|d| Sheet::new(self.gmap, d))
            .collect()
    }

    /// Returns the geometric curve attached to this edge, if present.
    ///
    /// `None` means the 1-cell has no registered edge attribute in the map.
    pub fn curve(&self) -> Option<&Curve> {
        self.gmap
            .attribute::<Cell1>(self.dart)
            .map(|attr| &attr.curve)
    }

    /// Returns the curve length between this edge view's oriented endpoints.
    ///
    /// The length is evaluated on the attached curve using the parameters of
    /// [`start`](Self::start) and [`end`](Self::end).
    ///
    /// # Panics
    ///
    /// Panics if the edge has no curve or either endpoint has no point
    /// geometry. This method should become fallible once the topology API has a
    /// project-wide error type for missing attributes.
    pub fn length(&self) -> Option<f64> {
        let t0 = self
            .start()
            .point()
            .map(|p| self.curve().map(|c| c.param_at(*p)))
            .unwrap()
            .unwrap();
        let t1 = self
            .end()
            .point()
            .map(|p| self.curve().map(|c| c.param_at(*p)))
            .unwrap()
            .unwrap();
        self.curve().map(|c| c.length(t0, t1))
    }
}

impl<P: Payload> MergeTopology<P> for Edge<'_, P> {
    fn merge_topology(&self) -> TopologyMerge<'_, P> {
        TopologyMerge::new(self.gmap, self.darts().collect(), self.dart)
    }
}

impl<P: Payload> Closeable for Edge<'_, P> {
    fn is_closed(&self) -> bool {
        match (self.start().point(), self.end().point()) {
            (Some(start), Some(end)) => start.coincides(*end, LINEAR_TOLERANCE),
            _ => false,
        }
    }
}
