use std::collections::HashSet;

use crate::geometry::{Curve, LINEAR_TOLERANCE, PointCoincidence};
use crate::topology::closed::Closeable;
use crate::topology::face::Face;
use crate::topology::gmap::{Cell1, Cell2, Dim, MergeTopology, TopologyMerge};
use crate::topology::shape_keys::EdgeKey;

use super::gmap::{Dart, GMap};
use super::payload::{Payload, StandardPayload};
use super::sheet::Sheet;
use super::vertex::Vertex;

/// A typed view over a 1-cell of a [`GMap`].
///
/// The view carries a stable [`EdgeKey`] and a contextual dart. The dart records
/// the local traversal direction relative to the edge's stored default dart.
///
/// # Default orientation
///
/// `GMap::edge(edge_key)` uses the dart stored in `EdgeAttr`. Traversals such as
/// [`Profile::edges`](crate::topology::profile::Profile::edges) or
/// [`Face::edges`](crate::topology::face::Face::edges) preserve the exact dart
/// reached in that traversal context.
pub struct Edge<'a, P: Payload = StandardPayload> {
    gmap: &'a GMap<P>,
    key: EdgeKey,
    dart: Dart,
}

impl<'a, P: Payload> Edge<'a, P> {
    /// Creates an edge view with the default (`Same`) orientation.
    pub fn new(gmap: &'a GMap<P>, key: EdgeKey) -> Self {
        let dart = gmap.edge_attr_unchecked(key).dart;
        Self { gmap, key, dart }
    }

    /// Creates an edge view from a dart, resolving the edge key and
    /// orientation relative to the stored default direction.
    ///
    /// Returns `None` if the dart does not belong to a registered edge.
    pub fn from_dart(gmap: &'a GMap<P>, dart: Dart) -> Option<Self> {
        let key = gmap.cell_key::<Cell1>(dart)?;
        Some(Self { gmap, key, dart })
    }

    /// Returns the stable key of this edge.
    pub fn key(&self) -> EdgeKey {
        self.key
    }

    /// Returns the dart that represents this edge view in the current
    /// traversal context.
    pub fn dart(&self) -> Dart {
        self.dart
    }

    /// Iterates every dart in this edge's 1-cell orbit.
    pub fn darts(&self) -> impl Iterator<Item = Dart> + '_ {
        let dart = self.dart();
        self.gmap.orbit(dart, self.gmap.orbit_indices(Dim::One))
    }

    /// Returns the vertex at the start of this edge view.
    pub fn start(&self) -> Vertex<'a, P> {
        Vertex::from_dart(self.gmap, self.dart()).expect("edge start must have a registered vertex")
    }

    /// Returns the vertex at the end of this edge view.
    pub fn end(&self) -> Vertex<'a, P> {
        Vertex::from_dart(self.gmap, self.gmap.alpha(Dim::Zero, self.dart()))
            .expect("edge end must have a registered vertex")
    }

    /// Returns the distinct vertices incident to this edge.
    pub fn vertices(&self) -> Vec<Vertex<'a, P>> {
        self.gmap
            .incident_cells(self.dart(), Dim::One, Dim::Zero)
            .filter_map(|d| Vertex::from_dart(self.gmap, d))
            .collect()
    }

    /// Returns the distinct domain faces incident to this edge.
    pub fn faces(&self) -> Vec<Face<'a, P>> {
        let mut seen = HashSet::new();
        self.gmap
            .incident_cells(self.dart(), Dim::One, Dim::Two)
            .filter_map(|dart| {
                let key = self.gmap.cell_key::<Cell2>(dart)?;
                seen.insert(key)
                    .then(|| Face::from_dart(self.gmap, dart))
                    .flatten()
            })
            .collect()
    }

    /// Returns all 2-dimensional sheets incident to this edge.
    ///
    /// Wrap a returned sheet with [`Closed::new`](super::closed::Closed::new)
    /// when the caller needs the stronger shell invariant.
    pub fn sheets(&self) -> Vec<Sheet<'a, P>> {
        self.gmap
            .incident_cells(self.dart(), Dim::One, Dim::Three)
            .filter_map(|d| Sheet::from_dart(self.gmap, d))
            .collect()
    }

    /// Returns the geometric curve attached to this edge.
    ///
    /// # Panics
    ///
    /// Panics if the edge has no stored attribute.
    pub fn curve(&self) -> Option<&Curve> {
        self.gmap.edge_attr(self.key).map(|attr| &attr.curve)
    }

    /// Returns the curve length between this edge view's oriented endpoints.
    ///
    /// The length is evaluated on the attached curve using the parameters of
    /// [`start`](Self::start) and [`end`](Self::end).
    ///
    /// # Panics
    ///
    /// Panics if the edge has no curve or either endpoint has no point
    /// geometry.
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

    /// Returns a new edge view with the opposite orientation.
    pub fn reversed(&self) -> Self {
        Self {
            gmap: self.gmap,
            key: self.key,
            dart: self.gmap.alpha(Dim::Zero, self.dart),
        }
    }
}

impl<P: Payload> MergeTopology<P> for Edge<'_, P> {
    fn merge_topology(&self) -> TopologyMerge<'_, P> {
        TopologyMerge::new(self.gmap, self.darts().collect(), self.dart())
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
