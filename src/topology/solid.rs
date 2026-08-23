use std::collections::HashSet;

use super::closed::Closed;
use super::edge::Edge;
use super::face::Face;
use super::gmap::{Dart, GMap, MergeTopology, TopologyMerge};
use super::payload::{Payload, StandardPayload};
use super::sheet::{Sheet, ShellRef};
use super::vertex::Vertex;
use crate::topology::shape_keys::SolidKey;

/// A domain-level solid view.
///
/// A solid is a bounded 3-dimensional region with one outer shell and zero or
/// more inner shells for cavities. It is backed by a stored [`SolidAttr`] in a
/// [`GMap`].
pub struct Solid<'g, P: Payload = StandardPayload> {
    gmap: &'g GMap<P>,
    key: SolidKey,
    dart: Dart,
}

impl<'g, P: Payload> Clone for Solid<'g, P> {
    fn clone(&self) -> Self {
        Self {
            gmap: self.gmap,
            key: self.key,
            dart: self.dart,
        }
    }
}

impl<'g, P: Payload> Solid<'g, P> {
    /// Creates a solid view from its key using the outer shell reference dart.
    pub fn new(gmap: &'g GMap<P>, key: SolidKey) -> Self {
        let dart = gmap.solid_attr_unchecked(key).outer_shell;
        Self { gmap, key, dart }
    }

    /// Creates a solid view from a dart on one of its registered shells.
    pub fn from_dart(gmap: &'g GMap<P>, dart: Dart) -> Option<Self> {
        let key = gmap.solid_key(dart)?;
        Some(Self { gmap, key, dart })
    }

    /// Returns the stable key of this solid in the source map.
    ///
    /// # Panics
    ///
    /// Panics if the solid's outer shell is not registered in the map's solid
    /// index.
    pub fn key(&self) -> SolidKey {
        self.key
    }

    /// Returns the user payload attached to this solid.
    pub fn data(&self) -> &P::S {
        &self.gmap.solid_attr_unchecked(self.key).data
    }

    /// Returns the outer closed shell of the solid.
    pub fn outer_shell(&self) -> ShellRef<'g, P> {
        let d = self.gmap.solid_attr_unchecked(self.key).outer_shell;
        Closed::new_unchecked(
            Sheet::from_dart(self.gmap, d).expect("solid outer shell must have a sheet"),
        )
    }

    /// Returns all inner closed shells of the solid.
    ///
    /// `None` means no inner-shell storage was provided. `Some(vec![])` means
    /// the solid explicitly stores an empty inner-shell list.
    pub fn inner_shells(&self) -> Option<Vec<ShellRef<'g, P>>> {
        self.gmap
            .solid_attr_unchecked(self.key)
            .inner_shells
            .as_ref()
            .map(|inner| {
                inner
                    .iter()
                    .map(|d| {
                        Closed::new_unchecked(
                            Sheet::from_dart(self.gmap, *d)
                                .expect("solid inner shell must have a sheet"),
                        )
                    })
                    .collect()
            })
    }

    /// Returns every shell of the solid, outer shell first.
    pub fn shells(&self) -> Vec<ShellRef<'g, P>> {
        let mut shells = vec![self.outer_shell()];
        if let Some(inners) = self.inner_shells() {
            shells.extend(inners);
        }
        shells
    }

    /// Returns the unique faces bounding this solid.
    ///
    /// Faces are deduplicated by [`FaceKey`](crate::topology::shape_keys::FaceKey)
    /// while preserving first-seen shell traversal order.
    pub fn faces(&self) -> Vec<Face<'g, P>> {
        let mut seen = HashSet::new();
        let mut faces = Vec::new();
        for shell in self.shells() {
            for face in shell.faces() {
                if seen.insert(face.key()) {
                    faces.push(face);
                }
            }
        }
        faces
    }

    /// Returns the unique edges bounding this solid.
    ///
    /// Edges are deduplicated by [`EdgeKey`](crate::topology::shape_keys::EdgeKey)
    /// while preserving first-seen shell traversal order.
    pub fn edges(&self) -> Vec<Edge<'g, P>> {
        let mut seen = HashSet::new();
        let mut edges = Vec::new();
        for shell in self.shells() {
            for edge in shell.edges() {
                if seen.insert(edge.key()) {
                    edges.push(edge);
                }
            }
        }
        edges
    }

    /// Returns the unique vertices bounding this solid.
    ///
    /// Vertices are deduplicated by
    /// [`VertexKey`](crate::topology::shape_keys::VertexKey) while preserving
    /// first-seen shell traversal order.
    pub fn vertices(&self) -> Vec<Vertex<'g, P>> {
        let mut seen = HashSet::new();
        let mut vertices = Vec::new();
        for shell in self.shells() {
            for vertex in shell.vertices() {
                if seen.insert(vertex.key()) {
                    vertices.push(vertex);
                }
            }
        }
        vertices
    }
}

impl<P: Payload> MergeTopology<P> for Solid<'_, P> {
    fn merge_topology(&self) -> TopologyMerge<'_, P> {
        let mut darts = Vec::new();
        for shell in self.shells() {
            darts.extend(shell.darts());
        }
        TopologyMerge::new(self.gmap, darts, self.dart)
    }
}
