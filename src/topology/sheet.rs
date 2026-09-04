use std::collections::{HashSet, VecDeque};

use crate::topology::gmap::Dim;

use super::closed::{Closeable, Closed};
use super::edge::Edge;
use super::face::Face;
use super::gmap::{Cell2, Dart, GMap, MergeTopology, TopologyMerge};
use super::orientation::Orientation;
use super::payload::{Payload, StandardPayload};
use super::vertex::Vertex;
use crate::topology::shape_keys::SheetKey;

/// A keyed 2-dimensional connected topology view with a contextual root dart.
///
/// A sheet contains the alpha0/alpha1/alpha2 component of its root plus every
/// component connected through another boundary loop of a domain face. Open
/// sheets can have free boundary darts; closed sheets are represented as
/// [`ShellRef`]. The view's dart determines the traversal orientation used when
/// producing incident face views.
pub struct Sheet<'a, P: Payload = StandardPayload> {
    gmap: &'a GMap<P>,
    key: SheetKey,
    /// A dart belonging to this logical sheet.
    pub dart: Dart,
}

impl<'a, P: Payload> Clone for Sheet<'a, P> {
    fn clone(&self) -> Self {
        Self {
            gmap: self.gmap,
            key: self.key,
            dart: self.dart,
        }
    }
}

impl<'a, P: Payload> Sheet<'a, P> {
    /// Creates a sheet view from its key using the attribute's reference dart.
    pub fn new(gmap: &'a GMap<P>, key: SheetKey) -> Self {
        let dart = gmap.sheet_attr_unchecked(key).dart;
        Self { gmap, key, dart }
    }

    /// Creates a sheet view from a dart in a registered sheet cell.
    pub fn from_dart(gmap: &'a GMap<P>, dart: Dart) -> Option<Self> {
        let key = gmap.sheet_key(dart)?;
        Some(Self { gmap, key, dart })
    }

    /// Returns this sheet's stable key.
    pub fn key(&self) -> SheetKey {
        self.key
    }

    /// Returns the same sheet with the opposite traversal orientation.
    pub fn reversed(&self) -> Self {
        Self {
            gmap: self.gmap,
            key: self.key,
            dart: self.gmap.alpha(Dim::Zero, self.dart),
        }
    }

    /// Returns the user payload attached to this sheet.
    pub fn data(&self) -> &P::Sheet {
        &self.gmap.sheet_attr_unchecked(self.key).data
    }

    /// Iterates every dart in this logical sheet.
    ///
    /// The traversal crosses between the disconnected boundary components of
    /// multi-loop faces through their stored face attributes.
    pub fn darts(&self) -> impl Iterator<Item = Dart> + '_ {
        self.gmap.sheet_darts(self.dart).into_iter()
    }

    /// Returns the domain faces attached to this sheet.
    ///
    /// Raw 2-cells without a registered [`Face`] are skipped.
    pub fn faces(&self) -> Vec<Face<'a, P>> {
        let mut pending = VecDeque::from([self.dart]);
        let mut seen_components = HashSet::new();
        let mut seen_faces = HashSet::new();
        let mut faces = Vec::new();

        while let Some(seed) = pending.pop_front() {
            let component = self.gmap.cell_representative(seed, Dim::Three);
            if !seen_components.insert(component) {
                continue;
            }

            for mut dart in self.gmap.incident_cells(seed, Dim::Three, Dim::Two) {
                let Some(key) = self.gmap.cell_key::<Cell2>(dart) else {
                    continue;
                };
                if self.gmap.cell_orientation_from_seed(seed, dart, Dim::Three)
                    == Some(Orientation::Reversed)
                {
                    dart = self.gmap.alpha(Dim::Zero, dart);
                }
                if !seen_faces.insert(key) {
                    continue;
                }

                let face = Face::from_dart(self.gmap, dart)
                    .expect("registered face key must produce a face view");
                pending.extend(face.loops().into_iter().map(|loop_| loop_.dart));
                faces.push(face);
            }
        }

        faces
    }

    /// Returns the unique edges used by this sheet's faces.
    ///
    /// Edges are deduplicated by [`EdgeKey`](crate::topology::shape_keys::EdgeKey)
    /// while preserving first-seen face traversal order.
    pub fn edges(&self) -> Vec<Edge<'a, P>> {
        let mut seen = HashSet::new();
        let mut edges = Vec::new();
        for face in self.faces() {
            for edge in face.edges() {
                if seen.insert(edge.key()) {
                    edges.push(edge);
                }
            }
        }
        edges
    }

    /// Returns the unique vertices used by this sheet's faces.
    ///
    /// Vertices are deduplicated by
    /// [`VertexKey`](crate::topology::shape_keys::VertexKey) while preserving
    /// first-seen face traversal order.
    pub fn vertices(&self) -> Vec<Vertex<'a, P>> {
        let mut seen = HashSet::new();
        let mut vertices = Vec::new();
        for face in self.faces() {
            for vertex in face.vertices() {
                if seen.insert(vertex.key()) {
                    vertices.push(vertex);
                }
            }
        }
        vertices
    }
}

impl<P: Payload> MergeTopology<P> for Sheet<'_, P> {
    fn merge_topology(&self) -> TopologyMerge<'_, P> {
        TopologyMerge::new(self.gmap, self.darts().collect(), self.dart)
    }
}

impl<'a, P: Payload> Closeable for Sheet<'a, P> {
    /// A sheet is closed when no dart in it is alpha0-, alpha1-, or alpha2-free.
    fn is_closed(&self) -> bool {
        self.darts().all(|d| {
            !self.gmap.is_free(d, Dim::Zero)
                && !self.gmap.is_free(d, Dim::One)
                && !self.gmap.is_free(d, Dim::Two)
        })
    }
}

/// A closed sheet used as a solid shell.
///
/// The closedness invariant is checked by [`Closed::new`] or trusted by
/// [`Closed::new_unchecked`].
pub type ShellRef<'a, P = StandardPayload> = Closed<Sheet<'a, P>>;

impl<'a, P: Payload> Closed<Sheet<'a, P>> {
    /// Returns the same closed shell with the opposite traversal orientation.
    pub fn reversed(&self) -> Self {
        Closed::new_unchecked(self.inner().reversed())
    }
}
