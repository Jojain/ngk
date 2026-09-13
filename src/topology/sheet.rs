use std::collections::{HashSet, VecDeque};

use super::closed::{Closeable, Closed};
use super::edge::Edge;
use super::face::Face;
use super::gmap::Dart;
use super::orientation::Orientation;
use super::payload::{Payload, StandardPayload};
use super::vertex::Vertex;
use crate::model::{Cell2, MergeHandle, MergeTopology, Model, TopologyMerge};
use crate::topology::attributes::ShellRoot;
use crate::topology::gmap::Dim;
use crate::topology::shape_keys::{FaceKey, SheetKey};

/// Where a sheet view is anchored, carrying this view's traversal orientation.
///
/// The stored [`ShellRoot`] says which of the two an attribute holds; this
/// mirrors it for a *view*, where a boundaryless face still has to be readable
/// in either orientation even though there is no dart to `alpha0`-flip.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SheetAnchor {
    /// A dart of this logical sheet; its direction is the view's.
    Dart(Dart),
    /// The sheet is exactly one boundaryless face, read in this orientation.
    Face(FaceKey, Orientation),
}

/// A keyed 2-dimensional connected topology view with a contextual anchor.
///
/// A sheet contains the alpha0/alpha1/alpha2 component of its root plus every
/// component connected through another boundary loop of a domain face. Open
/// sheets can have free boundary darts; closed sheets are represented as
/// [`ShellRef`]. The view's anchor determines the traversal orientation used
/// when producing incident face views.
///
/// A sheet holding a single boundaryless face has no darts at all, so
/// [`Self::dart`] answers `None` and every dart traversal is empty; its one
/// face still comes back from [`Self::faces`].
pub struct Sheet<'a, P: Payload = StandardPayload> {
    model: &'a Model<P>,
    key: SheetKey,
    anchor: SheetAnchor,
}

impl<'a, P: Payload> Clone for Sheet<'a, P> {
    fn clone(&self) -> Self {
        Self {
            model: self.model,
            key: self.key,
            anchor: self.anchor,
        }
    }
}

impl<'a, P: Payload> Sheet<'a, P> {
    /// Creates a sheet view from its key using the attribute's stored root.
    pub fn new(model: &'a Model<P>, key: SheetKey) -> Self {
        let anchor = match model.sheet_attr_unchecked(key).root {
            ShellRoot::Dart(dart) => SheetAnchor::Dart(dart),
            ShellRoot::Face { face, sense } => SheetAnchor::Face(face, sense),
        };
        Self { model, key, anchor }
    }

    /// Creates a sheet view from a dart in a registered sheet cell.
    pub fn from_dart(model: &'a Model<P>, dart: Dart) -> Option<Self> {
        let key = model.sheet_key(dart)?;
        Some(Self {
            model,
            key,
            anchor: SheetAnchor::Dart(dart),
        })
    }

    /// Returns this sheet's stable key.
    pub fn key(&self) -> SheetKey {
        self.key
    }

    /// Returns the dart this view is anchored at.
    ///
    /// A boundaryless sheet has none: it is one face with no incidences, and
    /// there is nothing for a dart to locate.
    pub fn dart(&self) -> Option<Dart> {
        match self.anchor {
            SheetAnchor::Dart(dart) => Some(dart),
            SheetAnchor::Face(_, _) => None,
        }
    }

    /// Returns the dart this view is anchored at.
    ///
    /// # Panics
    ///
    /// Panics on a boundaryless sheet, which has no dart to return.
    pub fn dart_unchecked(&self) -> Dart {
        self.dart()
            .expect("dart-rooted sheet should have an anchoring dart")
    }

    /// Returns the one boundaryless face this sheet is, if that is what it is.
    pub fn boundaryless_face(&self) -> Option<FaceKey> {
        match self.anchor {
            SheetAnchor::Face(face, _) => Some(face),
            SheetAnchor::Dart(_) => None,
        }
    }

    /// Returns the same sheet with the opposite traversal orientation.
    pub fn reversed(&self) -> Self {
        let anchor = match self.anchor {
            SheetAnchor::Dart(dart) => SheetAnchor::Dart(self.model.alpha(Dim::Zero, dart)),
            SheetAnchor::Face(face, sense) => SheetAnchor::Face(face, sense.flip()),
        };
        Self {
            model: self.model,
            key: self.key,
            anchor,
        }
    }

    /// Returns the user payload attached to this sheet.
    pub fn data(&self) -> &P::Sheet {
        &self.model.sheet_attr_unchecked(self.key).data
    }

    /// Iterates every dart in this logical sheet.
    ///
    /// The traversal crosses between the disconnected boundary components of
    /// multi-loop faces through their stored face attributes.
    pub fn darts(&self) -> impl Iterator<Item = Dart> + '_ {
        self.dart()
            .map(|dart| self.model.sheet_darts(dart))
            .unwrap_or_default()
            .into_iter()
    }

    /// Returns the domain faces attached to this sheet.
    ///
    /// Raw 2-cells without a registered [`Face`] are skipped.
    pub fn faces(&self) -> Vec<Face<'a, P>> {
        let seed = match self.anchor {
            SheetAnchor::Dart(dart) => dart,
            SheetAnchor::Face(face, sense) => {
                let face = Face::new(self.model, face);
                return vec![match sense {
                    Orientation::Same => face,
                    Orientation::Reversed => face.reversed(),
                }];
            }
        };
        let mut pending = VecDeque::from([seed]);
        let mut seen_components = HashSet::new();
        let mut seen_faces = HashSet::new();
        let mut faces = Vec::new();

        while let Some(seed) = pending.pop_front() {
            let component = self.model.cell_representative(seed, Dim::Three);
            if !seen_components.insert(component) {
                continue;
            }

            for mut dart in self.model.incident_cells(seed, Dim::Three, Dim::Two) {
                let Some(key) = self.model.cell_key::<Cell2>(dart) else {
                    continue;
                };
                if self
                    .model
                    .cell_orientation_from_seed(seed, dart, Dim::Three)
                    == Some(Orientation::Reversed)
                {
                    dart = self.model.alpha(Dim::Zero, dart);
                }
                if !seen_faces.insert(key) {
                    continue;
                }

                let face = Face::from_dart(self.model, dart)
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
        match self.anchor {
            SheetAnchor::Dart(dart) => TopologyMerge::new(self.model, self.darts().collect(), dart),
            SheetAnchor::Face(face, _) => TopologyMerge::with_faces(
                self.model,
                Vec::new(),
                vec![face],
                MergeHandle::Face(face),
            ),
        }
    }
}

impl<'a, P: Payload> Closeable for Sheet<'a, P> {
    /// A sheet is closed when no dart in it is alpha0-, alpha1-, or alpha2-free.
    ///
    /// A boundaryless sheet has no darts, which would make that test vacuously
    /// true and call any single face a shell. Its closedness is a question for
    /// the geometry instead: the surface has to close on itself in both
    /// parameter directions.
    fn is_closed(&self) -> bool {
        match self.anchor {
            SheetAnchor::Dart(_) => self.darts().all(|d| {
                !self.model.is_free(d, Dim::Zero)
                    && !self.model.is_free(d, Dim::One)
                    && !self.model.is_free(d, Dim::Two)
            }),
            SheetAnchor::Face(face, _) => self
                .model
                .face_attr(face)
                .is_some_and(|attr| attr.surface.is_closed()),
        }
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
