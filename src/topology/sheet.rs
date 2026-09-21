use std::collections::HashSet;

use super::closed::{Closeable, Closed};
use super::edge::Edge;
use super::face::Face;
use super::gmap::Dart;
use super::orientation::Orientation;
use super::payload::{Payload, StandardPayload};
use super::vertex::Vertex;
use crate::measure::{MeasureError, SurfaceProperties, sheet_surface_properties};
use crate::model::{Cell2, MergeTopology, Model, TopologyMerge};
use crate::topology::gmap::Dim;
use crate::topology::shape_keys::SheetKey;

/// A keyed 2-dimensional connected topology view with a contextual anchor.
///
/// A sheet is a logical boundary component. Its walk follows
/// `alpha0`/`alpha1`/`alpha2` and turns across solid-owned cut faces, so the
/// outer and inner sheets of one raw 3-cell remain distinct. Open sheets can
/// have free boundary darts; closed sheets are represented as [`ShellRef`].
/// The anchor determines the traversal orientation used when producing
/// incident face views.
///
/// A sheet that is one boundaryless face anchors inside the polygon that face
/// owns, so it has darts like any other — they are simply all scaffold, and
/// [`Self::edges`] and [`Self::vertices`] come back empty.
pub struct Sheet<'a, P: Payload = StandardPayload> {
    model: &'a Model<P>,
    key: SheetKey,
    anchor: Dart,
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
    /// Returns the canonical representative of the boundary component at
    /// `dart`.
    pub(crate) fn representative(model: &Model<P>, dart: Dart) -> Dart {
        model
            .sheet_darts(dart)
            .into_iter()
            .min()
            .expect("a sheet component contains its seed")
    }

    /// Creates a sheet view from its key using the attribute's stored root.
    pub fn new(model: &'a Model<P>, key: SheetKey) -> Self {
        let anchor = model.sheet_attr_unchecked(key).root;
        Self { model, key, anchor }
    }

    /// Creates a sheet view from a dart in a registered sheet cell.
    pub fn from_dart(model: &'a Model<P>, dart: Dart) -> Option<Self> {
        let key = model.sheet_key(dart)?;
        Some(Self {
            model,
            key,
            anchor: dart,
        })
    }

    /// Returns this sheet's stable key.
    pub fn key(&self) -> SheetKey {
        self.key
    }

    /// Returns the dart this view is anchored at.
    pub fn dart(&self) -> Dart {
        self.anchor
    }

    /// Returns the same sheet with the opposite traversal orientation.
    pub fn reversed(&self) -> Self {
        Self {
            model: self.model,
            key: self.key,
            anchor: self.model.alpha(Dim::Zero, self.anchor),
        }
    }

    /// Returns the user payload attached to this sheet.
    pub fn data(&self) -> &P::Sheet {
        self.model.sheet_attr_unchecked(self.key).data()
    }

    /// Iterates every dart in this logical sheet.
    ///
    /// One orbit: a sheet's faces are joined by `alpha2`, which generates the
    /// raw 3-cell along with `alpha0` and `alpha1`.
    pub fn darts(&self) -> impl Iterator<Item = Dart> + '_ {
        self.model.sheet_darts(self.anchor).into_iter()
    }

    /// Returns the domain faces attached to this sheet.
    ///
    /// Raw 2-cells without a registered [`Face`] are skipped.
    pub fn faces(&self) -> Vec<Face<'a, P>> {
        let mut seen_faces = HashSet::new();
        let mut faces = Vec::new();

        for mut dart in self.darts() {
            let Some(key) = self.model.cell_key::<Cell2>(dart) else {
                continue;
            };
            if !seen_faces.insert(key) {
                continue;
            }
            if self
                .model
                .cell_orientation_from_seed(self.anchor, dart, Dim::Three)
                == Some(Orientation::Reversed)
            {
                dart = self.model.alpha(Dim::Zero, dart);
            }
            faces.push(
                Face::from_dart(self.model, dart)
                    .expect("registered face key must produce a face view"),
            );
        }

        faces
    }

    /// Returns the total area of the sheet's faces.
    pub fn area(&self) -> Result<f64, MeasureError> {
        Ok(self.surface_properties()?.area)
    }

    /// Returns area, centroid and centroidal inertia for this sheet.
    pub fn surface_properties(&self) -> Result<SurfaceProperties, MeasureError> {
        sheet_surface_properties(self)
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
        TopologyMerge::new(self.model, self.darts().collect(), self.anchor)
    }
}

impl<'a, P: Payload> Closeable for Sheet<'a, P> {
    /// A sheet is closed when no dart in it is alpha0-, alpha1-, or alpha2-free.
    ///
    /// This answers for a boundaryless sheet too. Its one face covers a closed
    /// support and sits on the polygon schema of that surface — a bigon for a
    /// sphere, a square for a torus — whose every dart is linked in all three
    /// involutions precisely because the identifications are what close it.
    fn is_closed(&self) -> bool {
        self.darts().all(|d| {
            !self.model.is_free(d, Dim::Zero)
                && !self.model.is_free(d, Dim::One)
                && !self.model.is_free(d, Dim::Two)
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
