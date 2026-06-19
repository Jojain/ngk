use std::collections::HashMap;

use crate::geometry::dim2::curves::Curve2;
use crate::geometry::{Curve, Point3, Surface};
use crate::topology::dart::Dart;
use crate::topology::edge::Edge;
use crate::topology::face::Face;
use crate::topology::gmap::{Cell2, GMap};
use crate::topology::payload::Payload;
use crate::topology::shape_keys::EdgeKey;
use crate::topology::vertex::Vertex;

/// Stored data for a keyed vertex 0-cell.
#[derive(Clone)]
pub struct VertexAttr<T> {
    /// Representative dart of the vertex orbit.
    pub dart: Dart,
    /// Geometric point attached to the vertex.
    pub point: Point3,
    /// User payload attached to the vertex.
    pub data: T,
}

impl<T> VertexAttr<T> {
    /// Creates a vertex attribute rooted at `dart`.
    pub fn new(dart: Dart, point: Point3, data: T) -> Self {
        Self { dart, point, data }
    }

    /// Returns a typed vertex view over this attribute in `gmap`.
    pub fn vertex<'a, P: Payload>(&self, gmap: &'a GMap<P>) -> Vertex<'a, P> {
        Vertex::new(gmap, self.dart)
    }
}

/// Stored data for a keyed edge 1-cell.
#[derive(Clone)]
pub struct EdgeAttr<T> {
    /// Representative dart of the edge orbit.
    pub dart: Dart,
    /// Geometric curve attached to the edge.
    pub curve: Curve,
    /// User payload attached to the edge.
    pub data: T,
}

impl<T> EdgeAttr<T> {
    /// Creates an edge attribute rooted at `dart`.
    ///
    /// The caller's `dart` defines the edge's default orientation.
    pub fn new(dart: Dart, curve: Curve, data: T) -> Self {
        Self { dart, curve, data }
    }

    /// Returns a typed edge view over this attribute in `gmap`.
    pub fn edge<'a, P: Payload>(&self, gmap: &'a GMap<P>, key: EdgeKey) -> Edge<'a, P> {
        Edge::new(gmap, key)
    }
}

/// Stored data and default orientation for a profile.
#[derive(Clone)]
pub struct ProfileAttr<T> {
    /// Oriented dart used as the profile's default traversal root.
    pub dart: Dart,
    /// User payload attached to the profile.
    pub data: T,
}

impl<T> ProfileAttr<T> {
    /// Creates a profile attribute rooted at the given oriented dart.
    pub fn new(dart: Dart, data: T) -> Self {
        Self { dart, data }
    }
}

/// Stored data for a keyed domain face.
///
/// # Boundary orientation
///
/// `outer_loop` and `inner_loops` are oriented boundary seeds, not arbitrary
/// representatives of their loop cells. Traversing a stored seed determines
/// the direction of the corresponding boundary. Choosing `alpha0(seed)`
/// traverses the same loop in the opposite direction and reverses the face.
///
/// Each pcurve is keyed by the directed boundary dart that uses it, and its
/// parameter direction must match that dart from start vertex to end vertex.
/// In the support surface's UV space, the outer loop and every inner loop must
/// have opposite winding:
///
/// - outer CCW and inner CW: the face follows the support-surface orientation;
/// - outer CW and inner CCW: the face opposes the support-surface orientation.
///
/// Whether a loop is an outer boundary or a hole is determined structurally by
/// `outer_loop` versus `inner_loops`, not by winding alone.
///
/// Reversing a face is an atomic operation: replace every loop seed `d` with
/// `alpha0(d)`, and replace each pcurve entry `(d, curve)` with
/// `(alpha0(d), curve.reversed())`. Copy, merge, and topology-edit operations
/// must preserve these oriented darts rather than substitute canonical cell
/// representatives.
#[derive(Clone)]
pub struct FaceAttr<T> {
    /// Geometric support surface of the face.
    pub surface: Surface,
    /// User payload attached to the face.
    pub data: T,
    /// Oriented seed dart of the outer boundary loop.
    pub outer_loop: Dart,
    /// Oriented seed darts of inner boundary loops.
    pub inner_loops: Vec<Dart>,
    /// Directed boundary pcurves keyed by their oriented boundary darts.
    pub pcurves: HashMap<Dart, Curve2>,
}

impl<T> FaceAttr<T> {
    /// Creates a face attribute without boundary pcurves.
    ///
    /// The loop darts must follow the orientation contract documented on
    /// [`FaceAttr`].
    pub fn new(surface: Surface, data: T, outer_loop: Dart, inner_loops: Vec<Dart>) -> Self {
        Self {
            surface,
            data,
            outer_loop,
            inner_loops,
            pcurves: HashMap::new(),
        }
    }

    /// Creates a face attribute with explicit boundary pcurves.
    ///
    /// The loop darts and pcurves must follow the orientation contract
    /// documented on [`FaceAttr`].
    pub fn with_pcurves(
        surface: Surface,
        data: T,
        outer_loop: Dart,
        inner_loops: Vec<Dart>,
        pcurves: HashMap<Dart, Curve2>,
    ) -> Self {
        Self {
            surface,
            data,
            outer_loop,
            inner_loops,
            pcurves,
        }
    }

    /// Returns a typed face view over this attribute in `gmap`.
    pub fn face<'a, P: Payload<F = T>>(&'a self, gmap: &'a GMap<P>) -> Face<'a, P> {
        let key = gmap
            .cell_key::<Cell2>(self.outer_loop)
            .expect("FaceAttr must be registered to produce a Face view");
        Face::new(gmap, key)
    }
}

/// Stored data and default orientation for a sheet.
#[derive(Clone)]
pub struct SheetAttr<T> {
    /// Oriented dart used as the sheet's default traversal root.
    pub dart: Dart,
    /// User payload attached to the sheet.
    pub data: T,
}

impl<T> SheetAttr<T> {
    /// Creates a sheet attribute rooted at the given oriented dart.
    pub fn new(dart: Dart, data: T) -> Self {
        Self { dart, data }
    }
}

/// Stored data for a keyed domain solid.
#[derive(Clone)]
pub struct SolidAttr<T> {
    /// User payload attached to the solid.
    pub data: T,
    /// Representative dart of the outer shell.
    pub outer_shell: Dart,
    /// Representative darts of inner shells, when cavities are stored.
    pub inner_shells: Option<Vec<Dart>>,
}

impl<T> SolidAttr<T> {
    /// Creates a solid attribute from an outer shell and optional inner shells.
    pub fn new(data: T, outer_shell: Dart, inner_shells: Option<Vec<Dart>>) -> Self {
        Self {
            data,
            outer_shell,
            inner_shells,
        }
    }
}
