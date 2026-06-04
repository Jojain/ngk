use std::collections::HashMap;

use crate::geometry::dim2::curves::Curve2;
use crate::geometry::{Curve, Point3, Surface};
use crate::topology::dart::Dart;
use crate::topology::edge::Edge;
use crate::topology::face::Face;
use crate::topology::gmap::GMap;
use crate::topology::payload::Payload;
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
    pub fn new(dart: Dart, curve: Curve, data: T) -> Self {
        Self { dart, curve, data }
    }

    /// Returns a typed edge view over this attribute in `gmap`.
    pub fn edge<'a, P: Payload>(&self, gmap: &'a GMap<P>) -> Edge<'a, P> {
        Edge::new(gmap, self.dart)
    }
}

/// Stored data for a keyed domain face.
#[derive(Clone)]
pub struct FaceAttr<T> {
    /// Geometric support surface of the face.
    pub surface: Surface,
    /// User payload attached to the face.
    pub data: T,
    /// Representative dart of the outer boundary loop.
    pub outer_loop: Dart,
    /// Representative darts of inner boundary loops.
    pub inner_loops: Vec<Dart>,
    /// Boundary pcurves keyed by boundary dart.
    pub pcurves: HashMap<Dart, Curve2>,
}

impl<T> FaceAttr<T> {
    /// Creates a face attribute without boundary pcurves.
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
        Face::new(gmap, self)
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
