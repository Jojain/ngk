use std::collections::HashMap;

use crate::geometry::dim2::curves::Curve2;
use crate::geometry::{Curve, Point3, Surface};
use crate::topology::dart::Dart;
use crate::topology::edge::Edge;
use crate::topology::gmap::GMap;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::FacetKey;
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

/// Shared geometric data attached to one or more oriented faces.
///
/// Each pcurve is keyed by the directed boundary dart that uses it, and its
/// parameter direction must match that dart from start vertex to end vertex.
#[derive(Clone)]
pub struct FacetAttr<T> {
    /// Geometric support surface shared by the face occurrences.
    pub surface: Surface,
    /// User payload attached to the shared facet.
    pub data: T,
    /// Directed boundary pcurves keyed by their oriented boundary darts.
    pub pcurves: HashMap<Dart, Curve2>,
}

impl<T> FacetAttr<T> {
    /// Creates a facet attribute without boundary pcurves.
    pub fn new(surface: Surface, data: T) -> Self {
        Self {
            surface,
            data,
            pcurves: HashMap::new(),
        }
    }

    /// Creates a facet attribute with explicit boundary pcurves.
    pub fn with_pcurves(surface: Surface, data: T, pcurves: HashMap<Dart, Curve2>) -> Self {
        Self {
            surface,
            data,
            pcurves,
        }
    }
}

/// Stored topology of one oriented trimmed face occurrence.
#[derive(Clone, Debug)]
pub struct FaceAttr {
    /// Shared geometric facet used by this face.
    pub facet: FacetKey,
    /// Oriented seed dart of the outer boundary loop.
    pub outer_loop: Dart,
    /// Oriented seed darts of the inner boundary loops.
    pub inner_loops: Vec<Dart>,
}

impl FaceAttr {
    /// Creates an oriented face topology attribute.
    pub fn new(facet: FacetKey, outer_loop: Dart, inner_loops: Vec<Dart>) -> Self {
        Self {
            facet,
            outer_loop,
            inner_loops,
        }
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
