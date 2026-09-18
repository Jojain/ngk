//! `ShapeKey` → tessellated representation. Single dispatch entry point.

use super::{IndexedMesh, Polyline3, TessellateOpts, tessellate_curve, tessellate_face_key};
use crate::geometry::Point3;
use crate::model::Model;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, ShapeKey, VertexKey};

/// Tessellated representation of a topological shape.
#[derive(Debug, Clone)]
pub enum ShapeMesh {
    Vertex(Point3),
    Edge(Polyline3),
    Face(IndexedMesh),
}

/// Walk the shape pointed to by `key` in `g` and tessellate it.
///
/// - `Vertex` → its stored 3D position.
/// - `Edge` → polyline from start vertex's curve param to end vertex's.
/// - `Face` → indexed mesh via [`tessellate_face_key`].
pub fn tessellate_shape<P: Payload>(
    g: &Model<P>,
    key: ShapeKey,
    opts: TessellateOpts,
) -> Option<ShapeMesh> {
    match key {
        ShapeKey::Vertex(v) => tessellate_vertex(g, v).map(ShapeMesh::Vertex),
        ShapeKey::Edge(e) => tessellate_edge(g, e, opts).map(ShapeMesh::Edge),
        ShapeKey::Face(f) => tessellate_face_key(g, f, opts).ok().map(ShapeMesh::Face),
    }
}

pub fn tessellate_vertex<P: Payload>(g: &Model<P>, key: VertexKey) -> Option<Point3> {
    g.vertex_attr(key).map(|v| v.point)
}

pub fn tessellate_edge<P: Payload>(
    g: &Model<P>,
    key: EdgeKey,
    opts: TessellateOpts,
) -> Option<Polyline3> {
    let attr = g.edge_attr(key)?;
    let edge = attr.edge(g, key);
    let curve = edge.curve();
    let interval = edge.parameter_interval();
    Some(tessellate_curve(
        curve,
        interval.start.value(),
        interval.end.value(),
        opts.curve,
    ))
}
