//! `ShapeKey` → tessellated representation. Single dispatch entry point.

use super::{IndexedMesh, Polyline3, TessellateOpts, tessellate_curve, tessellate_face};
use crate::geometry::Point3;
use crate::topology::gmap::{Cell0, Cell1, Dim, GMap};
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
/// - `Face` → indexed mesh via [`tessellate_face`].
pub fn tessellate_shape<P: Payload>(
    g: &GMap<P>,
    key: ShapeKey,
    opts: TessellateOpts,
) -> Option<ShapeMesh> {
    match key {
        ShapeKey::Vertex(v) => tessellate_vertex(g, v).map(ShapeMesh::Vertex),
        ShapeKey::Edge(e) => tessellate_edge(g, e, opts).map(ShapeMesh::Edge),
        ShapeKey::Face(f) => tessellate_face(g, f, opts).map(ShapeMesh::Face),
    }
}

pub fn tessellate_vertex<P: Payload>(g: &GMap<P>, key: VertexKey) -> Option<Point3> {
    g.vertex(key).map(|v| v.point)
}

pub fn tessellate_edge<P: Payload>(
    g: &GMap<P>,
    key: EdgeKey,
    opts: TessellateOpts,
) -> Option<Polyline3> {
    let attr = g.edge(key)?;
    let edge = attr.edge(g);
    let v1 = edge.start();
    let v2 = edge.end();
    let p1 = v1.point()?;
    let p2 = v2.point()?;
    let curve = &edge.curve()?;
    let t0 = curve.param_at(*p1);
    let t1 = curve.param_at(*p2);
    Some(tessellate_curve(curve, t0, t1, opts.curve))
}
