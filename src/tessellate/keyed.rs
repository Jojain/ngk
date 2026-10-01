//! A shape → one mesh, every piece of it keyed by the cell it comes from.
//!
//! The layout is the one a three.js viewer draws and picks from directly:
//! every face's triangles in one indexed buffer, each face a range of it in
//! index units (`geometry.addGroup(start, count)`); every edge a polyline of
//! its own, one after another in one point buffer; every vertex one point.
//! A pick then names its cell by key, with nothing to interpret.

use std::collections::HashSet;
use std::hash::Hash;

use super::curve::tessellate_span;
use super::{
    CurveOpts, IndexedMesh, SurfaceOpts, TessellateError, TessellateOpts, tessellate_face,
};
use crate::geometry::Point3;
use crate::model::Model;
use crate::topology::edge::Edge;
use crate::topology::face::Face;
use crate::topology::payload::Payload;
use crate::topology::profile::Profile;
use crate::topology::shape_keys::{EdgeKey, FaceKey, VertexKey};
use crate::topology::sheet::Sheet;
use crate::topology::solid::Solid;
use crate::topology::vertex::Vertex;

/// The sampling [`tessellate`] uses: what an interactive CAD viewer needs to
/// show curved faces as curved.
pub const VIEWER: TessellateOpts = TessellateOpts {
    curve: CurveOpts { segments: 64 },
    surface: SurfaceOpts { nu: 64, nv: 32 },
};

/// A shape's mesh, with the key of the cell behind every range and point.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Tessellation {
    /// Every face's triangles. Positions are shared within a face, never
    /// across two.
    pub mesh: IndexedMesh,
    /// One range of `mesh.indices` per face, contiguous and covering it.
    pub faces: Vec<KeyedRange<FaceKey>>,
    /// Every edge's polyline, one after another.
    pub edge_points: Vec<Point3>,
    /// One range of `edge_points` per edge, contiguous and covering it. A
    /// closed edge's polyline ends on its first point.
    pub edges: Vec<KeyedRange<EdgeKey>>,
    /// One point per vertex: its stored position.
    pub vertices: Vec<KeyedPoint>,
}

/// The run of a buffer one cell owns.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KeyedRange<K> {
    pub key: K,
    pub start: usize,
    pub count: usize,
}

/// A vertex and where it is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KeyedPoint {
    pub key: VertexKey,
    pub point: Point3,
}

/// A face of the shape could not be meshed.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("face {face:?} could not be tessellated: {reason}")]
pub struct TessellationError {
    pub face: FaceKey,
    pub reason: TessellateError,
}

/// A shape that can be tessellated as the faces, edges and vertices it is
/// made of.
pub trait Tessellate {
    /// Tessellates at `opts` rather than at [`VIEWER`].
    fn tessellate_with(&self, opts: TessellateOpts) -> Result<Tessellation, TessellationError>;
}

/// Tessellates `shape` at [`VIEWER`] sampling.
///
/// Faces, edges and vertices each come in the shape's own `faces()`,
/// `edges()` and `vertices()` order, once each; a [`Model`] gives every cell
/// it holds, in store order. The same shape always gives the same lists.
pub fn tessellate<T: Tessellate + ?Sized>(shape: &T) -> Result<Tessellation, TessellationError> {
    shape.tessellate_with(VIEWER)
}

impl<P: Payload> Tessellate for Model<P> {
    fn tessellate_with(&self, opts: TessellateOpts) -> Result<Tessellation, TessellationError> {
        assemble(
            self.iter_faces().filter_map(|(key, _)| self.face(key)),
            self.iter_edges().filter_map(|(key, _)| self.edge(key)),
            self.iter_vertices().filter_map(|(key, _)| self.vertex(key)),
            opts,
        )
    }
}

impl<P: Payload> Tessellate for Solid<'_, P> {
    fn tessellate_with(&self, opts: TessellateOpts) -> Result<Tessellation, TessellationError> {
        assemble(self.faces(), self.edges(), self.vertices(), opts)
    }
}

impl<P: Payload> Tessellate for Sheet<'_, P> {
    fn tessellate_with(&self, opts: TessellateOpts) -> Result<Tessellation, TessellationError> {
        assemble(self.faces(), self.edges(), self.vertices(), opts)
    }
}

impl<P: Payload> Tessellate for Face<'_, P> {
    fn tessellate_with(&self, opts: TessellateOpts) -> Result<Tessellation, TessellationError> {
        assemble([self.clone()], self.edges(), self.vertices(), opts)
    }
}

impl<P: Payload> Tessellate for Profile<'_, P> {
    fn tessellate_with(&self, opts: TessellateOpts) -> Result<Tessellation, TessellationError> {
        assemble([], self.edges(), self.vertices(), opts)
    }
}

impl<P: Payload> Tessellate for Edge<'_, P> {
    fn tessellate_with(&self, opts: TessellateOpts) -> Result<Tessellation, TessellationError> {
        assemble([], [*self], self.vertices(), opts)
    }
}

/// Meshes each face, samples each edge and reads each vertex, skipping any
/// cell already seen: a face's loops walk a seam edge twice, a closed
/// profile meets its start vertex twice.
fn assemble<'g, P: Payload + 'g>(
    faces: impl IntoIterator<Item = Face<'g, P>>,
    edges: impl IntoIterator<Item = Edge<'g, P>>,
    vertices: impl IntoIterator<Item = Vertex<'g, P>>,
    opts: TessellateOpts,
) -> Result<Tessellation, TessellationError> {
    let mut out = Tessellation::default();
    for face in first_of_each(faces, |face| face.key()) {
        let mesh = tessellate_face(&face, opts).map_err(|reason| TessellationError {
            face: face.key(),
            reason,
        })?;
        out.faces.push(KeyedRange {
            key: face.key(),
            start: out.mesh.indices.len(),
            count: mesh.indices.len(),
        });
        append(&mut out.mesh, mesh);
    }
    for edge in first_of_each(edges, |edge| edge.key()) {
        let polyline = tessellate_span(&edge.trimmed_curve(), opts.curve);
        out.edges.push(KeyedRange {
            key: edge.key(),
            start: out.edge_points.len(),
            count: polyline.points.len(),
        });
        out.edge_points.extend(polyline.points);
    }
    out.vertices = first_of_each(vertices, |vertex| vertex.key())
        .map(|vertex| KeyedPoint {
            key: vertex.key(),
            point: *vertex.point(),
        })
        .collect();
    Ok(out)
}

fn append(into: &mut IndexedMesh, mesh: IndexedMesh) {
    let offset = into.positions.len() as u32;
    into.positions.extend(mesh.positions);
    into.normals.extend(mesh.normals);
    into.indices
        .extend(mesh.indices.into_iter().map(|index| index + offset));
}

fn first_of_each<T, K: Eq + Hash>(
    items: impl IntoIterator<Item = T>,
    key: impl Fn(&T) -> K,
) -> impl Iterator<Item = T> {
    let mut seen = HashSet::new();
    items.into_iter().filter(move |item| seen.insert(key(item)))
}
