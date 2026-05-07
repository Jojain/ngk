//! Geometry-only tessellation kernel.
//!
//! Lives below [`crate::viz`] and is the single entry point for turning
//! parametric geometry (`Curve`, `Surface`) and BRep references (a `FaceKey`,
//! `EdgeKey`, `VertexKey` into a [`GMap`](crate::topology::gmap::GMap)) into
//! flat polylines and indexed meshes the renderer can consume.
//!
//! - [`tessellate_curve`]: [`Curve`](crate::geometry::Curve) → [`Polyline3`].
//! - [`tessellate_surface_patch`]: [`Surface`](crate::geometry::Surface) over a
//!   `(u, v)` rectangle → [`IndexedMesh`].
//! - [`tessellate_face`]: a face attribute (surface + outer/inner pcurve loops)
//!   → [`IndexedMesh`]. **Real plan**: sample pcurves into a UV polygon with
//!   holes, run constrained Delaunay triangulation, lift via
//!   `surface.point_at`. **Today**: per-surface shortcuts (cylinder UV grid,
//!   plane fan / annulus strip) gated on the pcurve loop layout — see
//!   [`face`] for `// TODO: real CDT` markers.
//! - [`tessellate_shape`]: dispatch on a [`ShapeKey`](crate::topology::shape_keys::ShapeKey).

pub mod curve;
pub mod face;
pub mod shape;
pub mod surface;

use crate::geometry::Point3;
use nalgebra::Vector3;

/// A polyline in 3D. Edges and dart shafts share this type.
#[derive(Debug, Clone, Default)]
pub struct Polyline3 {
    pub points: Vec<Point3>,
}

impl Polyline3 {
    pub fn new(points: Vec<Point3>) -> Self {
        Self { points }
    }

    pub fn is_empty(&self) -> bool {
        self.points.len() < 2
    }
}

/// An indexed triangle mesh: positions, per-vertex normals, triangle indices.
#[derive(Debug, Clone, Default)]
pub struct IndexedMesh {
    pub positions: Vec<Point3>,
    pub normals: Vec<Vector3<f64>>,
    pub indices: Vec<u32>,
}

impl IndexedMesh {
    pub fn is_empty(&self) -> bool {
        self.positions.is_empty() || self.indices.is_empty()
    }
}

/// Sampling resolution for a single curve.
#[derive(Debug, Clone, Copy)]
pub struct CurveOpts {
    pub segments: usize,
}

impl Default for CurveOpts {
    fn default() -> Self {
        Self { segments: 16 }
    }
}

/// Sampling resolution for a surface patch grid.
#[derive(Debug, Clone, Copy)]
pub struct SurfaceOpts {
    pub nu: usize,
    pub nv: usize,
}

impl Default for SurfaceOpts {
    fn default() -> Self {
        Self { nu: 16, nv: 8 }
    }
}

/// Bundles every knob the tessellator currently exposes.
#[derive(Debug, Clone, Copy, Default)]
pub struct TessellateOpts {
    pub curve: CurveOpts,
    pub surface: SurfaceOpts,
}

pub use curve::tessellate_curve;
pub use face::tessellate_face;
pub use shape::{ShapeMesh, tessellate_edge, tessellate_shape, tessellate_vertex};
pub use surface::tessellate_surface_patch;
