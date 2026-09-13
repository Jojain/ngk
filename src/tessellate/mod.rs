//! Geometry-only tessellation kernel.
//!
//! Lives below [`crate::viz`] and is the single entry point for turning
//! parametric geometry (`Curve`, `Surface`) and BRep references (a `FaceKey`,
//! `EdgeKey`, `VertexKey` into a [`Model`](crate::model::Model)) into
//! flat polylines and indexed meshes the renderer can consume.
//!
//! - [`tessellate_curve`]: [`Curve`](crate::geometry::Curve) → [`Polyline3`].
//! - [`tessellate_surface_patch`]: [`Surface`](crate::geometry::Surface) over a
//!   `(u, v)` rectangle → [`IndexedMesh`].
//! - [`tessellate_face`]: a typed [`Face`](crate::topology::face::Face) view
//!   (surface + outer/inner pcurve loops) → [`IndexedMesh`]. **Real plan**:
//!   sample pcurves into a UV polygon with holes, run constrained Delaunay
//!   triangulation, lift via `surface.point_at`. **Today**: a UV grid where the
//!   boundary is the parameter rectangle, an ear-clipped and curvature-refined
//!   polygon where it is not, and a [`TessellateError`] naming the gap for
//!   anything neither covers — see [`face`] for `// TODO: real CDT` markers.
//! - [`tessellate_face_key`]: raw map/key bridge for callers that are still
//!   iterating a [`Model`](crate::model::Model) directly.
//! - [`tessellate_shape`]: dispatch on a [`ShapeKey`](crate::topology::shape_keys::ShapeKey).

pub mod curve;
pub mod face;
pub mod shape;
pub mod surface;

use crate::geometry::Point3;
use nalgebra::UnitVector3;

/// Why a face could not be meshed.
///
/// The tessellator ships shortcuts rather than a general constrained
/// triangulation, so it has gaps. A gap must be *reported*, not papered over: a
/// mesh that is not the face is read as the face. Each variant names what was
/// missing precisely enough to say what implementing it would take.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TessellateError {
    /// The face's boundary could not be read as a parameter-space polygon at
    /// all — a loop that would not unwrap, fewer than three sampled points.
    UnreadableBoundary,
    /// The boundary was read, and encloses nothing: every sample on one point,
    /// or all of them on one line. A face whose pcurves did not survive its
    /// import arrives like this, so this is a report about the *face*, not about
    /// a gap in the mesher.
    DegenerateBoundary,
    /// The boundary was read and is non-degenerate, but would not reduce to a
    /// simple polygon that could be triangulated: a self-intersecting loop, or
    /// a hole no bridge reaches. This is the case a real constrained Delaunay
    /// triangulation would carry and the ear clipper cannot.
    UntriangulableBoundary,
    /// A face with no enclosing loop takes its extent from its support's own
    /// domain, and this support's domain is unbounded.
    UnboundedDomain,
}

impl std::fmt::Display for TessellateError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnreadableBoundary => {
                write!(
                    f,
                    "the face's boundary could not be read in parameter space"
                )
            }
            Self::DegenerateBoundary => {
                write!(f, "the face's boundary encloses no area in parameter space")
            }
            Self::UntriangulableBoundary => write!(
                f,
                "the face's boundary would not reduce to a triangulable simple polygon"
            ),
            Self::UnboundedDomain => write!(
                f,
                "a face with no enclosing loop needs a bounded support domain"
            ),
        }
    }
}

impl std::error::Error for TessellateError {}

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
    pub normals: Vec<UnitVector3<f64>>,
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

/// What a face meshes to, or why it could not be meshed.
pub type TessellateResult<T> = Result<T, TessellateError>;

pub use curve::tessellate_curve;
pub use face::{tessellate_face, tessellate_face_key};
pub use shape::{ShapeMesh, tessellate_edge, tessellate_shape, tessellate_vertex};
pub use surface::tessellate_surface_patch;
