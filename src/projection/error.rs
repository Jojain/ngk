use thiserror::Error;

use crate::geometry::{IntersectionError, NurbsError};
use crate::topology::UnwrappedFaceDomainError;

/// Why a projection has no single answer.
#[derive(Debug, Error)]
pub enum ProjectionError {
    /// Nothing lies where the projection looks: the half-line meets no part of
    /// the target, or only behind its origin.
    #[error("the projection meets nothing")]
    Miss,
    /// Several feet are equally near within tolerance, as from a sphere's
    /// centre or a point on a cylinder's axis, so no one of them answers.
    #[error("several feet are equally near the point")]
    Ambiguous,
    /// The ray touches the surface without crossing it, so the hit is not a
    /// stable answer to a small change of the ray.
    #[error("the ray grazes the surface")]
    GrazingRay,
    /// The ray lies in the surface, so every point of it is a hit.
    #[error("the ray lies in the surface")]
    RayInSurface,
    /// A directional projection onto a surface the kernel has no closed-form
    /// line intersection for.
    #[error("no directional projection onto a {surface}")]
    UnsupportedSurface {
        /// The kind of surface refused.
        surface: &'static str,
    },
    /// A NURBS operation the projection relies on refused its input: the
    /// surface could not report the parameters of its nearest point, or a
    /// source curve could not be cut to a span.
    #[error("a NURBS operation failed")]
    Nurbs(#[from] NurbsError),
    /// An intersection the projection relies on, of a ray or of a curve with
    /// a surface, refused its options.
    #[error("the intersection failed")]
    Intersection(#[from] IntersectionError),
    /// The face's trim could not be read in its support's parameters.
    #[error("the face's trim could not be read")]
    Trim(#[from] UnwrappedFaceDomainError),
    /// The curve to project has no finite span, so it has no end to carry.
    #[error("the curve is unbounded")]
    UnboundedSource,
    /// The curve has no exact image the kernel can build.
    #[error("no exact projection of a {curve}")]
    UnsupportedCurve {
        /// The kind of curve refused.
        curve: &'static str,
    },
    /// The curve collapses under the projection: to a point, or, for a conic
    /// whose plane holds the direction of projection, to a segment that
    /// retraces itself.
    #[error("the curve projects to a degenerate image")]
    DegenerateImage,
    /// The places where the curve crosses the plane could not all be found, so
    /// the part of it in front of the plane is not certain.
    #[error("the crossings of the curve with the plane are not all known")]
    IncompleteCrossings,
}
