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
    /// The surface could not report the parameters of its nearest point.
    #[error("the surface could not locate its nearest point")]
    Nurbs(#[from] NurbsError),
    /// The ray/surface intersection refused its options.
    #[error("the ray/surface intersection failed")]
    Intersection(#[from] IntersectionError),
    /// The face's trim could not be read in its support's parameters.
    #[error("the face's trim could not be read")]
    Trim(#[from] UnwrappedFaceDomainError),
}
