use super::error::ProjectionError;
use super::options::ProjectOptions;
use super::result::PointProjection;
use crate::geometry::Point3;

/// A source that can be projected onto a target.
///
/// Implemented once per (source, target) pair; [`project`] dispatches on it.
pub trait Project<Target: ?Sized> {
    /// How the source is carried onto the target:
    /// [`Projection`](super::Projection) for a two-dimensional target, `()`
    /// for a curve, which only has a perpendicular.
    type Mode;
    /// What the target reports as touched; `()` for pure geometry.
    type Hit;
    /// The result of a successful projection.
    type Output;

    /// Projects `self` onto `target`.
    fn project(
        &self,
        target: &Target,
        mode: Self::Mode,
        options: &ProjectOptions,
    ) -> Result<Self::Output, ProjectionError>;
}

/// Projects `source` onto `target` in the given mode.
pub fn project<S, T>(
    source: &S,
    target: &T,
    mode: S::Mode,
    options: &ProjectOptions,
) -> Result<S::Output, ProjectionError>
where
    S: Project<T>,
    T: ?Sized,
{
    source.project(target, mode, options)
}

/// Projects a point onto `target` and keeps only the projected point.
pub fn project_point<T>(
    point: &Point3,
    target: &T,
    mode: <Point3 as Project<T>>::Mode,
    options: &ProjectOptions,
) -> Result<Point3, ProjectionError>
where
    T: ?Sized,
    Point3: Project<T, Output = PointProjection<<Point3 as Project<T>>::Hit>>,
{
    project(point, target, mode, options).map(PointProjection::into_point)
}
