use crate::geometry::{NativeParam, Point2, Point3};

/// Where on the target a projection landed, in the target's own parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TargetParameters {
    /// The parameter on a curve.
    Curve(NativeParam),
    /// The `(u, v)` parameters on a surface.
    Surface(Point2),
}

/// A point carried onto a target.
///
/// `parameters` is present when the algorithm computed them on the way to the
/// point, and absent when a caller can recompute them from [`Self::point`]
/// (the `(u, v)` on a plane, for one).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PointProjection<Hit> {
    /// The projected point, on the target.
    pub point: Point3,
    /// The parameters of [`Self::point`] on the target, when computed.
    pub parameters: Option<TargetParameters>,
    /// The distance from the source to [`Self::point`].
    pub distance: f64,
    /// What the target reports as touched.
    pub hit: Hit,
}

impl<Hit> PointProjection<Hit> {
    /// Narrows the result to the projected point.
    pub fn into_point(self) -> Point3 {
        self.point
    }
}
