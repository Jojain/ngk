pub mod axis;
pub mod dim2;
pub mod dim3;
pub mod interval;
pub mod nurbs;
pub mod tolerance;

pub use dim2::curves::{Circle2, Curve2, Line2};
pub use dim2::intersections::{
    CurveCurveIntersection2, CurveCurveIntersections2, CurveIntersectionError,
    CurveIntersectionOptions,
};
pub use dim2::nurbs::{ControlPolygon2, HPoint2, NurbsCurve2};
pub use dim2::utils::{Point2, Vector2};
pub use dim3::bbox::BBox;
pub use dim3::curves::{Bounded, Circle, Curve, Line, Periodicity};
pub use dim3::frame::Frame;
pub use dim3::intersections::{
    CurveCurveIntersection, CurveCurveIntersections, CurveSurfaceIntersection,
    CurveSurfaceIntersections, IntersectionCoverage, IntersectionError,
    IntersectionIncompleteReason, IntersectionOptions, IntersectionQuality, PreparedCurve,
    PreparedSurface, SurfaceIntersectionBranch, SurfaceIntersectionBranchKind,
    SurfaceIntersectionPoint, SurfaceIntersectionPointKind, SurfaceOverlapCandidate,
    SurfaceSurfaceIntersection, SurfaceSurfaceIntersections, intersect_prepared_curve_surface,
    intersect_surfaces, intersect_surfaces_with_options,
};
pub use dim3::nurbs::tessellate::{
    sample_curve_uniform, tessellate_curve_adaptive, tessellate_surface_grid,
};
pub use dim3::nurbs::{
    Bezier, BezierSurface, ControlNet, ControlPolygon, Degree, HPoint, KnotVector, NurbsCurve,
    NurbsSurface,
};
pub use dim3::surfaces::{
    Cylinder, Plane, RuledSurface, Surface, SurfaceOfRevolution, SurfacePeriodicity,
};
pub use dim3::utils::{IntoUnit, Point3, PointCoincidence};
pub use interval::Interval;
pub use nurbs::error::NurbsError;
pub use tolerance::{ANGULAR_TOLERANCE, LINEAR_TOLERANCE};
