pub mod axis;
pub mod counters;
pub mod dim2;
pub mod dim3;
pub mod interval;
pub mod nurbs;
pub mod parameter;
pub mod reparam;
pub mod tolerance;
pub mod traits;
pub mod transform;

pub use axis::{Axis, Axis3};
pub use counters::SolverCounters;
pub use dim2::curves::{Circle2, Curve2, Ellipse2, Line2};
pub use dim2::intersections::{
    CurveCurveIntersection2, CurveCurveIntersections2, CurveIntersectionError,
    CurveIntersectionOptions,
};
pub use dim2::nurbs::{ControlPolygon2, HPoint2, NurbsCurve2};
pub use dim2::trimmed::TrimmedCurve2;
pub use dim2::utils::{Axis2, DomainSide, Point2, Vector2};
pub use dim3::bbox::BBox;
pub use dim3::curves::{Circle, Curve, Ellipse, Helix, Line, Periodicity};

pub use dim3::frame::Frame;
pub use dim3::intersections::{
    AnalyticSection, AnalyticSurfaceIntersection, CurveCurveIntersection, CurveCurveIntersections,
    CurveSurfaceIntersection, CurveSurfaceIntersections, IntersectionCoverage, IntersectionError,
    IntersectionIncompleteReason, IntersectionOptions, IntersectionQuality, PcurveFidelity,
    PreparedCurve, PreparedSurface, SurfaceIntersectionBranch, SurfaceIntersectionBranchKind,
    SurfaceIntersectionPoint, SurfaceIntersectionPointKind, SurfaceOverlapCandidate,
    SurfaceSurfaceIntersection, SurfaceSurfaceIntersections, analytic_surface_intersections,
    intersect_analytic_curve_surface, intersect_analytic_curves, intersect_analytic_surfaces,
    intersect_prepared_curve_surface, intersect_prepared_surfaces, intersect_surfaces,
    intersect_surfaces_with_options, line_surface_is_analytic,
};
pub use dim3::nurbs::tessellate::{
    sample_curve_uniform, tessellate_curve_adaptive, tessellate_surface_grid,
};
pub use dim3::nurbs::{
    Bezier, BezierSurface, ControlNet, ControlPolygon, Degree, HPoint, InterpolationSystem,
    KNOT_TOLERANCE, KnotVector, NurbsCurve, NurbsSurface, interpolate_with_knots, make_compatible,
};
pub use dim3::surfaces::{
    Cone, Cylinder, Plane, RuledSurface, Sphere, Surface, SurfaceOfRevolution, SurfacePeriodicity,
    Torus,
};
pub use dim3::trimmed::TrimmedCurve;
pub use dim3::utils::{IntoUnit, Point3, PointCoincidence};
pub use interval::Interval;
pub use nurbs::error::{NurbsError, SkinningIncompatibility};
pub use parameter::{Fraction, Native, NativeParam, Normalized, Param};
pub use reparam::{ParamMap, Reparam};
pub use tolerance::{ANGULAR_TOLERANCE, LINEAR_TOLERANCE};
pub use traits::{Curve2Geometry, CurveGeometry, SurfaceGeometry};
pub use transform::Rigid;
