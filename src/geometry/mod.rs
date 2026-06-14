pub mod axis;
pub mod dim2;
pub mod dim3;
pub mod interval;
pub mod nurbs;
pub mod tolerance;

pub use dim2::curves::{Curve2, Line2};
pub use dim2::nurbs::{ControlPolygon2, HPoint2, NurbsCurve2};
pub use dim2::utils::{Point2, Vector2};
pub use dim3::bbox::BBox;
pub use dim3::curves::{Bounded, Circle, Curve, Line, Periodicity};
pub use dim3::frame::Frame;
pub use dim3::intersections::{
    CurveCurveIntersection, CurveCurveIntersections, CurveSurfaceIntersection,
    CurveSurfaceIntersections, IntersectionError, IntersectionOptions, SurfaceSurfaceIntersection,
    SurfaceSurfaceIntersections,
};
pub use dim3::surfaces::{Cylinder, Plane, RuledSurface, Surface, SurfaceOfRevolution};
pub use dim3::utils::{IntoUnit, Point3, PointCoincidence};
pub use interval::Interval;
pub use nurbs::tessellate::{
    sample_curve_uniform, tessellate_curve_adaptive, tessellate_surface_grid,
};
pub use nurbs::{
    Bezier, ControlNet, ControlPolygon, Degree, HPoint, KnotVector, NurbsCurve, NurbsError,
    NurbsSurface,
};
pub use tolerance::{ANGULAR_TOLERANCE, LINEAR_TOLERANCE};
