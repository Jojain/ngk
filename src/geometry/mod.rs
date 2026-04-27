pub mod dim2;
pub mod dim3;

pub use dim2::curves::{Curve2, Line2, Polyline2};
pub use dim2::utils::{Point2, Vector2};
pub use dim3::curves::{Circle, Curve, Line, Periodicity};
pub use dim3::frame::Frame;
pub use dim3::nurbs::{
    ControlNet, ControlPolygon, Degree, HPoint, KnotVector, NurbsCurve, NurbsError, NurbsSurface,
};
pub use dim3::nurbs::tessellate::{
    IndexedMesh, sample_curve_uniform, tessellate_curve_adaptive, tessellate_surface_grid,
};
pub use dim3::surfaces::{Cylinder, Plane, Surface};
pub use dim3::utils::{IntoUnit3, Point3};
