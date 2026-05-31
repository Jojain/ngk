mod curve_curve;
mod curve_surface;
mod error;
mod options;
mod surface_surface;

use crate::geometry::{Interval, Point3};

pub use curve_curve::{intersect_curves, intersect_curves_with_options};
pub use curve_surface::{intersect_curve_surface, intersect_curve_surface_with_options};
pub use error::IntersectionError;
pub use options::IntersectionOptions;
pub use surface_surface::{intersect_surfaces, intersect_surfaces_with_options};

pub type CurveCurveIntersections = Vec<CurveCurveIntersection>;
pub type CurveSurfaceIntersections = Vec<CurveSurfaceIntersection>;
pub type SurfaceSurfaceIntersections = Vec<SurfaceSurfaceIntersection>;

#[derive(Debug, Clone, PartialEq)]
pub enum CurveCurveIntersection {
    Point {
        point: Point3,
        u_a: f64,
        u_b: f64,
    },
    Overlap {
        interval_a: Interval,
        interval_b: Interval,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub enum CurveSurfaceIntersection {
    Point {
        point: Point3,
        curve_u: f64,
        surface_u: f64,
        surface_v: f64,
    },
    Overlap {
        curve_interval: Interval,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub enum SurfaceSurfaceIntersection {
    Point {
        point: Point3,
        surface_a_u: f64,
        surface_a_v: f64,
        surface_b_u: f64,
        surface_b_v: f64,
    },
    Curve {
        points: Vec<Point3>,
    },
    Region,
}
