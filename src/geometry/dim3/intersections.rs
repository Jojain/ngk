use crate::geometry::{Interval, Point3};

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
