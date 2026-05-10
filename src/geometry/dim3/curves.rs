use std::f64::consts::TAU;

use super::nurbs::NurbsCurve;
use super::nurbs::error::NurbsError;
use super::nurbs::points::{ControlPolygon, HPoint};
use super::surfaces::Plane;
use super::utils::{IntoUnit3, Point3, PointCoincidence};
use crate::geometry::LINEAR_TOLERANCE;
use crate::geometry::axis::Axis3;
use crate::geometry::tolerance::LINEAR_TOLERANCE_SQUARED;
use nalgebra::{Rotation3, UnitVector3, Vector3};

pub enum Periodicity {
    None,
    Periodic(f64),
}

#[derive(Clone)]
pub enum Curve {
    Line(Line),
    Circle(Circle),
    Nurbs(NurbsCurve),
}

impl Curve {
    pub fn periodicity(&self) -> Periodicity {
        match self {
            Curve::Line(_) => Periodicity::None,
            Curve::Circle(_) => Periodicity::Periodic(TAU),
            Curve::Nurbs(_) => Periodicity::None,
        }
    }
    pub fn point_at(&self, t: f64) -> Point3 {
        match self {
            Curve::Line(l) => l.point_at(t),
            Curve::Circle(c) => c.point_at(t),
            Curve::Nurbs(n) => n.point_at(t),
        }
    }
    pub fn param_at(&self, point: Point3) -> f64 {
        match self {
            Curve::Line(l) => l.param_at(point),
            Curve::Circle(c) => c.param_at(point),
            Curve::Nurbs(n) => closest_sample_parameter(n, point),
        }
    }

    pub fn parameters_between(&self, start: Point3, end: Point3) -> (f64, f64) {
        match self {
            Curve::Line(_) | Curve::Circle(_) => {
                let t0 = self.param_at(start);
                let mut t1 = self.param_at(end);
                if start.coincides(end, LINEAR_TOLERANCE) {
                    if let Periodicity::Periodic(period) = self.periodicity() {
                        t1 = t0 + period;
                    }
                }
                (t0, t1)
            }
            Curve::Nurbs(nurbs) => nurbs.domain(),
        }
    }

    pub fn length(&self, t0: f64, t1: f64) -> f64 {
        match self {
            Curve::Line(l) => l.length(t0, t1),
            Curve::Circle(c) => c.length(t0, t1),
            Curve::Nurbs(n) => sampled_length(n, t0, t1),
        }
    }

    pub fn project(&self, point: Point3) -> Point3 {
        match self {
            Curve::Line(l) => l.project(point),
            Curve::Circle(_c) => todo!(),
            Curve::Nurbs(_n) => todo!(),
        }
    }

    pub fn translated(&self, direction: Vector3<f64>) -> Result<Self, NurbsError> {
        match self {
            Curve::Line(line) => Ok(Curve::Line(Line::new(
                line.start + direction,
                line.end + direction,
            ))),
            Curve::Circle(circle) => Ok(Curve::Circle(Circle::new(
                Plane::new(
                    circle.plane.origin() + direction,
                    circle.plane.x_dir(),
                    circle.plane.normal(),
                ),
                circle.radius,
            ))),
            Curve::Nurbs(nurbs) => {
                let points = nurbs
                    .control_points()
                    .iter()
                    .map(|point| {
                        HPoint::from_cartesian(point.to_cartesian() + direction, point.weight())
                    })
                    .collect();
                let control_points = ControlPolygon::new(points)?;
                Ok(Curve::Nurbs(NurbsCurve::new(
                    nurbs.degree(),
                    control_points,
                    nurbs.knots().clone(),
                )?))
            }
        }
    }
}

fn closest_sample_parameter(curve: &NurbsCurve, point: Point3) -> f64 {
    let (u0, u1) = curve.domain();
    let segments = 128usize;
    let mut best_u = u0;
    let mut best_distance = f64::INFINITY;

    for i in 0..=segments {
        let u = u0 + (u1 - u0) * (i as f64 / segments as f64);
        let distance = (curve.point_at(u) - point).norm_squared();
        if distance < best_distance {
            best_distance = distance;
            best_u = u;
        }
    }

    best_u
}

fn sampled_length(curve: &NurbsCurve, t0: f64, t1: f64) -> f64 {
    let segments = 64usize;
    let mut length = 0.0;
    let mut previous = curve.point_at(t0);

    for i in 1..=segments {
        let t = t0 + (t1 - t0) * (i as f64 / segments as f64);
        let current = curve.point_at(t);
        length += (current - previous).norm();
        previous = current;
    }

    length
}

#[cfg(test)]
mod tests {
    use std::f64::consts::TAU;

    use nalgebra::Vector3;

    use super::{Circle, Curve};
    use crate::geometry::{ANGULAR_TOLERANCE, Plane, Point3};

    #[test]
    fn parameters_between_closed_circle_span_full_period() {
        let start = Point3::new(1.0, 0.0, 0.0);
        let curve = Curve::Circle(Circle::new(
            Plane::new(Point3::origin(), Vector3::x(), Vector3::z()),
            1.0,
        ));

        let (t0, t1) = curve.parameters_between(start, start);

        assert!((t0 - 0.0).abs() <= ANGULAR_TOLERANCE);
        assert!((t1 - TAU).abs() <= ANGULAR_TOLERANCE);
    }
}

#[derive(Clone)]
pub struct Line {
    start: Point3,
    end: Point3,
}

impl Line {
    pub fn new(start: Point3, end: Point3) -> Self {
        Self { start, end }
    }
    pub fn from_direction(start: Point3, direction: impl IntoUnit3) -> Self {
        let direction = direction.normalized();
        Self {
            start,
            end: start + *direction,
        }
    }

    pub fn direction(&self) -> UnitVector3<f64> {
        (self.end - self.start).normalized()
    }

    pub fn axis(&self) -> Axis3 {
        Axis3::new(self.start, self.direction())
    }

    pub fn point_at(&self, t: f64) -> Point3 {
        self.start + (self.end - self.start) * t
    }
    /// Inverse of [`Self::point_at`] — returns the `t ∈ [0, 1]` parameter
    /// such that `point_at(t)` is the closest point on the line.
    pub fn param_at(&self, point: Point3) -> f64 {
        let dir = self.end - self.start;
        let len_sq = dir.norm_squared();
        if len_sq < LINEAR_TOLERANCE_SQUARED {
            return 0.0;
        }
        (point - self.start).dot(&dir) / len_sq
    }
    /// Arc length between `t0` and `t1` (in distance units).
    pub fn length(&self, t0: f64, t1: f64) -> f64 {
        (t1 - t0).abs() * (self.end - self.start).norm()
    }

    pub fn project(&self, point: Point3) -> Point3 {
        self.axis().project(point)
    }
}

#[derive(Clone)]
pub struct Circle {
    plane: Plane,
    radius: f64,
}

impl Circle {
    pub fn new(plane: Plane, radius: f64) -> Self {
        Self { plane, radius }
    }
    /// Create a circle from an axis and radius. The normal of the circle is the axis direction, the X dir is chosen to be orthogonal to the axis.
    pub fn from_axis(axis: Axis3, radius: f64) -> Self {
        let normal = axis.direction;
        let reference = if normal.cross(&Vector3::z()).norm_squared() > LINEAR_TOLERANCE_SQUARED {
            Vector3::z()
        } else {
            Vector3::y()
        };
        let x_dir = normal.cross(&reference).normalized();
        let plane = Plane::new(axis.origin, x_dir, normal);
        Self::new(plane, radius)
    }

    pub fn point_at(&self, t: f64) -> Point3 {
        let rot = Rotation3::from_axis_angle(&self.plane.normal(), t);
        let vec = rot * self.plane.x_dir();
        self.plane.origin() + self.radius * *vec
    }
    /// Inverse of [`Self::point_at`]: returns the angle (in radians) of the
    /// projection of `point` onto the circle's plane, measured from `x_dir`
    /// counter-clockwise around `normal`. Range is `(-π, π]`.
    pub fn param_at(&self, point: Point3) -> f64 {
        let v = point - self.plane.origin();
        let x = v.dot(&self.plane.x_dir());
        let y = v.dot(&self.plane.y_dir());
        y.atan2(x)
    }
    /// Arc length between `t0` and `t1` (in distance units).
    pub fn length(&self, t0: f64, t1: f64) -> f64 {
        (t1 - t0).abs() * self.radius
    }
}
