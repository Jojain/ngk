use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, TAU};

use super::conics::conic_arc_nurbs;
use super::frame::Frame;
use super::intersections::{
    CurveCurveIntersections, CurveSurfaceIntersections, IntersectionError, IntersectionOptions,
    intersect_curve_surface, intersect_curves, intersect_curves_with_options,
};
use super::nurbs::points::{ControlPolygon, HPoint};
use super::nurbs::{Degree, KnotVector, NurbsCurve};
use super::surfaces::{Plane, Surface};
use super::utils::{IntoUnit, Point3, PointCoincidence};
use crate::geometry::axis::Axis3;
use crate::geometry::nurbs::error::NurbsError;
use crate::geometry::tolerance::{LINEAR_TOLERANCE_SQUARED, MAX_DISTANCE};
use crate::geometry::traits::CurveGeometry;
use crate::geometry::{ANGULAR_TOLERANCE, Interval, LINEAR_TOLERANCE};
use nalgebra::{Rotation3, UnitVector3, Vector3};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Periodicity {
    None,
    Periodic(f64),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Curve {
    Line(Line),
    Circle(Circle),
    Ellipse(Ellipse),
    Nurbs(NurbsCurve),
    Bounded(Box<Bounded<Curve>>),
}

impl Curve {
    pub fn line(start: Point3, end: Point3) -> Self {
        let line = Line::new(Axis3::from_points(start, end));
        let length = (end - start).norm();
        Curve::Bounded(Box::new(Bounded::new(
            Curve::Line(line),
            Interval::new(0.0, length),
        )))
    }
    pub fn circle(plane: Plane, radius: f64) -> Self {
        Curve::Circle(Circle::new(plane, radius))
    }

    /// Returns the arc of a circle spanning `bounds` (radians, measured from
    /// the plane's `x_dir`), parameterised over `[0, 1]`.
    ///
    /// Prefer this over [`Curve::circle`] for an edge that covers only part of
    /// a turn: a whole circle carries the `atan2` range `(-pi, pi]`, so
    /// [`Curve::parameters_between`] cannot express a span wider than half a
    /// turn and silently returns the complementary arc instead.
    pub fn arc(plane: Plane, radius: f64, bounds: Interval) -> Self {
        Curve::Bounded(Box::new(Bounded::new(
            Curve::Circle(Circle::new(plane, radius)),
            bounds,
        )))
    }

    /// Returns the innermost curve, unwrapping any [`Bounded`] trimming.
    ///
    /// Use it to ask what kind of geometry an edge carries — a trimmed arc is
    /// a `Bounded` wrapper around a [`Curve::Circle`], not a `Curve::Circle`.
    pub fn base(&self) -> &Curve {
        match self {
            Curve::Bounded(bounded) => bounded.inner().base(),
            curve => curve,
        }
    }

    pub fn to_nurbs(&self) -> Result<NurbsCurve, NurbsError> {
        match self {
            Curve::Line(line) => line.to_nurbs(),
            Curve::Circle(circle) => circle.to_nurbs(),
            Curve::Ellipse(ellipse) => ellipse.to_nurbs(),
            Curve::Nurbs(nurbs) => Ok(nurbs.clone()),
            Curve::Bounded(curve) => curve.to_nurbs(),
        }
    }

    pub fn periodicity(&self) -> Periodicity {
        match self {
            Curve::Line(_) => Periodicity::None,
            Curve::Circle(_) => Periodicity::Periodic(TAU),
            Curve::Ellipse(_) => Periodicity::Periodic(TAU),
            Curve::Nurbs(_) => Periodicity::None,
            Curve::Bounded(_) => Periodicity::None,
        }
    }
    pub fn point_at(&self, t: f64) -> Point3 {
        match self {
            Curve::Line(l) => l.point_at(t),
            Curve::Circle(c) => c.point_at(t),
            Curve::Ellipse(c) => c.point_at(t),
            Curve::Nurbs(n) => n.point_at(t),
            Curve::Bounded(c) => c.point_at(t),
        }
    }

    /// Returns whether the curve is geometrically closed (start coincides with end).
    pub fn is_closed(&self) -> bool {
        (self.point_at(0.0) - self.point_at(1.0)).norm() <= LINEAR_TOLERANCE
    }

    pub fn derivative_at(&self, t: f64, order: usize) -> Vector3<f64> {
        match self {
            Curve::Line(l) => l.derivative_at(t, order),
            Curve::Circle(c) => c.derivative_at(t, order),
            Curve::Ellipse(c) => c.derivative_at(t, order),
            Curve::Nurbs(n) => n.derivative_at(t, order),
            Curve::Bounded(c) => c.derivative_at(t, order),
        }
    }

    pub fn param_at(&self, point: Point3) -> f64 {
        match self {
            Curve::Line(l) => l.param_at(point),
            Curve::Circle(c) => c.param_at(point),
            Curve::Ellipse(c) => c.param_at(point),
            Curve::Nurbs(n) => closest_sample_parameter(n, point),
            Curve::Bounded(c) => c.param_at(point),
        }
    }

    pub fn parameters_between(&self, start: Point3, end: Point3) -> Interval {
        match self {
            Curve::Bounded(_) => Interval::new(self.param_at(start), self.param_at(end)),
            Curve::Line(_) | Curve::Circle(_) | Curve::Ellipse(_) => {
                let t0 = self.param_at(start);
                let mut t1 = self.param_at(end);
                if start.coincides(end, LINEAR_TOLERANCE)
                    && let Periodicity::Periodic(period) = self.periodicity()
                {
                    t1 = t0 + period;
                }
                Interval::new(t0, t1)
            }
            Curve::Nurbs(nurbs) => nurbs.domain(),
        }
    }

    /// Returns the exact subcurve over a normalized parameter interval.
    pub fn trimmed(&self, interval: Interval) -> Result<Self, NurbsError> {
        if (interval.end - interval.start).abs() <= LINEAR_TOLERANCE {
            return Err(NurbsError::DegenerateInterval {
                start: interval.start,
                end: interval.end,
            });
        }
        if interval.end < interval.start {
            return Ok(Curve::Nurbs(
                self.trimmed(Interval::new(interval.end, interval.start))?
                    .to_nurbs()?
                    .reversed(),
            ));
        }
        if interval.start < -LINEAR_TOLERANCE || interval.end > 1.0 + LINEAR_TOLERANCE {
            return Err(NurbsError::ParameterOutOfRange {
                u: if interval.start < 0.0 {
                    interval.start
                } else {
                    interval.end
                },
                min: 0.0,
                max: 1.0,
            });
        }

        let start = interval.start.clamp(0.0, 1.0);
        let end = interval.end.clamp(0.0, 1.0);
        if start <= LINEAR_TOLERANCE && end >= 1.0 - LINEAR_TOLERANCE {
            return Ok(self.clone());
        }

        let nurbs = self.to_nurbs()?;
        let domain = nurbs.domain();
        let native = |parameter: f64| domain.start + (domain.end - domain.start) * parameter;
        Ok(Curve::Nurbs(nurbs.trimmed(native(start), native(end))?))
    }

    pub fn length(&self, t0: f64, t1: f64) -> f64 {
        match self {
            Curve::Line(l) => l.length(t0, t1),
            Curve::Circle(c) => c.length(t0, t1),
            Curve::Ellipse(c) => c.length(t0, t1),
            Curve::Nurbs(n) => n.length(t0, t1),
            Curve::Bounded(c) => c.length(t0, t1),
        }
    }

    pub fn intersect_curve(
        &self,
        other: &Curve,
    ) -> Result<CurveCurveIntersections, IntersectionError> {
        intersect_curves(self, other)
    }

    pub fn intersect_curve_with_options(
        &self,
        other: &Curve,
        options: IntersectionOptions,
    ) -> Result<CurveCurveIntersections, IntersectionError> {
        intersect_curves_with_options(self, other, options)
    }

    pub fn intersect_surface(
        &self,
        surface: &Surface,
    ) -> Result<CurveSurfaceIntersections, IntersectionError> {
        intersect_curve_surface(self, surface)
    }

    /// Returns the point on the curve nearest `point`.
    pub fn project(&self, point: Point3) -> Point3 {
        match self {
            Curve::Line(curve) => curve.project(point),
            Curve::Circle(curve) => curve.project(point),
            Curve::Ellipse(curve) => curve.project(point),
            Curve::Nurbs(curve) => curve.project(point),
            Curve::Bounded(curve) => curve.project(point),
        }
    }

    /// Returns the parameter range over which the curve is defined.
    ///
    /// Unbounded curves report [`Interval::unbounded`]; a caller that needs a
    /// finite window clamps it with [`Interval::or_extent`].
    pub fn domain(&self) -> Interval {
        match self {
            Curve::Line(curve) => CurveGeometry::domain(curve),
            Curve::Circle(curve) => CurveGeometry::domain(curve),
            Curve::Ellipse(curve) => CurveGeometry::domain(curve),
            Curve::Nurbs(curve) => CurveGeometry::domain(curve),
            Curve::Bounded(curve) => CurveGeometry::domain(&**curve),
        }
    }

    /// Returns this curve rotated by `angle` radians around `axis`.
    ///
    /// The parameterisation is preserved: `rotated(..).point_at(t)` is
    /// `point_at(t)` rotated, for every `t`. Callers therefore keep any
    /// parameter interval computed on the source curve.
    pub fn rotated(&self, axis: Axis3, angle: f64) -> Result<Self, NurbsError> {
        let rotation = Rotation3::from_axis_angle(&axis.direction, angle);
        let rotate = |point: Point3| axis.origin + rotation * (point - axis.origin);
        match self {
            Curve::Line(line) => Ok(Curve::Line(Line::new(Axis3::new(
                rotate(line.origin()),
                rotation * *line.direction(),
            )))),
            Curve::Circle(circle) => Ok(Curve::Circle(Circle::new(
                Plane::new(
                    rotate(circle.plane().origin()),
                    rotation * *circle.plane().x_dir(),
                    rotation * *circle.plane().normal(),
                ),
                circle.radius(),
            ))),
            Curve::Ellipse(ellipse) => Ok(Curve::Ellipse(ellipse.rotated(axis, angle)?)),
            Curve::Bounded(curve) => Ok(Curve::Bounded(Box::new(Bounded::new(
                curve.inner().rotated(axis, angle)?,
                curve.bounds(),
            )))),
            Curve::Nurbs(nurbs) => {
                let points = nurbs
                    .control_points()
                    .iter()
                    .map(|point| {
                        HPoint::from_cartesian(rotate(point.to_cartesian()), point.weight())
                    })
                    .collect();
                Ok(Curve::Nurbs(NurbsCurve::new(
                    nurbs.degree(),
                    ControlPolygon::new(points)?,
                    nurbs.knots().clone(),
                )?))
            }
        }
    }

    pub fn translated(&self, direction: Vector3<f64>) -> Result<Self, NurbsError> {
        match self {
            Curve::Line(line) => Ok(Curve::Line(line.translated(direction))),
            Curve::Bounded(curve) => Ok(Curve::Bounded(Box::new(curve.translated(direction)?))),
            Curve::Circle(circle) => Ok(Curve::Circle(Circle::new(
                Plane::new(
                    circle.plane.origin() + direction,
                    circle.plane.x_dir(),
                    circle.plane.normal(),
                ),
                circle.radius,
            ))),
            Curve::Ellipse(ellipse) => Ok(Curve::Ellipse(ellipse.translated(direction)?)),
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

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Bounded<T> {
    inner: T,
    bounds: Interval,
}

impl<T> Bounded<T> {
    pub fn new(inner: T, bounds: Interval) -> Self {
        Self { inner, bounds }
    }

    pub fn inner(&self) -> &T {
        &self.inner
    }

    pub fn bounds(&self) -> Interval {
        self.bounds
    }

    fn global_parameter(&self, t: f64) -> f64 {
        self.bounds.start + (self.bounds.end - self.bounds.start) * t
    }

    fn local_parameter(&self, t: f64) -> f64 {
        let length = self.bounds.end - self.bounds.start;
        if length.abs() <= LINEAR_TOLERANCE {
            0.0
        } else {
            (t - self.bounds.start) / length
        }
    }
}

impl Bounded<Curve> {
    pub fn point_at(&self, t: f64) -> Point3 {
        self.inner.point_at(self.global_parameter(t))
    }

    pub fn derivative_at(&self, t: f64, order: usize) -> Vector3<f64> {
        let derivative = self.inner.derivative_at(self.global_parameter(t), order);
        if order == 1 {
            derivative * (self.bounds.end - self.bounds.start)
        } else {
            derivative
        }
    }

    /// Returns the normalised parameter of `point` on the trimmed curve.
    ///
    /// A periodic inner curve reports its parameter on one fixed branch — a
    /// circle uses `atan2`, so `(-pi, pi]` — which need not be the branch this
    /// trim lives on. The raw parameter is therefore shifted by whole periods
    /// until it lands at or after the start of the bounds, so an arc spanning
    /// more than half a turn reports its own span instead of the complementary
    /// one.
    pub fn param_at(&self, point: Point3) -> f64 {
        let raw = self.inner.param_at(point);
        let Periodicity::Periodic(period) = self.inner.periodicity() else {
            return self.local_parameter(raw);
        };

        // The nudge keeps a point sitting exactly on the start of the bounds
        // from wrapping to the far end when rounding puts it barely below.
        let start = self.bounds.start.min(self.bounds.end) - ANGULAR_TOLERANCE;
        self.local_parameter(start + (raw - start).rem_euclid(period))
    }

    pub fn length(&self, t0: f64, t1: f64) -> f64 {
        self.inner
            .length(self.global_parameter(t0), self.global_parameter(t1))
    }

    pub fn project(&self, point: Point3) -> Point3 {
        self.inner.project(point)
    }

    pub fn translated(&self, direction: Vector3<f64>) -> Result<Self, NurbsError> {
        Ok(Self::new(self.inner.translated(direction)?, self.bounds))
    }

    pub fn to_nurbs(&self) -> Result<NurbsCurve, NurbsError> {
        match self.inner() {
            Curve::Line(_) => NurbsCurve::new(
                Degree::new(1)?,
                ControlPolygon::new(vec![
                    HPoint::from_cartesian(self.point_at(0.0), 1.0),
                    HPoint::from_cartesian(self.point_at(1.0), 1.0),
                ])?,
                KnotVector::new(vec![0.0, 0.0, 1.0, 1.0])?,
            ),
            Curve::Circle(circle) => circle.to_nurbs_between(self.bounds.start, self.bounds.end),
            Curve::Ellipse(ellipse) => ellipse.to_nurbs_between(self.bounds.start, self.bounds.end),
            _ => self
                .inner
                .to_nurbs()?
                .trimmed(self.bounds.start, self.bounds.end),
        }
    }
}

pub(crate) fn circle_nurbs_control_points(plane: &Plane, radius: f64) -> (Vec<HPoint>, Vec<f64>) {
    let mut points = Vec::with_capacity(9);
    let mut weights = Vec::with_capacity(9);

    for i in 0..=8 {
        let angle = i as f64 * FRAC_PI_2 / 2.0;
        let is_midpoint = i % 2 == 1;
        let weight = if is_midpoint { FRAC_1_SQRT_2 } else { 1.0 };
        let radial_scale = if is_midpoint {
            radius / FRAC_1_SQRT_2
        } else {
            radius
        };
        let radial = angle.cos() * *plane.x_dir() + angle.sin() * *plane.y_dir();
        points.push(HPoint::from_cartesian(
            plane.origin() + radial * radial_scale,
            weight,
        ));
        weights.push(weight);
    }

    (points, weights)
}

pub(crate) fn circle_nurbs_knots() -> Result<KnotVector, NurbsError> {
    KnotVector::new(vec![
        0.0,
        0.0,
        0.0,
        FRAC_PI_2,
        FRAC_PI_2,
        std::f64::consts::PI,
        std::f64::consts::PI,
        3.0 * FRAC_PI_2,
        3.0 * FRAC_PI_2,
        TAU,
        TAU,
        TAU,
    ])
}

fn closest_sample_parameter(curve: &NurbsCurve, point: Point3) -> f64 {
    let domain = curve.domain();
    let segments = 128usize;
    let mut best_u = domain.start;
    let mut best_distance = f64::INFINITY;

    for i in 0..=segments {
        let u = domain.start + (domain.end - domain.start) * (i as f64 / segments as f64);
        let distance = (curve.point_at(u) - point).norm_squared();
        if distance < best_distance {
            best_distance = distance;
            best_u = u;
        }
    }

    for _ in 0..16 {
        let residual = curve.point_at(best_u) - point;
        let first = curve.derivative_at(best_u, 1);
        let second = curve.derivative_at(best_u, 2);
        let gradient = residual.dot(&first);
        let curvature = first.dot(&first) + residual.dot(&second);
        if curvature.abs() <= 1.0e-14 {
            break;
        }
        let next = (best_u - gradient / curvature).clamp(domain.start, domain.end);
        if (next - best_u).abs() <= 1.0e-12 {
            best_u = next;
            break;
        }
        best_u = next;
    }

    best_u
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

        let interval = curve.parameters_between(start, start);

        assert!((interval.start - 0.0).abs() <= ANGULAR_TOLERANCE);
        assert!((interval.end - TAU).abs() <= ANGULAR_TOLERANCE);
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Line {
    pub axis: Axis3,
}

impl Line {
    pub fn new(axis: Axis3) -> Self {
        Self { axis }
    }

    pub fn origin(&self) -> Point3 {
        self.axis.origin
    }

    pub fn direction(&self) -> UnitVector3<f64> {
        self.axis.direction
    }

    pub fn point_at(&self, t: f64) -> Point3 {
        self.axis.origin + *self.axis.direction * t
    }

    pub fn derivative_at(&self, t: f64, order: usize) -> Vector3<f64> {
        match order {
            0 => self.point_at(t).coords,
            1 => *self.axis.direction,
            _ => Vector3::zeros(),
        }
    }
    /// Inverse of [`Self::point_at`] — returns the `t ∈ [0, 1]` parameter
    /// such that `point_at(t)` is the closest point on the line.
    pub fn param_at(&self, point: Point3) -> f64 {
        let dir = *self.axis.direction;
        let len_sq = dir.norm_squared();
        if len_sq < LINEAR_TOLERANCE_SQUARED {
            return 0.0;
        }
        (point - self.axis.origin).dot(&dir) / len_sq
    }
    /// Arc length between `t0` and `t1` (in distance units).
    pub fn length(&self, t0: f64, t1: f64) -> f64 {
        (t1 - t0).abs()
    }

    pub fn project(&self, point: Point3) -> Point3 {
        self.axis.project(point)
    }

    pub fn translated(&self, direction: Vector3<f64>) -> Self {
        Self {
            axis: Axis3::new(self.axis.origin + direction, self.axis.direction),
        }
    }

    pub fn to_nurbs(&self) -> Result<NurbsCurve, NurbsError> {
        NurbsCurve::new(
            Degree::new(1)?,
            ControlPolygon::new(vec![
                HPoint::from_cartesian(self.axis.origin - *self.axis.direction * MAX_DISTANCE, 1.0),
                HPoint::from_cartesian(self.axis.origin + *self.axis.direction * MAX_DISTANCE, 1.0),
            ])?,
            KnotVector::new(vec![0.0, 0.0, 1.0, 1.0])?,
        )
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Circle {
    plane: Plane,
    radius: f64,
}

impl Circle {
    pub fn new(plane: Plane, radius: f64) -> Self {
        Self { plane, radius }
    }

    pub fn plane(&self) -> &Plane {
        &self.plane
    }

    pub fn radius(&self) -> f64 {
        self.radius
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

    pub fn derivative_at(&self, t: f64, order: usize) -> Vector3<f64> {
        if order == 0 {
            return self.point_at(t).coords;
        }
        let phase = t + order as f64 * FRAC_PI_2;
        self.radius * (phase.cos() * *self.plane.x_dir() + phase.sin() * *self.plane.y_dir())
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

    pub fn to_nurbs(&self) -> Result<NurbsCurve, NurbsError> {
        let (points, _) = circle_nurbs_control_points(&self.plane, self.radius);
        NurbsCurve::new(
            Degree::new(2)?,
            ControlPolygon::new(points)?,
            circle_nurbs_knots()?,
        )
    }

    /// Converts an angular interval of the circle to an exact rational NURBS arc.
    pub fn to_nurbs_between(&self, start: f64, end: f64) -> Result<NurbsCurve, NurbsError> {
        conic_arc_nurbs(
            start,
            end,
            FRAC_PI_2,
            |t| self.point_at(t),
            |t| self.derivative_at(t, 1),
        )
    }
}

/// A planar ellipse parameterized by angle in its local frame.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Ellipse {
    frame: Frame,
    major_radius: f64,
    minor_radius: f64,
}

impl Ellipse {
    /// Creates an ellipse whose axes follow the frame's X and Y directions.
    pub fn new(frame: Frame, major_radius: f64, minor_radius: f64) -> Self {
        Self {
            frame,
            major_radius,
            minor_radius,
        }
    }

    pub fn frame(&self) -> &Frame {
        &self.frame
    }

    pub fn major_radius(&self) -> f64 {
        self.major_radius
    }

    pub fn minor_radius(&self) -> f64 {
        self.minor_radius
    }

    pub fn point_at(&self, t: f64) -> Point3 {
        self.frame.origin
            + *self.frame.x_dir * (self.major_radius * t.cos())
            + *self.frame.y_dir * (self.minor_radius * t.sin())
    }

    pub fn derivative_at(&self, t: f64, order: usize) -> Vector3<f64> {
        if order == 0 {
            return self.point_at(t).coords;
        }
        let phase = t + order as f64 * FRAC_PI_2;
        *self.frame.x_dir * (self.major_radius * phase.cos())
            + *self.frame.y_dir * (self.minor_radius * phase.sin())
    }

    /// Returns the closest-point parameter, measured from the frame's X axis.
    pub fn param_at(&self, point: Point3) -> f64 {
        let local = self.frame.coordinates_of(point);
        let mut best = 0.0;
        let mut best_distance = f64::INFINITY;
        for index in 0..64 {
            let t = TAU * index as f64 / 64.0;
            let distance = (self.point_at(t) - point).norm_squared();
            if distance < best_distance {
                best_distance = distance;
                best = t;
            }
        }

        let a = self.major_radius;
        let b = self.minor_radius;
        for _ in 0..16 {
            let (sin, cos) = best.sin_cos();
            let first = (a * a - b * b) * sin * cos - a * local.x * sin + b * local.y * cos;
            let second =
                (a * a - b * b) * (cos * cos - sin * sin) - a * local.x * cos - b * local.y * sin;
            if second.abs() <= 1.0e-14 {
                break;
            }
            let next = best - first / second;
            if (next - best).abs() <= 1.0e-13 {
                best = next;
                break;
            }
            best = next;
        }
        best.rem_euclid(TAU)
    }

    pub fn project(&self, point: Point3) -> Point3 {
        self.point_at(self.param_at(point))
    }

    /// Numerically integrates the analytic speed over the interval.
    pub fn length(&self, t0: f64, t1: f64) -> f64 {
        let segments = 256usize;
        let start = t0.min(t1);
        let span = (t1 - t0).abs();
        let step = span / segments as f64;
        let speed = |t: f64| {
            ((self.major_radius * t.sin()).powi(2) + (self.minor_radius * t.cos()).powi(2)).sqrt()
        };
        let mut sum = speed(start) + speed(start + span);
        for index in 1..segments {
            let coefficient = if index % 2 == 0 { 2.0 } else { 4.0 };
            sum += coefficient * speed(start + index as f64 * step);
        }
        sum * step / 3.0
    }

    pub fn to_nurbs(&self) -> Result<NurbsCurve, NurbsError> {
        self.to_nurbs_between(0.0, TAU)
    }

    /// Converts an angular interval to an exact rational-quadratic NURBS arc.
    pub fn to_nurbs_between(&self, start: f64, end: f64) -> Result<NurbsCurve, NurbsError> {
        conic_arc_nurbs(
            start,
            end,
            FRAC_PI_2,
            |t| self.point_at(t),
            |t| self.derivative_at(t, 1),
        )
    }

    pub fn rotated(&self, axis: Axis3, angle: f64) -> Result<Self, NurbsError> {
        let rotation = Rotation3::from_axis_angle(&axis.direction, angle);
        Ok(Self::new(
            Frame::from_xy(
                axis.origin + rotation * (self.frame.origin - axis.origin),
                rotation * *self.frame.x_dir,
                rotation * *self.frame.y_dir,
            ),
            self.major_radius,
            self.minor_radius,
        ))
    }

    pub fn translated(&self, direction: Vector3<f64>) -> Result<Self, NurbsError> {
        Ok(Self::new(
            Frame::from_xy(
                self.frame.origin + direction,
                self.frame.x_dir,
                self.frame.y_dir,
            ),
            self.major_radius,
            self.minor_radius,
        ))
    }
}

impl CurveGeometry for Line {
    fn domain(&self) -> Interval {
        Interval::unbounded()
    }

    fn periodicity(&self) -> Periodicity {
        Periodicity::None
    }

    fn point_at(&self, t: f64) -> Point3 {
        Line::point_at(self, t)
    }

    fn derivative_at(&self, t: f64, order: usize) -> Vector3<f64> {
        Line::derivative_at(self, t, order)
    }

    fn param_at(&self, point: Point3) -> f64 {
        Line::param_at(self, point)
    }

    fn project(&self, point: Point3) -> Point3 {
        Line::project(self, point)
    }

    fn length(&self, t0: f64, t1: f64) -> f64 {
        Line::length(self, t0, t1)
    }

    fn to_nurbs(&self) -> Result<NurbsCurve, NurbsError> {
        Line::to_nurbs(self)
    }

    fn rotated(&self, axis: Axis3, angle: f64) -> Result<Self, NurbsError> {
        let rotation = Rotation3::from_axis_angle(&axis.direction, angle);
        Ok(Line::new(Axis3::new(
            axis.origin + rotation * (self.origin() - axis.origin),
            rotation * *self.direction(),
        )))
    }

    fn translated(&self, direction: Vector3<f64>) -> Result<Self, NurbsError> {
        Ok(Line::translated(self, direction))
    }
}

impl CurveGeometry for Circle {
    fn domain(&self) -> Interval {
        Interval::new(0.0, TAU)
    }

    fn periodicity(&self) -> Periodicity {
        Periodicity::Periodic(TAU)
    }

    fn point_at(&self, t: f64) -> Point3 {
        Circle::point_at(self, t)
    }

    fn derivative_at(&self, t: f64, order: usize) -> Vector3<f64> {
        Circle::derivative_at(self, t, order)
    }

    fn param_at(&self, point: Point3) -> f64 {
        Circle::param_at(self, point)
    }

    /// Projects onto the circle by dropping `point` to the circle's plane and
    /// rescaling the radial component.
    ///
    /// A point on the axis is equidistant from every point of the circle; the
    /// plane's `x_dir` is returned so the result stays deterministic.
    fn project(&self, point: Point3) -> Point3 {
        let origin = self.plane.origin();
        let normal = self.plane.normal();
        let offset = point - origin;
        let radial = offset - *normal * offset.dot(&normal);
        let distance = radial.norm();
        if distance <= LINEAR_TOLERANCE {
            return origin + *self.plane.x_dir() * self.radius;
        }
        origin + radial * (self.radius / distance)
    }

    fn length(&self, t0: f64, t1: f64) -> f64 {
        Circle::length(self, t0, t1)
    }

    fn to_nurbs(&self) -> Result<NurbsCurve, NurbsError> {
        Circle::to_nurbs(self)
    }

    fn rotated(&self, axis: Axis3, angle: f64) -> Result<Self, NurbsError> {
        let rotation = Rotation3::from_axis_angle(&axis.direction, angle);
        Ok(Circle::new(
            Plane::new(
                axis.origin + rotation * (self.plane.origin() - axis.origin),
                rotation * *self.plane.x_dir(),
                rotation * *self.plane.normal(),
            ),
            self.radius,
        ))
    }

    fn translated(&self, direction: Vector3<f64>) -> Result<Self, NurbsError> {
        Ok(Circle::new(
            Plane::new(
                self.plane.origin() + direction,
                self.plane.x_dir(),
                self.plane.normal(),
            ),
            self.radius,
        ))
    }
}

impl CurveGeometry for Ellipse {
    fn domain(&self) -> Interval {
        Interval::new(0.0, TAU)
    }

    fn periodicity(&self) -> Periodicity {
        Periodicity::Periodic(TAU)
    }

    fn point_at(&self, t: f64) -> Point3 {
        Ellipse::point_at(self, t)
    }

    fn derivative_at(&self, t: f64, order: usize) -> Vector3<f64> {
        Ellipse::derivative_at(self, t, order)
    }

    fn param_at(&self, point: Point3) -> f64 {
        Ellipse::param_at(self, point)
    }

    fn project(&self, point: Point3) -> Point3 {
        Ellipse::project(self, point)
    }

    fn length(&self, t0: f64, t1: f64) -> f64 {
        Ellipse::length(self, t0, t1)
    }

    fn to_nurbs(&self) -> Result<NurbsCurve, NurbsError> {
        Ellipse::to_nurbs(self)
    }

    fn rotated(&self, axis: Axis3, angle: f64) -> Result<Self, NurbsError> {
        Ellipse::rotated(self, axis, angle)
    }

    fn translated(&self, direction: Vector3<f64>) -> Result<Self, NurbsError> {
        Ellipse::translated(self, direction)
    }
}

impl CurveGeometry for NurbsCurve {
    fn domain(&self) -> Interval {
        NurbsCurve::domain(self)
    }

    fn periodicity(&self) -> Periodicity {
        Periodicity::None
    }

    fn point_at(&self, t: f64) -> Point3 {
        NurbsCurve::point_at(self, t)
    }

    fn derivative_at(&self, t: f64, order: usize) -> Vector3<f64> {
        NurbsCurve::derivative_at(self, t, order)
    }

    fn param_at(&self, point: Point3) -> f64 {
        closest_sample_parameter(self, point)
    }

    fn project(&self, point: Point3) -> Point3 {
        NurbsCurve::point_at(self, closest_sample_parameter(self, point))
    }

    fn length(&self, t0: f64, t1: f64) -> f64 {
        NurbsCurve::length(self, t0, t1)
    }

    fn to_nurbs(&self) -> Result<NurbsCurve, NurbsError> {
        Ok(self.clone())
    }

    fn rotated(&self, axis: Axis3, angle: f64) -> Result<Self, NurbsError> {
        let rotation = Rotation3::from_axis_angle(&axis.direction, angle);
        let points = self
            .control_points()
            .iter()
            .map(|point| {
                let rotated = axis.origin + rotation * (point.to_cartesian() - axis.origin);
                HPoint::from_cartesian(rotated, point.weight())
            })
            .collect();
        NurbsCurve::new(
            self.degree(),
            ControlPolygon::new(points)?,
            self.knots().clone(),
        )
    }

    fn translated(&self, direction: Vector3<f64>) -> Result<Self, NurbsError> {
        let points = self
            .control_points()
            .iter()
            .map(|point| HPoint::from_cartesian(point.to_cartesian() + direction, point.weight()))
            .collect();
        NurbsCurve::new(
            self.degree(),
            ControlPolygon::new(points)?,
            self.knots().clone(),
        )
    }
}

impl CurveGeometry for Bounded<Curve> {
    fn domain(&self) -> Interval {
        Interval::new(0.0, 1.0)
    }

    fn periodicity(&self) -> Periodicity {
        Periodicity::None
    }

    fn point_at(&self, t: f64) -> Point3 {
        Bounded::<Curve>::point_at(self, t)
    }

    fn derivative_at(&self, t: f64, order: usize) -> Vector3<f64> {
        Bounded::<Curve>::derivative_at(self, t, order)
    }

    fn param_at(&self, point: Point3) -> f64 {
        Bounded::<Curve>::param_at(self, point)
    }

    fn project(&self, point: Point3) -> Point3 {
        Bounded::<Curve>::project(self, point)
    }

    fn length(&self, t0: f64, t1: f64) -> f64 {
        Bounded::<Curve>::length(self, t0, t1)
    }

    fn to_nurbs(&self) -> Result<NurbsCurve, NurbsError> {
        Bounded::<Curve>::to_nurbs(self)
    }

    fn rotated(&self, axis: Axis3, angle: f64) -> Result<Self, NurbsError> {
        Ok(Bounded::new(
            self.inner().rotated(axis, angle)?,
            self.bounds(),
        ))
    }

    fn translated(&self, direction: Vector3<f64>) -> Result<Self, NurbsError> {
        Bounded::<Curve>::translated(self, direction)
    }
}

/// Forwards to whichever variant the curve holds.
///
/// The inherent methods on [`Curve`] shadow these, so call sites keep working
/// without importing the trait; the impl exists so generic code can be written
/// once over any curve.
impl CurveGeometry for Curve {
    fn domain(&self) -> Interval {
        Curve::domain(self)
    }

    fn periodicity(&self) -> Periodicity {
        Curve::periodicity(self)
    }

    fn point_at(&self, t: f64) -> Point3 {
        Curve::point_at(self, t)
    }

    fn derivative_at(&self, t: f64, order: usize) -> Vector3<f64> {
        Curve::derivative_at(self, t, order)
    }

    fn param_at(&self, point: Point3) -> f64 {
        Curve::param_at(self, point)
    }

    fn project(&self, point: Point3) -> Point3 {
        Curve::project(self, point)
    }

    fn length(&self, t0: f64, t1: f64) -> f64 {
        Curve::length(self, t0, t1)
    }

    fn to_nurbs(&self) -> Result<NurbsCurve, NurbsError> {
        Curve::to_nurbs(self)
    }

    fn rotated(&self, axis: Axis3, angle: f64) -> Result<Self, NurbsError> {
        Curve::rotated(self, axis, angle)
    }

    fn translated(&self, direction: Vector3<f64>) -> Result<Self, NurbsError> {
        Curve::translated(self, direction)
    }
}
