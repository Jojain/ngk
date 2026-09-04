use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, TAU};

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
use crate::geometry::{ANGULAR_TOLERANCE, Interval, LINEAR_TOLERANCE};
use nalgebra::{Rotation3, UnitVector3, Vector3};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Periodicity {
    None,
    Periodic(f64),
}

#[derive(Clone, Serialize, Deserialize)]
pub enum Curve {
    Line(Line),
    Circle(Circle),
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
            Curve::Nurbs(nurbs) => Ok(nurbs.clone()),
            Curve::Bounded(curve) => curve.to_nurbs(),
        }
    }

    pub fn periodicity(&self) -> Periodicity {
        match self {
            Curve::Line(_) => Periodicity::None,
            Curve::Circle(_) => Periodicity::Periodic(TAU),
            Curve::Nurbs(_) => Periodicity::None,
            Curve::Bounded(_) => Periodicity::None,
        }
    }
    pub fn point_at(&self, t: f64) -> Point3 {
        match self {
            Curve::Line(l) => l.point_at(t),
            Curve::Circle(c) => c.point_at(t),
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
            Curve::Nurbs(n) => n.derivative_at(t, order),
            Curve::Bounded(c) => c.derivative_at(t, order),
        }
    }

    pub fn param_at(&self, point: Point3) -> f64 {
        match self {
            Curve::Line(l) => l.param_at(point),
            Curve::Circle(c) => c.param_at(point),
            Curve::Nurbs(n) => closest_sample_parameter(n, point),
            Curve::Bounded(c) => c.param_at(point),
        }
    }

    pub fn parameters_between(&self, start: Point3, end: Point3) -> Interval {
        match self {
            Curve::Bounded(_) => Interval::new(self.param_at(start), self.param_at(end)),
            Curve::Line(_) | Curve::Circle(_) => {
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

    pub fn project(&self, point: Point3) -> Point3 {
        match self {
            Curve::Line(l) => l.project(point),
            Curve::Bounded(c) => c.project(point),
            Curve::Circle(_c) => todo!(),
            Curve::Nurbs(_n) => todo!(),
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

#[derive(Clone, Serialize, Deserialize)]
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

#[derive(Clone, Serialize, Deserialize)]
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

#[derive(Clone, Serialize, Deserialize)]
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
        if (end - start).abs() <= LINEAR_TOLERANCE {
            return Err(NurbsError::DegenerateInterval { start, end });
        }
        if end < start {
            return Ok(self.to_nurbs_between(end, start)?.reversed());
        }

        let span = end - start;
        let segment_count = (span / FRAC_PI_2).ceil() as usize;
        let segment_angle = span / segment_count as f64;
        let weight = (0.5 * segment_angle).cos();
        let mut points = Vec::with_capacity(2 * segment_count + 1);
        let mut knots = vec![start; 3];

        for segment in 0..segment_count {
            let angle_start = start + segment as f64 * segment_angle;
            let angle_end = angle_start + segment_angle;
            let angle_middle = 0.5 * (angle_start + angle_end);
            let start_point = self.point_at(angle_start);
            let middle_direction =
                angle_middle.cos() * *self.plane.x_dir() + angle_middle.sin() * *self.plane.y_dir();
            let middle_point = self.plane.origin() + middle_direction * (self.radius / weight);
            let end_point = self.point_at(angle_end);

            if segment == 0 {
                points.push(HPoint::from_cartesian(start_point, 1.0));
            }
            points.push(HPoint::from_cartesian(middle_point, weight));
            points.push(HPoint::from_cartesian(end_point, 1.0));

            if segment + 1 < segment_count {
                knots.extend(std::iter::repeat_n(angle_end, 2));
            }
        }
        knots.extend(std::iter::repeat_n(end, 3));

        NurbsCurve::new(
            Degree::new(2)?,
            ControlPolygon::new(points)?,
            KnotVector::new(knots)?,
        )
    }
}
