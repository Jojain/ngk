//! Analytic support curves in a surface's 2D parameter space.
//!
//! A [`Curve2`] is a **support**, exactly like its 3D counterpart
//! [`Curve`](crate::geometry::Curve): it is never cut down to the pcurve or
//! section resting on it. A [`Line2`] runs to infinity, a [`Circle2`] closes.
//! Saying *which part* is meant takes a second value — see
//! [`TrimmedCurve2`](crate::geometry::TrimmedCurve2).
//!
//! Parameters here are **native**: a line's parameter is affine (`0` and `1`
//! land on the two points it was built through), a circle's and an ellipse's is
//! the angle in radians, a NURBS curve's is its own knot domain. Nothing is
//! silently renormalized to `[0, 1]`; normalized traversal is what
//! `TrimmedCurve2` provides, over a stated span.

use std::f64::consts::{FRAC_PI_2, TAU};

use crate::geometry::{
    ControlPolygon2, Degree, HPoint2, Interval, KnotVector, LINEAR_TOLERANCE, NurbsError,
};
use nalgebra::{UnitVector2, Vector2};
use serde::{Deserialize, Serialize};

use super::conics::conic_arc_nurbs2;
use super::nurbs::NurbsCurve2;
use super::utils::Point2;
use crate::geometry::dim3::curves::Periodicity;
use crate::geometry::traits::Curve2Geometry;

/// An unbounded support curve in a surface's 2D parameter space.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Curve2 {
    Line(Line2),
    Circle(Circle2),
    Ellipse(Ellipse2),
    Nurbs(NurbsCurve2),
}

impl Curve2 {
    /// Returns the infinite line whose parameter maps `0` to `start` and `1` to `end`.
    pub fn line(start: Point2, end: Point2) -> Self {
        Curve2::Line(Line2::new(start, end - start))
    }

    /// Returns the full circle centred on `center`, starting along `x_dir`.
    pub fn circle(center: Point2, x_dir: Vector2<f64>, radius: f64) -> Self {
        Curve2::Circle(Circle2::new(center, x_dir, radius))
    }

    /// Converts the support to an exact 2D NURBS representation.
    ///
    /// Unbounded supports are represented over their `[0, 1]` window, periodic
    /// ones over a full period. The parameterization is **not** preserved — see
    /// [`crate::geometry::traits`].
    pub fn to_nurbs(&self) -> Result<NurbsCurve2, NurbsError> {
        match self {
            Curve2::Line(line) => line.to_nurbs(),
            Curve2::Circle(circle) => circle.to_nurbs(),
            Curve2::Ellipse(ellipse) => ellipse.to_nurbs(),
            Curve2::Nurbs(curve) => Ok(curve.clone()),
        }
    }

    /// Returns whether the parameter wraps, and with what period.
    pub fn periodicity(&self) -> Periodicity {
        match self {
            Curve2::Line(_) => Periodicity::None,
            Curve2::Circle(_) => Periodicity::Periodic(TAU),
            Curve2::Ellipse(_) => Periodicity::Periodic(TAU),
            Curve2::Nurbs(_) => Periodicity::None,
        }
    }

    /// Returns the parameter range over which the support is defined.
    ///
    /// Unbounded supports report [`Interval::unbounded`]; a caller that needs a
    /// finite window clamps it with [`Interval::or_extent`].
    pub fn domain(&self) -> Interval {
        match self {
            Curve2::Line(curve) => Curve2Geometry::domain(curve),
            Curve2::Circle(curve) => Curve2Geometry::domain(curve),
            Curve2::Ellipse(curve) => Curve2Geometry::domain(curve),
            Curve2::Nurbs(curve) => curve.domain(),
        }
    }

    /// Evaluates the support at a native parameter.
    pub fn point_at(&self, t: f64) -> Point2 {
        match self {
            Curve2::Line(line) => line.point_at(t),
            Curve2::Circle(circle) => circle.point_at(t),
            Curve2::Ellipse(ellipse) => ellipse.point_at(t),
            Curve2::Nurbs(curve) => curve.point_at(t),
        }
    }

    /// Returns the `order`-th derivative at a native parameter.
    pub fn derivative_at(&self, t: f64, order: usize) -> Vector2<f64> {
        match self {
            Curve2::Line(line) => line.derivative_at(t, order),
            Curve2::Circle(circle) => circle.derivative_at(t, order),
            Curve2::Ellipse(ellipse) => ellipse.derivative_at(t, order),
            Curve2::Nurbs(curve) => curve.derivative_at(t, order),
        }
    }

    /// Returns the native parameter of the support point nearest `point`.
    pub fn param_at(&self, point: Point2) -> f64 {
        match self {
            Curve2::Line(line) => line.param_at(point),
            Curve2::Circle(circle) => circle.param_at(point),
            Curve2::Ellipse(ellipse) => ellipse.param_at(point),
            Curve2::Nurbs(curve) => closest_sample_parameter(curve, point),
        }
    }

    /// Returns the point on the support nearest `point`.
    pub fn project(&self, point: Point2) -> Point2 {
        match self {
            Curve2::Line(line) => line.project(point),
            Curve2::Circle(circle) => circle.project(point),
            Curve2::Ellipse(ellipse) => ellipse.project(point),
            Curve2::Nurbs(curve) => curve.point_at(closest_sample_parameter(curve, point)),
        }
    }

    /// Returns the arc length between two native parameters.
    pub fn length(&self, t0: f64, t1: f64) -> f64 {
        match self {
            Curve2::Line(line) => line.length(t0, t1),
            Curve2::Circle(circle) => circle.length(t0, t1),
            Curve2::Ellipse(ellipse) => ellipse.length(t0, t1),
            Curve2::Nurbs(curve) => polyline_length(|t| curve.point_at(t), t0, t1),
        }
    }

    /// Returns the span running forward from `start` to `end`, in native parameters.
    ///
    /// On a periodic support only the arc reached by advancing from `start` is
    /// expressible here; a section meaning the other one states its span
    /// directly with [`TrimmedCurve2::new`](super::trimmed::TrimmedCurve2::new).
    pub fn interval_between(&self, start: Point2, end: Point2) -> Interval {
        let t0 = self.param_at(start);
        let raw_t1 = self.param_at(end);
        match self.periodicity() {
            Periodicity::Periodic(period) => {
                let delta = if (end - start).norm() <= LINEAR_TOLERANCE {
                    period
                } else {
                    (raw_t1 - t0).rem_euclid(period)
                };
                Interval::new(t0, t0 + delta)
            }
            Periodicity::None => Interval::new(t0, raw_t1),
        }
    }

    /// Returns an exact NURBS segment over an interval in this support's native
    /// parameterization, renormalized to `[0, 1]` for synchronized uses.
    pub fn trimmed_native(&self, interval: Interval) -> Result<Self, NurbsError> {
        let nurbs = match self {
            Curve2::Circle(circle) => conic_arc_nurbs2(
                interval.start,
                interval.end,
                FRAC_PI_2,
                |parameter| circle.point_at(parameter),
                |parameter| circle.derivative_at(parameter, 1),
            )?,
            Curve2::Ellipse(ellipse) => conic_arc_nurbs2(
                interval.start,
                interval.end,
                FRAC_PI_2,
                |parameter| ellipse.point_at(parameter),
                |parameter| ellipse.derivative_at(parameter, 1),
            )?,
            Curve2::Line(_) | Curve2::Nurbs(_) => {
                if interval.end < interval.start {
                    return Ok(Curve2::Nurbs(
                        self.trimmed_native(interval.reversed())?
                            .to_nurbs()?
                            .reversed(),
                    ));
                }
                self.to_nurbs()?.trimmed(interval.start, interval.end)?
            }
        };
        Ok(Curve2::Nurbs(renormalized(nurbs)?))
    }

    /// Returns the same support traversed in the opposite direction.
    ///
    /// Analytic variants stay analytic, so reversing never degrades support
    /// identity. The parameterization is **not** preserved: a parameter
    /// computed on the source has to be recomputed on the result.
    pub fn reversed(&self) -> Self {
        match self {
            Curve2::Line(line) => Curve2::Line(line.reversed()),
            Curve2::Circle(circle) => Curve2::Circle(circle.reversed()),
            Curve2::Ellipse(ellipse) => Curve2::Ellipse(ellipse.reversed()),
            Curve2::Nurbs(curve) => Curve2::Nurbs(curve.reversed()),
        }
    }

    /// Returns an exact Cartesian translation of this support.
    ///
    /// The parameterization is preserved, so a span computed on the source
    /// stays valid on the result.
    pub fn translated(&self, offset: Vector2<f64>) -> Result<Self, NurbsError> {
        match self {
            Curve2::Line(line) => Ok(Curve2::Line(line.translated(offset))),
            Curve2::Circle(circle) => Ok(Curve2::Circle(circle.translated(offset))),
            Curve2::Ellipse(ellipse) => Ok(Curve2::Ellipse(ellipse.translated(offset))),
            Curve2::Nurbs(curve) => Ok(Curve2::Nurbs(curve.translated(offset)?)),
        }
    }
}

/// Renormalizes a NURBS curve's knot vector onto `[0, 1]`.
fn renormalized(nurbs: NurbsCurve2) -> Result<NurbsCurve2, NurbsError> {
    let domain = nurbs.domain();
    let extent = domain.end - domain.start;
    let knots = KnotVector::new(
        nurbs
            .knots()
            .as_slice()
            .iter()
            .map(|knot| (knot - domain.start) / extent)
            .collect(),
    )?;
    NurbsCurve2::new(nurbs.degree(), nurbs.control_points().clone(), knots)
}

/// Samples used when a support has no closed-form arc length.
const LENGTH_SAMPLES: usize = 64;

/// Accumulates a polyline approximation of arc length between two parameters.
fn polyline_length(point_at: impl Fn(f64) -> Point2, t0: f64, t1: f64) -> f64 {
    let span = Interval::new(t0, t1);
    (0..LENGTH_SAMPLES)
        .map(|index| {
            let a = point_at(span.at(index as f64 / LENGTH_SAMPLES as f64));
            let b = point_at(span.at((index + 1) as f64 / LENGTH_SAMPLES as f64));
            (b - a).norm()
        })
        .sum()
}

/// Samples used to seed the closest-point search on a NURBS curve.
const CLOSEST_POINT_SAMPLES: usize = 64;

/// Returns the native parameter of the NURBS point nearest `point`.
fn closest_sample_parameter(curve: &NurbsCurve2, point: Point2) -> f64 {
    if let Some(parameter) = curve.parameter_at(point, LINEAR_TOLERANCE) {
        return parameter;
    }
    let domain = curve.domain();
    (0..=CLOSEST_POINT_SAMPLES)
        .map(|index| domain.at(index as f64 / CLOSEST_POINT_SAMPLES as f64))
        .map(|parameter| (parameter, (curve.point_at(parameter) - point).norm()))
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(parameter, _)| parameter)
        .unwrap_or(domain.start)
}

/// An infinite straight support in 2D parameter space.
///
/// The parameter is affine, not arc length: it counts the construction vector
/// given to [`Line2::new`], so a line built from `end - start` places `0` at
/// `start` and `1` at `end` while still extending past both in either
/// direction.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Line2 {
    origin: Point2,
    direction: UnitVector2<f64>,
    scale: f64,
}

impl Line2 {
    /// Creates the infinite line through `origin` along `direction`.
    ///
    /// `direction` is the parameter's unit rather than a bare heading:
    /// `point_at(1)` is `origin + direction`, so a line built from `end - start`
    /// places `0` at `start` and `1` at `end`. The line still runs to infinity
    /// both ways; the vector only says how fast the parameter travels.
    ///
    /// A zero vector names no line, so rather than normalize it into `NaN` —
    /// which would quietly poison every later comparison — the result collapses
    /// to the constant `origin`. A caller measuring a candidate against real
    /// geometry then sees a plainly wrong curve instead of `NaN`, which is the
    /// outcome it can actually act on.
    pub fn new(origin: Point2, direction: Vector2<f64>) -> Self {
        let scale = direction.norm();
        if scale <= LINEAR_TOLERANCE {
            return Self::with_scale(origin, UnitVector2::new_unchecked(Vector2::x()), 0.0);
        }
        Self {
            origin,
            direction: UnitVector2::new_unchecked(direction / scale),
            scale,
        }
    }

    fn with_scale(origin: Point2, direction: UnitVector2<f64>, scale: f64) -> Self {
        Self {
            origin,
            direction,
            scale,
        }
    }

    pub fn origin(&self) -> Point2 {
        self.origin
    }

    pub fn direction(&self) -> UnitVector2<f64> {
        self.direction
    }

    /// Evaluates the line at a native (affine) parameter.
    pub fn point_at(&self, t: f64) -> Point2 {
        self.origin + *self.direction * (self.scale * t)
    }

    pub fn derivative_at(&self, t: f64, order: usize) -> Vector2<f64> {
        match order {
            0 => self.point_at(t).coords,
            1 => *self.direction * self.scale,
            _ => Vector2::zeros(),
        }
    }

    /// Returns the parameter of the line point nearest `point`.
    pub fn param_at(&self, point: Point2) -> f64 {
        if self.scale.abs() <= LINEAR_TOLERANCE {
            return 0.0;
        }
        (point - self.origin).dot(&self.direction) / self.scale
    }

    pub fn project(&self, point: Point2) -> Point2 {
        self.point_at(self.param_at(point))
    }

    /// Returns the distance travelled between two parameters.
    pub fn length(&self, t0: f64, t1: f64) -> f64 {
        (t1 - t0).abs() * self.scale.abs()
    }

    /// Returns the same line traversed in the opposite direction.
    pub fn reversed(&self) -> Self {
        Self::with_scale(self.origin, -self.direction, self.scale)
    }

    pub fn translated(&self, offset: Vector2<f64>) -> Self {
        Self::with_scale(self.origin + offset, self.direction, self.scale)
    }

    /// Converts the `[0, 1]` window of the line to an exact degree-1 NURBS curve.
    pub fn to_nurbs(&self) -> Result<NurbsCurve2, NurbsError> {
        NurbsCurve2::new(
            Degree::new(1)?,
            ControlPolygon2::new(vec![
                HPoint2::from_cartesian(self.point_at(0.0), 1.0),
                HPoint2::from_cartesian(self.point_at(1.0), 1.0),
            ])?,
            KnotVector::new(vec![0.0, 0.0, 1.0, 1.0])?,
        )
    }
}

/// A full circle in 2D parameter space, parameterized by angle in radians.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Circle2 {
    center: Point2,
    x_dir: UnitVector2<f64>,
    /// The direction reached a quarter turn after `x_dir`.
    ///
    /// Stored rather than derived because it is what carries the circle's
    /// sense: [`reversed`](Self::reversed) negates it, the 2D analogue of
    /// flipping a 3D circle's plane normal.
    y_dir: UnitVector2<f64>,
    radius: f64,
}

impl Circle2 {
    /// Creates the counter-clockwise circle starting along `x_dir`.
    pub fn new(center: Point2, x_dir: Vector2<f64>, radius: f64) -> Self {
        let x_dir = UnitVector2::new_normalize(x_dir);
        Self {
            center,
            y_dir: UnitVector2::new_unchecked(Vector2::new(-x_dir.y, x_dir.x)),
            x_dir,
            radius,
        }
    }

    fn with_dirs(
        center: Point2,
        x_dir: UnitVector2<f64>,
        y_dir: UnitVector2<f64>,
        radius: f64,
    ) -> Self {
        Self {
            center,
            x_dir,
            y_dir,
            radius,
        }
    }

    pub fn center(&self) -> Point2 {
        self.center
    }

    pub fn x_dir(&self) -> UnitVector2<f64> {
        self.x_dir
    }

    pub fn y_dir(&self) -> UnitVector2<f64> {
        self.y_dir
    }

    pub fn radius(&self) -> f64 {
        self.radius
    }

    /// Evaluates the circle at an angle in radians.
    pub fn point_at(&self, t: f64) -> Point2 {
        let (sin, cos) = t.sin_cos();
        self.center + self.radius * (cos * *self.x_dir + sin * *self.y_dir)
    }

    pub fn derivative_at(&self, t: f64, order: usize) -> Vector2<f64> {
        if order == 0 {
            return self.point_at(t).coords;
        }
        let (sin, cos) = t.sin_cos();
        let x = *self.x_dir * self.radius;
        let y = *self.y_dir * self.radius;
        match order % 4 {
            0 => x * cos + y * sin,
            1 => -x * sin + y * cos,
            2 => -x * cos - y * sin,
            _ => x * sin - y * cos,
        }
    }

    /// Returns the angle of the circle point nearest `point`, in `(-pi, pi]`.
    pub fn param_at(&self, point: Point2) -> f64 {
        let radial = point - self.center;
        radial.dot(&self.y_dir).atan2(radial.dot(&self.x_dir))
    }

    pub fn project(&self, point: Point2) -> Point2 {
        let radial = point - self.center;
        if radial.norm() <= LINEAR_TOLERANCE {
            return self.center + *self.x_dir * self.radius;
        }
        self.center + radial * (self.radius / radial.norm())
    }

    /// Returns the arc length swept between two angles.
    pub fn length(&self, t0: f64, t1: f64) -> f64 {
        (t1 - t0).abs() * self.radius.abs()
    }

    /// Returns the circle traversed in the opposite sense.
    pub fn reversed(&self) -> Self {
        Self::with_dirs(self.center, self.x_dir, -self.y_dir, self.radius)
    }

    pub fn translated(&self, offset: Vector2<f64>) -> Self {
        Self::with_dirs(self.center + offset, self.x_dir, self.y_dir, self.radius)
    }

    /// Converts one full turn to an exact rational quadratic NURBS curve.
    pub fn to_nurbs(&self) -> Result<NurbsCurve2, NurbsError> {
        conic_arc_nurbs2(
            0.0,
            TAU,
            FRAC_PI_2,
            |parameter| self.point_at(parameter),
            |parameter| self.derivative_at(parameter, 1),
        )
    }
}

/// A full ellipse in 2D parameter space, parameterized by angle in radians.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Ellipse2 {
    center: Point2,
    x_dir: UnitVector2<f64>,
    /// The minor-axis direction; carries the ellipse's sense.
    y_dir: UnitVector2<f64>,
    major_radius: f64,
    minor_radius: f64,
}

impl Ellipse2 {
    /// Creates the counter-clockwise ellipse whose major axis lies along `x_dir`.
    pub fn new(center: Point2, x_dir: Vector2<f64>, major_radius: f64, minor_radius: f64) -> Self {
        let x_dir = UnitVector2::new_normalize(x_dir);
        Self {
            center,
            y_dir: UnitVector2::new_unchecked(Vector2::new(-x_dir.y, x_dir.x)),
            x_dir,
            major_radius,
            minor_radius,
        }
    }

    fn with_dirs(
        center: Point2,
        x_dir: UnitVector2<f64>,
        y_dir: UnitVector2<f64>,
        major_radius: f64,
        minor_radius: f64,
    ) -> Self {
        Self {
            center,
            x_dir,
            y_dir,
            major_radius,
            minor_radius,
        }
    }

    pub fn center(&self) -> Point2 {
        self.center
    }

    pub fn x_dir(&self) -> UnitVector2<f64> {
        self.x_dir
    }

    pub fn y_dir(&self) -> UnitVector2<f64> {
        self.y_dir
    }

    pub fn major_radius(&self) -> f64 {
        self.major_radius
    }

    pub fn minor_radius(&self) -> f64 {
        self.minor_radius
    }

    /// Evaluates the ellipse at an eccentric angle in radians.
    pub fn point_at(&self, t: f64) -> Point2 {
        let (sin, cos) = t.sin_cos();
        self.center
            + *self.x_dir * (self.major_radius * cos)
            + *self.y_dir * (self.minor_radius * sin)
    }

    pub fn derivative_at(&self, t: f64, order: usize) -> Vector2<f64> {
        if order == 0 {
            return self.point_at(t).coords;
        }
        let (sin, cos) = t.sin_cos();
        let x = *self.x_dir * self.major_radius;
        let y = *self.y_dir * self.minor_radius;
        match order % 4 {
            0 => x * cos + y * sin,
            1 => -x * sin + y * cos,
            2 => -x * cos - y * sin,
            _ => x * sin - y * cos,
        }
    }

    /// Returns the eccentric angle of the ellipse point matching `point`.
    ///
    /// This inverts the parameterization rather than minimizing distance: on
    /// the ellipse the two agree, which is the case the kernel asks about.
    pub fn param_at(&self, point: Point2) -> f64 {
        let offset = point - self.center;
        let x = offset.dot(&self.x_dir) / self.major_radius;
        let y = offset.dot(&self.y_dir) / self.minor_radius;
        y.atan2(x)
    }

    pub fn project(&self, point: Point2) -> Point2 {
        self.point_at(self.param_at(point))
    }

    /// Approximates the arc length swept between two angles.
    pub fn length(&self, t0: f64, t1: f64) -> f64 {
        polyline_length(|t| self.point_at(t), t0, t1)
    }

    /// Returns the ellipse traversed in the opposite sense.
    pub fn reversed(&self) -> Self {
        Self::with_dirs(
            self.center,
            self.x_dir,
            -self.y_dir,
            self.major_radius,
            self.minor_radius,
        )
    }

    pub fn translated(&self, offset: Vector2<f64>) -> Self {
        Self::with_dirs(
            self.center + offset,
            self.x_dir,
            self.y_dir,
            self.major_radius,
            self.minor_radius,
        )
    }

    /// Converts one full turn to an exact rational quadratic NURBS curve.
    pub fn to_nurbs(&self) -> Result<NurbsCurve2, NurbsError> {
        conic_arc_nurbs2(
            0.0,
            TAU,
            FRAC_PI_2,
            |parameter| self.point_at(parameter),
            |parameter| self.derivative_at(parameter, 1),
        )
    }
}

impl Curve2Geometry for Line2 {
    fn domain(&self) -> Interval {
        Interval::unbounded()
    }

    fn periodicity(&self) -> Periodicity {
        Periodicity::None
    }

    fn point_at(&self, t: f64) -> Point2 {
        Line2::point_at(self, t)
    }

    fn derivative_at(&self, t: f64, order: usize) -> Vector2<f64> {
        Line2::derivative_at(self, t, order)
    }

    fn param_at(&self, point: Point2) -> f64 {
        Line2::param_at(self, point)
    }

    fn project(&self, point: Point2) -> Point2 {
        Line2::project(self, point)
    }

    fn length(&self, t0: f64, t1: f64) -> f64 {
        Line2::length(self, t0, t1)
    }

    fn reversed(&self) -> Self {
        Line2::reversed(self)
    }

    fn translated(&self, offset: Vector2<f64>) -> Result<Self, NurbsError> {
        Ok(Line2::translated(self, offset))
    }

    fn to_nurbs(&self) -> Result<NurbsCurve2, NurbsError> {
        Line2::to_nurbs(self)
    }
}

impl Curve2Geometry for Circle2 {
    fn domain(&self) -> Interval {
        Interval::new(0.0, TAU)
    }

    fn periodicity(&self) -> Periodicity {
        Periodicity::Periodic(TAU)
    }

    fn point_at(&self, t: f64) -> Point2 {
        Circle2::point_at(self, t)
    }

    fn derivative_at(&self, t: f64, order: usize) -> Vector2<f64> {
        Circle2::derivative_at(self, t, order)
    }

    fn param_at(&self, point: Point2) -> f64 {
        Circle2::param_at(self, point)
    }

    fn project(&self, point: Point2) -> Point2 {
        Circle2::project(self, point)
    }

    fn length(&self, t0: f64, t1: f64) -> f64 {
        Circle2::length(self, t0, t1)
    }

    fn reversed(&self) -> Self {
        Circle2::reversed(self)
    }

    fn translated(&self, offset: Vector2<f64>) -> Result<Self, NurbsError> {
        Ok(Circle2::translated(self, offset))
    }

    fn to_nurbs(&self) -> Result<NurbsCurve2, NurbsError> {
        Circle2::to_nurbs(self)
    }
}

impl Curve2Geometry for Ellipse2 {
    fn domain(&self) -> Interval {
        Interval::new(0.0, TAU)
    }

    fn periodicity(&self) -> Periodicity {
        Periodicity::Periodic(TAU)
    }

    fn point_at(&self, t: f64) -> Point2 {
        Ellipse2::point_at(self, t)
    }

    fn derivative_at(&self, t: f64, order: usize) -> Vector2<f64> {
        Ellipse2::derivative_at(self, t, order)
    }

    fn param_at(&self, point: Point2) -> f64 {
        Ellipse2::param_at(self, point)
    }

    fn project(&self, point: Point2) -> Point2 {
        Ellipse2::project(self, point)
    }

    fn length(&self, t0: f64, t1: f64) -> f64 {
        Ellipse2::length(self, t0, t1)
    }

    fn reversed(&self) -> Self {
        Ellipse2::reversed(self)
    }

    fn translated(&self, offset: Vector2<f64>) -> Result<Self, NurbsError> {
        Ok(Ellipse2::translated(self, offset))
    }

    fn to_nurbs(&self) -> Result<NurbsCurve2, NurbsError> {
        Ellipse2::to_nurbs(self)
    }
}

/// Forwards to whichever variant the support holds.
///
/// The inherent methods on [`Curve2`] shadow these, so call sites keep working
/// without importing the trait; the impl exists so generic code can be written
/// once over any parameter-space support.
impl Curve2Geometry for Curve2 {
    fn domain(&self) -> Interval {
        Curve2::domain(self)
    }

    fn periodicity(&self) -> Periodicity {
        Curve2::periodicity(self)
    }

    fn point_at(&self, t: f64) -> Point2 {
        Curve2::point_at(self, t)
    }

    fn derivative_at(&self, t: f64, order: usize) -> Vector2<f64> {
        Curve2::derivative_at(self, t, order)
    }

    fn param_at(&self, point: Point2) -> f64 {
        Curve2::param_at(self, point)
    }

    fn project(&self, point: Point2) -> Point2 {
        Curve2::project(self, point)
    }

    fn length(&self, t0: f64, t1: f64) -> f64 {
        Curve2::length(self, t0, t1)
    }

    fn reversed(&self) -> Self {
        Curve2::reversed(self)
    }

    fn translated(&self, offset: Vector2<f64>) -> Result<Self, NurbsError> {
        Curve2::translated(self, offset)
    }

    fn to_nurbs(&self) -> Result<NurbsCurve2, NurbsError> {
        Curve2::to_nurbs(self)
    }
}
