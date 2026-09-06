use std::f64::consts::{FRAC_PI_2, TAU};

use crate::geometry::{
    ControlPolygon2, Degree, HPoint2, Interval, KnotVector, LINEAR_TOLERANCE, NurbsError,
};
use nalgebra::{UnitVector2, Vector2};
use serde::{Deserialize, Serialize};

use super::intersections::{
    CurveCurveIntersections2, CurveIntersectionError, CurveIntersectionOptions, intersect_curves,
    intersect_curves_with_options,
};
use super::nurbs::NurbsCurve2;
use super::utils::Point2;
use crate::geometry::traits::Curve2Geometry;

/// A curve in a surface's 2D parameter space.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Curve2 {
    Line(Line2),
    Circle(Circle2),
    Nurbs(NurbsCurve2),
}

impl Curve2 {
    /// Converts the curve to an exact 2D NURBS representation.
    pub fn to_nurbs(&self) -> Result<NurbsCurve2, NurbsError> {
        match self {
            Curve2::Line(line) => line.to_nurbs(),
            Curve2::Circle(circle) => circle.to_nurbs(),
            Curve2::Nurbs(curve) => Ok(curve.clone()),
        }
    }

    /// Evaluates the curve using a normalized parameter in `[0, 1]`.
    pub fn point_at(&self, parameter: f64) -> Point2 {
        match self {
            Curve2::Line(line) => line.point_at(parameter),
            Curve2::Circle(circle) => circle.point_at(parameter),
            Curve2::Nurbs(curve) => curve.point_at(native_parameter(curve.domain(), parameter)),
        }
    }

    /// Returns whether the curve is geometrically closed (start coincides with end).
    pub fn is_closed(&self) -> bool {
        (self.point_at(0.0) - self.point_at(1.0)).norm() <= LINEAR_TOLERANCE
    }

    /// Returns `segments + 1` uniformly parameterized points.
    pub fn sample(&self, segments: usize) -> Vec<Point2> {
        let segments = segments.max(1);
        (0..=segments)
            .map(|index| self.point_at(index as f64 / segments as f64))
            .collect()
    }

    /// Samples the curve adaptively and returns normalized parameters.
    pub fn adaptive_samples(&self, tolerance: f64, max_depth: usize) -> Vec<(f64, Point2)> {
        match self {
            Curve2::Line(line) => vec![(0.0, line.start), (1.0, line.end)],
            Curve2::Circle(circle) => circle.adaptive_samples(tolerance, max_depth),
            Curve2::Nurbs(curve) => {
                let domain = curve.domain();
                curve
                    .adaptive_samples(tolerance, max_depth)
                    .into_iter()
                    .map(|(parameter, point)| (normalized_parameter(domain, parameter), point))
                    .collect()
            }
        }
    }

    /// Returns the same curve with reversed direction.
    pub fn reversed(&self) -> Self {
        match self {
            Curve2::Line(line) => Curve2::Line(line.reversed()),
            Curve2::Circle(circle) => Curve2::Circle(circle.reversed()),
            Curve2::Nurbs(curve) => Curve2::Nurbs(curve.reversed()),
        }
    }

    /// Returns an exact Cartesian translation of this curve.
    pub fn translated(&self, offset: Vector2<f64>) -> Result<Self, NurbsError> {
        match self {
            Curve2::Line(line) => Ok(Curve2::Line(Line2::new(
                line.start + offset,
                line.end + offset,
            ))),
            Curve2::Circle(circle) => Ok(Curve2::Circle(circle.translated(offset))),
            Curve2::Nurbs(curve) => Ok(Curve2::Nurbs(curve.translated(offset)?)),
        }
    }

    /// Splits the curve at an interior normalized parameter.
    pub fn split_at(&self, parameter: f64) -> Result<(Self, Self), NurbsError> {
        if parameter <= LINEAR_TOLERANCE || parameter >= 1.0 - LINEAR_TOLERANCE {
            return Err(NurbsError::DegenerateInterval {
                start: 0.0,
                end: parameter,
            });
        }
        match self {
            Curve2::Line(line) => {
                let (first, second) = line.split_at(parameter);
                Ok((Curve2::Line(first), Curve2::Line(second)))
            }
            Curve2::Circle(circle) => {
                let (first, second) = circle.split_at(parameter);
                Ok((Curve2::Circle(first), Curve2::Circle(second)))
            }
            Curve2::Nurbs(curve) => {
                let (first, second) =
                    curve.split_at(native_parameter(curve.domain(), parameter))?;
                Ok((Curve2::Nurbs(first), Curve2::Nurbs(second)))
            }
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
            return Ok(self
                .trimmed(Interval::new(interval.end, interval.start))?
                .reversed());
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

        match self {
            Curve2::Line(line) => Ok(Curve2::Line(Line2::new(
                line.point_at(start),
                line.point_at(end),
            ))),
            Curve2::Circle(circle) => Ok(Curve2::Circle(circle.trimmed(start, end))),
            Curve2::Nurbs(curve) => {
                let domain = curve.domain();
                Ok(Curve2::Nurbs(curve.trimmed(
                    native_parameter(domain, start),
                    native_parameter(domain, end),
                )?))
            }
        }
    }

    /// Recovers the normalized parameter of a coincident point.
    pub fn parameter_at(&self, point: Point2, tolerance: f64) -> Option<f64> {
        match self {
            Curve2::Line(line) => line.parameter_at(point, tolerance),
            Curve2::Circle(circle) => circle.parameter_at(point, tolerance),
            Curve2::Nurbs(curve) => curve
                .parameter_at(point, tolerance)
                .map(|parameter| normalized_parameter(curve.domain(), parameter)),
        }
    }

    /// Intersects this curve with another curve using default tolerances.
    ///
    /// Returned parameters and intervals are normalized to each `Curve2`'s
    /// public `[0, 1]` parameter domain.
    pub fn intersect_curve(
        &self,
        other: &Curve2,
    ) -> Result<CurveCurveIntersections2, CurveIntersectionError> {
        intersect_curves(self, other)
    }

    /// Intersects this curve with another curve using explicit tolerances.
    ///
    /// Returned parameters and intervals are normalized to each `Curve2`'s
    /// public `[0, 1]` parameter domain.
    pub fn intersect_curve_with_options(
        &self,
        other: &Curve2,
        options: CurveIntersectionOptions,
    ) -> Result<CurveCurveIntersections2, CurveIntersectionError> {
        intersect_curves_with_options(self, other, options)
    }
}

/// A bounded circular arc in 2D, evaluated over the normalized `[0, 1]` domain.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Circle2 {
    center: Point2,
    x_dir: UnitVector2<f64>,
    radius: f64,
    sweep: f64,
}

impl Circle2 {
    /// Creates an arc starting along `x_dir` and rotating by `sweep` radians.
    pub fn new(center: Point2, x_dir: Vector2<f64>, radius: f64, sweep: f64) -> Self {
        Self {
            center,
            x_dir: UnitVector2::new_normalize(x_dir),
            radius,
            sweep,
        }
    }

    pub fn center(&self) -> Point2 {
        self.center
    }

    pub fn radius(&self) -> f64 {
        self.radius
    }

    pub fn sweep(&self) -> f64 {
        self.sweep
    }

    /// Evaluates the arc using a normalized parameter.
    pub fn point_at(&self, parameter: f64) -> Point2 {
        let angle = self.sweep * parameter;
        let y_dir = Vector2::new(-self.x_dir.y, self.x_dir.x);
        self.center + self.radius * (angle.cos() * *self.x_dir + angle.sin() * y_dir)
    }

    /// Returns adaptive samples with a chord sag bounded by `tolerance`.
    pub fn adaptive_samples(&self, tolerance: f64, max_depth: usize) -> Vec<(f64, Point2)> {
        let ratio = (1.0 - tolerance / self.radius.abs().max(tolerance)).clamp(-1.0, 1.0);
        let angle_step = (2.0 * ratio.acos()).max(1.0e-6);
        let depth_limit = 1usize.checked_shl(max_depth.min(20) as u32).unwrap_or(1);
        let segments = ((self.sweep.abs() / angle_step).ceil() as usize)
            .max(1)
            .min(depth_limit);
        (0..=segments)
            .map(|index| {
                let parameter = index as f64 / segments as f64;
                (parameter, self.point_at(parameter))
            })
            .collect()
    }

    pub fn reversed(&self) -> Self {
        Self::new(
            self.center,
            self.point_at(1.0) - self.center,
            self.radius,
            -self.sweep,
        )
    }

    pub fn translated(&self, offset: Vector2<f64>) -> Self {
        Self {
            center: self.center + offset,
            ..self.clone()
        }
    }

    pub fn split_at(&self, parameter: f64) -> (Self, Self) {
        let parameter = parameter.clamp(0.0, 1.0);
        (self.trimmed(0.0, parameter), self.trimmed(parameter, 1.0))
    }

    pub fn trimmed(&self, start: f64, end: f64) -> Self {
        Self::new(
            self.center,
            self.point_at(start) - self.center,
            self.radius,
            self.sweep * (end - start),
        )
    }

    /// Recovers the normalized parameter of a coincident point on the arc.
    pub fn parameter_at(&self, point: Point2, tolerance: f64) -> Option<f64> {
        let radial = point - self.center;
        if (radial.norm() - self.radius).abs() > tolerance {
            return None;
        }
        let y_dir = Vector2::new(-self.x_dir.y, self.x_dir.x);
        let mut angle = radial.dot(&y_dir).atan2(radial.dot(&self.x_dir));
        if self.sweep > 0.0 {
            while angle < 0.0 {
                angle += TAU;
            }
        } else {
            while angle > 0.0 {
                angle -= TAU;
            }
        }
        let parameter = angle / self.sweep;
        if !(-tolerance..=1.0 + tolerance).contains(&parameter) {
            return None;
        }
        let parameter = parameter.clamp(0.0, 1.0);
        ((self.point_at(parameter) - point).norm() <= tolerance).then_some(parameter)
    }

    /// Converts the arc to an exact rational quadratic NURBS representation.
    pub fn to_nurbs(&self) -> Result<NurbsCurve2, NurbsError> {
        if self.sweep.abs() <= LINEAR_TOLERANCE {
            return Err(NurbsError::DegenerateInterval {
                start: 0.0,
                end: self.sweep,
            });
        }
        let segment_count = (self.sweep.abs() / FRAC_PI_2).ceil() as usize;
        let segment_sweep = self.sweep / segment_count as f64;
        let middle_weight = (0.5 * segment_sweep).cos();
        let mut points = Vec::with_capacity(segment_count * 2 + 1);
        let mut knots = vec![0.0; 3];
        points.push(HPoint2::from_cartesian(self.point_at(0.0), 1.0));
        for segment in 0..segment_count {
            let middle = (segment as f64 + 0.5) / segment_count as f64;
            let end = (segment + 1) as f64 / segment_count as f64;
            let middle_point = self.center + (self.point_at(middle) - self.center) / middle_weight;
            points.push(HPoint2::from_cartesian(middle_point, middle_weight));
            points.push(HPoint2::from_cartesian(self.point_at(end), 1.0));
            if segment + 1 < segment_count {
                knots.extend([end, end]);
            }
        }
        knots.extend([1.0, 1.0, 1.0]);
        NurbsCurve2::new(
            Degree::new(2)?,
            ControlPolygon2::new(points)?,
            KnotVector::new(knots)?,
        )
    }
}

/// A bounded straight segment in 2D parameter space.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Line2 {
    pub start: Point2,
    pub end: Point2,
}

impl Line2 {
    /// Creates a line segment from its endpoints.
    pub fn new(start: Point2, end: Point2) -> Self {
        Self { start, end }
    }

    /// Evaluates the segment using a normalized parameter.
    pub fn point_at(&self, parameter: f64) -> Point2 {
        self.start + (self.end - self.start) * parameter
    }

    /// Returns `segments + 1` uniformly spaced samples.
    pub fn sample(&self, segments: usize) -> Vec<Point2> {
        let segments = segments.max(1);
        (0..=segments)
            .map(|index| self.point_at(index as f64 / segments as f64))
            .collect()
    }

    /// Returns the segment with reversed direction.
    pub fn reversed(&self) -> Self {
        Self {
            start: self.end,
            end: self.start,
        }
    }

    /// Splits the segment at a normalized parameter.
    pub fn split_at(&self, parameter: f64) -> (Self, Self) {
        let point = self.point_at(parameter.clamp(0.0, 1.0));
        (Self::new(self.start, point), Self::new(point, self.end))
    }

    /// Recovers the normalized parameter of a coincident point.
    pub fn parameter_at(&self, point: Point2, tolerance: f64) -> Option<f64> {
        let direction = self.end - self.start;
        let length_squared = direction.norm_squared();
        if length_squared <= tolerance * tolerance {
            return None;
        }
        let parameter = (point - self.start).dot(&direction) / length_squared;
        if !(-tolerance..=1.0 + tolerance).contains(&parameter) {
            return None;
        }
        let parameter = parameter.clamp(0.0, 1.0);
        ((self.point_at(parameter) - point).norm() <= tolerance).then_some(parameter)
    }

    /// Converts the segment to an exact degree-1 NURBS curve over `[0, 1]`.
    pub fn to_nurbs(&self) -> Result<NurbsCurve2, NurbsError> {
        NurbsCurve2::new(
            Degree::new(1)?,
            ControlPolygon2::new(vec![
                HPoint2::from_cartesian(self.start, 1.0),
                HPoint2::from_cartesian(self.end, 1.0),
            ])?,
            KnotVector::new(vec![0.0, 0.0, 1.0, 1.0])?,
        )
    }

    /// Returns an exact Cartesian translation of this segment.
    pub fn translated(&self, offset: Vector2<f64>) -> Self {
        Self::new(self.start + offset, self.end + offset)
    }
}

fn native_parameter(domain: Interval, normalized: f64) -> f64 {
    domain.start + (domain.end - domain.start) * normalized.clamp(0.0, 1.0)
}

fn normalized_parameter(domain: Interval, native: f64) -> f64 {
    let length = domain.end - domain.start;
    if length.abs() <= LINEAR_TOLERANCE {
        0.0
    } else {
        (native - domain.start) / length
    }
}

impl Curve2Geometry for Line2 {
    fn point_at(&self, parameter: f64) -> Point2 {
        Line2::point_at(self, parameter)
    }

    /// A segment is exactly represented by its endpoints at any tolerance.
    fn adaptive_samples(&self, _tolerance: f64, _max_depth: usize) -> Vec<(f64, Point2)> {
        vec![(0.0, self.start), (1.0, self.end)]
    }

    fn parameter_at(&self, point: Point2, tolerance: f64) -> Option<f64> {
        Line2::parameter_at(self, point, tolerance)
    }

    fn reversed(&self) -> Self {
        Line2::reversed(self)
    }

    fn translated(&self, offset: Vector2<f64>) -> Result<Self, NurbsError> {
        Ok(Line2::translated(self, offset))
    }

    fn split_at(&self, parameter: f64) -> Result<(Self, Self), NurbsError> {
        Ok(Line2::split_at(self, parameter))
    }

    fn to_nurbs(&self) -> Result<NurbsCurve2, NurbsError> {
        Line2::to_nurbs(self)
    }
}

impl Curve2Geometry for Circle2 {
    fn point_at(&self, parameter: f64) -> Point2 {
        Circle2::point_at(self, parameter)
    }

    fn adaptive_samples(&self, tolerance: f64, max_depth: usize) -> Vec<(f64, Point2)> {
        Circle2::adaptive_samples(self, tolerance, max_depth)
    }

    fn parameter_at(&self, point: Point2, tolerance: f64) -> Option<f64> {
        Circle2::parameter_at(self, point, tolerance)
    }

    fn reversed(&self) -> Self {
        Circle2::reversed(self)
    }

    fn translated(&self, offset: Vector2<f64>) -> Result<Self, NurbsError> {
        Ok(Circle2::translated(self, offset))
    }

    fn split_at(&self, parameter: f64) -> Result<(Self, Self), NurbsError> {
        Ok(Circle2::split_at(self, parameter))
    }

    fn to_nurbs(&self) -> Result<NurbsCurve2, NurbsError> {
        Circle2::to_nurbs(self)
    }
}

/// Forwards to whichever variant the curve holds.
///
/// The inherent methods on [`Curve2`] shadow these, so call sites keep working
/// without importing the trait; the impl exists so generic code can be written
/// once over any parameter-space curve.
impl Curve2Geometry for Curve2 {
    fn point_at(&self, parameter: f64) -> Point2 {
        Curve2::point_at(self, parameter)
    }

    fn adaptive_samples(&self, tolerance: f64, max_depth: usize) -> Vec<(f64, Point2)> {
        Curve2::adaptive_samples(self, tolerance, max_depth)
    }

    fn parameter_at(&self, point: Point2, tolerance: f64) -> Option<f64> {
        Curve2::parameter_at(self, point, tolerance)
    }

    fn reversed(&self) -> Self {
        Curve2::reversed(self)
    }

    fn translated(&self, offset: Vector2<f64>) -> Result<Self, NurbsError> {
        Curve2::translated(self, offset)
    }

    fn split_at(&self, parameter: f64) -> Result<(Self, Self), NurbsError> {
        Curve2::split_at(self, parameter)
    }

    fn to_nurbs(&self) -> Result<NurbsCurve2, NurbsError> {
        Curve2::to_nurbs(self)
    }
}
