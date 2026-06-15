use crate::geometry::{
    ControlPolygon2, Degree, HPoint2, Interval, KnotVector, LINEAR_TOLERANCE, NurbsError,
};
use nalgebra::Vector2;

use super::intersections::{
    CurveCurveIntersections2, CurveIntersectionError, CurveIntersectionOptions, intersect_curves,
    intersect_curves_with_options,
};
use super::nurbs::NurbsCurve2;
use super::utils::Point2;

/// A curve in a surface's 2D parameter space.
#[derive(Debug, Clone, PartialEq)]
pub enum Curve2 {
    Line(Line2),
    Nurbs(NurbsCurve2),
}

impl Curve2 {
    /// Converts the curve to an exact 2D NURBS representation.
    pub fn to_nurbs(&self) -> Result<NurbsCurve2, NurbsError> {
        match self {
            Curve2::Line(line) => NurbsCurve2::new(
                Degree::new(1)?,
                ControlPolygon2::new(vec![
                    HPoint2::from_cartesian(line.start, 1.0),
                    HPoint2::from_cartesian(line.end, 1.0),
                ])?,
                KnotVector::new(vec![0.0, 0.0, 1.0, 1.0])?,
            ),
            Curve2::Nurbs(curve) => Ok(curve.clone()),
        }
    }

    /// Evaluates the curve using a normalized parameter in `[0, 1]`.
    pub fn point_at(&self, parameter: f64) -> Point2 {
        match self {
            Curve2::Line(line) => line.point_at(parameter),
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

/// A bounded straight segment in 2D parameter space.
#[derive(Debug, Clone, PartialEq)]
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
