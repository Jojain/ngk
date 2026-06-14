use crate::geometry::{Interval, LINEAR_TOLERANCE, NurbsError};

use super::nurbs::NurbsCurve2;
use super::utils::Point2;

/// A curve in a surface's 2D parameter space.
#[derive(Debug, Clone, PartialEq)]
pub enum Curve2 {
    Line(Line2),
    Nurbs(NurbsCurve2),
}

impl Curve2 {
    /// Evaluates the curve using a normalized parameter in `[0, 1]`.
    pub fn point_at(&self, parameter: f64) -> Point2 {
        match self {
            Curve2::Line(line) => line.point_at(parameter),
            Curve2::Nurbs(curve) => curve.point_at(native_parameter(curve.domain(), parameter)),
        }
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

    /// Recovers the normalized parameter of a coincident point.
    pub fn parameter_at(&self, point: Point2, tolerance: f64) -> Option<f64> {
        match self {
            Curve2::Line(line) => line.parameter_at(point, tolerance),
            Curve2::Nurbs(curve) => curve
                .parameter_at(point, tolerance)
                .map(|parameter| normalized_parameter(curve.domain(), parameter)),
        }
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
