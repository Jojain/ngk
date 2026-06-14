use nalgebra::Vector2;

use super::nurbs::{ControlPolygon2, NurbsCurve2};
use crate::geometry::{Degree, Interval, KnotVector, LINEAR_TOLERANCE, NurbsError, Point2};

#[derive(Debug, Clone)]
pub(crate) struct Bezier2 {
    degree: Degree,
    control_points: ControlPolygon2,
    domain: Interval,
}

impl Bezier2 {
    pub(crate) fn new(
        degree: Degree,
        control_points: ControlPolygon2,
        domain: Interval,
    ) -> Result<Self, NurbsError> {
        let expected = degree.get() + 1;
        if control_points.len() != expected {
            return Err(NurbsError::BezierControlPointCountMismatch {
                expected,
                got: control_points.len(),
            });
        }
        if domain.is_degenerate(LINEAR_TOLERANCE) {
            return Err(NurbsError::DegenerateInterval {
                start: domain.start,
                end: domain.end,
            });
        }
        Ok(Self {
            degree,
            control_points,
            domain,
        })
    }

    pub(crate) fn degree(&self) -> Degree {
        self.degree
    }

    pub(crate) fn control_points(&self) -> &ControlPolygon2 {
        &self.control_points
    }

    pub(crate) fn domain(&self) -> Interval {
        self.domain
    }

    pub(crate) fn point_at(&self, parameter: f64) -> Point2 {
        self.to_nurbs()
            .expect("valid Bezier data must build a NURBS curve")
            .point_at(parameter)
    }

    pub(crate) fn derivative_at(&self, parameter: f64, order: usize) -> Vector2<f64> {
        self.to_nurbs()
            .expect("valid Bezier data must build a NURBS curve")
            .derivative_at(parameter, order)
    }

    pub(crate) fn parameter_at(&self, point: Point2, tolerance: f64) -> Option<f64> {
        self.to_nurbs()
            .expect("valid Bezier data must build a NURBS curve")
            .parameter_at(point, tolerance)
    }

    pub(crate) fn subdivide(&self, parameter: f64) -> Result<(Self, Self), NurbsError> {
        if !self.domain.contains(parameter, LINEAR_TOLERANCE) {
            return Err(NurbsError::ParameterOutOfRange {
                u: parameter,
                min: self.domain.start,
                max: self.domain.end,
            });
        }
        let (left, right) = self.to_nurbs()?.split_at(parameter)?;
        Ok((
            Self::new(
                self.degree,
                left.control_points().clone(),
                Interval::new(self.domain.start, parameter),
            )?,
            Self::new(
                self.degree,
                right.control_points().clone(),
                Interval::new(parameter, self.domain.end),
            )?,
        ))
    }

    fn to_nurbs(&self) -> Result<NurbsCurve2, NurbsError> {
        let knot_count = self.degree.get() + 1;
        let mut knots = Vec::with_capacity(2 * knot_count);
        knots.extend(std::iter::repeat_n(self.domain.start, knot_count));
        knots.extend(std::iter::repeat_n(self.domain.end, knot_count));
        NurbsCurve2::new(
            self.degree,
            self.control_points.clone(),
            KnotVector::new(knots)?,
        )
    }
}
