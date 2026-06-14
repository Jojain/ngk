use super::curve::NurbsCurve;
use super::degree::Degree;
use super::knots::KnotVector;
use super::points::ControlPolygon;
use crate::geometry::nurbs::error::NurbsError;
use crate::geometry::{BBox, Interval, LINEAR_TOLERANCE, Point3};
use nalgebra::Vector3;

#[derive(Debug, Clone)]
pub struct Bezier {
    degree: Degree,
    control_points: ControlPolygon,
    domain: Interval,
}

impl Bezier {
    pub fn new(
        degree: Degree,
        control_points: ControlPolygon,
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

    pub fn degree(&self) -> Degree {
        self.degree
    }

    pub fn control_points(&self) -> &ControlPolygon {
        &self.control_points
    }

    pub fn domain(&self) -> Interval {
        self.domain
    }

    pub fn point_at(&self, u: f64) -> Point3 {
        self.to_nurbs()
            .expect("valid Bezier data must build a NURBS curve")
            .point_at(u)
    }

    pub fn derivative_at(&self, u: f64, order: usize) -> Vector3<f64> {
        self.to_nurbs()
            .expect("valid Bezier data must build a NURBS curve")
            .derivative_at(u, order)
    }

    pub fn bbox(&self) -> BBox {
        BBox::from_points(self.control_points.iter().map(|point| point.to_cartesian()))
    }

    pub fn subdivide(&self, u: f64) -> Result<(Self, Self), NurbsError> {
        if !self.domain.contains(u, LINEAR_TOLERANCE) {
            return Err(NurbsError::ParameterOutOfRange {
                u,
                min: self.domain.start,
                max: self.domain.end,
            });
        }
        if u <= self.domain.start + LINEAR_TOLERANCE || u >= self.domain.end - LINEAR_TOLERANCE {
            return Err(NurbsError::DegenerateInterval {
                start: self.domain.start,
                end: u,
            });
        }

        let mut curve = self.to_nurbs()?;
        for _ in 0..self.degree.get() {
            curve.insert_knot(u);
        }

        let points = curve.control_points().as_slice();
        let degree = self.degree.get();
        let left = ControlPolygon::new(points[0..=degree].to_vec())?;
        let right = ControlPolygon::new(points[degree..=(degree * 2)].to_vec())?;

        Ok((
            Self::new(self.degree, left, Interval::new(self.domain.start, u))?,
            Self::new(self.degree, right, Interval::new(u, self.domain.end))?,
        ))
    }

    fn to_nurbs(&self) -> Result<NurbsCurve, NurbsError> {
        let p = self.degree.get();
        let mut knots = Vec::with_capacity(2 * (p + 1));
        knots.extend(std::iter::repeat_n(self.domain.start, p + 1));
        knots.extend(std::iter::repeat_n(self.domain.end, p + 1));
        NurbsCurve::new(
            self.degree,
            self.control_points.clone(),
            KnotVector::new(knots)?,
        )
    }
}
