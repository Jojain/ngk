use nalgebra::{Point4, Vector4};

use super::basis::basis_functions;
use super::degree::Degree;
use super::error::NurbsError;
use super::knots::KnotVector;
use super::points::{ControlPolygon, HPoint};
use crate::geometry::utils::Point3;

#[derive(Debug, Clone)]
pub struct NurbsCurve {
    degree: Degree,
    control_points: ControlPolygon,
    knots: KnotVector,
}

impl NurbsCurve {
    pub fn new(
        degree: Degree,
        control_points: ControlPolygon,
        knots: KnotVector,
    ) -> Result<Self, NurbsError> {
        let expected = control_points.len() + degree.get() + 1;
        if knots.len() != expected {
            return Err(NurbsError::KnotCountMismatch {
                expected,
                got: knots.len(),
            });
        }
        Ok(Self {
            degree,
            control_points,
            knots,
        })
    }

    /// Build a NURBS curve with a default clamped-uniform knot vector.
    pub fn with_uniform_knots(
        degree: Degree,
        control_points: ControlPolygon,
    ) -> Result<Self, NurbsError> {
        let knots = KnotVector::uniform_clamped(control_points.len(), degree);
        Self::new(degree, control_points, knots)
    }

    pub fn degree(&self) -> Degree {
        self.degree
    }

    pub fn control_points(&self) -> &ControlPolygon {
        &self.control_points
    }

    pub fn control_points_mut(&mut self) -> &mut ControlPolygon {
        &mut self.control_points
    }

    pub fn knots(&self) -> &KnotVector {
        &self.knots
    }

    pub fn domain(&self) -> (f64, f64) {
        self.knots.domain(self.degree)
    }

    pub fn is_rational(&self) -> bool {
        let first = self.control_points.get(0).map(|hp| hp.weight());
        self.control_points.iter().any(|hp| match first {
            Some(w0) => (hp.weight() - w0).abs() > f64::EPSILON,
            None => false,
        })
    }

    pub fn point_at(&self, u: f64) -> Point3 {
        let p = self.degree.get();
        let n = self.control_points.len() - 1;
        let u = self.clamp_parameter(u);
        let span = self.knots.find_span(n, self.degree, u);
        let basis = basis_functions(span, u, self.degree, &self.knots);

        let mut acc = Point4::origin();
        for i in 0..=p {
            let hp = self.control_points.get(span - p + i).unwrap();
            let contrib: Vector4<f64> = hp.0.coords * basis[i];
            acc.coords += contrib;
        }
        HPoint(acc).to_cartesian()
    }

    fn clamp_parameter(&self, u: f64) -> f64 {
        let (min, max) = self.domain();
        u.clamp(min, max)
    }

    /// Piegl & Tiller A5.1 — insert the knot `u` once (increasing its
    /// multiplicity by 1) and add the corresponding new control point.
    pub fn insert_knot(&mut self, u: f64) {
        let p = self.degree.get();
        let n = self.control_points.len() - 1;
        let u = self.clamp_parameter(u);
        let k = self.knots.find_span(n, self.degree, u);
        let r = self.knots.multiplicity(u);

        let old = self.control_points.clone();
        let new_len = old.len() + 1;
        let mut new_points: Vec<HPoint> = Vec::with_capacity(new_len);

        for i in 0..=(k - p) {
            new_points.push(*old.get(i).unwrap());
        }
        for _ in (k - p + 1)..=(k - r) {
            new_points.push(HPoint::new(0.0, 0.0, 0.0, 0.0));
        }
        for i in (k - r)..=n {
            new_points.push(*old.get(i).unwrap());
        }

        for i in (k - p + 1)..=(k - r) {
            let denom = self.knots.get(i + p) - self.knots.get(i);
            let alpha = if denom == 0.0 {
                0.0
            } else {
                (u - self.knots.get(i)) / denom
            };
            let p_i = old.get(i).unwrap().0;
            let p_im1 = old.get(i - 1).unwrap().0;
            let blended = Point4::from(alpha * p_i.coords + (1.0 - alpha) * p_im1.coords);
            new_points[i] = HPoint(blended);
        }

        self.control_points = ControlPolygon::new(new_points).unwrap();
        self.knots.insert(k + 1, u);
    }
}
