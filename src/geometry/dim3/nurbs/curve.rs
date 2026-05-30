use nalgebra::{Point4, Vector3};

use super::basis::{basis_function_derivatives, basis_functions};
use super::degree::Degree;
use super::error::NurbsError;
use super::knots::KnotVector;
use super::points::{ControlPolygon, HPoint};
use crate::geometry::{LINEAR_TOLERANCE, Point3};

const LENGTH_TOLERANCE: f64 = 1.0e-10;
const MAX_LENGTH_RECURSION: usize = 24;

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
            Some(w0) => (hp.weight() - w0).abs() > LINEAR_TOLERANCE,
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
        for (i, basis_value) in basis.iter().copied().enumerate().take(p + 1) {
            let hp = self.control_points.get(span - p + i).unwrap();
            acc.coords += hp.0.coords * basis_value;
        }
        HPoint(acc).to_cartesian()
    }

    /// Return the derivative of order `order` at parameter `u`.
    ///
    /// `order == 0` returns the curve point as a vector from the origin. Higher
    /// orders follow Piegl & Tiller A4.2: differentiate the homogeneous curve
    /// and project the rational derivatives back to 3D.
    pub fn derivative_at(&self, u: f64, order: usize) -> Vector3<f64> {
        self.derivatives_at(u, order)[order]
    }

    /// Return derivatives from order 0 through `max_order`.
    pub fn derivatives_at(&self, u: f64, max_order: usize) -> Vec<Vector3<f64>> {
        let p = self.degree.get();
        let n = self.control_points.len() - 1;
        let u = self.clamp_parameter(u);
        let span = self.knots.find_span(n, self.degree, u);
        let basis_order = max_order.min(p);
        let basis = basis_function_derivatives(span, u, self.degree, &self.knots, basis_order);

        let mut numerator_derivatives = vec![Vector3::zeros(); max_order + 1];
        let mut weight_derivatives = vec![0.0; max_order + 1];

        for (k, basis_row) in basis.iter().enumerate().take(basis_order + 1) {
            for (j, coefficient) in basis_row.iter().copied().enumerate().take(p + 1) {
                let hp = self.control_points.get(span - p + j).unwrap();
                numerator_derivatives[k] += hp.weighted_xyz() * coefficient;
                weight_derivatives[k] += hp.weight() * coefficient;
            }
        }

        let mut curve_derivatives = vec![Vector3::zeros(); max_order + 1];
        for k in 0..=max_order {
            let mut derivative = numerator_derivatives[k];
            for i in 1..=k {
                derivative -= curve_derivatives[k - i] * (binomial(k, i) * weight_derivatives[i]);
            }
            curve_derivatives[k] = derivative / weight_derivatives[0];
        }

        curve_derivatives
    }

    /// Arc length between two parameters, computed by integrating `|C'(u)|`
    /// independently on every non-empty knot span crossed by the interval.
    pub fn length(&self, u0: f64, u1: f64) -> f64 {
        let a = self.clamp_parameter(u0);
        let b = self.clamp_parameter(u1);
        let start = a.min(b);
        let end = a.max(b);
        if (end - start).abs() <= f64::EPSILON {
            return 0.0;
        }

        let mut breaks = vec![start];
        for &knot in self.knots.as_slice() {
            if knot > start && knot < end && breaks.last().is_none_or(|last| *last != knot) {
                breaks.push(knot);
            }
        }
        breaks.push(end);

        breaks
            .windows(2)
            .filter_map(|span| {
                let a = span[0];
                let b = span[1];
                (b > a).then(|| self.integrate_length_span(a, b))
            })
            .sum()
    }

    fn clamp_parameter(&self, u: f64) -> f64 {
        let (min, max) = self.domain();
        u.clamp(min, max)
    }

    fn integrate_length_span(&self, a: f64, b: f64) -> f64 {
        let span_width = b - a;
        let endpoint_offset = (span_width.abs() * 1.0e-12).max(f64::EPSILON);
        let speed = |u: f64| {
            let interior_u = if u <= a {
                a + endpoint_offset
            } else if u >= b {
                b - endpoint_offset
            } else {
                u
            };
            self.derivative_at(interior_u, 1).norm()
        };

        let midpoint = 0.5 * (a + b);
        let fa = speed(a);
        let fm = speed(midpoint);
        let fb = speed(b);
        let whole = simpson_estimate(a, b, fa, fm, fb);
        adaptive_simpson(
            &speed,
            SimpsonState {
                a,
                b,
                fa,
                fm,
                fb,
                whole,
                tolerance: LENGTH_TOLERANCE,
                depth: MAX_LENGTH_RECURSION,
            },
        )
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

struct SimpsonState {
    a: f64,
    b: f64,
    fa: f64,
    fm: f64,
    fb: f64,
    whole: f64,
    tolerance: f64,
    depth: usize,
}

fn adaptive_simpson(f: &impl Fn(f64) -> f64, state: SimpsonState) -> f64 {
    let SimpsonState {
        a,
        b,
        fa,
        fm,
        fb,
        whole,
        tolerance,
        depth,
    } = state;
    let midpoint = 0.5 * (a + b);
    let left_midpoint = 0.5 * (a + midpoint);
    let right_midpoint = 0.5 * (midpoint + b);
    let left_mid = f(left_midpoint);
    let right_mid = f(right_midpoint);
    let left = simpson_estimate(a, midpoint, fa, left_mid, fm);
    let right = simpson_estimate(midpoint, b, fm, right_mid, fb);
    let delta = left + right - whole;

    if depth == 0 || delta.abs() <= 15.0 * tolerance {
        return left + right + delta / 15.0;
    }

    adaptive_simpson(
        f,
        SimpsonState {
            a,
            b: midpoint,
            fa,
            fm: left_mid,
            fb: fm,
            whole: left,
            tolerance: 0.5 * tolerance,
            depth: depth - 1,
        },
    ) + adaptive_simpson(
        f,
        SimpsonState {
            a: midpoint,
            b,
            fa: fm,
            fm: right_mid,
            fb,
            whole: right,
            tolerance: 0.5 * tolerance,
            depth: depth - 1,
        },
    )
}

fn simpson_estimate(a: f64, b: f64, fa: f64, fm: f64, fb: f64) -> f64 {
    (b - a) * (fa + 4.0 * fm + fb) / 6.0
}

fn binomial(n: usize, k: usize) -> f64 {
    if k > n {
        return 0.0;
    }
    let k = k.min(n - k);
    (1..=k).fold(1.0, |acc, i| acc * (n + 1 - i) as f64 / i as f64)
}
