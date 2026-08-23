use nalgebra::{DMatrix, DVector, Point4, Vector3};
use serde::{Deserialize, Serialize};

use super::bezier::Bezier;
use super::degree::Degree;
use super::knots::KnotVector;
use super::points::{ControlPolygon, HPoint};
use crate::geometry::nurbs::basis::{basis_function_derivatives, basis_functions};
use crate::geometry::nurbs::error::NurbsError;
use crate::geometry::{Interval, LINEAR_TOLERANCE, Point3, PointCoincidence};

const LENGTH_TOLERANCE: f64 = 1.0e-10;
const MAX_LENGTH_RECURSION: usize = 24;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
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

    /// Interpolates 3D samples with chord-length parameters.
    pub fn interpolate(points: &[Point3]) -> Result<Self, NurbsError> {
        let parameters = Self::chord_length_parameters(points)?;
        Self::interpolate_with_parameters(points, &parameters)
    }

    /// Interpolates 3D samples using caller-provided normalized parameters.
    pub fn interpolate_with_parameters(
        points: &[Point3],
        parameters: &[f64],
    ) -> Result<Self, NurbsError> {
        validate_interpolation_input(points, parameters)?;
        if points
            .first()
            .zip(points.last())
            .is_some_and(|(first, last)| first.coincides(*last, LINEAR_TOLERANCE))
        {
            return interpolate_closed(points, parameters);
        }
        interpolate_open(points, parameters)
    }

    /// Returns chord-length parameters in `[0, 1]` for 3D samples.
    pub fn chord_length_parameters(points: &[Point3]) -> Result<Vec<f64>, NurbsError> {
        if points.len() < 2 {
            return Err(NurbsError::InsufficientInterpolationPoints {
                minimum: 2,
                got: points.len(),
            });
        }
        let lengths = points
            .windows(2)
            .map(|pair| (pair[1] - pair[0]).norm())
            .collect::<Vec<_>>();
        let total = lengths.iter().sum::<f64>();
        if total <= LINEAR_TOLERANCE {
            return Err(NurbsError::DegenerateInterpolationSamples);
        }

        let mut parameters = Vec::with_capacity(points.len());
        parameters.push(0.0);
        let mut accumulated = 0.0;
        for length in lengths {
            accumulated += length;
            parameters.push(accumulated / total);
        }
        *parameters.last_mut().unwrap() = 1.0;
        Ok(parameters)
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

    pub fn domain(&self) -> Interval {
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
        let domain = self.domain();
        u.clamp(domain.start, domain.end)
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

    pub fn bezier_spans(&self) -> Result<Vec<Bezier>, NurbsError> {
        let p = self.degree.get();
        let domain = self.domain();
        let mut refined = self.clone();
        let interior_knots = distinct_interior_knots(self.knots.as_slice(), domain);

        for knot in interior_knots {
            while refined.knots.multiplicity(knot) < p {
                refined.insert_knot(knot);
            }
        }

        let breaks = distinct_domain_knots(refined.knots.as_slice(), domain);
        let mut spans = Vec::new();
        let mut point_start = 0usize;

        for interval_values in breaks.windows(2) {
            let start = interval_values[0];
            let end = interval_values[1];
            if end <= start {
                continue;
            }
            let point_end = point_start + p;
            let points = refined.control_points.as_slice()[point_start..=point_end].to_vec();
            spans.push(Bezier::new(
                self.degree,
                ControlPolygon::new(points)?,
                Interval::new(start, end),
            )?);
            point_start += refined.knots.multiplicity(end).min(p + 1);
        }

        Ok(spans)
    }

    /// Returns the exact curve with reversed parameter direction.
    pub fn reversed(&self) -> Self {
        let domain = self.domain();
        let control_points = ControlPolygon::new(
            self.control_points
                .as_slice()
                .iter()
                .copied()
                .rev()
                .collect(),
        )
        .expect("reversing a non-empty control polygon remains non-empty");
        let knots = KnotVector::new(
            self.knots
                .as_slice()
                .iter()
                .rev()
                .map(|knot| domain.start + domain.end - knot)
                .collect(),
        )
        .expect("reversing a valid knot vector remains valid");
        Self {
            degree: self.degree,
            control_points,
            knots,
        }
    }

    /// Splits the curve exactly at an interior native-domain parameter.
    pub fn split_at(&self, parameter: f64) -> Result<(Self, Self), NurbsError> {
        let domain = self.domain();
        if parameter <= domain.start + LINEAR_TOLERANCE
            || parameter >= domain.end - LINEAR_TOLERANCE
        {
            return Err(NurbsError::DegenerateInterval {
                start: domain.start,
                end: parameter,
            });
        }

        let mut refined = self.clone();
        let multiplicity = refined.knots.multiplicity(parameter);
        for _ in multiplicity..self.degree.get() {
            refined.insert_knot(parameter);
        }

        let n = refined.control_points.len() - 1;
        let span = refined.knots.find_span(n, refined.degree, parameter);
        let split_index = span - refined.degree.get();
        let left_points =
            ControlPolygon::new(refined.control_points.as_slice()[..=split_index].to_vec())?;
        let right_points =
            ControlPolygon::new(refined.control_points.as_slice()[split_index..].to_vec())?;

        let mut left_knots = refined.knots.as_slice()[..=span].to_vec();
        left_knots.push(parameter);
        let mut right_knots = vec![parameter; refined.degree.get() + 1];
        right_knots.extend_from_slice(&refined.knots.as_slice()[span + 1..]);

        Ok((
            Self::new(refined.degree, left_points, KnotVector::new(left_knots)?)?,
            Self::new(refined.degree, right_points, KnotVector::new(right_knots)?)?,
        ))
    }

    /// Returns the exact subcurve over the requested native-domain interval.
    pub fn trimmed(&self, start: f64, end: f64) -> Result<Self, NurbsError> {
        if (end - start).abs() <= LINEAR_TOLERANCE {
            return Err(NurbsError::DegenerateInterval { start, end });
        }
        if end < start {
            return Ok(self.trimmed(end, start)?.reversed());
        }

        let domain = self.domain();
        if start < domain.start - LINEAR_TOLERANCE || end > domain.end + LINEAR_TOLERANCE {
            return Err(NurbsError::ParameterOutOfRange {
                u: if start < domain.start { start } else { end },
                min: domain.start,
                max: domain.end,
            });
        }

        let after_start = if start <= domain.start + LINEAR_TOLERANCE {
            self.clone()
        } else {
            self.split_at(start)?.1
        };
        if end >= domain.end - LINEAR_TOLERANCE {
            Ok(after_start)
        } else {
            Ok(after_start.split_at(end)?.0)
        }
    }
}

fn validate_interpolation_input(points: &[Point3], parameters: &[f64]) -> Result<(), NurbsError> {
    if points.len() < 2 {
        return Err(NurbsError::InsufficientInterpolationPoints {
            minimum: 2,
            got: points.len(),
        });
    }
    if points.len() != parameters.len() {
        return Err(NurbsError::InterpolationParameterCountMismatch {
            expected: points.len(),
            got: parameters.len(),
        });
    }
    if parameters.windows(2).any(|pair| pair[1] <= pair[0]) {
        return Err(NurbsError::InvalidInterpolationParameters);
    }
    Ok(())
}

fn interpolate_open(points: &[Point3], parameters: &[f64]) -> Result<NurbsCurve, NurbsError> {
    let n = points.len() - 1;
    let degree = Degree::new(3.min(n))?;
    let p = degree.get();
    let mut knots = vec![parameters[0]; p + 1];
    for index in 1..=n - p {
        knots.push(parameters[index..index + p].iter().sum::<f64>() / p as f64);
    }
    knots.extend(std::iter::repeat_n(parameters[n], p + 1));
    let knots = KnotVector::new(knots)?;

    let mut coefficients = DMatrix::zeros(n + 1, n + 1);
    for (row, parameter) in parameters.iter().copied().enumerate() {
        let span = knots.find_span(n, degree, parameter);
        let basis = basis_functions(span, parameter, degree, &knots);
        for (offset, value) in basis.into_iter().enumerate() {
            coefficients[(row, span - p + offset)] = value;
        }
    }
    let decomposition = coefficients.lu();
    let solve = |coordinate: fn(&Point3) -> f64| {
        decomposition
            .solve(&DVector::from_iterator(
                points.len(),
                points.iter().map(coordinate),
            ))
            .ok_or(NurbsError::SingularInterpolationSystem)
    };
    let x = solve(|point| point.x)?;
    let y = solve(|point| point.y)?;
    let z = solve(|point| point.z)?;
    let control_points = ControlPolygon::new(
        x.iter()
            .zip(y.iter())
            .zip(z.iter())
            .map(|((x, y), z)| HPoint::from_cartesian(Point3::new(*x, *y, *z), 1.0))
            .collect(),
    )?;
    NurbsCurve::new(degree, control_points, knots)
}

fn interpolate_closed(points: &[Point3], parameters: &[f64]) -> Result<NurbsCurve, NurbsError> {
    let unique = &points[..points.len() - 1];
    if unique.len() < 3 {
        return Err(NurbsError::InsufficientInterpolationPoints {
            minimum: 4,
            got: points.len(),
        });
    }

    let count = unique.len();
    let mut tangents = Vec::with_capacity(count);
    for index in 0..count {
        let previous = (index + count - 1) % count;
        let next = (index + 1) % count;
        let previous_parameter = if index == 0 {
            parameters[previous] - 1.0
        } else {
            parameters[previous]
        };
        let next_parameter = if index + 1 == count {
            1.0
        } else {
            parameters[next]
        };
        tangents.push((unique[next] - unique[previous]) / (next_parameter - previous_parameter));
    }

    let mut control_points = Vec::with_capacity(3 * count + 1);
    for index in 0..count {
        let next = (index + 1) % count;
        let start_parameter = parameters[index];
        let end_parameter = if next == 0 { 1.0 } else { parameters[next] };
        let duration = end_parameter - start_parameter;
        let segment = [
            unique[index],
            unique[index] + tangents[index] * (duration / 3.0),
            unique[next] - tangents[next] * (duration / 3.0),
            unique[next],
        ];
        if index == 0 {
            control_points.extend(segment);
        } else {
            control_points.extend_from_slice(&segment[1..]);
        }
    }

    let degree = Degree::new(3)?;
    let mut knots = vec![0.0; 4];
    for parameter in parameters.iter().copied().skip(1).take(count - 1) {
        knots.extend(std::iter::repeat_n(parameter, 3));
    }
    knots.extend(std::iter::repeat_n(1.0, 4));
    NurbsCurve::new(
        degree,
        ControlPolygon::from_cartesian(control_points, &vec![1.0; 3 * count + 1])?,
        KnotVector::new(knots)?,
    )
}

fn distinct_interior_knots(knots: &[f64], domain: Interval) -> Vec<f64> {
    distinct_domain_knots(knots, domain)
        .into_iter()
        .filter(|knot| *knot > domain.start && *knot < domain.end)
        .collect()
}

fn distinct_domain_knots(knots: &[f64], domain: Interval) -> Vec<f64> {
    let mut distinct = Vec::new();
    for &knot in knots {
        if knot < domain.start || knot > domain.end {
            continue;
        }
        if distinct.last().is_none_or(|last| *last != knot) {
            distinct.push(knot);
        }
    }
    distinct
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
