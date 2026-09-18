use nalgebra::{DMatrix, DVector, Point4, Vector3, Vector4};
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

/// Parameter distance below which two knots name the same break.
///
/// Not a distance in space, and deliberately not [`LINEAR_TOLERANCE`]: knots
/// are compared only between curves already mapped onto `[0, 1]`, so this is a
/// fraction of a whole domain. Two knots closer than this are one knot, and
/// keeping them apart would leave a span no evaluator can subdivide.
pub const KNOT_TOLERANCE: f64 = 1.0e-9;

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

    /// This curve with every control point moved by `f`.
    ///
    /// Degree, weights and knots are kept, so the result is the image of this
    /// curve under `f` with the same parameterization whenever `f` is affine.
    /// Infallible because the count invariant [`Self::new`] enforces is
    /// untouched by a point-wise map.
    pub(crate) fn map_control_points(&self, f: impl Fn(Point3) -> Point3) -> Self {
        Self {
            degree: self.degree,
            control_points: self.control_points.map_points(f),
            knots: self.knots.clone(),
        }
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
        u.clamp(domain.start.value(), domain.end.value())
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
        let k = self.knots.insertion_span(u);
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
                .map(|knot| domain.start.value() + domain.end.value() - knot)
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
        if parameter <= domain.start.value() + LINEAR_TOLERANCE
            || parameter >= domain.end.value() - LINEAR_TOLERANCE
        {
            return Err(NurbsError::DegenerateInterval {
                start: domain.start.value(),
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

    /// Returns the same curve over the same domain, with its ends clamped.
    ///
    /// A knot vector whose ends are not repeated `degree + 1` times describes a
    /// curve that runs on past `[U[p], U[m-p]]` — the form a periodic spline
    /// arrives in. The curve over its own domain is unchanged by clamping, but
    /// everything that takes a control polygon for a hull of the curve is not:
    /// the leading and trailing control points of an unclamped vector influence
    /// only the parts outside the domain, so a Bézier decomposition or a
    /// embedding bound computed from them is answering about a longer curve.
    ///
    /// Raising each end's multiplicity to `degree + 1` and dropping what falls
    /// outside is exact — knot insertion does not move the curve — so this
    /// changes the representation and nothing else.
    pub fn clamped(&self) -> Result<Self, NurbsError> {
        if self.knots.is_clamped(self.degree) {
            return Ok(self.clone());
        }

        let p = self.degree.get();
        let domain = self.domain();
        let mut refined = self.clone();
        for end in [domain.start.value(), domain.end.value()] {
            while refined.knots.multiplicity(end) < p + 1 {
                refined.insert_knot(end);
            }
        }

        let knots = refined.knots.as_slice();
        let (Some(first), Some(last)) = (
            knots.iter().position(|&knot| knot == domain.start.value()),
            knots.iter().rposition(|&knot| knot == domain.end.value()),
        ) else {
            return Err(NurbsError::UnsortedKnots);
        };
        Self::new(
            self.degree,
            ControlPolygon::new(refined.control_points.as_slice()[first..=last - p - 1].to_vec())?,
            KnotVector::new(knots[first..=last].to_vec())?,
        )
    }

    /// Returns the exact subcurve over the requested native-domain interval.
    ///
    /// Each end is first snapped onto a knot it already lands within
    /// [`LINEAR_TOLERANCE`] of. A parameter recovered from a point — which is
    /// how the piece of a curve one edge carries is named — reaches an interior
    /// knot an ulp to one side of it, and splitting there would leave a span
    /// narrower than the parameter can resolve: a Bezier piece with no control
    /// polygon to subdivide, which every later search refuses.
    pub fn trimmed(&self, start: f64, end: f64) -> Result<Self, NurbsError> {
        if (end - start).abs() <= LINEAR_TOLERANCE {
            return Err(NurbsError::DegenerateInterval { start, end });
        }
        if end < start {
            return Ok(self.trimmed(end, start)?.reversed());
        }

        let domain = self.domain();
        if start < domain.start.value() - LINEAR_TOLERANCE
            || end > domain.end.value() + LINEAR_TOLERANCE
        {
            return Err(NurbsError::ParameterOutOfRange {
                u: if start < domain.start.value() {
                    start
                } else {
                    end
                },
                min: domain.start.value(),
                max: domain.end.value(),
            });
        }

        let (start, end) = (self.snapped_to_knot(start), self.snapped_to_knot(end));
        if (end - start).abs() <= LINEAR_TOLERANCE {
            return Err(NurbsError::DegenerateInterval { start, end });
        }

        let after_start = if start <= domain.start.value() + LINEAR_TOLERANCE {
            self.clone()
        } else {
            self.split_at(start)?.1
        };
        if end >= domain.end.value() - LINEAR_TOLERANCE {
            Ok(after_start)
        } else {
            Ok(after_start.split_at(end)?.0)
        }
    }

    /// Returns the knot nearest `parameter` within [`LINEAR_TOLERANCE`], else `parameter`.
    fn snapped_to_knot(&self, parameter: f64) -> f64 {
        self.knots
            .as_slice()
            .iter()
            .copied()
            .filter(|knot| (knot - parameter).abs() <= LINEAR_TOLERANCE)
            .min_by(|first, second| {
                (first - parameter)
                    .abs()
                    .total_cmp(&(second - parameter).abs())
            })
            .unwrap_or(parameter)
    }

    /// Returns the same curve with its knot domain mapped affinely onto `[0, 1]`.
    ///
    /// An affine change of knots is a reparameterization and nothing else: the
    /// control points and weights are untouched, so the curve traces the same
    /// points in the same order. It exists because interior knots are only
    /// comparable between curves once both domains are the same — `0.5` of
    /// `[0, 5]` and `0.5` of `[0, 1]` name different places, and a knot union
    /// taken before this is a union of numbers rather than of breaks.
    pub fn normalized(&self) -> Result<Self, NurbsError> {
        let domain = self.domain();
        let (start, extent) = (domain.start.value(), domain.delta());
        if extent <= 0.0 || !extent.is_finite() {
            return Err(NurbsError::DegenerateInterval {
                start,
                end: domain.end.value(),
            });
        }
        if start == 0.0 && extent == 1.0 {
            return Ok(self.clone());
        }
        Self::new(
            self.degree,
            self.control_points.clone(),
            KnotVector::new(
                self.knots
                    .as_slice()
                    .iter()
                    .map(|knot| (knot - start) / extent)
                    .collect(),
            )?,
        )
    }

    /// Piegl & Tiller A5.4 — inserts every knot of `knots` in one pass.
    ///
    /// The same result as calling [`Self::insert_knot`] once per knot, at
    /// `O(n + m)` rather than `O(n · m)` and with one rebuild rather than one
    /// per knot. Knot insertion does not move a curve, so neither does this:
    /// the refined curve evaluates identically to this one everywhere.
    ///
    /// Every knot must fall strictly inside the domain. A knot at an end would
    /// raise an end multiplicity past `degree + 1`, which describes a different
    /// curve rather than the same one refined, so it is refused by name.
    pub fn refined(&self, knots: &[f64]) -> Result<Self, NurbsError> {
        if knots.is_empty() {
            return Ok(self.clone());
        }
        let domain = self.domain();
        let (min, max) = (domain.start.value(), domain.end.value());
        if let Some(&outside) = knots
            .iter()
            .find(|knot| **knot <= min || **knot >= max || !knot.is_finite())
        {
            return Err(NurbsError::ParameterOutOfRange {
                u: outside,
                min,
                max,
            });
        }

        let mut inserted = knots.to_vec();
        inserted.sort_by(f64::total_cmp);

        let p = self.degree.get();
        let n = self.control_points.len() - 1;
        let m = n + p + 1;
        let r = inserted.len() - 1;
        let u = self.knots.as_slice();
        let source = self.control_points.as_slice();

        let a = self.knots.find_span(n, self.degree, inserted[0]);
        let b = self.knots.find_span(n, self.degree, inserted[r]) + 1;

        let mut points = vec![Vector4::zeros(); n + r + 2];
        let mut refined = vec![0.0; m + r + 2];
        for j in 0..=a - p {
            points[j] = source[j].0.coords;
        }
        for j in (b - 1)..=n {
            points[j + r + 1] = source[j].0.coords;
        }
        for (j, knot) in u.iter().copied().enumerate().take(a + 1) {
            refined[j] = knot;
        }
        for j in (b + p)..=m {
            refined[j + r + 1] = u[j];
        }

        let mut i = b + p - 1;
        let mut k = b + p + r;
        for j in (0..=r).rev() {
            while inserted[j] <= u[i] && i > a {
                points[k - p - 1] = source[i - p - 1].0.coords;
                refined[k] = u[i];
                k -= 1;
                i -= 1;
            }
            points[k - p - 1] = points[k - p];
            for l in 1..=p {
                let index = k - p + l;
                let numerator = refined[k + l] - inserted[j];
                points[index - 1] = if numerator == 0.0 {
                    points[index]
                } else {
                    let alpha = numerator / (refined[k + l] - u[i - p + l]);
                    points[index - 1] * alpha + points[index] * (1.0 - alpha)
                };
            }
            refined[k] = inserted[j];
            k -= 1;
        }

        Self::new(
            self.degree,
            ControlPolygon::new(
                points
                    .into_iter()
                    .map(|point| HPoint(Point4::from(point)))
                    .collect(),
            )?,
            KnotVector::new(refined)?,
        )
    }

    /// Piegl & Tiller A5.9 — raises the degree to `degree`, leaving the curve
    /// where it is.
    ///
    /// Degree elevation is exact: the elevated curve evaluates identically to
    /// this one at every parameter. It costs control points and interior knots,
    /// which is why it runs before a knot union rather than after — the knots
    /// it inserts of its own would otherwise leave the vectors different again.
    ///
    /// Lowering a degree is a different operation with a different contract —
    /// it approximates — so a lower `degree` is refused rather than silently
    /// treated as a request for no change.
    pub fn elevated_degree(&self, degree: Degree) -> Result<Self, NurbsError> {
        if degree < self.degree {
            return Err(NurbsError::DegreeReductionRefused {
                from: self.degree.get(),
                to: degree.get(),
            });
        }
        let t = degree.get() - self.degree.get();
        if t == 0 {
            return self.clamped();
        }

        // A5.9 reads the source as a sequence of Bezier segments recovered from
        // knot multiplicities, and an unclamped end carries no segment boundary
        // to recover one from.
        let source = self.clamped()?;
        let p = source.degree.get();
        let n = source.control_points.len() - 1;
        let m = n + p + 1;
        let u = source.knots.as_slice();
        let control: Vec<Vector4<f64>> = source
            .control_points
            .iter()
            .map(|point| point.0.coords)
            .collect();

        let ph = p + t;
        let ph2 = ph / 2;
        let mut coefficients = vec![vec![0.0; p + 1]; ph + 1];
        coefficients[0][0] = 1.0;
        coefficients[ph][p] = 1.0;
        for i in 1..=ph2 {
            let inverse = 1.0 / binomial(ph, i);
            for j in i.saturating_sub(t)..=p.min(i) {
                coefficients[i][j] = inverse * binomial(p, j) * binomial(t, i - j);
            }
        }
        for i in (ph2 + 1)..ph {
            for j in i.saturating_sub(t)..=p.min(i) {
                coefficients[i][j] = coefficients[ph - i][p - j];
            }
        }

        // Every span of the source contributes at most `p + t + 1` control
        // points and knots to the result, which bounds both buffers without
        // predicting the multiplicities first.
        let capacity = (m + 1) * (t + 1) + 2 * (ph + 1);
        let mut elevated_points = vec![Vector4::zeros(); capacity];
        let mut elevated_knots = vec![0.0; capacity];
        let mut segment: Vec<Vector4<f64>> = control[0..=p].to_vec();
        let mut elevated_segment = vec![Vector4::zeros(); ph + 1];
        let mut next_segment = vec![Vector4::zeros(); p];
        let mut alphas = vec![0.0; p];

        let mut total = ph;
        let mut kind = ph + 1;
        let mut r: isize = -1;
        let mut a = p;
        let mut b = p + 1;
        let mut cursor = 1usize;
        let mut ua = u[0];
        elevated_points[0] = control[0];
        for knot in elevated_knots.iter_mut().take(ph + 1) {
            *knot = ua;
        }

        while b < m {
            let first_of_run = b;
            while b < m && u[b] == u[b + 1] {
                b += 1;
            }
            let multiplicity = b - first_of_run + 1;
            total += multiplicity + t;
            let ub = u[b];
            let old_r = r;
            r = p as isize - multiplicity as isize;

            let lbz = if old_r > 0 {
                ((old_r + 2) / 2) as usize
            } else {
                1
            };
            let rbz = if r > 0 {
                ph - ((r + 1) / 2) as usize
            } else {
                ph
            };

            if r > 0 {
                let numerator = ub - ua;
                for k in ((multiplicity + 1)..=p).rev() {
                    alphas[k - multiplicity - 1] = numerator / (u[a + k] - ua);
                }
                for j in 1..=r as usize {
                    let save = r as usize - j;
                    let s = multiplicity + j;
                    for k in (s..=p).rev() {
                        let alpha = alphas[k - s];
                        segment[k] = segment[k] * alpha + segment[k - 1] * (1.0 - alpha);
                    }
                    next_segment[save] = segment[p];
                }
            }

            for i in lbz..=ph {
                let mut accumulated = Vector4::zeros();
                for j in i.saturating_sub(t)..=p.min(i) {
                    accumulated += segment[j] * coefficients[i][j];
                }
                elevated_segment[i] = accumulated;
            }

            if old_r > 1 {
                let old_r = old_r as usize;
                let mut first = kind as isize - 2;
                let mut last = kind as isize;
                let denominator = ub - ua;
                let beta = (ub - elevated_knots[kind - 1]) / denominator;
                for removal in 1..old_r as isize {
                    let mut i = first;
                    let mut j = last;
                    let mut kj = j - kind as isize + 1;
                    while j - i > removal {
                        if (i as usize) < cursor {
                            let i = i as usize;
                            let alpha = (ub - elevated_knots[i]) / (ua - elevated_knots[i]);
                            elevated_points[i] =
                                elevated_points[i] * alpha + elevated_points[i - 1] * (1.0 - alpha);
                        }
                        if j >= lbz as isize {
                            let kj = kj as usize;
                            let factor =
                                if j - removal <= kind as isize - ph as isize + old_r as isize {
                                    (ub - elevated_knots[(j - removal) as usize]) / denominator
                                } else {
                                    beta
                                };
                            elevated_segment[kj] = elevated_segment[kj] * factor
                                + elevated_segment[kj + 1] * (1.0 - factor);
                        }
                        i += 1;
                        j -= 1;
                        kj -= 1;
                    }
                    first -= 1;
                    last += 1;
                }
            }

            if a != p {
                for _ in 0..(ph - old_r.max(0) as usize) {
                    elevated_knots[kind] = ua;
                    kind += 1;
                }
            }
            for j in lbz..=rbz {
                elevated_points[cursor] = elevated_segment[j];
                cursor += 1;
            }

            if b < m {
                let carried = r.max(0) as usize;
                segment[..carried].copy_from_slice(&next_segment[..carried]);
                for j in carried..=p {
                    segment[j] = control[b - p + j];
                }
                a = b;
                b += 1;
                ua = ub;
            } else {
                for i in 0..=ph {
                    elevated_knots[kind + i] = ub;
                }
            }
        }

        elevated_points.truncate(total - ph);
        elevated_knots.truncate(total + 1);
        Self::new(
            Degree::new(ph)?,
            ControlPolygon::new(
                elevated_points
                    .into_iter()
                    .map(|point| HPoint(Point4::from(point)))
                    .collect(),
            )?,
            KnotVector::new(elevated_knots)?,
        )
    }

    /// Returns this curve's interior knots, in order and with repeats kept.
    pub fn interior_knots(&self) -> Vec<f64> {
        let domain = self.domain();
        self.knots
            .as_slice()
            .iter()
            .copied()
            .filter(|knot| *knot > domain.start.value() && *knot < domain.end.value())
            .collect()
    }

    /// Returns this curve with each interior knot within [`KNOT_TOLERANCE`] of
    /// a value in `breaks` replaced by that value exactly.
    ///
    /// Two curves cut at the same place arrive with knots differing in the last
    /// few bits, and a union taken over the raw numbers admits both, leaving a
    /// span narrower than any evaluator can subdivide. The relabel moves the
    /// curve by less than the knot shift, which at this tolerance is far below
    /// [`LINEAR_TOLERANCE`] in space.
    fn with_snapped_knots(&self, breaks: &[f64]) -> Result<Self, NurbsError> {
        let domain = self.domain();
        Self::new(
            self.degree,
            self.control_points.clone(),
            KnotVector::new(
                self.knots
                    .as_slice()
                    .iter()
                    .copied()
                    .map(|knot| {
                        if knot <= domain.start.value() || knot >= domain.end.value() {
                            return knot;
                        }
                        breaks
                            .iter()
                            .copied()
                            .find(|break_at| (break_at - knot).abs() <= KNOT_TOLERANCE)
                            .unwrap_or(knot)
                    })
                    .collect(),
            )?,
        )
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
    let degree = Degree::new(3.min(points.len() - 1))?;
    let knots = KnotVector::averaged(parameters, degree)?;
    interpolate_with_knots(
        &points
            .iter()
            .map(|point| HPoint::from_cartesian(*point, 1.0))
            .collect::<Vec<_>>(),
        parameters,
        degree,
        &knots,
    )
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
        .filter(|knot| *knot > domain.start.value() && *knot < domain.end.value())
        .collect()
}

fn distinct_domain_knots(knots: &[f64], domain: Interval) -> Vec<f64> {
    let mut distinct = Vec::new();
    for &knot in knots {
        if knot < domain.start.value() || knot > domain.end.value() {
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

/// The factored linear system that interpolates points at stated parameters.
///
/// The coefficient matrix depends only on `(parameters, degree, knots)` and
/// not at all on the points, so a caller with many point sequences over one
/// parameterization factors once and solves many times. Skinning is exactly
/// that caller: it interpolates one row of the control grid per control point
/// index, all against the same v-parameters.
pub struct InterpolationSystem {
    degree: Degree,
    knots: KnotVector,
    decomposition: nalgebra::LU<f64, nalgebra::Dyn, nalgebra::Dyn>,
    count: usize,
}

impl InterpolationSystem {
    /// Factors the system interpolating `parameters.len()` points of degree
    /// `degree` over `knots`.
    ///
    /// `parameters` must be strictly increasing, and `knots` must be the
    /// vector for that many control points at that degree —
    /// [`KnotVector::averaged`] produces one.
    pub fn new(parameters: &[f64], degree: Degree, knots: &KnotVector) -> Result<Self, NurbsError> {
        let count = parameters.len();
        let p = degree.get();
        if count <= p {
            return Err(NurbsError::InsufficientInterpolationPoints {
                minimum: p + 1,
                got: count,
            });
        }
        let expected = count + p + 1;
        if knots.len() != expected {
            return Err(NurbsError::KnotCountMismatch {
                expected,
                got: knots.len(),
            });
        }
        if parameters.windows(2).any(|pair| pair[1] <= pair[0]) {
            return Err(NurbsError::InvalidInterpolationParameters);
        }

        let n = count - 1;
        let mut coefficients = DMatrix::zeros(count, count);
        for (row, parameter) in parameters.iter().copied().enumerate() {
            let span = knots.find_span(n, degree, parameter);
            let basis = basis_functions(span, parameter, degree, knots);
            for (offset, value) in basis.into_iter().enumerate().take(p + 1) {
                coefficients[(row, span - p + offset)] = value;
            }
        }

        Ok(Self {
            degree,
            knots: knots.clone(),
            decomposition: coefficients.lu(),
            count,
        })
    }

    /// The degree the interpolated curve has.
    pub fn degree(&self) -> Degree {
        self.degree
    }

    /// The knot vector the interpolated curve has.
    pub fn knots(&self) -> &KnotVector {
        &self.knots
    }

    /// Solves for the control points whose curve passes through `points`.
    ///
    /// **The solve runs in homogeneous coordinates.** Interpolating the
    /// cartesian positions and giving every control point weight `1` produces
    /// a curve that does not pass through a rational input at all — the
    /// weights are part of where the point is. Each of the four homogeneous
    /// components is solved as its own right-hand side and the divide happens
    /// only when the result is evaluated.
    ///
    /// Equal input weights are carried straight through rather than solved
    /// for. The weight row's solution is analytically that same constant,
    /// because the basis functions sum to one, and solving for it would return
    /// it with a rounding error that makes a non-rational curve report itself
    /// rational.
    pub fn solve(&self, points: &[HPoint]) -> Result<ControlPolygon, NurbsError> {
        if points.len() != self.count {
            return Err(NurbsError::InterpolationParameterCountMismatch {
                expected: self.count,
                got: points.len(),
            });
        }

        let solve = |component: fn(&HPoint) -> f64| {
            self.decomposition
                .solve(&DVector::from_iterator(
                    points.len(),
                    points.iter().map(component),
                ))
                .ok_or(NurbsError::SingularInterpolationSystem)
        };
        let x = solve(|point| point.0.x)?;
        let y = solve(|point| point.0.y)?;
        let z = solve(|point| point.0.z)?;

        let first = points[0].weight();
        let uniform = points
            .iter()
            .all(|point| (point.weight() - first).abs() <= LINEAR_TOLERANCE);
        let w = if uniform {
            DVector::from_element(points.len(), first)
        } else {
            solve(|point| point.0.w)?
        };

        ControlPolygon::new(
            (0..points.len())
                .map(|index| HPoint::new(x[index], y[index], z[index], w[index]))
                .collect(),
        )
    }
}

/// Interpolates homogeneous points at stated parameters over a stated knot
/// vector.
///
/// The one-shot form of [`InterpolationSystem`], for a caller with a single
/// sequence to interpolate.
pub fn interpolate_with_knots(
    points: &[HPoint],
    parameters: &[f64],
    degree: Degree,
    knots: &KnotVector,
) -> Result<NurbsCurve, NurbsError> {
    let system = InterpolationSystem::new(parameters, degree, knots)?;
    let control_points = system.solve(points)?;
    NurbsCurve::new(degree, control_points, knots.clone())
}

/// Brings a set of curves onto one degree, one domain and one knot vector,
/// without moving any of them.
///
/// Skinning reads the curves as a rectangular grid of control points, which
/// only means anything once index `i` names the same basis function on every
/// curve. Four steps get there, and the order is forced:
///
/// 1. **Clamp.** An unclamped end has no segment boundary for degree
///    elevation to recover, and its leading control points influence only the
///    curve outside its own domain.
/// 2. **Normalize.** Interior knots of `[0, 5]` and of `[0, 1]` are not
///    comparable until both domains are the same, so a union taken before
///    this is a union of unrelated numbers.
/// 3. **Elevate** every curve to the highest degree present. Elevation
///    inserts knots of its own, so refining first would leave the vectors
///    different again.
/// 4. **Refine** every curve to the union of all the knot vectors.
///
/// Every step is exact: each curve evaluates identically before and after,
/// which is the whole contract. The one exception is knots that differ by less
/// than [`KNOT_TOLERANCE`], which are relabelled onto a common value rather
/// than both admitted — see [`NurbsCurve::with_snapped_knots`].
pub fn make_compatible(curves: &mut [NurbsCurve]) -> Result<(), NurbsError> {
    for curve in curves.iter_mut() {
        *curve = curve.clamped()?.normalized()?;
    }
    let Some(degree) = curves.iter().map(NurbsCurve::degree).max() else {
        return Ok(());
    };
    for curve in curves.iter_mut() {
        *curve = curve.elevated_degree(degree)?;
    }

    let breaks = merged_breaks(curves);
    if breaks.is_empty() {
        return Ok(());
    }
    for curve in curves.iter_mut() {
        *curve = curve.with_snapped_knots(&breaks)?;
    }

    let multiplicities = breaks
        .iter()
        .map(|&break_at| {
            curves
                .iter()
                .map(|curve| curve.knots().multiplicity(break_at))
                .max()
                .unwrap_or(0)
                // An interior knot of multiplicity `degree` already splits the
                // curve; more than that describes a gap rather than a break.
                .min(degree.get())
        })
        .collect::<Vec<_>>();

    for curve in curves.iter_mut() {
        let mut missing = Vec::new();
        for (&break_at, &wanted) in breaks.iter().zip(&multiplicities) {
            let present = curve.knots().multiplicity(break_at);
            missing.extend(std::iter::repeat_n(
                break_at,
                wanted.saturating_sub(present),
            ));
        }
        *curve = curve.refined(&missing)?;
    }
    Ok(())
}

/// The distinct interior breaks of `curves`, merged within [`KNOT_TOLERANCE`].
///
/// One representative per cluster, placed at the cluster's mean so that no
/// curve is relabelled further than any other.
fn merged_breaks(curves: &[NurbsCurve]) -> Vec<f64> {
    let mut all = curves
        .iter()
        .flat_map(NurbsCurve::interior_knots)
        .collect::<Vec<_>>();
    all.sort_by(f64::total_cmp);

    let mut breaks = Vec::new();
    let mut cluster: Vec<f64> = Vec::new();
    for knot in all {
        if cluster
            .first()
            .is_some_and(|first| knot - first > KNOT_TOLERANCE)
        {
            breaks.push(cluster.iter().sum::<f64>() / cluster.len() as f64);
            cluster.clear();
        }
        cluster.push(knot);
    }
    if !cluster.is_empty() {
        breaks.push(cluster.iter().sum::<f64>() / cluster.len() as f64);
    }
    breaks
}
