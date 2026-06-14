use nalgebra::{DMatrix, DVector, Point3, Vector2, Vector3};

use crate::geometry::nurbs::basis::{basis_function_derivatives, basis_functions};
use crate::geometry::{Degree, Interval, KnotVector, LINEAR_TOLERANCE, NurbsError, Point2};

/// A homogeneous 2D control point stored as `(x*w, y*w, w)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HPoint2(Point3<f64>);

impl HPoint2 {
    /// Creates a homogeneous 2D point from pre-multiplied coordinates.
    pub fn new(x_w: f64, y_w: f64, w: f64) -> Self {
        Self(Point3::new(x_w, y_w, w))
    }

    /// Creates a homogeneous point from a Cartesian point and weight.
    pub fn from_cartesian(point: Point2, weight: f64) -> Self {
        Self::new(point.x * weight, point.y * weight, weight)
    }

    /// Converts this homogeneous point to Cartesian coordinates.
    pub fn to_cartesian(self) -> Point2 {
        Point2::new(self.0.x / self.0.z, self.0.y / self.0.z)
    }

    /// Returns the rational weight.
    pub fn weight(self) -> f64 {
        self.0.z
    }

    fn weighted_xy(self) -> Vector2<f64> {
        Vector2::new(self.0.x, self.0.y)
    }
}

/// The control-point sequence of a 2D NURBS curve.
#[derive(Debug, Clone, PartialEq)]
pub struct ControlPolygon2(Vec<HPoint2>);

impl ControlPolygon2 {
    /// Creates a non-empty 2D control polygon.
    pub fn new(points: Vec<HPoint2>) -> Result<Self, NurbsError> {
        if points.is_empty() {
            Err(NurbsError::EmptyControlPolygon)
        } else {
            Ok(Self(points))
        }
    }

    /// Creates a control polygon from Cartesian points and matching weights.
    pub fn from_cartesian(points: Vec<Point2>, weights: &[f64]) -> Result<Self, NurbsError> {
        if points.len() != weights.len() {
            return Err(NurbsError::WeightCountMismatch {
                expected: points.len(),
                got: weights.len(),
            });
        }
        Self::new(
            points
                .into_iter()
                .zip(weights.iter().copied())
                .map(|(point, weight)| HPoint2::from_cartesian(point, weight))
                .collect(),
        )
    }

    /// Returns the number of control points.
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// Returns whether the polygon has no control points.
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    /// Returns the control points as a slice.
    pub fn as_slice(&self) -> &[HPoint2] {
        &self.0
    }

    /// Returns the control point at `index`, if present.
    pub fn get(&self, index: usize) -> Option<&HPoint2> {
        self.0.get(index)
    }
}

/// A rational B-spline curve in a surface's 2D parameter space.
#[derive(Debug, Clone, PartialEq)]
pub struct NurbsCurve2 {
    degree: Degree,
    control_points: ControlPolygon2,
    knots: KnotVector,
    interpolation_parameters: Vec<f64>,
}

impl NurbsCurve2 {
    /// Creates a 2D NURBS curve after validating its knot count.
    pub fn new(
        degree: Degree,
        control_points: ControlPolygon2,
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
            interpolation_parameters: Vec::new(),
        })
    }

    /// Interpolates samples with a chord-length parameterization.
    ///
    /// Open inputs use a clamped curve of degree up to three. Inputs whose
    /// first and last points coincide produce a closed piecewise-cubic NURBS
    /// with matching tangent at the seam.
    pub fn interpolate(points: &[Point2]) -> Result<Self, NurbsError> {
        let parameters = chord_length_parameters(points)?;
        Self::interpolate_with_parameters(points, &parameters)
    }

    /// Interpolates samples using caller-provided normalized parameters.
    pub fn interpolate_with_parameters(
        points: &[Point2],
        parameters: &[f64],
    ) -> Result<Self, NurbsError> {
        validate_interpolation_input(points, parameters)?;
        if points
            .first()
            .zip(points.last())
            .is_some_and(|(first, last)| (*first - *last).norm() <= LINEAR_TOLERANCE)
        {
            return interpolate_closed(points, parameters);
        }
        interpolate_open(points, parameters)
    }

    /// Returns chord-length parameters in `[0, 1]` for the supplied samples.
    pub fn chord_length_parameters(points: &[Point2]) -> Result<Vec<f64>, NurbsError> {
        chord_length_parameters(points)
    }

    /// Returns the degree.
    pub fn degree(&self) -> Degree {
        self.degree
    }

    /// Returns the control polygon.
    pub fn control_points(&self) -> &ControlPolygon2 {
        &self.control_points
    }

    /// Returns the knot vector.
    pub fn knots(&self) -> &KnotVector {
        &self.knots
    }

    /// Returns the native parameter domain.
    pub fn domain(&self) -> Interval {
        self.knots.domain(self.degree)
    }

    /// Returns the source parameters used by interpolation, if any.
    pub fn interpolation_parameters(&self) -> &[f64] {
        &self.interpolation_parameters
    }

    /// Evaluates the curve at a native-domain parameter.
    pub fn point_at(&self, parameter: f64) -> Point2 {
        let p = self.degree.get();
        let n = self.control_points.len() - 1;
        let parameter = self.clamp_parameter(parameter);
        let span = self.knots.find_span(n, self.degree, parameter);
        let basis = basis_functions(span, parameter, self.degree, &self.knots);
        let mut weighted = Vector3::zeros();

        for (index, coefficient) in basis.iter().copied().enumerate().take(p + 1) {
            weighted += self.control_points.get(span - p + index).unwrap().0.coords * coefficient;
        }
        Point2::new(weighted.x / weighted.z, weighted.y / weighted.z)
    }

    /// Evaluates a derivative at a native-domain parameter.
    pub fn derivative_at(&self, parameter: f64, order: usize) -> Vector2<f64> {
        self.derivatives_at(parameter, order)[order]
    }

    /// Evaluates derivatives from order zero through `max_order`.
    pub fn derivatives_at(&self, parameter: f64, max_order: usize) -> Vec<Vector2<f64>> {
        let p = self.degree.get();
        let n = self.control_points.len() - 1;
        let parameter = self.clamp_parameter(parameter);
        let span = self.knots.find_span(n, self.degree, parameter);
        let basis_order = max_order.min(p);
        let basis =
            basis_function_derivatives(span, parameter, self.degree, &self.knots, basis_order);
        let mut numerator = vec![Vector2::zeros(); max_order + 1];
        let mut weights = vec![0.0; max_order + 1];

        for (order, row) in basis.iter().enumerate().take(basis_order + 1) {
            for (index, coefficient) in row.iter().copied().enumerate().take(p + 1) {
                let point = *self.control_points.get(span - p + index).unwrap();
                numerator[order] += point.weighted_xy() * coefficient;
                weights[order] += point.weight() * coefficient;
            }
        }

        let mut derivatives = vec![Vector2::zeros(); max_order + 1];
        for order in 0..=max_order {
            let mut derivative = numerator[order];
            for index in 1..=order {
                derivative -=
                    derivatives[order - index] * (binomial(order, index) * weights[index]);
            }
            derivatives[order] = derivative / weights[0];
        }
        derivatives
    }

    /// Returns the exact curve with reversed parameter direction.
    pub fn reversed(&self) -> Self {
        let domain = self.domain();
        let control_points = ControlPolygon2::new(
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
        let mut interpolation_parameters = self
            .interpolation_parameters
            .iter()
            .rev()
            .map(|parameter| domain.start + domain.end - parameter)
            .collect::<Vec<_>>();
        interpolation_parameters.sort_by(f64::total_cmp);
        Self {
            degree: self.degree,
            control_points,
            knots,
            interpolation_parameters,
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
            ControlPolygon2::new(refined.control_points.as_slice()[..=split_index].to_vec())?;
        let right_points =
            ControlPolygon2::new(refined.control_points.as_slice()[split_index..].to_vec())?;

        let mut left_knots = refined.knots.as_slice()[..=span].to_vec();
        left_knots.push(parameter);
        let mut right_knots = vec![parameter; refined.degree.get() + 1];
        right_knots.extend_from_slice(&refined.knots.as_slice()[span + 1..]);

        Ok((
            Self::new(refined.degree, left_points, KnotVector::new(left_knots)?)?,
            Self::new(refined.degree, right_points, KnotVector::new(right_knots)?)?,
        ))
    }

    /// Samples the curve adaptively and returns native parameters with points.
    pub fn adaptive_samples(&self, tolerance: f64, max_depth: usize) -> Vec<(f64, Point2)> {
        let domain = self.domain();
        let mut samples = vec![(domain.start, self.point_at(domain.start))];
        adaptive_sample(
            self,
            domain.start,
            domain.end,
            tolerance,
            max_depth,
            &mut samples,
        );
        samples
    }

    /// Finds a native parameter whose curve point coincides with `point`.
    pub fn parameter_at(&self, point: Point2, tolerance: f64) -> Option<f64> {
        let domain = self.domain();
        let sample_count = 128usize;
        let mut parameter = (0..=sample_count)
            .map(|index| {
                let fraction = index as f64 / sample_count as f64;
                let u = domain.start + (domain.end - domain.start) * fraction;
                (u, (self.point_at(u) - point).norm_squared())
            })
            .min_by(|left, right| left.1.total_cmp(&right.1))?
            .0;

        for _ in 0..16 {
            let curve_point = self.point_at(parameter);
            let first = self.derivative_at(parameter, 1);
            let second = self.derivative_at(parameter, 2);
            let delta = curve_point - point;
            let denominator = first.norm_squared() + delta.dot(&second);
            if denominator.abs() <= f64::EPSILON {
                break;
            }
            let next =
                (parameter - delta.dot(&first) / denominator).clamp(domain.start, domain.end);
            if (next - parameter).abs() <= 1.0e-12 {
                parameter = next;
                break;
            }
            parameter = next;
        }

        ((self.point_at(parameter) - point).norm() <= tolerance).then_some(parameter)
    }

    fn clamp_parameter(&self, parameter: f64) -> f64 {
        let domain = self.domain();
        parameter.clamp(domain.start, domain.end)
    }

    fn insert_knot(&mut self, parameter: f64) {
        let p = self.degree.get();
        let n = self.control_points.len() - 1;
        let parameter = self.clamp_parameter(parameter);
        let span = self.knots.find_span(n, self.degree, parameter);
        let multiplicity = self.knots.multiplicity(parameter);
        let old = self.control_points.clone();
        let mut points = Vec::with_capacity(old.len() + 1);

        points.extend_from_slice(&old.as_slice()[..=span - p]);
        points.extend(std::iter::repeat_n(
            HPoint2::new(0.0, 0.0, 0.0),
            span - multiplicity - (span - p),
        ));
        points.extend_from_slice(&old.as_slice()[span - multiplicity..]);

        for (index, point) in points
            .iter_mut()
            .enumerate()
            .take(span - multiplicity + 1)
            .skip(span - p + 1)
        {
            let denominator = self.knots.get(index + p) - self.knots.get(index);
            let alpha = if denominator == 0.0 {
                0.0
            } else {
                (parameter - self.knots.get(index)) / denominator
            };
            let current = old.get(index).unwrap().0.coords;
            let previous = old.get(index - 1).unwrap().0.coords;
            *point = HPoint2(Point3::from(current * alpha + previous * (1.0 - alpha)));
        }

        self.control_points =
            ControlPolygon2::new(points).expect("knot insertion preserves control points");
        let mut knots = self.knots.as_slice().to_vec();
        knots.insert(span + 1, parameter);
        self.knots = KnotVector::new(knots).expect("knot insertion preserves ordering");
    }
}

fn chord_length_parameters(points: &[Point2]) -> Result<Vec<f64>, NurbsError> {
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

fn validate_interpolation_input(points: &[Point2], parameters: &[f64]) -> Result<(), NurbsError> {
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

fn interpolate_open(points: &[Point2], parameters: &[f64]) -> Result<NurbsCurve2, NurbsError> {
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
    let x = DVector::from_iterator(points.len(), points.iter().map(|point| point.x));
    let y = DVector::from_iterator(points.len(), points.iter().map(|point| point.y));
    let decomposition = coefficients.lu();
    let solved_x = decomposition
        .solve(&x)
        .ok_or(NurbsError::SingularInterpolationSystem)?;
    let solved_y = decomposition
        .solve(&y)
        .ok_or(NurbsError::SingularInterpolationSystem)?;
    let control_points = ControlPolygon2::new(
        solved_x
            .iter()
            .zip(solved_y.iter())
            .map(|(x, y)| HPoint2::from_cartesian(Point2::new(*x, *y), 1.0))
            .collect(),
    )?;
    let mut curve = NurbsCurve2::new(degree, control_points, knots)?;
    curve.interpolation_parameters = parameters.to_vec();
    Ok(curve)
}

fn interpolate_closed(points: &[Point2], parameters: &[f64]) -> Result<NurbsCurve2, NurbsError> {
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
    let mut curve = NurbsCurve2::new(
        degree,
        ControlPolygon2::from_cartesian(control_points, &vec![1.0; 3 * count + 1])?,
        KnotVector::new(knots)?,
    )?;
    curve.interpolation_parameters = parameters.to_vec();
    Ok(curve)
}

fn adaptive_sample(
    curve: &NurbsCurve2,
    start: f64,
    end: f64,
    tolerance: f64,
    depth: usize,
    samples: &mut Vec<(f64, Point2)>,
) {
    let midpoint = 0.5 * (start + end);
    let start_point = samples.last().unwrap().1;
    let end_point = curve.point_at(end);
    let midpoint_point = curve.point_at(midpoint);
    let chord_midpoint = Point2::from((start_point.coords + end_point.coords) * 0.5);

    if depth == 0 || (midpoint_point - chord_midpoint).norm() <= tolerance {
        samples.push((end, end_point));
        return;
    }
    adaptive_sample(curve, start, midpoint, tolerance, depth - 1, samples);
    adaptive_sample(curve, midpoint, end, tolerance, depth - 1, samples);
}

fn binomial(n: usize, k: usize) -> f64 {
    if k > n {
        return 0.0;
    }
    let k = k.min(n - k);
    (1..=k).fold(1.0, |accumulator, index| {
        accumulator * (n + 1 - index) as f64 / index as f64
    })
}
