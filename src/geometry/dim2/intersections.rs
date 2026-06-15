use nalgebra::Matrix2;
use thiserror::Error;

use super::bezier::Bezier2;
use super::curves::Curve2;
use crate::geometry::{Interval, LINEAR_TOLERANCE, NurbsError, Point2};

const OVERLAP_SAMPLES: [f64; 5] = [0.0, 0.25, 0.5, 0.75, 1.0];
const OVERLAP_TANGENT_DOT_TOLERANCE: f64 = 1.0e-6;
const EARLY_REFINEMENT_DEPTH: usize = 8;

/// A point or coincident interval shared by two 2D curves.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CurveCurveIntersection2 {
    Point {
        point: Point2,
        u_a: f64,
        u_b: f64,
    },
    Overlap {
        interval_a: Interval,
        interval_b: Interval,
    },
}

/// All intersections between two 2D curves.
pub type CurveCurveIntersections2 = Vec<CurveCurveIntersection2>;

/// Numerical controls for 2D curve-curve intersection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurveIntersectionOptions {
    pub linear_tolerance: f64,
    pub parameter_tolerance: f64,
    pub bbox_tolerance: f64,
    pub max_subdivision_depth: usize,
    pub leaf_diagonal_tolerance: f64,
    pub newton_max_iterations: usize,
}

impl CurveIntersectionOptions {
    /// Returns whether every option is finite and usable.
    pub fn validate(self) -> bool {
        self.linear_tolerance.is_finite()
            && self.linear_tolerance > 0.0
            && self.parameter_tolerance.is_finite()
            && self.parameter_tolerance > 0.0
            && self.bbox_tolerance.is_finite()
            && self.bbox_tolerance >= 0.0
            && self.leaf_diagonal_tolerance.is_finite()
            && self.leaf_diagonal_tolerance > 0.0
            && self.max_subdivision_depth > 0
            && self.newton_max_iterations > 0
    }

    fn linear_tolerance_squared(self) -> f64 {
        self.linear_tolerance * self.linear_tolerance
    }
}

impl Default for CurveIntersectionOptions {
    fn default() -> Self {
        Self {
            linear_tolerance: LINEAR_TOLERANCE,
            parameter_tolerance: 1.0e-10,
            bbox_tolerance: LINEAR_TOLERANCE,
            max_subdivision_depth: 32,
            leaf_diagonal_tolerance: LINEAR_TOLERANCE * 10.0,
            newton_max_iterations: 48,
        }
    }
}

/// Failure to prepare or execute a 2D curve intersection.
#[derive(Debug, Clone, Error, PartialEq)]
pub enum CurveIntersectionError {
    #[error("invalid 2D curve intersection options")]
    InvalidOptions,
    #[error("2D NURBS conversion failed")]
    Nurbs(#[from] NurbsError),
}

#[derive(Debug, Clone, Copy)]
struct Bounds2 {
    min: Point2,
    max: Point2,
}

impl Bounds2 {
    fn from_bezier(bezier: &Bezier2) -> Self {
        let mut points = bezier
            .control_points()
            .as_slice()
            .iter()
            .map(|point| point.to_cartesian());
        let first = points
            .next()
            .expect("Bezier control polygons are non-empty");
        points.fold(
            Self {
                min: first,
                max: first,
            },
            |bounds, point| Self {
                min: Point2::new(bounds.min.x.min(point.x), bounds.min.y.min(point.y)),
                max: Point2::new(bounds.max.x.max(point.x), bounds.max.y.max(point.y)),
            },
        )
    }

    fn diagonal_length(self) -> f64 {
        (self.max - self.min).norm()
    }

    fn intersects(self, other: Self, tolerance: f64) -> bool {
        self.min.x <= other.max.x + tolerance
            && self.max.x + tolerance >= other.min.x
            && self.min.y <= other.max.y + tolerance
            && self.max.y + tolerance >= other.min.y
    }
}

#[derive(Clone)]
struct CurvePiece {
    bezier: Bezier2,
    bounds: Bounds2,
    depth: usize,
}

impl CurvePiece {
    fn new(bezier: Bezier2) -> Self {
        let bounds = Bounds2::from_bezier(&bezier);
        Self {
            bezier,
            bounds,
            depth: 0,
        }
    }

    fn diagonal_length(&self) -> f64 {
        self.bounds.diagonal_length()
    }

    fn split(&self) -> Option<(Self, Self)> {
        let domain = self.bezier.domain();
        if domain.is_degenerate(LINEAR_TOLERANCE) {
            return None;
        }
        let midpoint = 0.5 * (domain.start + domain.end);
        let (left, right) = self.bezier.subdivide(midpoint).ok()?;
        Some((
            Self {
                bounds: Bounds2::from_bezier(&left),
                bezier: left,
                depth: self.depth + 1,
            },
            Self {
                bounds: Bounds2::from_bezier(&right),
                bezier: right,
                depth: self.depth + 1,
            },
        ))
    }
}

/// Intersects two 2D curves using default tolerances.
pub fn intersect_curves(
    a: &Curve2,
    b: &Curve2,
) -> Result<CurveCurveIntersections2, CurveIntersectionError> {
    intersect_curves_with_options(a, b, CurveIntersectionOptions::default())
}

/// Intersects two 2D curves using explicit numerical controls.
pub fn intersect_curves_with_options(
    a: &Curve2,
    b: &Curve2,
    options: CurveIntersectionOptions,
) -> Result<CurveCurveIntersections2, CurveIntersectionError> {
    if !options.validate() {
        return Err(CurveIntersectionError::InvalidOptions);
    }

    let nurbs_a = a.to_nurbs()?;
    let nurbs_b = b.to_nurbs()?;
    let domain_a = nurbs_a.domain();
    let domain_b = nurbs_b.domain();
    let spans_a = nurbs_a
        .bezier_spans()?
        .into_iter()
        .map(CurvePiece::new)
        .collect::<Vec<_>>();
    let spans_b = nurbs_b
        .bezier_spans()?
        .into_iter()
        .map(CurvePiece::new)
        .collect::<Vec<_>>();
    let mut intersections = Vec::new();

    for span_a in &spans_a {
        for span_b in &spans_b {
            intersect_pieces(span_a.clone(), span_b.clone(), options, &mut intersections);
        }
    }

    Ok(dedup_intersections(intersections, options)
        .into_iter()
        .map(|intersection| normalize_intersection(intersection, domain_a, domain_b))
        .collect())
}

fn intersect_pieces(
    a: CurvePiece,
    b: CurvePiece,
    options: CurveIntersectionOptions,
    intersections: &mut CurveCurveIntersections2,
) {
    if !a.bounds.intersects(b.bounds, options.bbox_tolerance) {
        return;
    }

    if let Some(intersection) = straight_span_intersection(&a.bezier, &b.bezier, options) {
        intersections.push(intersection);
        return;
    }
    if a.bezier.degree().get() == 1 && b.bezier.degree().get() == 1 {
        return;
    }
    if let Some(overlap) = matching_bezier_overlap(&a.bezier, &b.bezier, options) {
        intersections.push(overlap);
        return;
    }

    if a.depth + b.depth >= EARLY_REFINEMENT_DEPTH
        && let Some(point) = refine_point(&a.bezier, &b.bezier, options)
    {
        intersections.push(point);
        return;
    }

    let leaf_a = a.diagonal_length() <= options.leaf_diagonal_tolerance;
    let leaf_b = b.diagonal_length() <= options.leaf_diagonal_tolerance;
    let max_depth =
        a.depth >= options.max_subdivision_depth && b.depth >= options.max_subdivision_depth;
    if (leaf_a && leaf_b) || max_depth {
        if let Some(point) = refine_point(&a.bezier, &b.bezier, options) {
            intersections.push(point);
        }
        return;
    }

    let split_a = !leaf_a
        && a.depth < options.max_subdivision_depth
        && (leaf_b || a.diagonal_length() >= b.diagonal_length());
    if split_a && let Some((left, right)) = a.split() {
        intersect_pieces(left, b.clone(), options, intersections);
        intersect_pieces(right, b, options, intersections);
        return;
    }

    if !leaf_b
        && b.depth < options.max_subdivision_depth
        && let Some((left, right)) = b.split()
    {
        intersect_pieces(a.clone(), left, options, intersections);
        intersect_pieces(a, right, options, intersections);
    } else if let Some(point) = refine_point(&a.bezier, &b.bezier, options) {
        intersections.push(point);
    }
}

fn straight_span_intersection(
    a: &Bezier2,
    b: &Bezier2,
    options: CurveIntersectionOptions,
) -> Option<CurveCurveIntersection2> {
    if a.degree().get() != 1 || b.degree().get() != 1 {
        return None;
    }

    let a0 = a.point_at(a.domain().start);
    let a1 = a.point_at(a.domain().end);
    let b0 = b.point_at(b.domain().start);
    let b1 = b.point_at(b.domain().end);
    let direction_a = a1 - a0;
    let direction_b = b1 - b0;
    let denominator = cross(direction_a, direction_b);
    let offset = b0 - a0;

    if denominator.abs() > options.linear_tolerance * direction_a.norm() * direction_b.norm() {
        let fraction_a = cross(offset, direction_b) / denominator;
        let fraction_b = cross(offset, direction_a) / denominator;
        if !parameter_fraction_is_bounded(fraction_a, options)
            || !parameter_fraction_is_bounded(fraction_b, options)
        {
            return None;
        }
        let point = a0 + direction_a * fraction_a.clamp(0.0, 1.0);
        return Some(CurveCurveIntersection2::Point {
            point,
            u_a: a.parameter_at(point, options.linear_tolerance)?,
            u_b: b.parameter_at(point, options.linear_tolerance)?,
        });
    }

    if cross(offset, direction_a).abs() > options.linear_tolerance * direction_a.norm().max(1.0) {
        return None;
    }

    straight_span_overlap(a, b, a0, a1, b0, b1, options)
}

fn straight_span_overlap(
    a: &Bezier2,
    b: &Bezier2,
    a0: Point2,
    a1: Point2,
    b0: Point2,
    b1: Point2,
    options: CurveIntersectionOptions,
) -> Option<CurveCurveIntersection2> {
    let direction = a1 - a0;
    let length = direction.norm();
    if length <= options.linear_tolerance {
        return None;
    }
    let axis = direction / length;
    let b0_distance = (b0 - a0).dot(&axis);
    let b1_distance = (b1 - a0).dot(&axis);
    let overlap_start = 0.0_f64.max(b0_distance.min(b1_distance));
    let overlap_end = length.min(b0_distance.max(b1_distance));
    if overlap_start > overlap_end + options.linear_tolerance {
        return None;
    }

    let start_point = a0 + axis * overlap_start;
    if overlap_end - overlap_start <= options.linear_tolerance {
        return Some(CurveCurveIntersection2::Point {
            point: start_point,
            u_a: a.parameter_at(start_point, options.linear_tolerance)?,
            u_b: b.parameter_at(start_point, options.linear_tolerance)?,
        });
    }

    let end_point = a0 + axis * overlap_end;
    Some(CurveCurveIntersection2::Overlap {
        interval_a: Interval::new(
            a.parameter_at(start_point, options.linear_tolerance)?,
            a.parameter_at(end_point, options.linear_tolerance)?,
        ),
        interval_b: Interval::new(
            b.parameter_at(start_point, options.linear_tolerance)?,
            b.parameter_at(end_point, options.linear_tolerance)?,
        ),
    })
}

fn parameter_fraction_is_bounded(fraction: f64, options: CurveIntersectionOptions) -> bool {
    fraction >= -options.parameter_tolerance && fraction <= 1.0 + options.parameter_tolerance
}

fn cross(a: nalgebra::Vector2<f64>, b: nalgebra::Vector2<f64>) -> f64 {
    a.x * b.y - a.y * b.x
}

fn matching_bezier_overlap(
    a: &Bezier2,
    b: &Bezier2,
    options: CurveIntersectionOptions,
) -> Option<CurveCurveIntersection2> {
    if a.degree() != b.degree() {
        return None;
    }
    if same_bezier_samples(a, b, false, options) {
        return Some(CurveCurveIntersection2::Overlap {
            interval_a: a.domain(),
            interval_b: b.domain(),
        });
    }
    same_bezier_samples(a, b, true, options).then_some(CurveCurveIntersection2::Overlap {
        interval_a: a.domain(),
        interval_b: Interval::new(b.domain().end, b.domain().start),
    })
}

fn same_bezier_samples(
    a: &Bezier2,
    b: &Bezier2,
    reverse_b: bool,
    options: CurveIntersectionOptions,
) -> bool {
    OVERLAP_SAMPLES.iter().copied().all(|fraction| {
        let u = lerp_domain(a.domain(), fraction);
        let b_fraction = if reverse_b { 1.0 - fraction } else { fraction };
        let v = lerp_domain(b.domain(), b_fraction);
        (a.point_at(u) - b.point_at(v)).norm() <= options.linear_tolerance
            && tangents_are_compatible(a, b, u, v, reverse_b)
    })
}

fn tangents_are_compatible(a: &Bezier2, b: &Bezier2, u: f64, v: f64, reverse_b: bool) -> bool {
    let derivative_a = a.derivative_at(u, 1);
    let derivative_b = b.derivative_at(v, 1);
    let denominator = derivative_a.norm() * derivative_b.norm();
    if denominator <= LINEAR_TOLERANCE {
        return false;
    }
    let dot = derivative_a.dot(&derivative_b) / denominator;
    if reverse_b {
        dot <= -1.0 + OVERLAP_TANGENT_DOT_TOLERANCE
    } else {
        dot >= 1.0 - OVERLAP_TANGENT_DOT_TOLERANCE
    }
}

fn lerp_domain(domain: Interval, fraction: f64) -> f64 {
    domain.start + (domain.end - domain.start) * fraction
}

fn refine_point(
    a: &Bezier2,
    b: &Bezier2,
    options: CurveIntersectionOptions,
) -> Option<CurveCurveIntersection2> {
    let mut u = 0.5 * (a.domain().start + a.domain().end);
    let mut v = 0.5 * (b.domain().start + b.domain().end);

    for _ in 0..options.newton_max_iterations {
        let point_a = a.point_at(u);
        let point_b = b.point_at(v);
        let residual = point_a - point_b;
        let derivative_a = a.derivative_at(u, 1);
        let derivative_b = b.derivative_at(v, 1);
        let jacobian = Matrix2::from_columns(&[derivative_a, -derivative_b]);
        let Some(delta) = jacobian.lu().solve(&residual) else {
            break;
        };

        u = (u - delta.x).clamp(a.domain().start, a.domain().end);
        v = (v - delta.y).clamp(b.domain().start, b.domain().end);
        if delta.norm() <= options.parameter_tolerance {
            break;
        }
    }

    let point_a = a.point_at(u);
    let point_b = b.point_at(v);
    ((point_a - point_b).norm_squared() <= options.linear_tolerance_squared()).then(|| {
        CurveCurveIntersection2::Point {
            point: Point2::from((point_a.coords + point_b.coords) * 0.5),
            u_a: u,
            u_b: v,
        }
    })
}

fn dedup_intersections(
    intersections: CurveCurveIntersections2,
    options: CurveIntersectionOptions,
) -> CurveCurveIntersections2 {
    let mut deduped = Vec::new();
    let mut counts = Vec::new();
    for intersection in intersections {
        if let Some(index) = deduped
            .iter()
            .position(|existing| same_intersection(existing, &intersection, options))
        {
            merge_intersection(&mut deduped[index], intersection, counts[index]);
            counts[index] += 1;
        } else {
            deduped.push(intersection);
            counts.push(1);
        }
    }
    deduped
}

fn merge_intersection(
    existing: &mut CurveCurveIntersection2,
    incoming: CurveCurveIntersection2,
    existing_count: usize,
) {
    let (
        CurveCurveIntersection2::Point { point, u_a, u_b },
        CurveCurveIntersection2::Point {
            point: incoming_point,
            u_a: incoming_u_a,
            u_b: incoming_u_b,
        },
    ) = (existing, incoming)
    else {
        return;
    };

    let count = existing_count as f64;
    *point = Point2::from((point.coords * count + incoming_point.coords) / (count + 1.0));
    *u_a = (*u_a * count + incoming_u_a) / (count + 1.0);
    *u_b = (*u_b * count + incoming_u_b) / (count + 1.0);
}

fn same_intersection(
    a: &CurveCurveIntersection2,
    b: &CurveCurveIntersection2,
    options: CurveIntersectionOptions,
) -> bool {
    match (a, b) {
        (
            CurveCurveIntersection2::Point {
                point: point_a,
                u_a: a_u_a,
                u_b: a_u_b,
            },
            CurveCurveIntersection2::Point {
                point: point_b,
                u_a: b_u_a,
                u_b: b_u_b,
            },
        ) => {
            let parameter_merge_tolerance = options
                .linear_tolerance
                .sqrt()
                .max(options.parameter_tolerance * 10.0);
            let point_merge_tolerance = options
                .linear_tolerance
                .sqrt()
                .max(options.linear_tolerance * 10.0);
            (point_a - point_b).norm() <= point_merge_tolerance
                && (a_u_a - b_u_a).abs() <= parameter_merge_tolerance
                && (a_u_b - b_u_b).abs() <= parameter_merge_tolerance
        }
        (
            CurveCurveIntersection2::Overlap {
                interval_a: a_interval_a,
                interval_b: a_interval_b,
            },
            CurveCurveIntersection2::Overlap {
                interval_a: b_interval_a,
                interval_b: b_interval_b,
            },
        ) => {
            same_interval(*a_interval_a, *b_interval_a, options)
                && same_interval(*a_interval_b, *b_interval_b, options)
        }
        _ => false,
    }
}

fn same_interval(a: Interval, b: Interval, options: CurveIntersectionOptions) -> bool {
    (a.start - b.start).abs() <= options.parameter_tolerance
        && (a.end - b.end).abs() <= options.parameter_tolerance
}

fn normalize_intersection(
    intersection: CurveCurveIntersection2,
    domain_a: Interval,
    domain_b: Interval,
) -> CurveCurveIntersection2 {
    match intersection {
        CurveCurveIntersection2::Point { point, u_a, u_b } => CurveCurveIntersection2::Point {
            point,
            u_a: normalize_parameter(domain_a, u_a),
            u_b: normalize_parameter(domain_b, u_b),
        },
        CurveCurveIntersection2::Overlap {
            interval_a,
            interval_b,
        } => CurveCurveIntersection2::Overlap {
            interval_a: Interval::new(
                normalize_parameter(domain_a, interval_a.start),
                normalize_parameter(domain_a, interval_a.end),
            ),
            interval_b: Interval::new(
                normalize_parameter(domain_b, interval_b.start),
                normalize_parameter(domain_b, interval_b.end),
            ),
        },
    }
}

fn normalize_parameter(domain: Interval, parameter: f64) -> f64 {
    (parameter - domain.start) / (domain.end - domain.start)
}
