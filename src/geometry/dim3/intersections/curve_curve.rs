use nalgebra::{Matrix2, Vector2};

use super::analytic::intersect_analytic_curves;
use super::error::IntersectionError;
use super::options::IntersectionOptions;
use super::{
    CurveCurveIntersection, CurveCurveIntersections, IntersectionCoverage,
    IntersectionIncompleteReason,
};
use crate::geometry::{Bezier, Curve, Interval, LINEAR_TOLERANCE, Point3, PointCoincidence};

const OVERLAP_SAMPLES: [f64; 5] = [0.0, 0.25, 0.5, 0.75, 1.0];
const OVERLAP_TANGENT_DOT_TOLERANCE: f64 = 1.0e-6;
const EARLY_REFINEMENT_DEPTH: usize = 8;

/// Node visits allowed per query before the search reports itself incomplete.
///
/// Transverse candidates isolate within a few dozen nodes; only contacts this
/// solver cannot resolve approach the cap.
const SEARCH_NODE_BUDGET: usize = 4_096;

#[derive(Clone)]
struct CurvePiece {
    bezier: Bezier,
    depth: usize,
}

impl CurvePiece {
    fn new(bezier: Bezier) -> Self {
        Self { bezier, depth: 0 }
    }

    fn domain(&self) -> Interval {
        self.bezier.domain()
    }

    fn diagonal_length(&self) -> f64 {
        self.bezier.bbox().diagonal_length()
    }

    fn midpoint(&self) -> f64 {
        let domain = self.domain();
        0.5 * (domain.start.value() + domain.end.value())
    }

    fn split(&self) -> Option<(Self, Self)> {
        let domain = self.domain();
        if domain.is_degenerate(LINEAR_TOLERANCE) {
            return None;
        }
        let midpoint = self.midpoint();
        let (left, right) = self.bezier.subdivide(midpoint).ok()?;
        Some((
            Self {
                bezier: left,
                depth: self.depth + 1,
            },
            Self {
                bezier: right,
                depth: self.depth + 1,
            },
        ))
    }
}

pub fn intersect_curves(
    a: &Curve,
    b: &Curve,
) -> Result<CurveCurveIntersections, IntersectionError> {
    intersect_curves_with_options(a, b, IntersectionOptions::default())
}

pub fn intersect_curves_with_options(
    a: &Curve,
    b: &Curve,
    options: IntersectionOptions,
) -> Result<CurveCurveIntersections, IntersectionError> {
    if !options.validate() {
        return Err(IntersectionError::InvalidOptions);
    }
    if let Some(analytic) = intersect_analytic_curves(a, b, options) {
        return analytic;
    }

    let spans_a = a
        .to_nurbs()?
        .bezier_spans()?
        .into_iter()
        .map(CurvePiece::new)
        .collect::<Vec<_>>();
    let spans_b = b
        .to_nurbs()?
        .bezier_spans()?
        .into_iter()
        .map(CurvePiece::new)
        .collect::<Vec<_>>();
    let mut search = Search {
        options,
        intersections: Vec::new(),
        reasons: Vec::new(),
        budget: SEARCH_NODE_BUDGET,
    };

    for span_a in &spans_a {
        for span_b in &spans_b {
            search.visit(span_a.clone(), span_b.clone());
        }
    }

    Ok(CurveCurveIntersections::new(
        dedup_intersections(search.intersections, options),
        if search.reasons.is_empty() {
            IntersectionCoverage::Complete
        } else {
            IntersectionCoverage::Incomplete(search.reasons)
        },
    ))
}

/// One bounded embedding search over a single curve pair.
struct Search {
    options: IntersectionOptions,
    intersections: Vec<CurveCurveIntersection>,
    reasons: Vec<IntersectionIncompleteReason>,
    /// Remaining node visits for this query.
    ///
    /// Bounding each side's depth separately admits the product of both
    /// budgets, which a pair that keeps overlapping however far it is split --
    /// two tangent or coincident curves -- never finishes. Running out is
    /// reported rather than passed off as an empty result.
    budget: usize,
}

impl Search {
    fn push_reason(&mut self, reason: IntersectionIncompleteReason) {
        if !self.reasons.contains(&reason) {
            self.reasons.push(reason);
        }
    }

    fn visit(&mut self, a: CurvePiece, b: CurvePiece) {
        let options = self.options;
        let Some(remaining) = self.budget.checked_sub(1) else {
            self.push_reason(IntersectionIncompleteReason::SubdivisionBudgetExhausted);
            return;
        };
        self.budget = remaining;

        if !a
            .bezier
            .bbox()
            .expanded(options.bbox_tolerance)
            .intersects(&b.bezier.bbox(), options.bbox_tolerance)
        {
            return;
        }

        if let Some(overlap) = line_overlap(&a.bezier, &b.bezier, options) {
            self.intersections.push(overlap);
            return;
        }
        if let Some(overlap) = matching_bezier_overlap(&a.bezier, &b.bezier, options) {
            self.intersections.push(overlap);
            return;
        }

        if a.depth + b.depth >= EARLY_REFINEMENT_DEPTH
            && let Some(point) = refine_point(&a.bezier, &b.bezier, options)
        {
            self.intersections.push(point);
            return;
        }

        let leaf_a = a.diagonal_length() <= options.leaf_diagonal_tolerance;
        let leaf_b = b.diagonal_length() <= options.leaf_diagonal_tolerance;
        let max_depth =
            a.depth >= options.max_subdivision_depth && b.depth >= options.max_subdivision_depth;
        if (leaf_a && leaf_b) || max_depth {
            if let Some(point) = refine_point(&a.bezier, &b.bezier, options) {
                self.intersections.push(point);
            }
            return;
        }

        let split_a = (!leaf_a && a.depth < options.max_subdivision_depth)
            && (leaf_b || a.diagonal_length() >= b.diagonal_length());
        if split_a && let Some((left, right)) = a.split() {
            self.visit(left, b.clone());
            self.visit(right, b);
            return;
        }

        if !leaf_b
            && b.depth < options.max_subdivision_depth
            && let Some((left, right)) = b.split()
        {
            self.visit(a.clone(), left);
            self.visit(a, right);
        } else if let Some(point) = refine_point(&a.bezier, &b.bezier, options) {
            self.intersections.push(point);
        }
    }
}

fn line_overlap(
    a: &Bezier,
    b: &Bezier,
    options: IntersectionOptions,
) -> Option<CurveCurveIntersection> {
    if a.degree().get() != 1 || b.degree().get() != 1 {
        return None;
    }
    let a0 = a.point_at(a.domain().start.value());
    let a1 = a.point_at(a.domain().end.value());
    let b0 = b.point_at(b.domain().start.value());
    let b1 = b.point_at(b.domain().end.value());
    let a_dir = a1 - a0;
    let b_dir = b1 - b0;
    let a_len = a_dir.norm();
    let b_len = b_dir.norm();
    if a_len <= options.linear_tolerance || b_len <= options.linear_tolerance {
        return None;
    }
    if a_dir.cross(&b_dir).norm() > options.linear_tolerance * a_len * b_len {
        return None;
    }
    if (b0 - a0).cross(&a_dir).norm() > options.linear_tolerance * a_len {
        return None;
    }

    let axis = a_dir / a_len;
    let b0_s = (b0 - a0).dot(&axis);
    let b1_s = (b1 - a0).dot(&axis);
    let overlap_start = 0.0_f64.max(b0_s.min(b1_s));
    let overlap_end = a_len.min(b0_s.max(b1_s));
    if overlap_start > overlap_end + options.linear_tolerance {
        return None;
    }

    if (overlap_end - overlap_start).abs() <= options.linear_tolerance {
        let point = a0 + axis * overlap_start;
        return Some(CurveCurveIntersection::Point {
            point,
            u_a: parameter_from_line_distance(a.domain(), overlap_start / a_len),
            u_b: parameter_on_second_line(b.domain(), b0_s, b1_s, overlap_start),
        });
    }

    Some(CurveCurveIntersection::Overlap {
        interval_a: Interval::new(
            parameter_from_line_distance(a.domain(), overlap_start / a_len),
            parameter_from_line_distance(a.domain(), overlap_end / a_len),
        ),
        interval_b: Interval::new(
            parameter_on_second_line(b.domain(), b0_s, b1_s, overlap_start),
            parameter_on_second_line(b.domain(), b0_s, b1_s, overlap_end),
        ),
    })
}

fn parameter_from_line_distance(domain: Interval, t: f64) -> f64 {
    domain.start.value() + (domain.end.value() - domain.start.value()) * t
}

fn parameter_on_second_line(domain: Interval, b0_s: f64, b1_s: f64, target: f64) -> f64 {
    let denom = b1_s - b0_s;
    if denom.abs() <= f64::EPSILON {
        return domain.start.value();
    }
    domain.start.value() + (domain.end.value() - domain.start.value()) * ((target - b0_s) / denom)
}

fn matching_bezier_overlap(
    a: &Bezier,
    b: &Bezier,
    options: IntersectionOptions,
) -> Option<CurveCurveIntersection> {
    if a.degree() != b.degree() {
        return None;
    }
    if same_bezier_samples(a, b, false, options) || same_bezier_samples(a, b, true, options) {
        return Some(CurveCurveIntersection::Overlap {
            interval_a: a.domain(),
            interval_b: b.domain(),
        });
    }
    None
}

fn same_bezier_samples(
    a: &Bezier,
    b: &Bezier,
    reverse_b: bool,
    options: IntersectionOptions,
) -> bool {
    OVERLAP_SAMPLES.iter().copied().all(|t| {
        let u = normalized_parameter(a.domain(), t);
        let v = normalized_parameter(b.domain(), if reverse_b { 1.0 - t } else { t });
        a.point_at(u)
            .coincides(b.point_at(v), options.linear_tolerance)
            && tangents_are_compatible(a, b, u, v, reverse_b)
    })
}

fn tangents_are_compatible(a: &Bezier, b: &Bezier, u: f64, v: f64, reverse_b: bool) -> bool {
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

fn normalized_parameter(domain: Interval, t: f64) -> f64 {
    domain.start.value() + (domain.end.value() - domain.start.value()) * t
}

fn refine_point(
    a: &Bezier,
    b: &Bezier,
    options: IntersectionOptions,
) -> Option<CurveCurveIntersection> {
    let mut u = 0.5 * (a.domain().start.value() + a.domain().end.value());
    let mut v = 0.5 * (b.domain().start.value() + b.domain().end.value());

    for _ in 0..options.newton_max_iterations {
        let point_a = a.point_at(u);
        let point_b = b.point_at(v);
        let residual = point_a - point_b;
        let a_derivatives = [a.derivative_at(u, 1), a.derivative_at(u, 2)];
        let b_derivatives = [b.derivative_at(v, 1), b.derivative_at(v, 2)];
        let c1 = a_derivatives[0];
        let c2 = a_derivatives[1];
        let d1 = b_derivatives[0];
        let d2 = b_derivatives[1];

        let f = Vector2::new(residual.dot(&c1), residual.dot(&d1));
        let jacobian = Matrix2::new(
            c1.dot(&c1) + residual.dot(&c2),
            -d1.dot(&c1),
            c1.dot(&d1),
            residual.dot(&d2) - d1.dot(&d1),
        );
        let Some(delta) = jacobian.lu().solve(&f) else {
            break;
        };

        u = (u - delta.x).clamp(a.domain().start.value(), a.domain().end.value());
        v = (v - delta.y).clamp(b.domain().start.value(), b.domain().end.value());
        if delta.norm() <= options.parameter_tolerance {
            break;
        }
    }

    let point_a = a.point_at(u);
    let point_b = b.point_at(v);
    (point_a - point_b)
        .norm_squared()
        .le(&options.linear_tolerance_squared())
        .then(|| CurveCurveIntersection::Point {
            point: midpoint(point_a, point_b),
            u_a: u,
            u_b: v,
        })
}

fn midpoint(a: Point3, b: Point3) -> Point3 {
    Point3::from((a.coords + b.coords) * 0.5)
}

fn dedup_intersections(
    intersections: Vec<CurveCurveIntersection>,
    options: IntersectionOptions,
) -> Vec<CurveCurveIntersection> {
    let mut deduped = Vec::new();
    let mut counts = Vec::new();
    for intersection in intersections {
        if let Some(index) = deduped
            .iter()
            .position(|existing| same_intersection(existing, &intersection, options))
        {
            merge_intersection(&mut deduped[index], &intersection, counts[index]);
            counts[index] += 1;
        } else {
            deduped.push(intersection);
            counts.push(1);
        }
    }
    deduped
}

fn merge_intersection(
    existing: &mut CurveCurveIntersection,
    incoming: &CurveCurveIntersection,
    existing_count: usize,
) {
    if let (
        CurveCurveIntersection::Point { point, u_a, u_b },
        CurveCurveIntersection::Point {
            point: incoming_point,
            u_a: incoming_u_a,
            u_b: incoming_u_b,
        },
    ) = (existing, incoming)
    {
        let count = existing_count as f64;
        *point = Point3::from((point.coords * count + incoming_point.coords) / (count + 1.0));
        *u_a = (*u_a * count + *incoming_u_a) / (count + 1.0);
        *u_b = (*u_b * count + *incoming_u_b) / (count + 1.0);
    }
}

fn same_intersection(
    a: &CurveCurveIntersection,
    b: &CurveCurveIntersection,
    options: IntersectionOptions,
) -> bool {
    match (a, b) {
        (
            CurveCurveIntersection::Point {
                point: pa,
                u_a: ua0,
                u_b: ub0,
            },
            CurveCurveIntersection::Point {
                point: pb,
                u_a: ua1,
                u_b: ub1,
            },
        ) => {
            let _ = (ua0, ub0, ua1, ub1);
            pa.coincides(*pb, point_merge_tolerance(options))
        }
        (
            CurveCurveIntersection::Overlap {
                interval_a: a0,
                interval_b: b0,
            },
            CurveCurveIntersection::Overlap {
                interval_a: a1,
                interval_b: b1,
            },
        ) => same_interval(*a0, *a1, options) && same_interval(*b0, *b1, options),
        _ => false,
    }
}

fn point_merge_tolerance(options: IntersectionOptions) -> f64 {
    (options.linear_tolerance.sqrt() * 10.0).max(options.linear_tolerance)
}

fn same_interval(a: Interval, b: Interval, options: IntersectionOptions) -> bool {
    let a = a.ordered();
    let b = b.ordered();
    (a.start - b.start).abs() <= options.parameter_tolerance
        && (a.end - b.end).abs() <= options.parameter_tolerance
}
