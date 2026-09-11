//! Shared exact rational-quadratic construction for 2D conic arcs.
//!
//! The 3D counterpart lives in [`crate::geometry::dim3::conics`]; the two are
//! the same construction in different ambient spaces, kept separate because
//! neither point type is a projection of the other.

use crate::geometry::dim2::nurbs::{ControlPolygon2, HPoint2, NurbsCurve2};
use crate::geometry::dim2::utils::Point2;
use crate::geometry::dim3::nurbs::{Degree, KnotVector};
use crate::geometry::nurbs::error::NurbsError;
use crate::geometry::tolerance::LINEAR_TOLERANCE;
use nalgebra::Vector2;

/// Builds an exact piecewise rational-quadratic conic over `[start, end]`.
pub(crate) fn conic_arc_nurbs2(
    start: f64,
    end: f64,
    max_span: f64,
    point_at: impl Fn(f64) -> Point2,
    derivative_at: impl Fn(f64) -> Vector2<f64>,
) -> Result<NurbsCurve2, NurbsError> {
    if (end - start).abs() <= LINEAR_TOLERANCE {
        return Err(NurbsError::DegenerateInterval { start, end });
    }
    if end < start {
        return Ok(conic_arc_nurbs2(end, start, max_span, point_at, derivative_at)?.reversed());
    }

    let span_count = ((end - start) / max_span).ceil().max(1.0) as usize;
    let span = (end - start) / span_count as f64;
    let mut points = Vec::with_capacity(2 * span_count + 1);
    let mut knots = vec![start; 3];

    for index in 0..span_count {
        let t0 = start + index as f64 * span;
        let t2 = t0 + span;
        let midpoint_parameter = 0.5 * (t0 + t2);
        let p0 = point_at(t0);
        let p2 = point_at(t2);
        let p1 = tangent_intersection(p0, derivative_at(t0), p2, derivative_at(t2));
        let midpoint = point_at(midpoint_parameter);
        let chord_midpoint = p0 + 0.5 * (p2 - p0);
        let weight = (chord_midpoint - midpoint).norm() / (midpoint - p1).norm();

        if index == 0 {
            points.push(HPoint2::from_cartesian(p0, 1.0));
        }
        points.push(HPoint2::from_cartesian(p1, weight));
        points.push(HPoint2::from_cartesian(p2, 1.0));
        if index + 1 < span_count {
            knots.extend([t2, t2]);
        }
    }
    knots.extend([end, end, end]);

    NurbsCurve2::new(
        Degree::new(2)?,
        ControlPolygon2::new(points)?,
        KnotVector::new(knots)?,
    )
}

/// Intersects two tangent lines using their Gram matrix.
fn tangent_intersection(
    point_a: Point2,
    tangent_a: Vector2<f64>,
    point_b: Point2,
    tangent_b: Vector2<f64>,
) -> Point2 {
    let offset = point_b - point_a;
    let aa = tangent_a.dot(&tangent_a);
    let ab = tangent_a.dot(&tangent_b);
    let bb = tangent_b.dot(&tangent_b);
    let rhs_a = offset.dot(&tangent_a);
    let rhs_b = offset.dot(&tangent_b);
    let determinant = aa * bb - ab * ab;
    point_a + tangent_a * ((rhs_a * bb - rhs_b * ab) / determinant)
}
