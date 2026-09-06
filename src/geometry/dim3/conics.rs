//! Shared exact rational-quadratic construction for planar conic arcs.

use crate::geometry::dim3::nurbs::points::{ControlPolygon, HPoint};
use crate::geometry::dim3::nurbs::{Degree, KnotVector, NurbsCurve};
use crate::geometry::dim3::utils::Point3;
use crate::geometry::nurbs::error::NurbsError;
use crate::geometry::tolerance::LINEAR_TOLERANCE;
use nalgebra::Vector3;

/// Builds an exact piecewise rational-quadratic conic over `[start, end]`.
pub(crate) fn conic_arc_nurbs(
    start: f64,
    end: f64,
    max_span: f64,
    point_at: impl Fn(f64) -> Point3,
    derivative_at: impl Fn(f64) -> Vector3<f64>,
) -> Result<NurbsCurve, NurbsError> {
    if (end - start).abs() <= LINEAR_TOLERANCE {
        return Err(NurbsError::DegenerateInterval { start, end });
    }
    if end < start {
        return Ok(conic_arc_nurbs(end, start, max_span, point_at, derivative_at)?.reversed());
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
            points.push(HPoint::from_cartesian(p0, 1.0));
        }
        points.push(HPoint::from_cartesian(p1, weight));
        points.push(HPoint::from_cartesian(p2, 1.0));
        if index + 1 < span_count {
            knots.extend([t2, t2]);
        }
    }
    knots.extend([end, end, end]);

    NurbsCurve::new(
        Degree::new(2)?,
        ControlPolygon::new(points)?,
        KnotVector::new(knots)?,
    )
}

/// Intersects two coplanar tangent lines using their Gram matrix.
fn tangent_intersection(
    point_a: Point3,
    tangent_a: Vector3<f64>,
    point_b: Point3,
    tangent_b: Vector3<f64>,
) -> Point3 {
    let offset = point_b - point_a;
    let aa = tangent_a.dot(&tangent_a);
    let ab = tangent_a.dot(&tangent_b);
    let bb = tangent_b.dot(&tangent_b);
    let rhs_a = offset.dot(&tangent_a);
    let rhs_b = offset.dot(&tangent_b);
    let determinant = aa * bb - ab * ab;
    point_a + tangent_a * ((rhs_a * bb - rhs_b * ab) / determinant)
}
