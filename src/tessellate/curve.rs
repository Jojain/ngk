//! Curve sampling: any [`Curve`] from `t0` to `t1` → [`Polyline3`].
//!
//! Single uniform-sampling implementation for now. NURBS curves get the same
//! treatment via [`Curve::point_at`] — for adaptive NURBS sampling reach for
//! [`crate::geometry::tessellate_curve_adaptive`] directly.

use super::{CurveOpts, Polyline3};
use crate::geometry::Curve;

/// Sample `curve` uniformly between `t0` and `t1` into `opts.segments + 1`
/// points. The endpoints are always present.
pub fn tessellate_curve(curve: &Curve, t0: f64, t1: f64, opts: CurveOpts) -> Polyline3 {
    let segments = opts.segments.max(1);
    let mut points = Vec::with_capacity(segments + 1);
    for i in 0..=segments {
        let t = t0 + (t1 - t0) * (i as f64 / segments as f64);
        points.push(curve.point_at(t));
    }
    Polyline3 { points }
}
