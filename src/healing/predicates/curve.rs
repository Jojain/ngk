//! Deciding whether two edge curves describe one continuous piece of geometry.
//!
//! Splitting an edge does not preserve its representation: `Curve::trimmed`
//! converts to NURBS, so the two halves of a line are two degree-1 NURBS
//! curves, not two `Bounded` views of the parent. Structural comparison
//! therefore misses the very case healing exists for, and the tests here work
//! on sampled geometry instead: both curves are sampled, one analytic support
//! is fitted through the three interesting points, and the support is accepted
//! only when every sample lies on it.
//!
//! Lines and circles are the supports this module knows. A free-form pair is
//! reported as not joinable rather than approximated, so healing leaves it
//! alone.

use std::f64::consts::TAU;

use nalgebra::{UnitVector3, Vector3};

use crate::geometry::{Curve, Interval, Plane, Point3};

/// Samples taken per curve when testing a candidate support.
pub const SUPPORT_SAMPLES: usize = 12;

/// Samples a curve over the parameter span between two of its points.
///
/// The span follows the curve's own parameterization, so the samples run from
/// `start` to `end` whichever way the curve is stored.
pub fn sample_between(curve: &Curve, start: Point3, end: Point3, segments: usize) -> Vec<Point3> {
    let span = curve.parameters_between(start, end);
    let segments = segments.max(1);
    (0..=segments)
        .map(|index| {
            let fraction = index as f64 / segments as f64;
            curve.point_at(span.start + (span.end - span.start) * fraction)
        })
        .collect()
}

/// Builds the single curve that replaces two edges meeting at `through`.
///
/// The result runs from `start` to `end` and passes through `through`. It is
/// `None` when the two curves do not share one line or circle, when `through`
/// is not interior to the result, or when the result would close on itself.
pub fn join_curves(
    first: &Curve,
    second: &Curve,
    start: Point3,
    through: Point3,
    end: Point3,
    linear: f64,
    angular: f64,
) -> Option<Curve> {
    let mut samples = sample_between(first, start, through, SUPPORT_SAMPLES);
    samples.extend(sample_between(second, through, end, SUPPORT_SAMPLES));
    join_on_line(&samples, start, through, end, linear)
        .or_else(|| join_on_circle(&samples, start, through, end, linear, angular))
}

/// Accepts the straight support when every sample lies on the `start`-`end`
/// segment and `through` is interior to it.
fn join_on_line(
    samples: &[Point3],
    start: Point3,
    through: Point3,
    end: Point3,
    linear: f64,
) -> Option<Curve> {
    let chord = end - start;
    let length = chord.norm();
    if length <= linear {
        return None;
    }
    let direction = chord / length;
    let offset = |point: Point3| {
        let along = (point - start).dot(&direction);
        (along, (point - start - direction * along).norm())
    };

    let (along, away) = offset(through);
    if away > linear || along <= linear || along >= length - linear {
        return None;
    }
    for &sample in samples {
        let (along, away) = offset(sample);
        if away > linear || along < -linear || along > length + linear {
            return None;
        }
    }
    Some(Curve::line(start, end))
}

/// Accepts the circular support when every sample lies on the circle through
/// `start`, `through` and `end`.
fn join_on_circle(
    samples: &[Point3],
    start: Point3,
    through: Point3,
    end: Point3,
    linear: f64,
    angular: f64,
) -> Option<Curve> {
    let (center, normal, radius) = circle_through(start, through, end, linear)?;
    if samples
        .iter()
        .any(|&sample| !on_circle(center, &normal, radius, sample, linear))
    {
        return None;
    }

    let plane = Plane::new(center, start - center, normal);
    let angle = |point: Point3| {
        let radial = point - center;
        radial.dot(&plane.y_dir()).atan2(radial.dot(&plane.x_dir()))
    };
    let interior = angle(through);
    if interior.abs() <= angular {
        return None;
    }

    let closing = angle(end);
    let sweep = if interior > 0.0 {
        if closing > interior {
            closing
        } else {
            closing + TAU
        }
    } else if closing < interior {
        closing
    } else {
        closing - TAU
    };
    if sweep.abs() <= angular || (sweep.abs() - TAU).abs() <= angular {
        return None;
    }
    Some(Curve::arc(plane, radius, Interval::new(0.0, sweep)))
}

/// Returns the circle through three points as `(center, normal, radius)`.
fn circle_through(
    a: Point3,
    b: Point3,
    c: Point3,
    linear: f64,
) -> Option<(Point3, UnitVector3<f64>, f64)> {
    let u = b - a;
    let v = c - a;
    let normal: Vector3<f64> = u.cross(&v);
    let scale = normal.norm_squared();
    if scale <= linear * linear {
        return None;
    }
    let center = a
        + (v.cross(&normal) * u.norm_squared() + normal.cross(&u) * v.norm_squared())
            / (2.0 * scale);
    let radius = (a - center).norm();
    (radius.is_finite() && radius > linear)
        .then(|| (center, UnitVector3::new_normalize(normal), radius))
}

/// Reports whether a point lies on the circle within `linear`.
fn on_circle(
    center: Point3,
    normal: &UnitVector3<f64>,
    radius: f64,
    point: Point3,
    linear: f64,
) -> bool {
    let radial = point - center;
    let axial = radial.dot(normal);
    axial.abs() <= linear
        && ((radial - normal.into_inner() * axial).norm() - radius).abs() <= linear
}

/// Returns the same curve traversed the other way.
///
/// [`Curve`] has no in-place reversal, so this goes through the exact NURBS
/// form. Used when a face traverses a fused edge against its stored direction.
pub fn reversed(curve: &Curve) -> Option<Curve> {
    curve
        .to_nurbs()
        .ok()
        .map(|nurbs| Curve::Nurbs(nurbs.reversed()))
}
