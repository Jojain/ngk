//! Rebuilding the parameter curve of a fused boundary.
//!
//! A fused edge invalidates the parameter curves of the pieces it replaced, so
//! every face that carries it needs a new one. Planar faces get an exact answer
//! from the same projection the profile builders use. Other surfaces get a
//! fitted line or arc in parameter space, accepted only when lifting it back to
//! three dimensions traces the fused edge within tolerance — a check in model
//! units, which keeps it meaningful on a surface whose parameters are angles.

use crate::builders::profiles::curve_pcurve;
use crate::geometry::{
    Circle2, Curve, Curve2, Interval, Point2, Point3, Surface, SurfacePeriodicity, TrimmedCurve2,
    Vector2,
};

use super::curve::{SUPPORT_SAMPLES, sample_between};

/// Builds the parameter curve of a boundary running along `curve` from `start`
/// to `end` on `surface`.
pub fn boundary_pcurve(
    surface: &Surface,
    curve: &Curve,
    start: Point3,
    end: Point3,
    linear: f64,
) -> Option<TrimmedCurve2> {
    if let Surface::Plane(plane) = surface {
        return curve_pcurve(curve, start, end, plane).ok();
    }

    let samples = sample_between(curve, start, end, SUPPORT_SAMPLES);
    let mut parameters = samples
        .iter()
        .map(|&point| surface.param_at(point).ok())
        .collect::<Option<Vec<_>>>()?;
    // `param_at` answers inside the surface's own domain, so a boundary running
    // across a closed direction comes back folded — a sawtooth that neither a
    // segment nor an arc can trace. Unwrapping puts the samples back on one
    // branch, the way the intersection tracer does to its states before handing
    // them to its own fitter, so that a span one whole period long is
    // expressible here at all.
    unwrap_parameters(surface.periodicity(), &mut parameters);
    let last = parameters.len() - 1;

    let straight = TrimmedCurve2::segment(parameters[0], parameters[last]);
    if traces(surface, &straight, &samples, linear) {
        return Some(straight);
    }
    let curved = arc2_through(
        parameters[0],
        parameters[last / 2],
        parameters[last],
        linear,
    )?;
    traces(surface, &curved, &samples, linear).then_some(curved)
}

/// Shifts every parameter onto the branch of the one before it.
///
/// Nothing here is normalized back afterwards: a pcurve is stated in the
/// surface's *native* parameters and a span one whole period long is exactly
/// what a wrapping loop needs, so the unwrapped run is the answer rather than a
/// working form of it.
fn unwrap_parameters(periodicity: SurfacePeriodicity, parameters: &mut [Point2]) {
    let periods = match periodicity {
        SurfacePeriodicity::None => return,
        SurfacePeriodicity::UPeriodic(u) => [Some(u), None],
        SurfacePeriodicity::VPeriodic(v) => [None, Some(v)],
        SurfacePeriodicity::UVPeriodic(u, v) => [Some(u), Some(v)],
    };
    for (axis, period) in periods.into_iter().enumerate() {
        let Some(period) = period else {
            continue;
        };
        for current in 1..parameters.len() {
            let previous = parameters[current - 1][axis];
            let value = &mut parameters[current][axis];
            while *value + period * 0.5 < previous {
                *value += period;
            }
            while *value - period * 0.5 > previous {
                *value -= period;
            }
        }
    }
}

/// Reports whether lifting `candidate` onto `surface` traces `samples`.
///
/// The comparison is a two-sided polyline distance in model units: every lifted
/// point must sit on the sampled edge and every sampled point must be reached
/// by the lifted curve.
///
/// This is a weak test, and on a curved support it is weak enough that nothing
/// passes it: every lifted point sits off the chords joining the samples by a
/// sagitta that is about how many samples were taken, not about whether the fit
/// is right. Comparing the two at matching fractions instead — which is what
/// the synchronized-halves rule would ask for — is correct and does let a rim
/// on a cylinder be rebuilt, but the fused rims that then reach the Boolean's
/// trim domain flatten to degenerate polygons and its ray classification stops
/// being able to place a point. So a fused boundary on a curved support is
/// reported as `PcurveNotJoinable` rather than rebuilt wrongly.
fn traces(surface: &Surface, candidate: &TrimmedCurve2, samples: &[Point3], linear: f64) -> bool {
    let lifted = candidate
        .sample(4 * SUPPORT_SAMPLES)
        .into_iter()
        .map(|uv| surface.point_at(uv.x, uv.y))
        .collect::<Vec<_>>();
    polylines_agree(&lifted, samples, linear) && polylines_agree(samples, &lifted, linear)
}

/// Reports whether every point of `points` lies within `tolerance` of the
/// polyline through `polyline`.
fn polylines_agree(points: &[Point3], polyline: &[Point3], tolerance: f64) -> bool {
    points.iter().all(|&point| {
        polyline
            .windows(2)
            .map(|segment| segment_distance(point, segment[0], segment[1]))
            .fold(f64::INFINITY, f64::min)
            <= tolerance
    })
}

/// Returns the distance from a point to a segment.
fn segment_distance(point: Point3, start: Point3, end: Point3) -> f64 {
    let direction = end - start;
    let length_squared = direction.norm_squared();
    if length_squared <= f64::EPSILON {
        return (point - start).norm();
    }
    let t = ((point - start).dot(&direction) / length_squared).clamp(0.0, 1.0);
    (point - (start + direction * t)).norm()
}

/// Returns the parameter-space arc through three points, sweeping from the
/// first through the second to the third.
///
/// The support is the full circle those three points determine, and the span
/// states which way round it runs: the two arcs joining the first and third
/// points share both endpoints, so only the sweep names the one through the
/// second.
fn arc2_through(
    first: Point2,
    second: Point2,
    third: Point2,
    linear: f64,
) -> Option<TrimmedCurve2> {
    let determinant = 2.0
        * (first.x * (second.y - third.y)
            + second.x * (third.y - first.y)
            + third.x * (first.y - second.y));
    if determinant.abs() <= linear {
        return None;
    }
    let square = |point: Point2| point.x * point.x + point.y * point.y;
    let center = Point2::new(
        (square(first) * (second.y - third.y)
            + square(second) * (third.y - first.y)
            + square(third) * (first.y - second.y))
            / determinant,
        (square(first) * (third.x - second.x)
            + square(second) * (first.x - third.x)
            + square(third) * (second.x - first.x))
            / determinant,
    );

    let x_dir = first - center;
    let radius = x_dir.norm();
    if radius <= linear {
        return None;
    }
    let y_dir = Vector2::new(-x_dir.y, x_dir.x);
    let angle = |point: Point2| {
        let radial = point - center;
        radial.dot(&y_dir).atan2(radial.dot(&x_dir))
    };

    let interior = angle(second);
    if interior == 0.0 {
        return None;
    }
    let closing = angle(third);
    let sweep = if interior > 0.0 {
        if closing > interior {
            closing
        } else {
            closing + std::f64::consts::TAU
        }
    } else if closing < interior {
        closing
    } else {
        closing - std::f64::consts::TAU
    };
    Some(TrimmedCurve2::new(
        Curve2::Circle(Circle2::new(center, x_dir, radius)),
        Interval::new(0.0, sweep),
    ))
}
