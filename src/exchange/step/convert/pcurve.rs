//! Rebuilding a parameter curve from the 3D curve that lies on the support.
//!
//! STEP does not require a file to carry parameter curves: `SURFACE_CURVE`'s
//! associated geometry is optional, and a writer that thinks its 3D curves say
//! everything omits them. `FaceAttr` requires one per boundary dart, so the
//! importer has to rebuild what the file left out.
//!
//! On a plane that is a *projection*: an isometry onto the plane's own
//! coordinates, exact for every support and leaving the parameterization where
//! it was. Everywhere else it is a *lift*: invert the curve onto the support
//! and find the image it traces through the support's parameters. Lifting is
//! one-directional and every `Surface` can invert a point, so it needs no
//! intersection machinery — what it needs is care about the two things
//! inversion gets wrong on its own.
//!
//! **A periodic parameter folds.** Inversion answers within one period, so a
//! curve crossing the fold comes back as a jump of a whole period rather than
//! as a continuous walk. Both directions can fold — a torus folds in both at
//! once — so the unwrapping runs per axis rather than on longitude alone.
//!
//! **A collapsed row loses a parameter.** Every longitude names a sphere's
//! pole, so inversion there returns an arbitrary one and a curve arriving at a
//! pole would swing sideways across the whole domain in its last step. A
//! sample on a collapsed row therefore keeps the parameter that pins the row
//! and takes the other from its neighbour, which is the direction the curve was
//! actually travelling when it got there.

use crate::builders::profiles::curve_pcurve;
use crate::geometry::parameter::Fraction;
use crate::geometry::{
    ANGULAR_TOLERANCE, Axis2, Curve, Curve2, Interval, LINEAR_TOLERANCE, NurbsCurve2, NurbsError,
    Point2, PointCoincidence, Surface, TrimmedCurve, TrimmedCurve2,
};

/// How many points a lift inverts before it decides what it is looking at.
///
/// Enough to tell a parameter line from a curve that merely starts and ends on
/// one, and to seed an interpolation that is then measured and refined.
const SAMPLES: usize = 16;

/// How many times a fitted lift is resampled while it is still too far out.
const REFINEMENTS: usize = 4;

/// A parameter curve rebuilt on a support, and how faithful it is.
///
/// The two travel together because a caller that reports its imports has to
/// say which boundaries are exact and which were approximated, and a fidelity
/// that arrived separately could be attached to the wrong curve.
#[derive(Debug, Clone)]
pub struct LiftedPcurve {
    /// The image of the curve in the support's parameter space.
    pub pcurve: TrimmedCurve2,
    /// How far the image strays from the curve, when it was fitted.
    ///
    /// `None` where the image is a closed form — a projection onto a plane, or
    /// a parameter line verified against the curve — and those are exact
    /// rather than merely close, so there is no residual to state.
    pub residual: Option<f64>,
}

/// Lifts the section of `curve` over `interval` onto `surface`.
///
/// The result runs in the same direction as `interval`, so a boundary walked
/// backwards lifts to a parameter curve walked backwards, which is what makes
/// the face's winding come out of the file rather than being reasoned about.
pub fn lift_pcurve(
    surface: &Surface,
    curve: &Curve,
    interval: Interval,
    tolerance: f64,
) -> Result<LiftedPcurve, NurbsError> {
    if let Surface::Plane(plane) = surface {
        return Ok(LiftedPcurve {
            pcurve: curve_pcurve(&TrimmedCurve::new(curve.clone(), interval), plane)?,
            residual: None,
        });
    }

    let section = TrimmedCurve::new(curve.clone(), interval);
    let fractions: Vec<f64> = (0..=SAMPLES)
        .map(|index| index as f64 / SAMPLES as f64)
        .collect();
    let image = invert(surface, &section, &fractions)?;

    if let Some(pcurve) = parameter_line(surface, &section, &image, tolerance) {
        return Ok(LiftedPcurve {
            pcurve,
            residual: None,
        });
    }
    fit(surface, &section, &image, tolerance)
}

/// Inverts a section onto a support at the given fractions of its span.
fn invert(
    surface: &Surface,
    section: &TrimmedCurve,
    fractions: &[f64],
) -> Result<Vec<Point2>, NurbsError> {
    let mut image = Vec::with_capacity(fractions.len());
    for &fraction in fractions {
        image.push(surface.param_at(section.point_at(Fraction::new(fraction)))?);
    }
    resolve_collapsed_rows(surface, &mut image);
    unwrap_periods(surface, &mut image);
    resolve_closing_edges(surface, &mut image);
    Ok(image)
}

/// Puts every sample that sits where the support closes on one side of it.
///
/// A support can close on itself without a period: a swept or lofted spline
/// whose first and last columns are one row of points. Inversion there returns
/// either column, arbitrarily, so a whole-turn rim can come back as a walk out
/// and back, and a cut running along the closing column as a zig-zag between
/// the two. Moving a sample to the other column is free, because the two are
/// one point of the surface, so each goes to the side its nearest sample off
/// the column is on -- and a curve lying wholly along the column stays on the
/// side it started on, which the loop it belongs to may later move across.
fn resolve_closing_edges(surface: &Surface, image: &mut [Point2]) {
    let (u, v) = surface.domain();
    for (axis, span) in [(Axis2::U, u), (Axis2::V, v)] {
        if periods(surface)[axis.index()].is_some() || !span.is_finite() {
            continue;
        }
        let (low, high) = (span.ordered().start.value(), span.ordered().end.value());
        let across = |point: Point2, value: f64| {
            let mut moved = point;
            moved[axis.index()] = value;
            moved
        };
        // The other side of the closing column, for a sample that sits on it.
        let other_side = |point: Point2| {
            let at = point[axis.index()];
            let other = if (at - low).abs() <= ANGULAR_TOLERANCE {
                high
            } else if (at - high).abs() <= ANGULAR_TOLERANCE {
                low
            } else {
                return None;
            };
            let moved = across(point, other);
            surface
                .point_at(point.x, point.y)
                .coincides(surface.point_at(moved.x, moved.y), LINEAR_TOLERANCE)
                .then_some(other)
        };

        let on_column: Vec<bool> = image
            .iter()
            .map(|point| other_side(*point).is_some())
            .collect();
        let Some(first) = image.first().map(|point| point[axis.index()]) else {
            continue;
        };
        for index in 0..image.len() {
            let Some(other) = other_side(image[index]) else {
                continue;
            };
            let heading = (1..image.len())
                .flat_map(|step| [index.checked_sub(step), index.checked_add(step)])
                .flatten()
                .find(|near| *near < image.len() && !on_column[*near])
                .map_or(first, |near| image[near][axis.index()]);
            let at = image[index][axis.index()];
            if (heading - other).abs() < (heading - at).abs() {
                image[index] = across(image[index], other);
            }
        }
    }
}

/// Replaces the parameter a collapsed row does not determine.
///
/// A row that collapses to a point carries every value of the parameter
/// running along it, so inversion there returns whichever one its arithmetic
/// happened to produce. The neighbouring sample knows which one the curve was
/// heading for.
fn resolve_collapsed_rows(surface: &Surface, image: &mut [Point2]) {
    for index in 0..image.len() {
        let point = image[index];
        if !surface.is_degenerate_at(point.x, point.y) {
            continue;
        }
        let Some(along) = collapsed_axis(surface, point) else {
            continue;
        };
        let Some(neighbour) = nearest_defined(surface, image, index) else {
            continue;
        };
        image[index][along.index()] = neighbour[along.index()];
    }
}

/// Which parameter runs along the collapsed row through a point, if any.
fn collapsed_axis(surface: &Surface, point: Point2) -> Option<Axis2> {
    // A row listed along one axis is a value that axis holds fixed, so what
    // varies freely along it is the *other* one.
    for axis in [Axis2::U, Axis2::V] {
        let held = point[axis.index()];
        if surface
            .degenerate_rows(axis)
            .iter()
            .any(|row| (row - held).abs() <= crate::geometry::ANGULAR_TOLERANCE)
        {
            return Some(axis.transverse());
        }
    }
    None
}

/// The nearest sample the support does not collapse at.
fn nearest_defined(surface: &Surface, image: &[Point2], from: usize) -> Option<Point2> {
    (1..image.len()).find_map(|step| {
        [from.checked_sub(step), from.checked_add(step)]
            .into_iter()
            .flatten()
            .filter_map(|index| image.get(index).copied())
            .find(|point| !surface.is_degenerate_at(point.x, point.y))
    })
}

/// Lifts each periodic parameter onto one continuous branch.
///
/// Inversion folds a periodic parameter into one period, so a curve crossing
/// the fold comes back with a jump of very nearly a whole period in it. Undoing
/// that per sample, per axis, is what turns the samples back into a walk.
fn unwrap_periods(surface: &Surface, image: &mut [Point2]) {
    for (axis, period) in periods(surface).into_iter().enumerate() {
        let Some(period) = period else {
            continue;
        };
        for index in 1..image.len() {
            let previous = image[index - 1][axis];
            let jump = image[index][axis] - previous;
            image[index][axis] -= (jump / period).round() * period;
        }
    }
}

/// The support's periods, in parameter order.
fn periods(surface: &Surface) -> [Option<f64>; 2] {
    use crate::geometry::SurfacePeriodicity::{
        None as Aperiodic, UPeriodic, UVPeriodic, VPeriodic,
    };
    match surface.periodicity() {
        Aperiodic => [None, None],
        UPeriodic(u) => [Some(u), None],
        VPeriodic(v) => [None, Some(v)],
        UVPeriodic(u, v) => [Some(u), Some(v)],
    }
}

/// Returns the straight parameter-space segment the samples lie on, if they do.
///
/// The candidate is proposed from the samples and then *verified* against the
/// section itself, in model units — so a segment that happens to join the two
/// ends of a curve that bulges away from it is rejected rather than silently
/// returned. Most boundaries on an analytic support are parameter lines, which
/// is why this is tried before anything is fitted.
fn parameter_line(
    surface: &Surface,
    section: &TrimmedCurve,
    image: &[Point2],
    tolerance: f64,
) -> Option<TrimmedCurve2> {
    let (&first, &last) = (image.first()?, image.last()?);
    let candidate = TrimmedCurve2::segment(first, last);
    (deviation(surface, section, &candidate) <= tolerance).then_some(candidate)
}

/// Interpolates the samples, refining until the fit is within tolerance.
///
/// The error is measured by putting the parameter curve back on the surface
/// and comparing against the section in model units, so it means the same
/// thing as any other linear tolerance rather than being a parameter-space
/// residual whose size depends on the support.
fn fit(
    surface: &Surface,
    section: &TrimmedCurve,
    image: &[Point2],
    tolerance: f64,
) -> Result<LiftedPcurve, NurbsError> {
    let mut samples = image.len() - 1;
    let mut best: Option<(TrimmedCurve2, f64)> = None;
    for _ in 0..REFINEMENTS {
        let fractions: Vec<f64> = (0..=samples)
            .map(|index| index as f64 / samples as f64)
            .collect();
        let points = invert(surface, section, &fractions)?;
        let fitted = NurbsCurve2::interpolate_with_parameters(&points, &fractions)?;
        let span = fitted.domain();
        let candidate = TrimmedCurve2::new(Curve2::Nurbs(fitted), span);

        let error = deviation(surface, section, &candidate);
        if best.as_ref().is_none_or(|(_, best)| error < *best) {
            best = Some((candidate, error));
        }
        if error <= tolerance {
            break;
        }
        samples *= 2;
    }

    let (pcurve, residual) = best.expect("at least one fit is attempted");
    Ok(LiftedPcurve {
        pcurve,
        residual: Some(residual),
    })
}

/// The furthest a parameter curve's image strays from the section.
fn deviation(surface: &Surface, section: &TrimmedCurve, pcurve: &TrimmedCurve2) -> f64 {
    /// Measured more finely than the lift was sampled, so a candidate cannot
    /// be verified only at the points that proposed it.
    const CHECKS: usize = 4 * SAMPLES;

    (0..=CHECKS)
        .map(|index| {
            let fraction = index as f64 / CHECKS as f64;
            let uv = pcurve.point_at(Fraction::new(fraction));
            (surface.point_at(uv.x, uv.y) - section.point_at(Fraction::new(fraction))).norm()
        })
        .fold(0.0_f64, |worst, distance| {
            if distance.is_finite() {
                worst.max(distance)
            } else {
                f64::INFINITY
            }
        })
}
