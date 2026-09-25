//! Pcurves of the curves a blend builds.

use super::errors::BlendError;
use crate::builders::profiles::curve_pcurve;
use crate::geometry::parameter::Fraction;
use crate::geometry::{
    Curve2, IntersectionError, IntersectionOptions, LINEAR_TOLERANCE, NurbsCurve2, Point2, Surface,
    SurfacePeriodicity, TrimmedCurve, TrimmedCurve2, pcurve_on_surface,
};

/// Samples a closed rail is traced with, and the fit is checked between.
const CLOSED_SAMPLES: usize = 128;

/// Writes the pcurve of a curve lying on `surface`, over the curve's span and
/// in its direction.
///
/// A plane's is exact: its parameters are Cartesian coordinates in it. On any
/// other support the curve is traced through the analytic section machinery,
/// which is exact on an isoline and a measured fit otherwise; a fit straying
/// further than the fitting tolerance is refused rather than written.
pub(crate) fn pcurve_on(
    surface: &Surface,
    curve: &TrimmedCurve,
) -> Result<TrimmedCurve2, BlendError> {
    if let Surface::Plane(plane) = surface {
        return curve_pcurve(curve, plane)
            .map_err(|error| BlendError::Pcurve(IntersectionError::from(error)));
    }
    let options = IntersectionOptions::default();
    let (pcurve, fidelity) = pcurve_on_surface(curve, surface, options)?;
    let deviation = fidelity.deviation();
    if deviation > options.fit_tolerance {
        return Err(BlendError::PcurveDeviation { deviation });
    }
    Ok(pcurve)
}

/// Writes the pcurve of a whole closed rail lying on `surface`, in the rail's
/// direction, starting on the branch nearest `reference`.
///
/// On a plane the rail closes in the parameters too, and its pcurve is exact.
/// Round a periodic direction it does not: a cylinder wall's rim is a line
/// one period long. The rail is followed sample by sample, each lifted onto
/// the branch of the one before, so it comes out as that line rather than as
/// pieces cut at the seam. A line through every sample is written as one
/// segment; anything else is interpolated and refused if it strays between
/// the samples. `reference` is where the face's old rim began, so the new one
/// starts on the same branch and the face's loops keep bounding one band.
pub(crate) fn closed_pcurve_on(
    surface: &Surface,
    rail: &TrimmedCurve,
    reference: Option<Point2>,
) -> Result<TrimmedCurve2, BlendError> {
    if let Surface::Plane(plane) = surface {
        return curve_pcurve(rail, plane)
            .map_err(|error| BlendError::Pcurve(IntersectionError::from(error)));
    }
    let fractions = (0..=CLOSED_SAMPLES)
        .map(|index| index as f64 / CLOSED_SAMPLES as f64)
        .collect::<Vec<_>>();
    let mut samples: Vec<Point2> = Vec::with_capacity(fractions.len());
    for &fraction in &fractions {
        let point = rail.point_at(Fraction::new(fraction));
        let uv = match samples.last() {
            Some(&previous) => {
                let uv = surface
                    .param_near(point, previous, LINEAR_TOLERANCE)
                    .map_err(|error| BlendError::Pcurve(IntersectionError::from(error)))?;
                onto_branch(surface, uv, previous)
            }
            None => {
                let uv = surface
                    .param_at(point)
                    .map_err(|error| BlendError::Pcurve(IntersectionError::from(error)))?;
                reference.map_or(uv, |reference| onto_branch(surface, uv, reference))
            }
        };
        samples.push(uv);
    }

    let (first, last) = (samples[0], samples[CLOSED_SAMPLES]);
    let chord = last - first;
    let straight = chord.norm() > LINEAR_TOLERANCE
        && samples.iter().all(|&sample| {
            let offset = sample - first;
            (offset.x * chord.y - offset.y * chord.x).abs() / chord.norm() <= LINEAR_TOLERANCE
        });
    if straight {
        return Ok(TrimmedCurve2::segment(first, last));
    }

    // Every other sample is fitted; the ones between are what the fit is
    // checked against.
    let fitted = samples.iter().copied().step_by(2).collect::<Vec<_>>();
    let parameters = fractions.iter().copied().step_by(2).collect::<Vec<_>>();
    let curve = NurbsCurve2::interpolate_with_parameters(&fitted, &parameters)
        .map_err(|error| BlendError::Pcurve(IntersectionError::from(error)))?;
    let pcurve = TrimmedCurve2::whole(Curve2::Nurbs(curve));
    let deviation = fractions
        .iter()
        .skip(1)
        .step_by(2)
        .map(|&fraction| {
            let uv = pcurve.point_at(Fraction::new(fraction));
            (surface.point_at(uv.x, uv.y) - rail.point_at(Fraction::new(fraction))).norm()
        })
        .fold(0.0, f64::max);
    if deviation > IntersectionOptions::default().fit_tolerance {
        return Err(BlendError::PcurveDeviation { deviation });
    }
    Ok(pcurve)
}

/// Shifts `uv` by whole periods to the branch nearest `reference`.
pub(crate) fn onto_branch(surface: &Surface, uv: Point2, reference: Point2) -> Point2 {
    let nearest =
        |value: f64, target: f64, period: f64| value + ((target - value) / period).round() * period;
    match surface.periodicity() {
        SurfacePeriodicity::None => uv,
        SurfacePeriodicity::UPeriodic(period) => {
            Point2::new(nearest(uv.x, reference.x, period), uv.y)
        }
        SurfacePeriodicity::VPeriodic(period) => {
            Point2::new(uv.x, nearest(uv.y, reference.y, period))
        }
        SurfacePeriodicity::UVPeriodic(u_period, v_period) => Point2::new(
            nearest(uv.x, reference.x, u_period),
            nearest(uv.y, reference.y, v_period),
        ),
    }
}
