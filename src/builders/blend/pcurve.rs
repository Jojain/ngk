//! Pcurves of the curves a blend builds.

use super::errors::BlendError;
use crate::builders::profiles::curve_pcurve;
use crate::geometry::{
    IntersectionError, IntersectionOptions, Surface, TrimmedCurve, TrimmedCurve2, pcurve_on_surface,
};

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
