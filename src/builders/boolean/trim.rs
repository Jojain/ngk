//! Trim queries the Boolean asks of its faces.
//!
//! The trim domain itself is a topology query, shared with the face splitter,
//! which builds below the Boolean; it is re-exported here so the Boolean's
//! modules reach it beside the queries that are the Boolean's own.

use crate::geometry::TrimmedCurve2;
use crate::geometry::parameter::Fraction;
use crate::topology::face::Face;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::EdgeKey;

pub(crate) use crate::topology::trim_domain::{FaceTrimDomain, TrimLocation};

/// Locates the boundary edge of `face` that already realizes `pcurve`.
///
/// A contact section running along a face's own trim loop must never be
/// imprinted: the edge already carries that geometry, and splitting the face
/// along its own loop yields a degenerate fragment with no interior probe.
/// Such a section is realized on the existing edge instead.
pub(crate) fn boundary_edge_for<P: Payload>(
    face: &Face<'_, P>,
    pcurve: &TrimmedCurve2,
    tolerance: f64,
) -> Option<EdgeKey> {
    let samples = [0.0, 0.25, 0.5, 0.75, 1.0].map(|t| pcurve.point_at(Fraction::new(t)));
    face.edges()
        .into_iter()
        .find(|edge| {
            face.pcurve(edge.dart()).is_some_and(|boundary| {
                samples
                    .iter()
                    .all(|point| boundary.try_parameter_at(*point, tolerance).is_some())
            })
        })
        .map(|edge| edge.key())
}
