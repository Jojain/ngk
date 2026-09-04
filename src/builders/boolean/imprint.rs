//! Derivation of topology-subdivision inputs from the intersection network.

use std::collections::HashMap;

use crate::builders::faces::FaceImprint;
use crate::geometry::{Curve, Point3};
use crate::topology::shape_keys::{EdgeKey, FaceKey};

use super::{BooleanCell, IntersectionNetwork, IntersectionOrientation, IntersectionSpanUse};

/// Collects edge split points from canonical event incidences.
pub(crate) fn edge_points(network: &IntersectionNetwork) -> HashMap<EdgeKey, Vec<Point3>> {
    let mut points = HashMap::<EdgeKey, Vec<Point3>>::new();
    for event in network.events() {
        for event_use in &event.uses {
            if let BooleanCell::Edge(edge) = event_use.cell {
                points.entry(edge).or_default().push(event.point);
            }
        }
    }
    points
}

/// Reconstructs face-local imprints from the shared three-dimensional spans.
pub(crate) fn face_imprints(network: &IntersectionNetwork) -> HashMap<FaceKey, Vec<FaceImprint>> {
    let mut imprints = HashMap::<FaceKey, Vec<FaceImprint>>::new();
    for span in network.spans() {
        for span_use in &span.uses {
            let IntersectionSpanUse::Face {
                face,
                pcurve,
                orientation,
                ..
            } = span_use
            else {
                continue;
            };
            let (curve, pcurve) = match orientation {
                IntersectionOrientation::Forward => ((*span.curve).clone(), (**pcurve).clone()),
                IntersectionOrientation::Reversed => (
                    Curve::Nurbs(
                        span.curve
                            .to_nurbs()
                            .expect("validated intersection curve")
                            .reversed(),
                    ),
                    pcurve.reversed(),
                ),
            };
            imprints
                .entry(*face)
                .or_default()
                .push(FaceImprint::new(curve, pcurve));
        }
    }
    imprints
}
