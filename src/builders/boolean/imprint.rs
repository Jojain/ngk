//! Derivation of topology-subdivision inputs from the intersection network.

use std::collections::HashMap;

use super::graph::SpanSubdivision;
use crate::builders::faces::{FaceImprint, FaceImprintSection, split_face_edge_staged};
use crate::geometry::{Curve, Point3, PointCoincidence};
use crate::topology::shape_keys::{EdgeKey, FaceKey};
use crate::topology::{TopologyEdit, gmap::GMap, payload::Payload};

use super::{
    BooleanCell, BooleanError, BooleanSide, IntersectionNetwork, IntersectionOrientation,
    IntersectionSpanId, IntersectionSpanUse,
};

/// A face imprint retaining its canonical network identity.
#[derive(Clone)]
pub(crate) struct SpanImprint {
    pub(crate) span: IntersectionSpanId,
    pub(crate) pieces: Vec<SpanSubdivision>,
    pub(crate) side: BooleanSide,
    pub(crate) imprint: FaceImprint,
    pub(crate) orientation: IntersectionOrientation,
}

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
pub(crate) fn face_imprints(network: &IntersectionNetwork) -> HashMap<FaceKey, Vec<SpanImprint>> {
    let mut imprints = HashMap::<FaceKey, Vec<SpanImprint>>::new();
    for (index, span) in network.spans().iter().enumerate() {
        for span_use in &span.uses {
            let IntersectionSpanUse::Face {
                face,
                side,
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
            imprints.entry(*face).or_default().push(SpanImprint {
                span: IntersectionSpanId(index),
                pieces: Vec::new(),
                side: *side,
                imprint: FaceImprint::new(curve, pcurve),
                orientation: *orientation,
            });
        }
    }
    imprints
}

/// Splits a known section at canonical span boundaries, retaining its exact parent map.
pub(crate) fn realize_section<P: Payload>(
    edit: &mut TopologyEdit<'_, P>,
    imprint: &SpanImprint,
    section: &FaceImprintSection,
    tolerance: f64,
) -> Result<Vec<(IntersectionSpanId, f64, EdgeKey)>, BooleanError> {
    let start = section.interval.start;
    let end = section.interval.end;
    let mut cuts = vec![start, end];
    for piece in &imprint.pieces {
        for t in [piece.interval.start, piece.interval.end] {
            if t > start.min(end) + tolerance && t < start.max(end) - tolerance {
                cuts.push(t);
            }
        }
    }
    cuts.sort_by(f64::total_cmp);
    cuts.dedup_by(|a, b| (*a - *b).abs() <= tolerance);
    if start > end {
        cuts.reverse();
    }
    let mut edge = section.edge;
    let mut output = Vec::new();
    for (index, pair) in cuts.windows(2).enumerate() {
        let current = edge;
        if index + 2 < cuts.len() {
            let point = imprint.imprint.curve.point_at(pair[1]);
            let view = edit.edge_unchecked(edge);
            let parameter = view.curve().expect("section geometry").param_at(point);
            let face = view.faces()[0].key();
            edge = split_face_edge_staged(edit, face, edge, parameter)?.second;
        }
        let middle = (pair[0] + pair[1]) * 0.5;
        let piece = imprint
            .pieces
            .iter()
            .find(|piece| piece.interval.ordered().contains(middle, tolerance))
            .ok_or(BooleanError::UnrealizedSpan { span: imprint.span })?;
        let mut a = (pair[0] - piece.interval.start) / (piece.interval.end - piece.interval.start);
        let mut b = (pair[1] - piece.interval.start) / (piece.interval.end - piece.interval.start);
        if piece.reversed {
            a = 1.0 - a;
            b = 1.0 - b;
        }
        output.push((piece.span, a.min(b), current));
    }
    Ok(output)
}

/// Records the existing edge fragment realizing each edge-borne canonical span.
///
/// Contacts running along an operand's own boundary are never imprinted, so the
/// only realization available is the fragment the edge split pass produced.
/// Spans with no matching fragment are left unrealized; assembly rejects the
/// operation when a surviving span is then missing a side.
pub(crate) fn realize_edge_spans<P: Payload>(
    map: &GMap<P>,
    network: &IntersectionNetwork,
    lineage: &HashMap<EdgeKey, Vec<EdgeKey>>,
    tolerance: f64,
) -> Vec<(IntersectionSpanId, BooleanSide, EdgeKey)> {
    let mut realized = Vec::new();
    for (index, span) in network.spans().iter().enumerate() {
        let start = span.curve.point_at(0.0);
        let end = span.curve.point_at(1.0);
        for span_use in &span.uses {
            let IntersectionSpanUse::Edge { side, edge, .. } = span_use else {
                continue;
            };
            let Some(fragments) = lineage.get(edge) else {
                continue;
            };
            let fragment = fragments.iter().copied().find(|fragment| {
                let view = map.edge_unchecked(*fragment);
                let (start_vertex, end_vertex) = (view.start(), view.end());
                let (Some(a), Some(b)) = (start_vertex.point(), end_vertex.point()) else {
                    return false;
                };
                let (a, b) = (*a, *b);
                (a.coincides(start, tolerance) && b.coincides(end, tolerance))
                    || (a.coincides(end, tolerance) && b.coincides(start, tolerance))
            });
            if let Some(fragment) = fragment {
                realized.push((IntersectionSpanId(index), *side, fragment));
            }
        }
    }
    realized
}
