//! Canonical intersection network shared by both Boolean operands.

use crate::geometry::{
    Curve, Curve2, Interval, KnotVector, NurbsCurve, NurbsError, Point2, Point3, PointCoincidence,
};
use crate::topology::shape_keys::{EdgeKey, FaceKey};
use thiserror::Error;

use super::{BooleanCell, BooleanError, BooleanSide, PointContactKind};

/// Stable index of a remarkable point in an intersection network.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct IntersectionEventId(pub(crate) usize);

/// Stable index of a curve section in an intersection network.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct IntersectionSpanId(pub(crate) usize);

/// Position of an intersection event inside one source cell.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum IntersectionEventLocation {
    Vertex,
    Edge { parameter: f64 },
    Face { uv: Point2 },
}

/// One operand-local interpretation of a canonical intersection event.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IntersectionEventUse {
    pub side: BooleanSide,
    pub cell: BooleanCell,
    pub location: IntersectionEventLocation,
}

/// A canonical point where an intersection section starts, ends, or changes context.
#[derive(Debug, Clone)]
pub struct IntersectionEvent {
    pub point: Point3,
    pub kind: PointContactKind,
    pub uses: Vec<IntersectionEventUse>,
}

/// Orientation of an intersection section in one local parameter space.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IntersectionOrientation {
    Forward,
    Reversed,
}

/// One operand-local representation of an intersection section.
#[derive(Clone)]
pub enum IntersectionSpanUse {
    Edge {
        side: BooleanSide,
        edge: EdgeKey,
        interval: Interval,
    },
    Face {
        side: BooleanSide,
        face: FaceKey,
        pcurve: Box<Curve2>,
        orientation: IntersectionOrientation,
    },
}

/// Nature of a one-dimensional intersection section.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IntersectionSpanKind {
    Transverse,
    Overlap,
}

/// An indivisible curve section between two canonical events.
#[derive(Clone)]
pub struct IntersectionSpan {
    pub start: IntersectionEventId,
    pub end: IntersectionEventId,
    pub curve: Box<Curve>,
    pub kind: IntersectionSpanKind,
    pub uses: Vec<IntersectionSpanUse>,
}

/// A two-dimensional overlap retained alongside the one-dimensional network.
#[derive(Debug, Clone)]
pub struct IntersectionRegion {
    pub first_face: FaceKey,
    pub second_face: FaceKey,
    pub boundary: Vec<IntersectionSpanId>,
}

/// Canonical source of truth for all contacts between two Boolean operands.
#[derive(Clone, Default)]
pub struct IntersectionNetwork {
    events: Vec<IntersectionEvent>,
    spans: Vec<IntersectionSpan>,
    regions: Vec<IntersectionRegion>,
}

/// Structural inconsistency detected before topology mutation.
#[derive(Debug, Error, Clone, PartialEq)]
pub enum IntersectionNetworkValidationError {
    #[error("intersection event {event} has no operand incidence")]
    EventWithoutUse { event: usize },
    #[error("intersection span {span} references a missing endpoint")]
    MissingSpanEndpoint { span: usize },
    #[error("intersection span {span} has no operand-local representation")]
    SpanWithoutUse { span: usize },
    #[error("intersection span {span} does not meet its canonical endpoint")]
    SpanEndpointMismatch { span: usize },
}

impl IntersectionNetwork {
    /// Returns every canonical remarkable point.
    pub fn events(&self) -> &[IntersectionEvent] {
        &self.events
    }

    /// Returns every connected curve section.
    pub fn spans(&self) -> &[IntersectionSpan] {
        &self.spans
    }

    /// Returns every two-dimensional overlap.
    pub fn regions(&self) -> &[IntersectionRegion] {
        &self.regions
    }

    /// Resolves an event identifier.
    pub fn event(&self, id: IntersectionEventId) -> Option<&IntersectionEvent> {
        self.events.get(id.0)
    }

    /// Checks connectivity and geometric endpoint consistency.
    pub fn validate(&self, tolerance: f64) -> Result<(), IntersectionNetworkValidationError> {
        for (index, event) in self.events.iter().enumerate() {
            if event.uses.is_empty() {
                return Err(IntersectionNetworkValidationError::EventWithoutUse { event: index });
            }
        }
        for (index, span) in self.spans.iter().enumerate() {
            let Some(start) = self.event(span.start) else {
                return Err(IntersectionNetworkValidationError::MissingSpanEndpoint {
                    span: index,
                });
            };
            let Some(end) = self.event(span.end) else {
                return Err(IntersectionNetworkValidationError::MissingSpanEndpoint {
                    span: index,
                });
            };
            if span.uses.is_empty() {
                return Err(IntersectionNetworkValidationError::SpanWithoutUse { span: index });
            }
            if !span.curve.point_at(0.0).coincides(start.point, tolerance)
                || !span.curve.point_at(1.0).coincides(end.point, tolerance)
            {
                return Err(IntersectionNetworkValidationError::SpanEndpointMismatch {
                    span: index,
                });
            }
        }
        Ok(())
    }
}

/// Collects raw intersection observations into one incidence-aware network.
pub(crate) struct IntersectionNetworkBuilder {
    tolerance: f64,
    network: IntersectionNetwork,
}

impl IntersectionNetworkBuilder {
    pub(crate) fn new(tolerance: f64) -> Self {
        Self {
            tolerance,
            network: IntersectionNetwork::default(),
        }
    }

    pub(crate) fn record_event(
        &mut self,
        point: Point3,
        kind: PointContactKind,
        uses: impl IntoIterator<Item = IntersectionEventUse>,
    ) -> IntersectionEventId {
        let uses = uses.into_iter().collect::<Vec<_>>();
        if let Some((index, event)) =
            self.network
                .events
                .iter_mut()
                .enumerate()
                .find(|(_, event)| {
                    event.point.coincides(point, self.tolerance)
                        && uses_are_compatible(&event.uses, &uses, self.tolerance)
                })
        {
            for event_use in uses {
                if !event
                    .uses
                    .iter()
                    .any(|existing| uses_match(*existing, event_use, self.tolerance))
                {
                    event.uses.push(event_use);
                }
            }
            if kind == PointContactKind::Tangent {
                event.kind = PointContactKind::Tangent;
            }
            return IntersectionEventId(index);
        }

        let id = IntersectionEventId(self.network.events.len());
        self.network
            .events
            .push(IntersectionEvent { point, kind, uses });
        id
    }

    pub(crate) fn record_span(
        &mut self,
        curve: Curve,
        kind: IntersectionSpanKind,
        start_uses: impl IntoIterator<Item = IntersectionEventUse>,
        end_uses: impl IntoIterator<Item = IntersectionEventUse>,
        uses: impl IntoIterator<Item = IntersectionSpanUse>,
    ) -> Option<IntersectionSpanId> {
        let start_point = curve.point_at(0.0);
        let end_point = curve.point_at(1.0);
        if start_point.coincides(end_point, self.tolerance) {
            return None;
        }
        let start = self.record_event(start_point, PointContactKind::Transverse, start_uses);
        let end = self.record_event(end_point, PointContactKind::Transverse, end_uses);
        let incoming_uses = uses.into_iter().collect::<Vec<_>>();
        if let Some((index, span)) = self.network.spans.iter_mut().enumerate().find(|(_, span)| {
            let same_direction = span.start == start && span.end == end;
            let reversed_direction = span.start == end && span.end == start;
            (same_direction || reversed_direction)
                && curves_coincide(&span.curve, &curve, reversed_direction, self.tolerance)
        }) {
            let reversed = span.start == end && span.end == start;
            for span_use in incoming_uses {
                let span_use = align_span_use(span_use, reversed);
                if !span
                    .uses
                    .iter()
                    .any(|existing| same_span_cell(existing, &span_use))
                {
                    span.uses.push(span_use);
                }
            }
            return Some(IntersectionSpanId(index));
        }

        let id = IntersectionSpanId(self.network.spans.len());
        self.network.spans.push(IntersectionSpan {
            start,
            end,
            curve: Box::new(curve),
            kind,
            uses: incoming_uses,
        });
        Some(id)
    }

    pub(crate) fn record_region(
        &mut self,
        first_face: FaceKey,
        second_face: FaceKey,
        boundary: Vec<IntersectionSpanId>,
    ) {
        self.network.regions.push(IntersectionRegion {
            first_face,
            second_face,
            boundary,
        });
    }

    pub(crate) fn finish(self) -> Result<IntersectionNetwork, IntersectionNetworkValidationError> {
        self.network.validate(self.tolerance)?;
        Ok(self.network)
    }
}

fn curves_coincide(left: &Curve, right: &Curve, right_is_reversed: bool, tolerance: f64) -> bool {
    [0.25, 0.5, 0.75].into_iter().all(|parameter| {
        let right_parameter = if right_is_reversed {
            1.0 - parameter
        } else {
            parameter
        };
        left.point_at(parameter)
            .coincides(right.point_at(right_parameter), tolerance)
    })
}

fn align_span_use(span_use: IntersectionSpanUse, reversed: bool) -> IntersectionSpanUse {
    if !reversed {
        return span_use;
    }
    match span_use {
        IntersectionSpanUse::Face {
            side,
            face,
            pcurve,
            orientation,
        } => IntersectionSpanUse::Face {
            side,
            face,
            pcurve: Box::new(pcurve.reversed()),
            orientation,
        },
        IntersectionSpanUse::Edge {
            side,
            edge,
            interval,
        } => IntersectionSpanUse::Edge {
            side,
            edge,
            interval: Interval::new(interval.end, interval.start),
        },
    }
}

fn same_span_cell(left: &IntersectionSpanUse, right: &IntersectionSpanUse) -> bool {
    match (left, right) {
        (
            IntersectionSpanUse::Edge {
                side: left_side,
                edge: left_edge,
                ..
            },
            IntersectionSpanUse::Edge {
                side: right_side,
                edge: right_edge,
                ..
            },
        ) => left_side == right_side && left_edge == right_edge,
        (
            IntersectionSpanUse::Face {
                side: left_side,
                face: left_face,
                ..
            },
            IntersectionSpanUse::Face {
                side: right_side,
                face: right_face,
                ..
            },
        ) => left_side == right_side && left_face == right_face,
        _ => false,
    }
}

fn uses_are_compatible(
    existing: &[IntersectionEventUse],
    incoming: &[IntersectionEventUse],
    tolerance: f64,
) -> bool {
    existing.iter().all(|left| {
        incoming
            .iter()
            .filter(|right| left.side == right.side && left.cell == right.cell)
            .all(|right| locations_are_compatible(left.location, right.location, tolerance))
    })
}

fn uses_match(left: IntersectionEventUse, right: IntersectionEventUse, tolerance: f64) -> bool {
    left.side == right.side
        && left.cell == right.cell
        && locations_are_compatible(left.location, right.location, tolerance)
}

fn locations_are_compatible(
    left: IntersectionEventLocation,
    right: IntersectionEventLocation,
    tolerance: f64,
) -> bool {
    match (left, right) {
        (IntersectionEventLocation::Vertex, IntersectionEventLocation::Vertex) => true,
        (
            IntersectionEventLocation::Edge { parameter: left },
            IntersectionEventLocation::Edge { parameter: right },
        ) => (left - right).abs() <= tolerance,
        (
            IntersectionEventLocation::Face { uv: left },
            IntersectionEventLocation::Face { uv: right },
        ) => (left - right).norm() <= tolerance,
        _ => false,
    }
}

pub(crate) fn vertex_use(side: BooleanSide, cell: BooleanCell) -> IntersectionEventUse {
    IntersectionEventUse {
        side,
        cell,
        location: IntersectionEventLocation::Vertex,
    }
}

pub(crate) fn edge_use(side: BooleanSide, edge: EdgeKey, parameter: f64) -> IntersectionEventUse {
    IntersectionEventUse {
        side,
        cell: BooleanCell::Edge(edge),
        location: IntersectionEventLocation::Edge { parameter },
    }
}

pub(crate) fn face_use(side: BooleanSide, face: FaceKey, uv: Point2) -> IntersectionEventUse {
    IntersectionEventUse {
        side,
        cell: BooleanCell::Face(face),
        location: IntersectionEventLocation::Face { uv },
    }
}

/// An original span interval mapped to a canonical subdivided span.
#[derive(Clone)]
pub(crate) struct SpanSubdivision {
    pub(crate) span: IntersectionSpanId,
    pub(crate) interval: Interval,
    pub(crate) reversed: bool,
}

/// Nodes all span interiors at compatible events, then deduplicates the exact pieces.
/// Input events already include every observed span endpoint, so partial overlaps
/// gain common endpoints before canonicalization.
pub(crate) fn finalize_network(
    network: &IntersectionNetwork,
    linear: f64,
    parameter: f64,
) -> Result<(IntersectionNetwork, Vec<Vec<SpanSubdivision>>), BooleanError> {
    let mut builder = IntersectionNetworkBuilder::new(linear);
    for event in &network.events {
        builder.record_event(event.point, event.kind, event.uses.clone());
    }
    let mut mapping = Vec::new();
    for span in &network.spans {
        let mut parameters = vec![0.0, 1.0];
        for event in &network.events {
            let t = span.curve.param_at(event.point);
            if t <= parameter
                || t >= 1.0 - parameter
                || !span.curve.point_at(t).coincides(event.point, linear)
            {
                continue;
            }
            let uses = span_event_uses(&span.uses, t);
            if uses_are_compatible(&event.uses, &uses, linear) {
                parameters.push(t);
            }
        }
        parameters.sort_by(f64::total_cmp);
        parameters.dedup_by(|a, b| (*a - *b).abs() <= parameter);
        let mut pieces = Vec::new();
        for pair in parameters.windows(2) {
            let interval = Interval::new(pair[0], pair[1]);
            let curve = normalized_subcurve(&span.curve, interval)?;
            let start = curve.point_at(0.0);
            let uses = span
                .uses
                .iter()
                .map(|span_use| match span_use {
                    IntersectionSpanUse::Face {
                        side,
                        face,
                        pcurve,
                        orientation,
                    } => Ok(IntersectionSpanUse::Face {
                        side: *side,
                        face: *face,
                        pcurve: Box::new(pcurve.trimmed(interval)?),
                        orientation: *orientation,
                    }),
                    IntersectionSpanUse::Edge {
                        side,
                        edge,
                        interval: source,
                    } => Ok(IntersectionSpanUse::Edge {
                        side: *side,
                        edge: *edge,
                        interval: Interval::new(
                            source.start + (source.end - source.start) * pair[0],
                            source.start + (source.end - source.start) * pair[1],
                        ),
                    }),
                })
                .collect::<Result<Vec<_>, NurbsError>>()?;
            if let Some(id) = builder.record_span(
                curve,
                span.kind,
                span_event_uses(&span.uses, pair[0]),
                span_event_uses(&span.uses, pair[1]),
                uses,
            ) {
                let canonical = &builder.network.spans[id.0];
                let reversed = !builder.network.events[canonical.start.0]
                    .point
                    .coincides(start, linear);
                pieces.push(SpanSubdivision {
                    span: id,
                    interval,
                    reversed,
                });
            }
        }
        mapping.push(pieces);
    }
    for region in &network.regions {
        builder.record_region(region.first_face, region.second_face, Vec::new());
    }
    Ok((builder.finish()?, mapping))
}

/// Converts a span parameter into each of its operand-local event incidences.
fn span_event_uses(uses: &[IntersectionSpanUse], t: f64) -> Vec<IntersectionEventUse> {
    uses.iter()
        .map(|usage| match usage {
            IntersectionSpanUse::Face {
                side, face, pcurve, ..
            } => face_use(*side, *face, pcurve.point_at(t)),
            IntersectionSpanUse::Edge {
                side,
                edge,
                interval,
            } => edge_use(
                *side,
                *edge,
                interval.start + (interval.end - interval.start) * t,
            ),
        })
        .collect()
}

/// Restores a normalized parameter domain after exact NURBS trimming.
pub(crate) fn normalized_subcurve(curve: &Curve, interval: Interval) -> Result<Curve, NurbsError> {
    let trimmed = curve.trimmed(interval)?.to_nurbs()?;
    let domain = trimmed.domain();
    let knots = KnotVector::new(
        trimmed
            .knots()
            .as_slice()
            .iter()
            .map(|knot| (knot - domain.start) / (domain.end - domain.start))
            .collect(),
    )?;
    Ok(Curve::Nurbs(NurbsCurve::new(
        trimmed.degree(),
        trimmed.control_points().clone(),
        knots,
    )?))
}
