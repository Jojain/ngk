//! Canonical intersection network shared by both Boolean operands.

use crate::geometry::{
    Bounded, Curve, Curve2, Interval, KnotVector, NurbsCurve, NurbsError, Point2, Point3,
    PointCoincidence,
};
use crate::topology::gmap::GMap;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey};
use nalgebra::Vector3;
use std::collections::HashSet;
use thiserror::Error;

use super::{BooleanCell, BooleanError, BooleanSide, BooleanTolerances, PointContactKind};

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
    Tangent,
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
    /// Oriented boundary cycle, counterclockwise in the first face's parameter domain.
    pub boundary: Vec<(IntersectionSpanId, IntersectionOrientation)>,
    /// Whether the two coincident faces are oriented the same way on the overlap.
    pub normals_agree: bool,
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
    #[error("coincident region {region} is not bounded by one closed oriented cycle")]
    UnboundedRegion { region: usize },
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

    /// Resolves a span identifier.
    pub fn span(&self, id: IntersectionSpanId) -> Option<&IntersectionSpan> {
        self.spans.get(id.0)
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
        let point_kind = match kind {
            IntersectionSpanKind::Tangent => PointContactKind::Tangent,
            IntersectionSpanKind::Transverse | IntersectionSpanKind::Overlap => {
                PointContactKind::Transverse
            }
        };
        let start = self.record_event(start_point, point_kind, start_uses);
        let end = self.record_event(end_point, point_kind, end_uses);
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

    /// Records a coincident face pair; its oriented boundary is closed after noding.
    pub(crate) fn record_region(&mut self, first_face: FaceKey, second_face: FaceKey) {
        if self
            .network
            .regions
            .iter()
            .any(|region| region.first_face == first_face && region.second_face == second_face)
        {
            return;
        }
        self.network.regions.push(IntersectionRegion {
            first_face,
            second_face,
            boundary: Vec::new(),
            normals_agree: false,
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
            .all(|right| {
                matches!(
                    (left.location, right.location),
                    (
                        IntersectionEventLocation::Face { .. },
                        IntersectionEventLocation::Face { .. }
                    )
                ) || locations_are_compatible(left.location, right.location, tolerance)
            })
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

/// Bounded guard against tolerance thrash in the noding fixed point.
const MAX_NODING_PASSES: usize = 8;

/// Nodes every span interior at compatible events, repeating until no pass adds an
/// event. A pass can split a span at a point whose incidences only became
/// compatible once an earlier pass merged them, so one pass is not a fixed point.
pub(crate) fn finalize_network(
    network: &IntersectionNetwork,
    linear: f64,
    parameter: f64,
) -> Result<(IntersectionNetwork, Vec<Vec<SpanSubdivision>>), BooleanError> {
    let mut events = network.events.clone();
    for _ in 0..MAX_NODING_PASSES {
        let (builder, mapping) = node_spans(network, &events, linear, parameter)?;
        if builder.network.events.len() <= events.len() {
            return Ok((builder.finish()?, mapping));
        }
        events = builder.network.events.clone();
    }
    Err(BooleanError::NodingDidNotConverge {
        passes: MAX_NODING_PASSES,
    })
}

/// One noding pass of the original spans against `events`, which already include
/// every observed span endpoint, so partial overlaps gain common endpoints before
/// canonicalization.
fn node_spans(
    network: &IntersectionNetwork,
    events: &[IntersectionEvent],
    linear: f64,
    parameter: f64,
) -> Result<(IntersectionNetworkBuilder, Vec<Vec<SpanSubdivision>>), BooleanError> {
    let mut builder = IntersectionNetworkBuilder::new(linear);
    for event in events {
        builder.record_event(event.point, event.kind, event.uses.clone());
    }
    let mut mapping = Vec::new();
    for span in &network.spans {
        let mut parameters = vec![0.0, 1.0];
        for event in events {
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
        builder.record_region(region.first_face, region.second_face);
    }
    Ok((builder, mapping))
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
    if let Curve::Bounded(bounded) = curve
        && matches!(bounded.inner(), Curve::Line(_) | Curve::Circle(_))
    {
        let bounds = bounded.bounds();
        let parameter = |t: f64| bounds.start + (bounds.end - bounds.start) * t;
        return Ok(Curve::Bounded(Box::new(Bounded::new(
            bounded.inner().clone(),
            Interval::new(parameter(interval.start), parameter(interval.end)),
        ))));
    }
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

/// Edge keys bounding one face, used to recognize a section a face already carries.
fn face_edge_keys<P: Payload>(map: &GMap<P>, face: FaceKey) -> HashSet<EdgeKey> {
    map.face_unchecked(face)
        .edges()
        .into_iter()
        .map(|edge| edge.key())
        .collect()
}

/// Whether one operand realizes `span` on `face`, either as an imprint or as one
/// of that face's own boundary edges.
fn span_lies_on_face(
    span: &IntersectionSpan,
    side: BooleanSide,
    face: FaceKey,
    edges: &HashSet<EdgeKey>,
) -> bool {
    span.uses.iter().any(|span_use| match span_use {
        IntersectionSpanUse::Face {
            side: use_side,
            face: use_face,
            ..
        } => *use_side == side && *use_face == face,
        IntersectionSpanUse::Edge {
            side: use_side,
            edge,
            ..
        } => *use_side == side && edges.contains(edge),
    })
}

/// Chains the region's spans into a single closed cycle, or reports that they do
/// not form exactly one.
fn walk_cycle(
    network: &IntersectionNetwork,
    candidates: &[IntersectionSpanId],
) -> Option<Vec<(IntersectionSpanId, IntersectionOrientation)>> {
    let (&seed, rest) = candidates.split_first()?;
    let mut remaining = rest.to_vec();
    let origin = network.spans[seed.0].start;
    let mut current = network.spans[seed.0].end;
    let mut cycle = vec![(seed, IntersectionOrientation::Forward)];
    while current != origin {
        let position = remaining.iter().position(|id| {
            let span = &network.spans[id.0];
            span.start == current || span.end == current
        })?;
        let id = remaining.remove(position);
        let span = &network.spans[id.0];
        let (orientation, next) = if span.start == current {
            (IntersectionOrientation::Forward, span.end)
        } else {
            (IntersectionOrientation::Reversed, span.start)
        };
        cycle.push((id, orientation));
        current = next;
    }
    remaining.is_empty().then_some(cycle)
}

/// Signed area of the cycle in one face's parameter domain; positive is counterclockwise.
fn cycle_signed_area<P: Payload>(
    map: &GMap<P>,
    network: &IntersectionNetwork,
    face: FaceKey,
    cycle: &[(IntersectionSpanId, IntersectionOrientation)],
) -> Result<f64, BooleanError> {
    let view = map.face_unchecked(face);
    let mut area = 0.0;
    for (id, orientation) in cycle {
        let span = &network.spans[id.0];
        let (start, end) = match orientation {
            IntersectionOrientation::Forward => (span.start, span.end),
            IntersectionOrientation::Reversed => (span.end, span.start),
        };
        let start = view
            .surface()
            .closest_parameter(network.events[start.0].point)?;
        let end = view
            .surface()
            .closest_parameter(network.events[end.0].point)?;
        area += start.x * end.y - end.x * start.y;
    }
    Ok(area)
}

/// Walks every coincident region into one counterclockwise cycle in the first
/// face's domain and records whether the two faces are oriented alike there.
///
/// This is the closure step classification and selection rely on: a coincident
/// region with no oriented boundary is indistinguishable from an unresolved overlap.
pub(crate) fn close_regions<P: Payload>(
    network: &mut IntersectionNetwork,
    map: &GMap<P>,
) -> Result<(), BooleanError> {
    for index in 0..network.regions.len() {
        let first_face = network.regions[index].first_face;
        let second_face = network.regions[index].second_face;
        let first_edges = face_edge_keys(map, first_face);
        let second_edges = face_edge_keys(map, second_face);
        let candidates = network
            .spans
            .iter()
            .enumerate()
            .filter(|(_, span)| {
                span_lies_on_face(span, BooleanSide::First, first_face, &first_edges)
                    && span_lies_on_face(span, BooleanSide::Second, second_face, &second_edges)
            })
            .map(|(id, _)| IntersectionSpanId(id))
            .collect::<Vec<_>>();
        let mut cycle = walk_cycle(network, &candidates)
            .ok_or(IntersectionNetworkValidationError::UnboundedRegion { region: index })?;
        if cycle_signed_area(map, network, first_face, &cycle)? < 0.0 {
            cycle.reverse();
            for (_, orientation) in &mut cycle {
                *orientation = match orientation {
                    IntersectionOrientation::Forward => IntersectionOrientation::Reversed,
                    IntersectionOrientation::Reversed => IntersectionOrientation::Forward,
                };
            }
        }
        let normals_agree = region_normals_agree(map, network, first_face, second_face, &cycle)?;
        let region = &mut network.regions[index];
        region.boundary = cycle;
        region.normals_agree = normals_agree;
    }
    Ok(())
}

/// Compares the two oriented face normals at the region's boundary centroid.
fn region_normals_agree<P: Payload>(
    map: &GMap<P>,
    network: &IntersectionNetwork,
    first_face: FaceKey,
    second_face: FaceKey,
    cycle: &[(IntersectionSpanId, IntersectionOrientation)],
) -> Result<bool, BooleanError> {
    let mut centroid = Vector3::zeros();
    for (id, orientation) in cycle {
        let span = &network.spans[id.0];
        let start = match orientation {
            IntersectionOrientation::Forward => span.start,
            IntersectionOrientation::Reversed => span.end,
        };
        centroid += network.events[start.0].point.coords;
    }
    let centroid = Point3::from(centroid / cycle.len() as f64);
    let first = map.face_unchecked(first_face);
    let second = map.face_unchecked(second_face);
    let first_uv = first.surface().closest_parameter(centroid)?;
    let second_uv = second.surface().closest_parameter(centroid)?;
    let first_normal = *first.normal_at(first_uv.x, first_uv.y);
    let second_normal = *second.normal_at(second_uv.x, second_uv.y);
    Ok(first_normal.dot(&second_normal) > 0.0)
}

/// Checks the contract a regularized solid Boolean requires of a finalized network:
/// every section is realized on both operands, agrees with its pcurves, closes into
/// loops, and every coincident region is bounded.
///
/// The general preparation facility deliberately admits open, one-sided contacts,
/// so this is checked only where a closed result solid must follow.
pub fn validate_solid_network<P: Payload>(
    map: &GMap<P>,
    network: &IntersectionNetwork,
    tolerances: BooleanTolerances,
) -> Result<(), BooleanError> {
    let mut valence = vec![0usize; network.events.len()];
    let mut coincident = vec![false; network.events.len()];
    for (index, span) in network.spans.iter().enumerate() {
        for side in [BooleanSide::First, BooleanSide::Second] {
            if !span
                .uses
                .iter()
                .any(|span_use| span_use_side(span_use) == side)
            {
                return Err(BooleanError::SpanNotTwoSided { span: index });
            }
        }
        validate_span_pcurves(map, span, index, tolerances)?;
        // Parity is a statement about transverse loops only: a coincident section
        // ends where the two boundaries stop sharing area, not on another crossing.
        match span.kind {
            IntersectionSpanKind::Transverse => {
                valence[span.start.0] += 1;
                valence[span.end.0] += 1;
            }
            IntersectionSpanKind::Tangent => {}
            IntersectionSpanKind::Overlap => {
                coincident[span.start.0] = true;
                coincident[span.end.0] = true;
            }
        }
    }
    for (index, event) in network.events.iter().enumerate() {
        if event.kind == PointContactKind::Tangent
            || coincident[index]
            || valence[index].is_multiple_of(2)
        {
            continue;
        }
        return Err(BooleanError::OpenIntersectionLoop {
            event: index,
            point: event.point,
        });
    }
    for region in &network.regions {
        if region.boundary.is_empty() {
            return Err(BooleanError::RegionWithoutBoundary {
                first: region.first_face,
                second: region.second_face,
            });
        }
    }
    Ok(())
}

/// Which operand a section representation belongs to.
fn span_use_side(span_use: &IntersectionSpanUse) -> BooleanSide {
    match span_use {
        IntersectionSpanUse::Edge { side, .. } | IntersectionSpanUse::Face { side, .. } => *side,
    }
}

/// Rejects a pcurve that does not evaluate onto the canonical section curve.
fn validate_span_pcurves<P: Payload>(
    map: &GMap<P>,
    span: &IntersectionSpan,
    index: usize,
    tolerances: BooleanTolerances,
) -> Result<(), BooleanError> {
    const SAMPLES: usize = 8;
    for span_use in &span.uses {
        let IntersectionSpanUse::Face { face, pcurve, .. } = span_use else {
            continue;
        };
        let view = map.face_unchecked(*face);
        for step in 0..=SAMPLES {
            let t = step as f64 / SAMPLES as f64;
            let uv = pcurve.point_at(t);
            let residual = (view.point_at(uv.x, uv.y) - span.curve.point_at(t)).norm();
            if residual > tolerances.section_fit {
                return Err(BooleanError::PcurveDisagreesWithCurve {
                    span: index,
                    residual,
                });
            }
        }
    }
    Ok(())
}
