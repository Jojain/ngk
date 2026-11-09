//! A source curve walked edge by edge, and the chains its projection makes.
//!
//! A source is a sequence of spans, each between two vertices or none, that
//! is either closed on itself or open. Each span is cut into the parts that
//! have a shadow and the parts that do not, and the parts with a shadow are
//! joined into chains wherever the walk passes from one to the next without a
//! reason to stop. What a part with a shadow is carried onto is the caller's:
//! the chaining is the same for any target, given a test of whether two
//! neighbouring segments continue each other.

use std::collections::HashMap;

use super::curve_result::{
    ChainEnd, CurveProjection, Exactness, ProjectedChain, ProjectedSegment, SegmentSource,
    SourceSpan,
};
use super::error::ProjectionError;
use super::mode::Projection;
use super::options::ProjectOptions;
use super::plane_carrier::{Piece, PlaneCarrier};
use crate::geometry::{Curve, LINEAR_TOLERANCE, Plane, PointCoincidence, TrimmedCurve};
use crate::topology::closed::Closeable;
use crate::topology::edge::Edge;
use crate::topology::payload::Payload;
use crate::topology::profile::Profile;
use crate::topology::shape_keys::VertexKey;

/// One span of the walk, with the vertices it runs between.
pub(super) struct WalkEdge {
    pub(super) span: TrimmedCurve,
    pub(super) source: SegmentSource,
    /// The vertex the span leaves, when it has one.
    pub(super) start: Option<VertexKey>,
    /// The vertex the span arrives at, when it has one.
    pub(super) end: Option<VertexKey>,
}

impl WalkEdge {
    /// A span that is no edge of a model, and so touches no vertex.
    pub(super) fn of_curve(span: TrimmedCurve) -> Self {
        Self {
            span,
            source: SegmentSource::Curve,
            start: None,
            end: None,
        }
    }

    /// The edge as it is oriented in the walk.
    pub(super) fn of_edge<P: Payload>(edge: &Edge<'_, P>) -> Self {
        let (start, end) = match edge {
            Edge::Bounded(bounded) => {
                let (start, end) = bounded.vertices();
                (Some(start.key()), Some(end.key()))
            }
            Edge::Marked(marked) => {
                let corner = marked.corner().key();
                (Some(corner), Some(corner))
            }
            Edge::Unmarked(_) => (None, None),
        };
        Self {
            span: edge.trimmed_curve(),
            source: SegmentSource::Edge(edge.key()),
            start,
            end,
        }
    }
}

/// Spans in the order they are walked, the last arriving where the first
/// leaves when `cyclic`.
pub(super) struct Walk {
    pub(super) edges: Vec<WalkEdge>,
    pub(super) cyclic: bool,
}

impl Walk {
    /// A single span, closed when its ends coincide.
    pub(super) fn of_span(span: TrimmedCurve) -> Self {
        let cyclic = span.start().coincides(span.end(), LINEAR_TOLERANCE);
        Self {
            edges: vec![WalkEdge::of_curve(span)],
            cyclic,
        }
    }

    /// The whole of a curve's domain; an unbounded curve has no finite span
    /// and is refused.
    pub(super) fn of_curve(curve: &Curve) -> Result<Self, ProjectionError> {
        if !curve.domain().is_finite() {
            return Err(ProjectionError::UnboundedSource);
        }
        Ok(Self::of_span(TrimmedCurve::whole(curve.clone())))
    }

    /// An edge as it is oriented in its view.
    pub(super) fn of_edge<P: Payload>(edge: &Edge<'_, P>) -> Self {
        Self {
            edges: vec![WalkEdge::of_edge(edge)],
            cyclic: !matches!(edge, Edge::Bounded(_)),
        }
    }

    /// A profile's edges in the order of its walk.
    pub(super) fn of_profile<P: Payload>(profile: &Profile<'_, P>) -> Self {
        Self {
            edges: profile.edges().iter().map(WalkEdge::of_edge).collect(),
            cyclic: profile.is_closed(),
        }
    }

    /// Refuses a walk with a span that has no finite extent.
    pub(super) fn ensure_bounded(&self) -> Result<(), ProjectionError> {
        if self
            .edges
            .iter()
            .any(|edge| !edge.span.interval().is_finite())
        {
            return Err(ProjectionError::UnboundedSource);
        }
        Ok(())
    }
}

/// One part of one span, and the segment it projects to when it has a shadow.
pub(super) struct Item<Hit> {
    pub(super) edge: usize,
    /// Whether the part runs to the end of its span rather than to a cut
    /// inside it.
    pub(super) last_in_edge: bool,
    pub(super) segment: Option<ProjectedSegment<Hit>>,
}

/// What lies between a part and the one after it.
#[derive(Clone, Copy)]
enum Link {
    /// The two are continuous.
    Join,
    /// They are not, for this reason.
    Break(ChainEnd),
}

/// Projects `walk` onto `plane`, cut into chains.
///
/// The parts of each span with a shadow are chained as described for
/// [`chain_items`]; every two of them continue each other on a plane.
pub(super) fn project_walk(
    walk: &Walk,
    plane: &Plane,
    mode: Projection,
    options: &ProjectOptions,
) -> Result<CurveProjection<()>, ProjectionError> {
    walk.ensure_bounded()?;
    let Some(first) = walk.edges.first() else {
        return Err(ProjectionError::Miss);
    };
    let carrier = PlaneCarrier::new(plane, mode, options, first.span.start())?;

    let mut items = Vec::new();
    for (index, edge) in walk.edges.iter().enumerate() {
        let pieces = carrier.pieces(&edge.span)?;
        let count = pieces.len();
        for (position, piece) in pieces.into_iter().enumerate() {
            items.push(Item {
                edge: index,
                last_in_edge: position + 1 == count,
                segment: segment_of(&carrier, edge, piece)?,
            });
        }
    }
    if items.iter().all(|item| item.segment.is_none()) {
        return Err(ProjectionError::Miss);
    }
    Ok(CurveProjection {
        chains: chain_items(items, walk, |_, _| true),
    })
}

/// Cuts the parts of a walk into chains.
///
/// Two consecutive parts are continuous when both have a shadow, meet at a
/// vertex where exactly two ends of the walk meet (or inside a span), and
/// `continues` says the first's segment runs on into the second's. Where a part
/// with a shadow meets one without, or a span is cut inside, the chain stops at
/// a [`Miss`](ChainEnd::Miss); where both have a shadow and `continues` denies
/// it, at a [`Jump`](ChainEnd::Jump). Where more or fewer than two ends meet at
/// a vertex, it stops on that [`SourceVertex`](ChainEnd::SourceVertex). The
/// ends of an open walk are [`SourceEnd`](ChainEnd::SourceEnd), and a closed
/// walk all of whose parts are continuous is one chain with
/// [`Closed`](ChainEnd::Closed) ends.
///
/// The parts must be in walk order, and at least one must have a segment.
pub(super) fn chain_items<Hit>(
    mut items: Vec<Item<Hit>>,
    walk: &Walk,
    continues: impl Fn(&ProjectedSegment<Hit>, &ProjectedSegment<Hit>) -> bool,
) -> Vec<ProjectedChain<Hit>> {
    let mut degrees: HashMap<VertexKey, usize> = HashMap::new();
    for key in walk
        .edges
        .iter()
        .flat_map(|edge| [edge.start, edge.end])
        .flatten()
    {
        *degrees.entry(key).or_default() += 1;
    }

    let count = items.len();
    let next = |index: usize| {
        if index + 1 < count {
            Some(index + 1)
        } else if walk.cyclic {
            Some(0)
        } else {
            None
        }
    };
    let previous = |index: usize| {
        if index > 0 {
            Some(index - 1)
        } else if walk.cyclic {
            Some(count - 1)
        } else {
            None
        }
    };
    let links: Vec<Link> = (0..count)
        .map(|index| match next(index) {
            Some(following) => link(&items, index, following, &walk.edges, &degrees, &continues),
            None => Link::Break(ChainEnd::SourceEnd),
        })
        .collect();
    // Why a chain through `index` begins there, or `None` when a part with a
    // shadow leads into it.
    let start_of = |index: usize| match previous(index) {
        None => Some(ChainEnd::SourceEnd),
        Some(before) => match links[before] {
            Link::Join => None,
            Link::Break(end) => Some(end),
        },
    };

    let mut chains = Vec::new();
    for index in 0..count {
        if items[index].segment.is_none() {
            continue;
        }
        let Some(start) = start_of(index) else {
            continue;
        };
        let mut segments = Vec::new();
        let mut at = index;
        let end = loop {
            segments.extend(items[at].segment.take());
            match (links[at], next(at)) {
                (Link::Join, Some(following)) => at = following,
                (Link::Join, None) => break ChainEnd::SourceEnd,
                (Link::Break(end), _) => break end,
            }
        };
        chains.push(ProjectedChain {
            segments,
            start,
            end,
        });
    }
    // Every part with a shadow was led into by another: the walk is a closed
    // loop of them, and it is one chain.
    if chains.is_empty() {
        let segments = items
            .iter_mut()
            .filter_map(|item| item.segment.take())
            .collect();
        chains.push(ProjectedChain {
            segments,
            start: ChainEnd::Closed,
            end: ChainEnd::Closed,
        });
    }
    chains
}

fn segment_of(
    carrier: &PlaneCarrier<'_>,
    edge: &WalkEdge,
    piece: Piece,
) -> Result<Option<ProjectedSegment<()>>, ProjectionError> {
    if !piece.front {
        return Ok(None);
    }
    let (curve, pcurve) = carrier.project_piece(edge.span.curve(), piece.interval)?;
    Ok(Some(ProjectedSegment {
        curve,
        pcurve,
        hit: (),
        source: SourceSpan {
            source: edge.source,
            interval: piece.interval,
        },
        fit: Exactness::Exact,
    }))
}

/// What lies between the part at `index` and the one at `following`.
fn link<Hit>(
    items: &[Item<Hit>],
    index: usize,
    following: usize,
    edges: &[WalkEdge],
    degrees: &HashMap<VertexKey, usize>,
    continues: &impl Fn(&ProjectedSegment<Hit>, &ProjectedSegment<Hit>) -> bool,
) -> Link {
    let part = &items[index];
    // A cut inside a span is no vertex: only the end of a span can stop a
    // chain on one.
    if part.last_in_edge
        && let Some(vertex) = edges[part.edge].end
        && degrees.get(&vertex).is_some_and(|degree| *degree != 2)
    {
        return Link::Break(ChainEnd::SourceVertex(vertex));
    }
    match (&part.segment, &items[following].segment) {
        (Some(before), Some(after)) if continues(before, after) => Link::Join,
        (Some(_), Some(_)) => Link::Break(ChainEnd::Jump),
        _ => Link::Break(ChainEnd::Miss),
    }
}

#[cfg(test)]
mod tests {
    use slotmap::SlotMap;

    use super::*;
    use crate::geometry::Point3;

    fn edge(from: Point3, to: Point3, start: VertexKey, end: VertexKey) -> WalkEdge {
        WalkEdge {
            span: TrimmedCurve::segment(from, to),
            source: SegmentSource::Curve,
            start: Some(start),
            end: Some(end),
        }
    }

    #[test]
    fn a_vertex_where_more_than_two_ends_meet_stops_the_chains_on_it() {
        let mut keys = SlotMap::<VertexKey, ()>::with_key();
        let (outer, joint, far) = (keys.insert(()), keys.insert(()), keys.insert(()));
        let a = Point3::new(0.0, 0.0, 0.0);
        let b = Point3::new(1.0, 0.0, 0.0);
        let c = Point3::new(2.0, 1.0, 0.0);
        // Three ends meet at `joint`: the first edge's, the second's and the
        // third's.
        let walk = Walk {
            edges: vec![
                edge(a, b, outer, joint),
                edge(b, c, joint, far),
                edge(c, b, far, joint),
            ],
            cyclic: false,
        };
        let result = project_walk(
            &walk,
            &Plane::xy(),
            Projection::Normal,
            &ProjectOptions::default(),
        )
        .unwrap();
        assert_eq!(result.chains.len(), 2);
        assert_eq!(result.chains[0].segments.len(), 1);
        assert_eq!(result.chains[0].start, ChainEnd::SourceEnd);
        assert_eq!(result.chains[0].end, ChainEnd::SourceVertex(joint));
        assert_eq!(result.chains[1].segments.len(), 2);
        assert_eq!(result.chains[1].start, ChainEnd::SourceVertex(joint));
        assert_eq!(result.chains[1].end, ChainEnd::SourceEnd);
    }
}
