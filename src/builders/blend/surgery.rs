//! The executor's vocabulary: everything a blend changes, with every point,
//! curve and pcurve already computed.
//!
//! A surgery says what happens to the map in four words — cut an edge open,
//! insert an edge at a corner, add a face, and the corners all of those meet
//! at — and carries the geometry of each. A closed edge is cut like any other
//! but has no corner, and the face that fills it is a band between its two
//! rails rather than a walk round corners. Planning writes it; the executor
//! builds it without computing anything. Every treatment of a vertex, every
//! section of an edge and every 2D corner is written in this vocabulary, so a
//! new one extends what can be planned without changing how it is built.

use crate::geometry::{Point3, Surface, TrimmedCurve, TrimmedCurve2};
use crate::topology::edit::EditKey;
use crate::topology::gmap::Dart;
use crate::topology::shape_keys::{EdgeKey, FaceKey, VertexKey};

/// Index of a [`PlannedCorner`] in [`Surgery::corners`].
pub(crate) type CornerId = usize;

/// Index of a [`Joint`] in [`Surgery::joints`].
pub(crate) type JointId = usize;

/// A vertex the surgery leaves: where it sits, and what it derives from.
#[derive(Debug, Clone)]
pub(crate) struct PlannedCorner {
    pub(crate) point: Point3,
    pub(crate) sources: Vec<EditKey>,
}

/// A selected edge opened along its two faces.
///
/// Each side becomes an edge of its own, the rail its face now ends at.
#[derive(Debug, Clone)]
pub(crate) struct Cut {
    pub(crate) edge: EdgeKey,
    pub(crate) sides: [CutSide; 2],
}

/// One face's side of a cut edge, and the rail it becomes.
#[derive(Debug, Clone)]
pub(crate) struct CutSide {
    pub(crate) face: FaceKey,
    /// This face's dart on the edge, at the edge's start.
    pub(crate) start: Dart,
    /// Where the rail ends.
    pub(crate) ends: RailEnds,
    /// The rail, running from its first corner to its second, or once round
    /// from where it closes.
    pub(crate) rail: TrimmedCurve,
    /// The rail's pcurve on `face`, in the rail's direction.
    pub(crate) pcurve: TrimmedCurve2,
}

/// Where one side of a cut edge ends.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RailEnds {
    /// At two corners: at the edge's start, then at its end.
    Corners([CornerId; 2]),
    /// Nowhere: the rail is a closed edge with no corner, as the cut edge
    /// was. The 0-cell where it closes is interior to it.
    Closed,
}

impl RailEnds {
    /// The corners, when the rail has any.
    pub(crate) fn corners(self) -> Option<[CornerId; 2]> {
        match self {
            Self::Corners(corners) => Some(corners),
            Self::Closed => None,
        }
    }
}

/// An edge the surgery adds, other than a rail.
///
/// A joint is used by two faces, or by one face or wire when it closes a
/// planar corner: either inserted into an existing corner and bounding a new
/// face, or bounding two new faces.
#[derive(Debug, Clone)]
pub(crate) struct Joint {
    /// The corners it runs between, from the first to the second.
    pub(crate) corners: [CornerId; 2],
    pub(crate) curve: TrimmedCurve,
    pub(crate) sources: Vec<EditKey>,
    /// The existing corner it is inserted into, if any.
    pub(crate) insertion: Option<Insertion>,
}

/// An existing corner a joint is inserted into.
#[derive(Debug, Clone)]
pub(crate) struct Insertion {
    /// The face whose loop receives the joint, or `None` on a wire.
    pub(crate) face: Option<FaceKey>,
    /// The incoming edge's dart at the corner. The joint follows it, so it
    /// runs from the incoming edge's side of the corner to the outgoing one.
    pub(crate) after: Dart,
    /// The joint's pcurve on `face`, in the joint's direction.
    pub(crate) pcurve: Option<TrimmedCurve2>,
}

/// A face the surgery adds.
#[derive(Debug, Clone)]
pub(crate) struct NewFace {
    pub(crate) surface: Surface,
    pub(crate) sources: Vec<EditKey>,
    pub(crate) boundary: NewBoundary,
}

/// How a new face is bounded.
#[derive(Debug, Clone)]
pub(crate) enum NewBoundary {
    /// One loop, walked once round.
    Walk(Vec<Bound>),
    /// Two closed rails, each running once round the surface's `u`, with the
    /// face the band between them: the blend of a closed edge. The first is
    /// walked with its rail and the second against it, so the band lies on
    /// one side of both. A scaffold cut joins them so the face is one 2-cell.
    Band(Box<[Bound; 2]>),
}

impl NewBoundary {
    /// Every boundary edge, in walk order.
    pub(crate) fn bounds(&self) -> &[Bound] {
        match self {
            Self::Walk(bounds) => bounds,
            Self::Band(bounds) => bounds.as_slice(),
        }
    }
}

/// One edge of a new face's boundary walk.
#[derive(Debug, Clone)]
pub(crate) struct Bound {
    pub(crate) kind: BoundKind,
    /// Whether the walk runs against the rail's or the joint's own direction.
    pub(crate) reversed: bool,
    /// The pcurve on the new face's surface, in the walk's direction.
    pub(crate) pcurve: TrimmedCurve2,
}

/// What a new face's boundary edge is shared with.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum BoundKind {
    /// One side of a cut edge.
    Rail { cut: usize, side: usize },
    /// A joint.
    Joint(JointId),
}

/// Everything one blend changes.
#[derive(Debug, Clone, Default)]
pub(crate) struct Surgery {
    pub(crate) corners: Vec<PlannedCorner>,
    /// Vertices the surgery replaces. Every piece of them must end up at a
    /// planned corner.
    pub(crate) consumed_vertices: Vec<VertexKey>,
    pub(crate) cuts: Vec<Cut>,
    pub(crate) joints: Vec<Joint>,
    pub(crate) faces: Vec<NewFace>,
}

impl Surgery {
    /// Adds a corner and returns its id.
    pub(crate) fn corner(&mut self, point: Point3, sources: Vec<EditKey>) -> CornerId {
        self.corners.push(PlannedCorner { point, sources });
        self.corners.len() - 1
    }

    /// Adds a joint and returns its id.
    pub(crate) fn joint(&mut self, joint: Joint) -> JointId {
        self.joints.push(joint);
        self.joints.len() - 1
    }

    /// The corners a bound's own direction runs between, when it has any.
    pub(crate) fn bound_corners(&self, kind: BoundKind) -> Option<[CornerId; 2]> {
        match kind {
            BoundKind::Rail { cut, side } => self.cuts[cut].sides[side].ends.corners(),
            BoundKind::Joint(joint) => Some(self.joints[joint].corners),
        }
    }
}
