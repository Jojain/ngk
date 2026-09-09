//! The ordering boundary between geometric roles and operand roles.
//!
//! A contact between two cells is a symmetric relation: whether a line meets a
//! cylinder, and where, does not depend on which operand each came from. Only
//! classification and assembly care about that, so the narrow phase is written
//! against the cells' geometric roles -- the vertex, the edge, the face -- and
//! the mapping to first/second happens exactly once, in [`record`].
//!
//! Operand order lives in the [`PairKind`] variant itself: **the first key of
//! every variant comes from the first operand**. `VertexEdge` and `EdgeVertex`
//! are the same geometric question asked of operands in opposite order, and one
//! probe answers both, so the mirrored variants cost no dispatch. Ordering is
//! then field order rather than a flag something has to remember to apply.

use crate::builders::faces::FaceImprint;
use crate::geometry::{Curve, Interval, Point3};
use crate::topology::shape_keys::{EdgeKey, FaceKey, VertexKey};

use super::{BooleanCell, BooleanSide, IntersectionAccumulator, PointContactKind, RawIntersection};

/// The two cells to probe, named by what they are and ordered by operand.
///
/// The first key comes from the first operand and the second from the second,
/// which is the whole of the ordering convention.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum PairKind {
    VertexVertex(VertexKey, VertexKey),
    VertexEdge(VertexKey, EdgeKey),
    EdgeVertex(EdgeKey, VertexKey),
    VertexFace(VertexKey, FaceKey),
    FaceVertex(FaceKey, VertexKey),
    EdgeEdge(EdgeKey, EdgeKey),
    EdgeFace(EdgeKey, FaceKey),
    FaceEdge(FaceKey, EdgeKey),
    FaceFace(FaceKey, FaceKey),
}

/// Which of a pair's two cells a contact attaches to.
///
/// A pair whose halves are different kinds has exactly one edge and at most one
/// face, so a probe names the role and never learns which operand supplied it.
/// Positional addressing is needed only when both halves are the same kind, and
/// there [`ContactCell::A`] is the first operand's cell by construction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum ContactCell {
    /// The pair's only edge.
    Edge,
    /// The pair's only face.
    Face,
    /// The first operand's cell, on a pair whose halves are the same kind.
    First,
    /// The second operand's cell, likewise.
    Second,
}

impl PairKind {
    /// Returns the pair's cells in operand order: first operand, then second.
    pub(super) fn ordered_cells(self) -> (BooleanCell, BooleanCell) {
        use BooleanCell::{Edge, Face, Vertex};
        match self {
            PairKind::VertexVertex(a, b) => (Vertex(a), Vertex(b)),
            PairKind::VertexEdge(vertex, edge) => (Vertex(vertex), Edge(edge)),
            PairKind::EdgeVertex(edge, vertex) => (Edge(edge), Vertex(vertex)),
            PairKind::VertexFace(vertex, face) => (Vertex(vertex), Face(face)),
            PairKind::FaceVertex(face, vertex) => (Face(face), Vertex(vertex)),
            PairKind::EdgeEdge(a, b) => (Edge(a), Edge(b)),
            PairKind::EdgeFace(edge, face) => (Edge(edge), Face(face)),
            PairKind::FaceEdge(face, edge) => (Face(face), Edge(edge)),
            PairKind::FaceFace(a, b) => (Face(a), Face(b)),
        }
    }

    /// Resolves what `cell` names, and the operand it came from.
    ///
    /// Panics when a role names nothing or would name both halves, which only a
    /// contact built for a different [`PairKind`] can produce.
    fn resolve(self, cell: ContactCell) -> (BooleanCell, BooleanSide) {
        let (first, second) = self.ordered_cells();
        let from_first = match cell {
            ContactCell::First => true,
            ContactCell::Second => false,
            ContactCell::Edge => match (first, second) {
                (BooleanCell::Edge(_), BooleanCell::Edge(_)) => {
                    panic!("{self:?} has two edges; address one of them by slot")
                }
                (BooleanCell::Edge(_), _) => true,
                (_, BooleanCell::Edge(_)) => false,
                _ => panic!("{self:?} has no edge"),
            },
            ContactCell::Face => match (first, second) {
                (BooleanCell::Face(_), BooleanCell::Face(_)) => {
                    panic!("{self:?} has two faces; address one of them by slot")
                }
                (BooleanCell::Face(_), _) => true,
                (_, BooleanCell::Face(_)) => false,
                _ => panic!("{self:?} has no face"),
            },
        };
        if from_first {
            (first, BooleanSide::First)
        } else {
            (second, BooleanSide::Second)
        }
    }

    /// Returns the edge `cell` names.
    pub(super) fn edge(self, cell: ContactCell) -> EdgeKey {
        match self.resolve(cell).0 {
            BooleanCell::Edge(edge) => edge,
            other => panic!("{cell:?} of {self:?} is {other:?}, not an edge"),
        }
    }

    /// Returns the face `cell` names.
    pub(super) fn face(self, cell: ContactCell) -> FaceKey {
        match self.resolve(cell).0 {
            BooleanCell::Face(face) => face,
            other => panic!("{cell:?} of {self:?} is {other:?}, not a face"),
        }
    }

    /// Returns the operand that supplied `cell`.
    pub(super) fn side(self, cell: ContactCell) -> BooleanSide {
        self.resolve(cell).1
    }
}

/// One narrow-phase observation, phrased in the pair's own geometric roles.
///
/// Nothing here names an operand: a probe reports what it saw about the pair's
/// edge or face, and [`record`] resolves that against the pair's variant.
#[derive(Clone)]
pub(super) enum Contact {
    /// A point where the pair's two cells meet.
    Point {
        point: Point3,
        kind: PointContactKind,
    },
    /// A point at which one cell's edge must gain a vertex.
    EdgePoint { cell: ContactCell, point: Point3 },
    /// Two edges resting on each other over an interval of each, `a` being the
    /// interval of the first operand's edge.
    EdgeOverlap { a: Interval, b: Interval },
    /// A contact section one cell's edge already realizes.
    EdgeSection {
        cell: ContactCell,
        curve: Curve,
        interval: Interval,
    },
    /// A section to imprint on one cell's face.
    Imprint {
        cell: ContactCell,
        imprint: FaceImprint,
        /// Whether the two surfaces are tangent along the section.
        tangent: bool,
    },
    /// Two coplanar faces sharing area rather than only touching.
    Region,
}

/// Writes a pair's contacts into the accumulator in operand order.
///
/// This is the only place in contact computation that produces a
/// [`BooleanSide`]; everything upstream of it is indifferent to which operand
/// is which.
pub(super) fn record(
    plan: &mut IntersectionAccumulator,
    pair: PairKind,
    contacts: impl IntoIterator<Item = Contact>,
) {
    for contact in contacts {
        match contact {
            Contact::Point { point, kind } => {
                let (first, second) = pair.ordered_cells();
                plan.contacts.push(RawIntersection::Point {
                    point,
                    first,
                    second,
                    kind,
                });
            }
            Contact::EdgePoint { cell, point } => {
                plan.edge_points
                    .entry(pair.edge(cell))
                    .or_default()
                    .push(point);
            }
            Contact::EdgeOverlap { a, b } => {
                plan.contacts.push(RawIntersection::Overlap {
                    first_edge: pair.edge(ContactCell::First),
                    second_edge: pair.edge(ContactCell::Second),
                    first_interval: a,
                    second_interval: b,
                });
            }
            Contact::EdgeSection {
                cell,
                curve,
                interval,
            } => {
                plan.contacts.push(RawIntersection::EdgeSection {
                    side: pair.side(cell),
                    edge: pair.edge(cell),
                    curve,
                    interval,
                });
            }
            Contact::Imprint {
                cell,
                imprint,
                tangent,
            } => {
                let sink = if tangent {
                    &mut plan.tangent_face_imprints
                } else {
                    &mut plan.face_imprints
                };
                sink.entry(pair.face(cell)).or_default().push(imprint);
            }
            Contact::Region => {
                let (first, second) = pair.ordered_cells();
                let (BooleanCell::Face(first_face), BooleanCell::Face(second_face)) =
                    (first, second)
                else {
                    panic!("a region contact needs two faces, got {first:?} and {second:?}");
                };
                plan.contacts.push(RawIntersection::Region {
                    first_face,
                    second_face,
                });
            }
        }
    }
}
