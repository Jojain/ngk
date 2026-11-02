//! Welding two faces along an edge each of them runs.

use std::collections::HashMap;

use thiserror::Error;

use crate::builders::edges::{EdgeSplitError, split_face_boundary_edge};
use crate::geometry::parameter::Fraction;
use crate::geometry::{Point3, PointCoincidence};
use crate::topology::ModelEdit;
use crate::topology::edge::Edge;
use crate::topology::edit::ModelEditError;
use crate::topology::gmap::{Dart, Dim};
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey, VertexKey};

/// How the two faces of a weld ran the edge they now share.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Traversal {
    /// Each face ran the edge the other way, as two consistently wound
    /// neighbours always do.
    Opposed,
    /// Both faces ran the edge the same way: one of them is wound against the
    /// other.
    Aligned,
}

/// Why two edges could not be welded.
#[derive(Debug, Error)]
pub(crate) enum FaceSewError {
    /// One of the edges is not on its face's boundary, its ends do not lie on
    /// the other's, or it is already welded to a third face.
    #[error("the two edges do not cover the same span")]
    Mismatch,
    #[error(transparent)]
    EdgeSplit(#[from] EdgeSplitError),
    #[error(transparent)]
    ModelEdit(#[from] ModelEditError),
}

/// The edges and vertices a run of welds folded into a survivor.
///
/// A key consumed by one weld can still be named by a later one, so each map
/// is followed to its end before a key is used.
#[derive(Debug, Default)]
pub(crate) struct SewMerges {
    pub(crate) edges: HashMap<EdgeKey, EdgeKey>,
    pub(crate) vertices: HashMap<VertexKey, VertexKey>,
}

/// Welds `second`'s face onto `first`'s across their two edges, which must
/// cover the same span within `tolerance`.
///
/// The second edge and its vertices are merged into the first's. Which way the
/// faces ran the edge is reported, not judged: welding two faces wound against
/// each other is a valid map, and only the caller knows whether it wanted one.
pub(crate) fn sew_faces_along<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    first: (EdgeKey, FaceKey),
    second: (EdgeKey, FaceKey),
    tolerance: f64,
    merges: &mut SewMerges,
) -> Result<Traversal, FaceSewError> {
    let ((first, first_face), (second, second_face)) = (first, second);
    mark_to_match(edit, first, second)?;
    // Which way each fragment runs is a question about the face that keeps it,
    // not about the edge: an edge's own reference dart is whichever one it was
    // built from, and a fragment copied in from another map need not have been
    // built from the same side as one made here. Reading the direction off the
    // edge instead of off the loop is then right exactly half the time — the
    // half where the two happen to agree — and sews the second face on
    // backwards whenever they disagree.
    let (da, av, [a0, a1, aq]) =
        loop_traversal(edit, first, first_face).ok_or(FaceSewError::Mismatch)?;
    let (mut db, mut bv, [b0, b1, bq]) =
        loop_traversal(edit, second, second_face).ok_or(FaceSewError::Mismatch)?;
    let traversal = if a0.coincides(a1, tolerance) {
        // A closed span: its ends coincide, so only a point partway along can
        // say whether the other side runs with it or against it.
        if !aq.coincides(b0, tolerance) && !b0.coincides(a0, tolerance) {
            return Err(FaceSewError::Mismatch);
        }
        if aq.coincides(bq, tolerance) {
            Traversal::Aligned
        } else {
            Traversal::Opposed
        }
    } else if a0.coincides(b1, tolerance) && a1.coincides(b0, tolerance) {
        Traversal::Opposed
    } else if a0.coincides(b0, tolerance) && a1.coincides(b1, tolerance) {
        Traversal::Aligned
    } else {
        return Err(FaceSewError::Mismatch);
    };
    if traversal == Traversal::Opposed {
        db = edit.alpha(Dim::Zero, db);
        bv.swap(0, 1);
    }
    if !edit.is_free(da, Dim::Two) || !edit.is_free(db, Dim::Two) {
        return Err(FaceSewError::Mismatch);
    }
    edit.sew(Dim::Two, da, db)?;
    edit.merge_edges_into(first, second);
    merges.edges.insert(second, first);
    for (a, b) in av.into_iter().zip(bv) {
        // Nothing to reconcile where an end carries no logical vertex: the two
        // sides meet along the edge, not at a corner either of them has.
        let (Some(mut a), Some(mut b)) = (a, b) else {
            continue;
        };
        while let Some(&next) = merges.vertices.get(&a) {
            a = next;
        }
        while let Some(&next) = merges.vertices.get(&b) {
            b = next;
        }
        if a != b {
            edit.merge_vertices_into(a, b);
            merges.vertices.insert(b, a);
        }
    }
    Ok(traversal)
}

/// The dart `face` traverses `edge` with, and the edge's ends in that order.
///
/// The points always exist; the vertex keys need not. A whole circle with
/// nothing marked on it is bounded by no vertex at all, and both its ends are
/// the one place it closes -- so there is a span to match the other side
/// against, and nothing to merge.
fn loop_traversal<P: Payload>(
    edit: &ModelEdit<'_, P>,
    edge: EdgeKey,
    face: FaceKey,
) -> Option<(Dart, [Option<VertexKey>; 2], [Point3; 3])> {
    let traversed = edit
        .face_unchecked(face)
        .loops()
        .into_iter()
        .flat_map(|boundary| boundary.edges())
        .find(|candidate| candidate.key() == edge)?;
    let dart = traversed.dart();
    let section = traversed.trimmed_curve();
    // The quarter sample is what orients a closed edge. Its two ends are the
    // same point, so they cannot say which way round the other side runs; a
    // point partway along can, and agrees with the ends everywhere else.
    let samples = [
        section.point_at(Fraction::new(0.0)),
        section.point_at(Fraction::new(1.0)),
        section.point_at(Fraction::new(0.25)),
    ];
    let keys = match traversed {
        Edge::Bounded(bounded) => {
            let (start, end) = bounded.vertices();
            [Some(start.key()), Some(end.key())]
        }
        Edge::Marked(marked) => [Some(marked.corner().key()); 2],
        Edge::Unmarked(_) => [None, None],
    };
    Some((dart, keys, samples))
}

/// Gives both sides of a closed span a corner when only one side has it.
///
/// Sewing the two sides joins the 0-cells they close at into one orbit. With
/// one side marked and the other unmarked, that orbit would be held by the
/// marked side's corner and owned by the unmarked edge's interior at once. So
/// the unmarked side is marked first, where the other side's corner sits:
/// marking is a pure relabel of its own closing 0-cell, and afterwards both
/// sides close at one point that both call a corner.
fn mark_to_match<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    first: EdgeKey,
    second: EdgeKey,
) -> Result<(), EdgeSplitError> {
    for (marked, unmarked) in [(first, second), (second, first)] {
        let corner = match edit.edge_unchecked(marked) {
            Edge::Marked(edge) => *edge.corner().point(),
            Edge::Bounded(_) | Edge::Unmarked(_) => continue,
        };
        let Edge::Unmarked(edge) = edit.edge_unchecked(unmarked) else {
            continue;
        };
        let fraction = edge.trimmed_curve().parameter_at(corner);
        split_face_boundary_edge(edit, unmarked, fraction, false)?;
    }
    Ok(())
}
