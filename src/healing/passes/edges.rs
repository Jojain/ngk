//! The 1-removal pass: fusing the faces a shape-free edge separates.
//!
//! An edge carries no shape when the faces meeting along it sit on one support
//! surface. Usually those are two distinct faces and removing the edge fuses
//! them. Sometimes one face bounds the edge on both sides — the slit left
//! behind when an earlier fusion joined two faces that shared more than one
//! edge — and removing it rejoins that face's own boundary instead.
//!
//! Either way the pass has to leave the face with a coherent set of parameter
//! curves: it carries the consumed face's curves over unchanged when both
//! faces shared one surface value, and otherwise rebuilds every curve of the
//! surviving face in its own plane. A closed interface between an inner loop
//! and the face filling it is removed as a sequence of 1-removals; the final
//! removal also drops the exhausted inner-loop identity.

use std::collections::HashMap;

use crate::builders::profiles::curve_pcurve;
use crate::builders::removal::{
    CellRemovalError, MergeKind, MergedCell, is_removable, planned_merge, remove_cell_staged,
};
use crate::geometry::{Plane, Surface};
use crate::model::Model;
use crate::topology::attributes::LoopKind;
use crate::topology::gmap::{Dart, Dim};
use crate::topology::orientation::Orientation;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey};
use crate::topology::{ModelEdit, ModelEditError};

use super::super::errors::HealingError;
use super::super::options::HealingOptions;
use super::super::predicates::curve::reversed;
use super::super::predicates::{SurfaceMatch, surfaces_match};
use super::super::report::{HealedCell, HealingReport, SkipReason};
use super::{edge_dart_in_face, incident_faces};

/// Offers every scoped edge that is not a seam to the 1-removal operation.
///
/// A seam is left to [`super::seams`]: it is not a redundant edge but a cut in
/// a parameterization, and the two are worth asking for separately.
pub(in crate::healing) fn run<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    options: &HealingOptions,
    report: &mut HealingReport,
) -> Result<(), HealingError> {
    run_over(edit, options, report, |kind| !kind.is_seam_removal())
}

/// Offers every scoped edge whose planned removal `wanted` accepts.
pub(in crate::healing::passes) fn run_over<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    options: &HealingOptions,
    report: &mut HealingReport,
    wanted: impl Fn(MergeKind) -> bool,
) -> Result<(), HealingError> {
    for key in super::scoped_edges(edit.model(), options)? {
        if edit.model().edge_attr(key).is_none() {
            continue;
        }
        match plan(edit.model(), key, options) {
            // The other pass's business, not a refusal: recording a skip here
            // would report a cell as declined that is about to be removed.
            Ok(fusion) if !wanted(fusion.kind) => continue,
            Ok(fusion) => apply(edit, fusion, report)?,
            Err(reason) => report.skip(HealedCell::Edge(key), reason),
        }
    }
    Ok(())
}

/// Everything the fusion needs, resolved before the topology changes.
pub(in crate::healing::passes) struct FaceFusion {
    edge: EdgeKey,
    dart: Dart,
    survivor: FaceKey,
    /// The face the fusion consumes, or `None` when one face bounds the edge on
    /// both sides and the removal only rejoins its boundary.
    consumed: Option<FaceKey>,
    surfaces: SurfaceMatch,
    /// The survivor's plane, when the fused face's curves must be rebuilt.
    plane: Option<Plane>,
    /// What the removal will do, which is also what decides whose pass this is.
    kind: MergeKind,
}

/// Decides whether the edge carries shape.
pub(in crate::healing::passes) fn plan<P: Payload>(
    g: &Model<P>,
    edge: EdgeKey,
    options: &HealingOptions,
) -> Result<FaceFusion, SkipReason> {
    let dart = g.edge_attr(edge).ok_or(SkipReason::Unregistered)?.dart;
    if !is_removable(g, dart, Dim::One) {
        return Err(SkipReason::NotRemovable);
    }

    let (survivor, consumed) = match incident_faces(g, dart)[..] {
        // One face on both sides: the edge is a slit the boundary can close
        // over, or the wall between the face's outer loop and one of its holes.
        // An edge on a free boundary also reports one face, but it has one side
        // rather than two, and nothing to rejoin.
        [_] if bounds_a_free_side(g, dart) => return Err(SkipReason::NotBetweenTwoCells),
        [face] => (face, None),
        // Matches the survivor rule of `remove_cell_staged`.
        [first, second]
            if options.remove_filled_inner_loops && fills_inner_loop(g, dart, first, second) =>
        {
            (first, Some(second))
        }
        [first, second]
            if options.remove_filled_inner_loops && fills_inner_loop(g, dart, second, first) =>
        {
            (second, Some(first))
        }
        [first, second] if first <= second => (first, Some(second)),
        [first, second] => (second, Some(first)),
        _ => return Err(SkipReason::NotBetweenTwoCells),
    };

    let surface = &g
        .face_attr(survivor)
        .ok_or(SkipReason::Unregistered)?
        .surface;
    let surfaces = match consumed {
        // A seam is the parameterization's own cut, not a slit, so removing it
        // does not close the boundary over — it lets the loop fall into the two
        // wrapping loops the face really has. `remove_cell_staged` recognises
        // that shape and refuses any other two-way split, so the decision is
        // left to it rather than guarded by periodicity here.
        None => SurfaceMatch::Identical,
        Some(consumed) => {
            let both_outer = [survivor, consumed]
                .into_iter()
                .all(|face| fuses_outer_loop(g, dart, face));
            let filled_inner_loop =
                options.remove_filled_inner_loops && fills_inner_loop(g, dart, survivor, consumed);
            if !both_outer && !filled_inner_loop {
                return Err(SkipReason::NotOuterLoop);
            }
            let other = &g
                .face_attr(consumed)
                .ok_or(SkipReason::Unregistered)?
                .surface;
            surfaces_match(
                surface,
                other,
                options.linear_tolerance,
                options.angular_tolerance,
            )
            .ok_or(SkipReason::SurfacesNotJoinable)?
        }
    };

    // A rejoined boundary re-keys the parameter curves it kept, so a planar face
    // is rebuilt whether or not the surfaces needed reconciling.
    let plane = match (surfaces, surface, consumed) {
        (_, Surface::Plane(plane), _) => Some(plane.clone()),
        (SurfaceMatch::Coplanar, _, _) => return Err(SkipReason::SurfacesNotJoinable),
        (SurfaceMatch::Identical, _, _) => None,
    };
    if plane.is_some() && !has_rebuildable_boundary(g, survivor, consumed) {
        return Err(SkipReason::MissingGeometry);
    }

    // Every other refusal the removal can raise — a boundary that would fall
    // into two loops, an unregistered incidence — is decided here so a declined
    // candidate never disturbs the map. The answer also says what the removal
    // will be, which is what sorts the candidate into one pass or the other.
    let kind = planned_merge(g, dart, Dim::One).map_err(|error| match error {
        CellRemovalError::LoopWouldSplit { .. } => SkipReason::LoopWouldSplit,
        CellRemovalError::WouldLeaveWrappingLoop { .. } => SkipReason::PeriodicSurface,
        CellRemovalError::WouldUnboundFace { .. } => SkipReason::WouldUnboundFace,
        CellRemovalError::NotRemovable { .. } => SkipReason::NotRemovable,
        _ => SkipReason::Unregistered,
    })?;

    Ok(FaceFusion {
        edge,
        dart,
        survivor,
        consumed,
        surfaces,
        plane,
        kind,
    })
}

/// Reports whether the edge at `dart` has a side no face bounds.
fn bounds_a_free_side<P: Payload>(g: &Model<P>, dart: Dart) -> bool {
    g.orbit(dart, g.orbit_indices(Dim::One))
        .any(|d| g.is_free(d, Dim::Two))
}

/// Reports whether `face` carries the edge on a loop that bounds it from outside.
///
/// That is its outer loop, or a wrapping loop, which bounds a ring face the
/// same way without closing in parameter space. Inner loops are excluded:
/// filled inner loops are recognized separately because their surrounding face
/// must survive.
fn fuses_outer_loop<P: Payload>(g: &Model<P>, dart: Dart, face: FaceKey) -> bool {
    let Some(attr) = g.face_attr(face) else {
        return false;
    };
    let Some(incident) = edge_dart_in_face(g, dart, face) else {
        return false;
    };
    let Some(profile) = g.profile_key(incident) else {
        return false;
    };
    attr.loops
        .iter()
        .filter(|boundary| boundary.kind() != LoopKind::Inner)
        .any(|boundary| g.profile_key(boundary.seed()) == Some(profile))
}

/// Reports whether `consumed` completely fills one inner loop of `survivor`.
///
/// The low-level removal keeps the lower face key. Imprint splits preserve the
/// source face as that survivor, so requiring this direction also prevents an
/// island's outer loop from accidentally becoming the fused face's exterior.
fn fills_inner_loop<P: Payload>(
    g: &Model<P>,
    dart: Dart,
    survivor: FaceKey,
    consumed: FaceKey,
) -> bool {
    if fuses_outer_loop(g, dart, survivor) || !fuses_outer_loop(g, dart, consumed) {
        return false;
    }
    let Some(incident) = edge_dart_in_face(g, dart, survivor) else {
        return false;
    };
    let Some(profile) = g.profile_key(incident) else {
        return false;
    };
    let Some(face) = g.face(survivor) else {
        return false;
    };
    let inner_loops = face.inner_loops();
    let Some(inner) = inner_loops
        .iter()
        .find(|boundary| boundary.profile_key() == Some(profile))
    else {
        return false;
    };
    let Some(island) = g.face(consumed) else {
        return false;
    };
    let inner_edges = inner.edges();
    let mut inner_keys = inner_edges
        .iter()
        .map(|edge| edge.key())
        .collect::<Vec<_>>();
    let island_edges = island.edges();
    let mut island_keys = island_edges
        .iter()
        .map(|edge| edge.key())
        .collect::<Vec<_>>();
    inner_keys.sort();
    inner_keys.dedup();
    island_keys.sort();
    island_keys.dedup();
    inner_keys == island_keys
        && inner_edges.iter().all(|edge| {
            let mut faces = incident_faces(g, edge.dart());
            faces.sort();
            let mut expected = [survivor, consumed];
            expected.sort();
            faces == expected
        })
}

/// Reports whether every boundary edge of both faces carries the geometry a
/// rebuild needs.
fn has_rebuildable_boundary<P: Payload>(
    g: &Model<P>,
    survivor: FaceKey,
    consumed: Option<FaceKey>,
) -> bool {
    std::iter::once(survivor).chain(consumed).all(|face| {
        g.face(face).is_some_and(|view| {
            view.loops()
                .iter()
                .flat_map(|boundary| boundary.edges())
                .all(|edge| edge.trimmed_curve().is_some())
        })
    })
}

/// Removes the edge and restores the fused face's parameter curves.
pub(in crate::healing::passes) fn apply<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    fusion: FaceFusion,
    report: &mut HealingReport,
) -> Result<(), HealingError> {
    let carried = match (fusion.surfaces, fusion.consumed) {
        (SurfaceMatch::Identical, Some(consumed)) => edit
            .face_attr_unchecked(consumed)
            .pcurves
            .iter()
            .map(|(dart, pcurve)| (*dart, pcurve.clone()))
            .collect::<Vec<_>>(),
        _ => Vec::new(),
    };

    let removal = remove_cell_staged(edit, fusion.dart, Dim::One)?;
    let (survivor, consumed, orientation) = match removal.merged {
        MergedCell::Faces {
            survivor,
            consumed,
            orientation,
            ..
        } => (survivor, Some(consumed), orientation),
        MergedCell::Loops { face, .. } => (face, None, Orientation::Same),
        // A seam removal reshapes one face's boundary into two wrapping loops,
        // consuming nothing: the same rejoin bookkeeping applies.
        MergedCell::Ring { face, .. } => (face, None, Orientation::Same),
        // And into one, when a degeneracy closes the face's far side.
        MergedCell::Cap { face, .. } => (face, None, Orientation::Same),
        MergedCell::BoundaryRemoved { face, .. } => (face, None, Orientation::Same),
        // And the face's last boundary going leaves it covering its support.
        MergedCell::Unbounded { face, .. } => (face, None, Orientation::Same),
        MergedCell::Edges { .. } => {
            return Err(ModelEditError::MissingLineageAttribute {
                key: crate::topology::EditKey::Face(fusion.survivor),
            }
            .into());
        }
    };
    debug_assert_eq!(survivor, fusion.survivor, "survivor rules must agree");

    match fusion.plane {
        None => {
            for (dart, pcurve) in carried {
                let Some(dart) = removal.remap(dart) else {
                    continue;
                };
                let (dart, pcurve) = match orientation {
                    Orientation::Same => (dart, pcurve),
                    Orientation::Reversed => (edit.alpha(Dim::Zero, dart), pcurve.reversed()),
                };
                edit.face_attr_mut_unchecked(survivor)
                    .pcurves
                    .insert(dart, pcurve);
            }
        }
        Some(plane) => rebuild_pcurves(edit, survivor, &plane)
            .ok_or(HealingError::PcurveRebuildFailed { face: survivor })?,
    }

    match fusion.kind.is_seam_removal() {
        true => report.removed_seams.push(fusion.edge),
        false => report.removed_edges.push(fusion.edge),
    }
    match consumed {
        Some(consumed) => report.fused_faces.push((survivor, consumed)),
        None => report.rejoined_faces.push(survivor),
    }
    Ok(())
}

/// Projects every boundary of `face` into `plane` to replace its parameter
/// curves.
///
/// Each curve is oriented along the boundary before it is projected, so the
/// stored direction of a shared edge does not leak into the face's own
/// parameter space.
fn rebuild_pcurves<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    plane: &Plane,
) -> Option<()> {
    let mut pcurves = HashMap::new();
    {
        let view = edit.model().face(face)?;
        for boundary in view.loops() {
            for edge in boundary.edges() {
                let dart = edge.dart();
                // The ends of the section the edge *is*, not of the vertices it
                // happens to carry: a closed edge has one vertex or none, and
                // still runs from somewhere to somewhere.
                let section = edge.trimmed_curve()?;
                let (start, end) = (section.point_at(0.0), section.point_at(1.0));
                let stored = edge.curve()?;
                let oriented = match edit.model().edge_orientation_at_dart(edge.key(), dart) {
                    Orientation::Same => stored.clone(),
                    Orientation::Reversed => reversed(stored)?,
                };
                pcurves.insert(dart, curve_pcurve(&oriented, start, end, plane).ok()?);
            }
        }
    }
    edit.face_attr_mut_unchecked(face).pcurves = pcurves;
    Some(())
}
