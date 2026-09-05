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
//! surviving face in its own plane.

use std::collections::HashMap;

use crate::builders::profiles::curve_pcurve;
use crate::builders::removal::{
    CellRemovalError, MergedCell, can_remove_cell, is_removable, remove_cell_staged,
};
use crate::geometry::{Plane, Surface, SurfacePeriodicity};
use crate::topology::gmap::{Dart, Dim, GMap};
use crate::topology::orientation::Orientation;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey};
use crate::topology::{TopologyEdit, TopologyEditError};

use super::super::errors::HealingError;
use super::super::options::HealingOptions;
use super::super::predicates::curve::reversed;
use super::super::predicates::{SurfaceMatch, surfaces_match};
use super::super::report::{HealedCell, HealingReport, SkipReason};
use super::{edge_dart_in_face, incident_faces};

/// Offers every scoped edge to the 1-removal operation.
pub(in crate::healing) fn run<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    options: &HealingOptions,
    report: &mut HealingReport,
) -> Result<(), HealingError> {
    for key in super::scoped_edges(g.map(), options)? {
        if g.map().edge_attr(key).is_none() {
            continue;
        }
        match plan(g.map(), key, options) {
            Ok(fusion) => apply(g, fusion, report)?,
            Err(reason) => report.skip(HealedCell::Edge(key), reason),
        }
    }
    Ok(())
}

/// Everything the fusion needs, resolved before the topology changes.
struct FaceFusion {
    edge: EdgeKey,
    dart: Dart,
    survivor: FaceKey,
    /// The face the fusion consumes, or `None` when one face bounds the edge on
    /// both sides and the removal only rejoins its boundary.
    consumed: Option<FaceKey>,
    surfaces: SurfaceMatch,
    /// The survivor's plane, when the fused face's curves must be rebuilt.
    plane: Option<Plane>,
}

/// Decides whether the edge carries shape.
fn plan<P: Payload>(
    g: &GMap<P>,
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
        [first, second] if first <= second => (first, Some(second)),
        [first, second] => (second, Some(first)),
        _ => return Err(SkipReason::NotBetweenTwoCells),
    };

    let surface = &g
        .face_attr(survivor)
        .ok_or(SkipReason::Unregistered)?
        .surface;
    let surfaces = match consumed {
        None => {
            // A seam is the parameterization's own boundary, not a slit: closing
            // it over would leave the face with no way to express its trimming.
            if !matches!(surface.periodicity(), SurfacePeriodicity::None) {
                return Err(SkipReason::PeriodicSurface);
            }
            SurfaceMatch::Identical
        }
        Some(consumed) => {
            for face in [survivor, consumed] {
                if !fuses_outer_loops(g, dart, face) {
                    return Err(SkipReason::NotOuterLoop);
                }
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
    // candidate never disturbs the map.
    can_remove_cell(g, dart, Dim::One).map_err(|error| match error {
        CellRemovalError::LoopWouldSplit { .. } => SkipReason::LoopWouldSplit,
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
    })
}

/// Reports whether the edge at `dart` has a side no face bounds.
fn bounds_a_free_side<P: Payload>(g: &GMap<P>, dart: Dart) -> bool {
    g.orbit(dart, g.orbit_indices(Dim::One))
        .any(|d| g.is_free(d, Dim::Two))
}

/// Reports whether `face` carries the edge on its outer boundary.
///
/// Fusing an outer boundary with a hole reshapes which loop is outer, which
/// this pass does not model.
fn fuses_outer_loops<P: Payload>(g: &GMap<P>, dart: Dart, face: FaceKey) -> bool {
    let Some(attr) = g.face_attr(face) else {
        return false;
    };
    let Some(incident) = edge_dart_in_face(g, dart, face) else {
        return false;
    };
    g.profile_key(incident).is_some() && g.profile_key(incident) == g.profile_key(attr.outer_loop)
}

/// Reports whether every boundary edge of both faces carries the geometry a
/// rebuild needs.
fn has_rebuildable_boundary<P: Payload>(
    g: &GMap<P>,
    survivor: FaceKey,
    consumed: Option<FaceKey>,
) -> bool {
    std::iter::once(survivor).chain(consumed).all(|face| {
        g.face(face).is_some_and(|view| {
            view.loops()
                .iter()
                .flat_map(|boundary| boundary.edges())
                .all(|edge| {
                    edge.curve().is_some()
                        && edge.start().point().is_some()
                        && edge.end().point().is_some()
                })
        })
    })
}

/// Removes the edge and restores the fused face's parameter curves.
fn apply<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    fusion: FaceFusion,
    report: &mut HealingReport,
) -> Result<(), HealingError> {
    let carried = match (fusion.surfaces, fusion.consumed) {
        (SurfaceMatch::Identical, Some(consumed)) => g
            .face_attr_unchecked(consumed)
            .pcurves
            .iter()
            .map(|(dart, pcurve)| (*dart, pcurve.clone()))
            .collect::<Vec<_>>(),
        _ => Vec::new(),
    };

    let removal = remove_cell_staged(g, fusion.dart, Dim::One)?;
    let (survivor, consumed, orientation) = match removal.merged {
        MergedCell::Faces {
            survivor,
            consumed,
            orientation,
            ..
        } => (survivor, Some(consumed), orientation),
        MergedCell::Loops { face, .. } => (face, None, Orientation::Same),
        MergedCell::Edges { .. } => {
            return Err(TopologyEditError::MissingLineageAttribute {
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
                    Orientation::Reversed => (g.alpha(Dim::Zero, dart), pcurve.reversed()),
                };
                g.face_attr_mut_unchecked(survivor)
                    .pcurves
                    .insert(dart, pcurve);
            }
        }
        Some(plane) => rebuild_pcurves(g, survivor, &plane)
            .ok_or(HealingError::PcurveRebuildFailed { face: survivor })?,
    }

    report.removed_edges.push(fusion.edge);
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
    g: &mut TopologyEdit<'_, P>,
    face: FaceKey,
    plane: &Plane,
) -> Option<()> {
    let mut pcurves = HashMap::new();
    {
        let view = g.map().face(face)?;
        for boundary in view.loops() {
            for edge in boundary.edges() {
                let dart = edge.dart();
                let start = *edge.start().point()?;
                let end = *edge.end().point()?;
                let stored = edge.curve()?;
                let oriented = match g.map().edge_orientation_at_dart(edge.key(), dart) {
                    Orientation::Same => stored.clone(),
                    Orientation::Reversed => reversed(stored)?,
                };
                pcurves.insert(dart, curve_pcurve(&oriented, start, end, plane).ok()?);
            }
        }
    }
    g.face_attr_mut_unchecked(face).pcurves = pcurves;
    Some(())
}
