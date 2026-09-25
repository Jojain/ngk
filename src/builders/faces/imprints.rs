use std::collections::{HashMap, HashSet};

use super::*;
use crate::builders::edges::EdgeSplitError;
use crate::builders::scaffold::{CutAttachment, cut_between_loops};
use crate::geometry::parameter::{Fraction, NativeParam, Normalized};
use crate::geometry::{
    Axis2, DomainSide, Interval, LINEAR_TOLERANCE, NurbsError, Periodicity, Point2, Surface,
    SurfacePeriodicity, TrimmedCurve, TrimmedCurve2, Vector2,
};
use crate::model::Model;
use crate::topology::ModelEdit;
use crate::topology::attributes::{
    EdgeAttr, FaceAttr, LoopDefinition, LoopKind, ProfileAttr, VertexAttr,
};
use crate::topology::edge::Edge;
use crate::topology::edit::EditKey;
use crate::topology::embedding::EntityOwner;
use crate::topology::gmap::{Dart, Dim};
use crate::topology::payload::Payload;
use crate::topology::profile::Profile;
use crate::topology::shape_keys::{EdgeKey, FaceKey, VertexKey};

pub fn split_face_by_imprints<P: Payload>(
    g: &mut Model<P>,
    face: FaceKey,
    imprints: &[FaceImprint],
) -> Result<Vec<FaceImprintSplit>, FaceImprintSplitError> {
    g.transaction(|edit| split_face_by_imprints_edit(edit, face, imprints))
}

/// Applies every open and closed imprint before the outer transaction commits.
pub(crate) fn split_face_by_imprints_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    imprints: &[FaceImprint],
) -> Result<Vec<FaceImprintSplit>, FaceImprintSplitError> {
    let (closed_indices, open_indices): (Vec<_>, Vec<_>) =
        (0..imprints.len()).partition(|&index| imprints[index].pcurve.is_closed());
    let (closed_imprints, open_imprints) = imprints.iter().fold(
        (Vec::new(), Vec::new()),
        |(mut closed, mut open), imprint| {
            if imprint.pcurve.is_closed() {
                closed.push(imprint);
            } else {
                open.push(imprint.clone());
            }
            (closed, open)
        },
    );
    let open_imprints = imprints_on_one_periodic_image(
        edit.face_attr(face)
            .ok_or(FaceImprintSplitError::MissingFace { face })?
            .surface
            .periodicity(),
        &open_imprints,
    )?;
    if closed_imprints.is_empty() {
        let mut splits = split_ring_face_by_wrapping_chains(edit, face, &open_imprints)?;
        if !splits.is_empty() {
            remap_section_indices(&mut splits, &open_indices);
            return Ok(splits);
        }
    }

    let pcurves = open_imprints
        .iter()
        .map(|imprint| imprint.pcurve.clone())
        .collect::<Vec<_>>();
    let graph = FaceImprintGraph::from_curves(&pcurves)?;
    // Cut for the imprints as handed in, so a corner is placed once, and read
    // back against the open ones the walk uses: a periodic image is the same
    // imprint written elsewhere in the domain, and lands on the same vertex.
    let placed = split_imprint_boundary_endpoints(edit, face, imprints)?;
    let open_placed = open_indices
        .iter()
        .map(|index| placed[*index])
        .collect::<Vec<_>>();
    let mut splits = add_closed_curve_imprint_loops(edit, face, &closed_imprints)?;
    remap_section_indices(&mut splits, &closed_indices);
    let mut open_splits = add_closed_imprint_loops(edit, face, &graph, &open_imprints)?;
    open_splits.extend(split_open_imprints(
        edit,
        vec![face],
        &open_imprints,
        &open_placed,
    )?);
    remap_section_indices(&mut open_splits, &open_indices);
    splits.extend(open_splits);
    check_imprints_realized(face, &splits, &graph, &closed_indices, &open_indices)?;
    Ok(splits)
}

/// Refuses a split that left an imprint with nothing to show for it.
///
/// An imprint the splitter cannot place is a cut the caller asked for and did
/// not get, and returning the other cuts alone hands back a face that is wrong
/// rather than incomplete — the Boolean that asked for it then fails much later,
/// where the span it cannot sew says nothing about the imprint that went
/// missing here.
///
/// An imprint that contributed no graph edge is *not* missing: the graph drops a
/// fragment that duplicates one already recorded, so a chord handed in twice,
/// or in both directions, is realized once and the copy has nothing left to
/// realize.
pub(crate) fn check_imprints_realized(
    face: FaceKey,
    splits: &[FaceImprintSplit],
    graph: &FaceImprintGraph,
    closed_indices: &[usize],
    open_indices: &[usize],
) -> Result<(), FaceImprintSplitError> {
    let realized = splits
        .iter()
        .flat_map(|split| &split.sections)
        .map(|section| section.imprint)
        .collect::<HashSet<_>>();
    let contributing = graph
        .edges()
        .iter()
        .filter_map(|edge| open_indices.get(edge.source_curve).copied())
        .collect::<HashSet<_>>();
    let mut expected = closed_indices
        .iter()
        .copied()
        .chain(contributing)
        .collect::<Vec<_>>();
    expected.sort_unstable();
    match expected.into_iter().find(|index| !realized.contains(index)) {
        Some(imprint) => Err(FaceImprintSplitError::ImprintNotRealized { face, imprint }),
        None => Ok(()),
    }
}

/// Rewrites imprints onto one continuous image of a periodic face's domain.
///
/// Two parameter points a whole period apart name the same point of a periodic
/// surface, so imprints written on different images still meet end to end on
/// the face itself — an arc crossing the seam has to run past the domain's edge
/// to stay continuous, and lands a period away from where the arc it meets was
/// written. Joining them by position alone then finds no walk at all, and a
/// face the walk should cut is left whole: this is the whole of why a torus
/// survives a Boolean that any other support would be split by.
///
/// Only whole periods are ever added, so every imprint still names the points it
/// named, and its 3D curve is left exactly as it was.
pub(crate) fn imprints_on_one_periodic_image(
    periodicity: SurfacePeriodicity,
    imprints: &[FaceImprint],
) -> Result<Vec<FaceImprint>, NurbsError> {
    let periods = match periodicity {
        SurfacePeriodicity::None => return Ok(imprints.to_vec()),
        SurfacePeriodicity::UPeriodic(u) => [Some(u), None],
        SurfacePeriodicity::VPeriodic(v) => [None, Some(v)],
        SurfacePeriodicity::UVPeriodic(u, v) => [Some(u), Some(v)],
    };
    let mut placed = vec![None; imprints.len()];
    for seed in 0..imprints.len() {
        if placed[seed].is_some() {
            continue;
        }
        // The seed anchors its own walk: which image that walk is written on is
        // arbitrary, only that the walk agrees with itself matters.
        let mut frontier = endpoints(&imprints[seed]).to_vec();
        placed[seed] = Some(imprints[seed].clone());
        while let Some(anchor) = frontier.pop() {
            for index in 0..imprints.len() {
                if placed[index].is_some() {
                    continue;
                }
                let Some(offset) = endpoints(&imprints[index])
                    .into_iter()
                    .find_map(|end| period_offset(periods, anchor, end))
                else {
                    continue;
                };
                let moved = FaceImprint::with_section(
                    imprints[index].curve.clone(),
                    imprints[index].pcurve.translated(offset)?,
                );
                frontier.extend(endpoints(&moved));
                placed[index] = Some(moved);
            }
        }
    }
    Ok(placed
        .into_iter()
        .map(|imprint| {
            imprint.expect("every imprint is placed, by its own walk if by nothing else")
        })
        .collect())
}

pub(crate) fn endpoints(imprint: &FaceImprint) -> [Point2; 2] {
    [
        imprint.pcurve.point_at(Fraction::new(0.0)),
        imprint.pcurve.point_at(Fraction::new(1.0)),
    ]
}

/// The whole-period translation carrying `point` onto `anchor`, if one does.
pub(crate) fn period_offset(
    periods: [Option<f64>; 2],
    anchor: Point2,
    point: Point2,
) -> Option<Vector2> {
    let mut offset = Vector2::zeros();
    for (axis, period) in periods.into_iter().enumerate() {
        let Some(period) = period.filter(|period| period.is_finite() && *period > 0.0) else {
            continue;
        };
        offset[axis] = ((anchor[axis] - point[axis]) / period).round() * period;
    }
    ((point + offset) - anchor)
        .norm()
        .le(&LINEAR_TOLERANCE)
        .then_some(offset)
}

/// Restores original input indices after partitioning closed and open curves.
pub(crate) fn remap_section_indices(splits: &mut [FaceImprintSplit], indices: &[usize]) {
    for section in splits.iter_mut().flat_map(|split| &mut split.sections) {
        section.imprint = indices[section.imprint];
    }
}

pub(crate) fn split_open_imprints<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    mut active_faces: Vec<FaceKey>,
    imprints: &[FaceImprint],
    placed: &[[Option<VertexKey>; 2]],
) -> Result<Vec<FaceImprintSplit>, FaceImprintSplitError> {
    let mut splits = Vec::new();
    loop {
        let mut next_faces = Vec::new();
        let mut progressed = false;

        for face in active_faces {
            let Some(split) = split_one_face_by_imprints(edit, face, imprints, placed)? else {
                next_faces.push(face);
                continue;
            };

            next_faces.push(split.first);
            next_faces.push(split.second);
            splits.push(split);
            progressed = true;
        }

        if !progressed {
            break;
        }
        active_faces = next_faces;
    }
    Ok(splits)
}

/// Cuts the boundary at every imprint end, returning the corner each became.
///
/// The result is indexed like `imprints`, each entry holding the corner for the
/// imprint's start and for its end. An endpoint that meets no boundary has none.
pub(crate) fn split_imprint_boundary_endpoints<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    imprints: &[FaceImprint],
) -> Result<Vec<[Option<VertexKey>; 2]>, FaceImprintSplitError> {
    let mut placed = Vec::with_capacity(imprints.len());
    for imprint in imprints {
        let mut corners = [None, None];
        for (slot, fraction) in [0.0, 1.0].into_iter().enumerate() {
            let uv = imprint.pcurve.point_at(Fraction::new(fraction));
            corners[slot] = split_boundary_at_uv(edit, face, uv)?;
        }
        placed.push(corners);
    }
    Ok(placed)
}

pub(crate) fn add_closed_curve_imprint_loops<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    imprints: &[&FaceImprint],
) -> Result<Vec<FaceImprintSplit>, FaceImprintSplitError> {
    let boundary_uvs = face_boundary_uvs(edit, face)?;
    // Read off the boundary's curves, not its corners: a face bounded by one
    // whole circle has a single 0-cell, whose polygon encloses nothing and
    // would state no winding at all.
    let boundary_area = boundary_winding(edit, face);
    let mut splits = Vec::new();

    for (index, imprint) in imprints.iter().enumerate() {
        let samples = imprint
            .pcurve
            .adaptive_samples(LINEAR_TOLERANCE, 16)
            .into_iter()
            .map(|(_, point)| point)
            .collect::<Vec<_>>();
        if samples.len() < 4
            || samples
                .iter()
                .any(|point| snap_boundary_corner(&boundary_uvs, *point).is_some())
        {
            continue;
        }

        let imprint_area = signed_area(&samples[..samples.len() - 1]);
        let outside = if boundary_area.signum() == imprint_area.signum() {
            reverse_imprint(imprint)?
        } else {
            (*imprint).clone()
        };
        let mut split = split_face_by_closed_curve_imprint(edit, face, &outside)?;
        let reversed = boundary_area.signum() == imprint_area.signum();
        for section in &mut split.sections {
            section.imprint = index;
            section.interval = if reversed {
                Interval::new(1.0, 0.0)
            } else {
                Interval::new(0.0, 1.0)
            };
        }
        splits.push(split);
    }
    Ok(splits)
}

pub(crate) fn reverse_imprint(imprint: &FaceImprint) -> Result<FaceImprint, NurbsError> {
    imprint.reversed()
}

/// One link of a wrapping chain: the imprint as travelled, and where it came from.
#[derive(Clone)]
pub(crate) struct ChainLink {
    imprint: FaceImprint,
    /// Index of the input imprint this link was cut from.
    source: usize,
    /// The part of that input used, backwards when the chain travels it so.
    interval: Interval<Normalized>,
}

/// Imprints that together close on a face's periodic quotient.
///
/// Such a chain bounds no island: in parameter space it is a line one period
/// long, not a closed polygon, so it cuts a ring face in two rather than
/// carving a hole out of one.
pub(crate) struct WrappingChain {
    /// The axis the chain spans exactly one period of.
    axis: Axis2,
    links: Vec<ChainLink>,
}

impl WrappingChain {
    /// The imprints in travel order.
    fn imprints(&self) -> Vec<FaceImprint> {
        self.links.iter().map(|link| link.imprint.clone()).collect()
    }

    /// How far the chain travels along its axis, signed.
    fn travel(&self) -> f64 {
        travel_along(self.axis, &self.imprints())
    }
}

/// Signed travel of a chain of imprints along one parameter axis.
pub(crate) fn travel_along(axis: Axis2, imprints: &[FaceImprint]) -> f64 {
    imprints
        .iter()
        .map(|imprint| {
            axis.of(imprint.pcurve.point_at(Fraction::new(1.0)))
                - axis.of(imprint.pcurve.point_at(Fraction::new(0.0)))
        })
        .sum()
}

/// Cuts a ring face by every imprint chain that wraps its periodic direction.
///
/// Returns no splits — leaving the imprints to the planar paths — unless the
/// face is a ring bounded by exactly two wrapping loops and *every* imprint
/// belongs to a chain spanning one whole period of the wrapped axis. A chain
/// that does not wrap is a chord or an island, which those paths already know
/// how to apply.
pub(crate) fn split_ring_face_by_wrapping_chains<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    imprints: &[FaceImprint],
) -> Result<Vec<FaceImprintSplit>, FaceImprintSplitError> {
    let Some(chains) = wrapping_chains(edit, face, imprints)? else {
        return Ok(Vec::new());
    };
    // A face with no loops at all is cut into two caps rather than two rings:
    // each half is bounded by one copy of the chain and closed on its far side
    // by the degeneracy there. It has no existing loop to pair a copy with, so
    // only the first chain can be taken this way — after it there are caps, not
    // a boundaryless face.
    if edit
        .face_attr(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?
        .is_boundaryless()
    {
        let [chain] = &chains[..] else {
            return Ok(Vec::new());
        };
        return Ok(
            split_boundaryless_face_by_wrapping_chain(edit, face, chain)?
                .into_iter()
                .collect(),
        );
    }
    let mut rings = vec![face];
    let mut splits = Vec::new();
    for chain in chains {
        let Some(target) = ring_face_for_chain(edit, &rings, &chain)? else {
            return Ok(Vec::new());
        };
        let split = split_ring_face_by_wrapping_chain(edit, target, &chain)?;
        rings.push(split.second);
        splits.push(split);
    }
    Ok(splits)
}

/// Cuts a boundaryless face in two along a chain that wraps a periodic
/// direction.
///
/// A sphere cut by a plane: the chain is a latitude circle, and each half is a
/// cap — bounded by one copy of it and closed on its far side by a pole. Which
/// copy bounds which half follows from direction alone, and needs no sampling: a
/// face's interior lies to the left of its boundary, so the copy travelling
/// forward along the wrapped axis bounds the half above it and the reversed copy
/// bounds the half below.
///
/// Returns `None` when the support names no degeneracy on one of the two sides,
/// which would leave a half nothing closes.
pub(crate) fn split_boundaryless_face_by_wrapping_chain<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    chain: &WrappingChain,
) -> Result<Option<FaceImprintSplit>, FaceImprintSplitError> {
    let old_face = edit
        .face_attr(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?
        .clone();
    let axis = chain.axis;
    let transverse = axis.transverse();
    let rows = old_face.surface.degenerate_rows(transverse);
    let at = chain
        .links
        .first()
        .map(|link| transverse.of(link.imprint.pcurve.point_at(Fraction::new(0.0))))
        .ok_or(FaceImprintSplitError::MissingFace { face })?;
    if [DomainSide::Low, DomainSide::High]
        .into_iter()
        .any(|side| side.nearest(at, rows.iter().copied()).is_none())
    {
        return Ok(None);
    }

    let forward = chain.imprints();
    let backward = reversed_imprint_loop(&forward)?;
    let forward_loop = add_section_loop(edit, face, &old_face.surface, &forward);
    let backward_loop = add_section_loop(edit, face, &old_face.surface, &backward);
    let section_edges = sew_section_loops(edit, face, &forward_loop, &backward_loop)?;
    for loop_ in [&forward_loop, &backward_loop] {
        edit.add_profile_derived_from(vec![EditKey::Face(face)], ProfileAttr::new(loop_.loop_dart));
    }

    // The forward copy travels the chain's own direction; the reversed one
    // travels against it.
    let (high, low) = if chain.travel() > 0.0 {
        (&forward_loop, &backward_loop)
    } else {
        (&backward_loop, &forward_loop)
    };
    let cap = |loop_: &SectionLoop, side: DomainSide| {
        vec![LoopDefinition::from_kind(
            loop_.loop_dart,
            LoopKind::Capping { axis, side },
        )]
    };

    let face_attr = edit
        .face_attr_mut(face)
        .expect("source face must remain staged during a cap split");
    let anchor = face_attr.seed();
    face_attr.set_boundary(cap(high, DomainSide::High), anchor);
    face_attr.pcurves = high.pcurves.clone();

    let second = edit.add_face_split_from(
        face,
        FaceAttr::with_loops(
            old_face.surface,
            cap(low, DomainSide::Low),
            low.pcurves.clone(),
        ),
    );

    Ok(Some(FaceImprintSplit {
        first: face,
        second,
        sections: section_edges
            .into_iter()
            .zip(&chain.links)
            .map(|(edge, link)| FaceImprintSection {
                edge,
                imprint: link.source,
                interval: link.interval,
            })
            .collect(),
    }))
}

/// Reads the imprints as chains wrapping `face`'s periodic direction, if they all are.
pub(crate) fn wrapping_chains<P: Payload>(
    edit: &ModelEdit<'_, P>,
    face: FaceKey,
    imprints: &[FaceImprint],
) -> Result<Option<Vec<WrappingChain>>, FaceImprintSplitError> {
    if imprints.is_empty() {
        return Ok(None);
    }
    let attr = edit
        .face_attr(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?;
    let periods = match attr.surface.periodicity() {
        SurfacePeriodicity::None => return Ok(None),
        SurfacePeriodicity::UPeriodic(u) => [Some(u), None],
        SurfacePeriodicity::VPeriodic(v) => [None, Some(v)],
        SurfacePeriodicity::UVPeriodic(u, v) => [Some(u), Some(v)],
    };
    // A ring names the axis through the loops that wrap it. A boundaryless face
    // has no loops to name one, so every periodic axis is a candidate and the
    // chain's own travel picks the one it spans.
    let candidates = match attr.wrapping().collect::<Vec<_>>()[..] {
        [(_, axis), (_, second_axis)] if axis == second_axis && attr.loops().len() == 2 => {
            vec![axis]
        }
        [] if attr.is_boundaryless() => Axis2::ALL.into_iter().collect(),
        _ => return Ok(None),
    };

    let Some(links) = chain_imprints(imprints) else {
        return Ok(None);
    };
    for axis in candidates {
        let Some(period) = periods[axis.index()] else {
            continue;
        };
        let chains = links
            .iter()
            .cloned()
            .map(|links| WrappingChain { axis, links })
            .collect::<Vec<_>>();
        if chains
            .iter()
            .all(|chain| (chain.travel().abs() - period).abs() <= LINEAR_TOLERANCE)
        {
            return Ok(Some(chains));
        }
    }
    Ok(None)
}

/// Joins imprints end to end into walks, reversing any written backwards.
///
/// Returns `None` when an imprint joins nothing, which is the normal answer
/// for a chord ending on the face's own boundary.
pub(crate) fn chain_imprints(imprints: &[FaceImprint]) -> Option<Vec<Vec<ChainLink>>> {
    let mut remaining = (0..imprints.len()).collect::<Vec<_>>();
    let mut chains = Vec::new();
    while !remaining.is_empty() {
        let source = remaining.remove(0);
        let mut links = vec![ChainLink {
            imprint: imprints[source].clone(),
            source,
            interval: Interval::UNIT,
        }];
        loop {
            let end = links.last()?.imprint.pcurve.point_at(Fraction::new(1.0));
            let meets = |index: &usize| {
                let pcurve = &imprints[*index].pcurve;
                [
                    pcurve.point_at(Fraction::new(0.0)),
                    pcurve.point_at(Fraction::new(1.0)),
                ]
                .iter()
                .any(|point| (point - end).norm() <= LINEAR_TOLERANCE)
            };
            let Some(position) = remaining.iter().position(meets) else {
                break;
            };
            let index = remaining.remove(position);
            let backwards = (imprints[index].pcurve.point_at(Fraction::new(0.0)) - end).norm()
                > LINEAR_TOLERANCE;
            links.push(ChainLink {
                imprint: if backwards {
                    imprints[index].reversed().ok()?
                } else {
                    imprints[index].clone()
                },
                source: index,
                interval: if backwards {
                    Interval::new(1.0, 0.0)
                } else {
                    Interval::new(0.0, 1.0)
                },
            });
        }
        chains.push(links);
    }
    Some(chains)
}

/// The ring among `rings` that `chain` runs across.
///
/// A wrapping chain spans its axis entirely, so what places it is the axis it
/// is transverse to: the chain lies on the ring whose two wrapping loops it
/// runs between.
pub(crate) fn ring_face_for_chain<P: Payload>(
    edit: &ModelEdit<'_, P>,
    rings: &[FaceKey],
    chain: &WrappingChain,
) -> Result<Option<FaceKey>, FaceImprintSplitError> {
    let transverse = chain.axis.transverse();
    let at = |pcurve: &TrimmedCurve2| transverse.of(pcurve.point_at(Fraction::new(0.5)));
    let position = chain
        .links
        .iter()
        .map(|link| at(&link.imprint.pcurve))
        .sum::<f64>()
        / chain.links.len() as f64;
    for &face in rings {
        let attr = edit
            .face_attr(face)
            .ok_or(FaceImprintSplitError::MissingFace { face })?;
        let bounds = attr
            .wrapping()
            .filter_map(|(seed, _)| attr.pcurves.get(&seed).map(at))
            .collect::<Vec<_>>();
        let [low, high] = bounds[..] else {
            continue;
        };
        let (low, high) = (low.min(high), low.max(high));
        if position > low + LINEAR_TOLERANCE && position < high - LINEAR_TOLERANCE {
            return Ok(Some(face));
        }
    }
    Ok(None)
}

/// Cuts a ring face in two along a chain that wraps its periodic direction.
///
/// Each half keeps one of the original wrapping loops and gains one copy of the
/// chain. Which copy goes where follows from direction alone: a face's boundary
/// runs one way round, so the new loop bounding a half must travel against the
/// original loop it is paired with. No seam anchors anything, and no vertex is
/// created beyond the chain's own junctions.
pub(crate) fn split_ring_face_by_wrapping_chain<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    chain: &WrappingChain,
) -> Result<FaceImprintSplit, FaceImprintSplitError> {
    let old_face = edit
        .face_attr(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?
        .clone();
    let axis = chain.axis;
    let forward = chain.imprints();
    let backward = reversed_imprint_loop(&forward)?;

    let forward_loop = add_section_loop(edit, face, &old_face.surface, &forward);
    let backward_loop = add_section_loop(edit, face, &old_face.surface, &backward);
    let section_edges = sew_section_loops(edit, face, &forward_loop, &backward_loop)?;
    edit.add_profile_derived_from(
        vec![EditKey::Face(face)],
        ProfileAttr::new(forward_loop.loop_dart),
    );
    edit.add_profile_derived_from(
        vec![EditKey::Face(face)],
        ProfileAttr::new(backward_loop.loop_dart),
    );

    let seeds = old_face.wrapping().collect::<Vec<_>>();
    let [(first_seed, _), (second_seed, _)] = seeds[..] else {
        return Err(FaceImprintSplitError::MissingFace { face });
    };
    // The half keeping `first_seed` is bounded by whichever copy runs against it.
    let first_travel = loop_travel(edit, &old_face.pcurves, first_seed, axis)?;
    let (first_new, second_new) = if first_travel * chain.travel() < 0.0 {
        (&forward_loop, &backward_loop)
    } else {
        (&backward_loop, &forward_loop)
    };

    let mut first_pcurves = wrapping_loop_pcurves(edit, face, &old_face.pcurves, first_seed)?;
    first_pcurves.extend(first_new.pcurves.clone());
    let mut second_pcurves = wrapping_loop_pcurves(edit, face, &old_face.pcurves, second_seed)?;
    second_pcurves.extend(second_new.pcurves.clone());
    let ring = |seed: Dart, added: Dart| {
        vec![
            LoopDefinition::from_kind(seed, LoopKind::Wrapping { axis }),
            LoopDefinition::from_kind(added, LoopKind::Wrapping { axis }),
        ]
    };
    let second_boundary = ring(second_seed, second_new.loop_dart);
    let first_boundary = ring(first_seed, first_new.loop_dart);

    let boundary = edit
        .face_unchecked(face)
        .loop_from_seed(second_seed)
        .starting_at(second_seed)
        .ok_or(FaceImprintSplitError::SeedNotOnBoundary {
            face,
            dart: second_seed,
        })?;
    let attachment = CutAttachment::on_loop(edit, &boundary).ok_or(
        FaceImprintSplitError::RingCutAttachment {
            face,
            dart: second_seed,
        },
    )?;
    attachment.move_after(edit, first_new.loop_dart)?;

    let face_attr = edit
        .face_attr_mut(face)
        .expect("source face must remain staged during a ring split");
    let anchor = face_attr.seed();
    face_attr.set_boundary(first_boundary, anchor);
    face_attr.pcurves = first_pcurves;

    let second = edit.add_face_split_from(
        face,
        FaceAttr::with_loops(old_face.surface, second_boundary, second_pcurves),
    );
    cut_between_loops(edit, second, second_seed, second_new.loop_dart)?;

    Ok(FaceImprintSplit {
        first: face,
        second,
        sections: section_edges
            .into_iter()
            .zip(&chain.links)
            .map(|(edge, link)| FaceImprintSection {
                edge,
                imprint: link.source,
                interval: link.interval,
            })
            .collect(),
    })
}

/// Signed travel of one stored boundary loop along `axis`.
pub(crate) fn loop_travel<P: Payload>(
    edit: &ModelEdit<'_, P>,
    pcurves: &HashMap<Dart, TrimmedCurve2>,
    seed: Dart,
    axis: Axis2,
) -> Result<f64, FaceImprintSplitError> {
    let profile = Profile::from_dart(edit, seed).expect("face loop must have a registered profile");
    Ok(profile
        .darts()
        .step_by(2)
        .filter_map(|dart| pcurves.get(&dart))
        .map(|pcurve| {
            axis.of(pcurve.point_at(Fraction::new(1.0)))
                - axis.of(pcurve.point_at(Fraction::new(0.0)))
        })
        .sum())
}

/// What the two halves of a chord split bound, given what the chorded loop did.
///
/// Chording an outer loop splits a disk into two disks. Chording a wrapping loop
/// splits a ring into a ring and a disk: one half still walks a whole period of
/// the axis and the other closes back on itself, so the halves are told apart by
/// how far each travels — no tolerance to tune, since one runs a period and the
/// other runs nothing.
pub(crate) fn chord_loop_kinds<P: Payload>(
    edit: &ModelEdit<'_, P>,
    chorded: LoopKind,
    source: (Dart, &HashMap<Dart, TrimmedCurve2>),
    created: (Dart, &HashMap<Dart, TrimmedCurve2>),
) -> Result<(LoopKind, LoopKind), FaceImprintSplitError> {
    let Some(axis) = chorded.wrapped_axis() else {
        return Ok((chorded, chorded));
    };
    let source_travel = loop_travel(edit, source.1, source.0, axis)?.abs();
    let created_travel = loop_travel(edit, created.1, created.0, axis)?.abs();
    Ok(if source_travel > created_travel {
        (LoopKind::Wrapping { axis }, LoopKind::Outer)
    } else {
        (LoopKind::Outer, LoopKind::Wrapping { axis })
    })
}

/// The stored pcurves of one boundary loop, keyed by its own darts.
pub(crate) fn wrapping_loop_pcurves<P: Payload>(
    edit: &ModelEdit<'_, P>,
    face: FaceKey,
    old_pcurves: &HashMap<Dart, TrimmedCurve2>,
    seed: Dart,
) -> Result<HashMap<Dart, TrimmedCurve2>, FaceImprintSplitError> {
    let profile = Profile::from_dart(edit, seed).expect("face loop must have a registered profile");
    profile
        .darts()
        .step_by(2)
        .map(|dart| {
            let candidates = [
                dart,
                edit.alpha(Dim::Zero, dart),
                edit.alpha(Dim::Two, dart),
            ];
            candidates
                .iter()
                .find_map(|d| old_pcurves.get(d))
                .cloned()
                .map(|pcurve| (dart, pcurve))
                .ok_or(FaceImprintSplitError::MissingPcurve { face, dart })
        })
        .collect()
}

pub(crate) fn split_face_by_closed_curve_imprint<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    imprint: &FaceImprint,
) -> Result<FaceImprintSplit, FaceImprintSplitError> {
    let old_face = edit
        .face_attr(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?
        .clone();
    let outside_loop = add_imprint_section_loop(edit, face, &old_face.surface, imprint);
    let island_loop =
        add_imprint_section_loop(edit, face, &old_face.surface, &reverse_imprint(imprint)?);
    finish_closed_imprint_split(edit, face, old_face, outside_loop, island_loop)
}

pub(crate) fn add_closed_imprint_loops<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    graph: &FaceImprintGraph,
    imprints: &[FaceImprint],
) -> Result<Vec<FaceImprintSplit>, FaceImprintSplitError> {
    let boundary_uvs = face_boundary_uvs(edit, face)?;
    // The winding every hole added here must turn against, read once before
    // any of them is added and while the face is still whole.
    let boundary_area = boundary_winding(edit, face);
    let mut splits = Vec::new();
    for component in graph.closed_edge_components() {
        let mut loop_imprints = component
            .iter()
            .map(|oriented| {
                let edge = &graph.edges[oriented.edge];
                let imprint = imprints[edge.source_curve].trimmed(edge.interval)?;
                if oriented.reversed {
                    imprint.reversed()
                } else {
                    Ok(imprint)
                }
            })
            .collect::<Result<Vec<_>, NurbsError>>()?;
        let uvs = loop_imprints
            .iter()
            .map(|imprint| imprint.pcurve.point_at(Fraction::new(0.0)))
            .collect::<Vec<_>>();
        if uvs.len() < 2
            || uvs
                .iter()
                .any(|uv| snap_boundary_corner(&boundary_uvs, *uv).is_some())
        {
            continue;
        }

        let mut provenance = component
            .iter()
            .map(|oriented| {
                let edge = &graph.edges[oriented.edge];
                let interval = if oriented.reversed {
                    Interval::new(edge.interval.end.value(), edge.interval.start.value())
                } else {
                    edge.interval
                };
                (edge.source_curve, interval)
            })
            .collect::<Vec<_>>();
        if orient_imprint_loop_against_boundary(boundary_area, &mut loop_imprints)? {
            provenance.reverse();
            for (_, interval) in &mut provenance {
                *interval = interval.reversed();
            }
        }
        let mut split = split_face_by_closed_imprint_loop(edit, face, &loop_imprints)?;
        for (section, (imprint, interval)) in split.sections.iter_mut().zip(provenance) {
            section.imprint = imprint;
            section.interval = interval;
        }
        splits.push(split);
    }

    Ok(splits)
}

pub(crate) fn split_face_by_closed_imprint_loop<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    imprints: &[FaceImprint],
) -> Result<FaceImprintSplit, FaceImprintSplitError> {
    let old_face = edit
        .face_attr(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?
        .clone();
    let island_imprints = reversed_imprint_loop(imprints)?;

    let outside_loop = add_section_loop(edit, face, &old_face.surface, imprints);
    let island_loop = add_section_loop(edit, face, &old_face.surface, &island_imprints);
    finish_closed_imprint_split(edit, face, old_face, outside_loop, island_loop)
}

pub(crate) fn finish_closed_imprint_split<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    old_face: FaceAttr<P::F>,
    outside_loop: SectionLoop,
    island_loop: SectionLoop,
) -> Result<FaceImprintSplit, FaceImprintSplitError> {
    let section_edges = sew_section_loops(edit, face, &outside_loop, &island_loop)?;
    edit.add_profile_derived_from(
        vec![EditKey::Face(face)],
        ProfileAttr::new(outside_loop.loop_dart),
    );
    edit.add_profile_derived_from(
        vec![EditKey::Face(face)],
        ProfileAttr::new(island_loop.loop_dart),
    );

    // The face the island sits in reaches it along a cut it owns. Without one
    // the new hole would sit in a 2-cell of its own, leaving the face two
    // disconnected pieces that only its loop list joined up.
    let reached_from = old_face.seed();
    cut_between_loops(edit, face, reached_from, outside_loop.loop_dart)?;

    let face_attr = edit
        .face_attr_mut(face)
        .expect("source face must remain staged during a closed-loop split");
    face_attr.push_inner(outside_loop.loop_dart);
    face_attr.pcurves.extend(outside_loop.pcurves);

    let second = edit.add_face_split_from(
        face,
        FaceAttr::with_pcurves(
            old_face.surface,
            island_loop.loop_dart,
            Vec::new(),
            island_loop.pcurves,
        ),
    );

    Ok(FaceImprintSplit {
        first: face,
        second,
        sections: section_edges
            .into_iter()
            .enumerate()
            .map(|(imprint, edge)| FaceImprintSection {
                edge,
                imprint,
                interval: Interval::UNIT,
            })
            .collect(),
    })
}

pub(crate) struct SectionLoop {
    loop_dart: Dart,
    edges: Vec<SectionLoopEdge>,
    pcurves: HashMap<Dart, TrimmedCurve2>,
}

#[derive(Clone)]
pub(crate) struct SectionLoopEdge {
    dart: Dart,
    start_uv: Point2,
    end_uv: Point2,
    curve: TrimmedCurve,
    pcurve: TrimmedCurve2,
}

pub(crate) fn add_section_loop<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    source_face: FaceKey,
    surface: &Surface,
    imprints: &[FaceImprint],
) -> SectionLoop {
    let n = imprints.len();
    let darts = (0..2 * n).map(|_| edit.add_dart()).collect::<Vec<_>>();

    for edge in 0..n {
        edit.link(Dim::Zero, darts[2 * edge], darts[2 * edge + 1])
            .expect("fresh section edge darts must be alpha0-free");
    }
    for edge in 0..n {
        let end = darts[2 * edge + 1];
        let next_start = darts[2 * ((edge + 1) % n)];
        edit.link(Dim::One, end, next_start)
            .expect("fresh section loop darts must be alpha1-free");
    }

    // A loop of one section closes on itself, so its single 0-cell is where
    // that section comes back round rather than a place two of them meet.
    // Nothing is bounded there, so no vertex is registered; `sew_section_loops`
    // classifies the 0-cell inside the edge instead, which is what leaves a
    // closed section an unmarked edge rather than a circle with an invented
    // corner.
    if n > 1 {
        for vertex in 0..n {
            let dart = edit.cell_representative(darts[2 * vertex], Dim::Zero);
            let uv = imprints[vertex].pcurve.point_at(Fraction::new(0.0));
            edit.add_vertex_derived_from(
                vec![EditKey::Face(source_face)],
                VertexAttr::new(dart, surface.point_at(uv.x, uv.y)),
            );
        }
    }

    let edges = (0..n)
        .map(|edge| {
            let imprint = &imprints[edge];
            SectionLoopEdge {
                dart: darts[2 * edge],
                start_uv: imprint.pcurve.point_at(Fraction::new(0.0)),
                end_uv: imprint.pcurve.point_at(Fraction::new(1.0)),
                curve: imprint.curve.clone(),
                pcurve: imprint.pcurve.clone(),
            }
        })
        .collect::<Vec<_>>();
    let pcurves = edges
        .iter()
        .map(|edge| (edge.dart, edge.pcurve.clone()))
        .collect();

    SectionLoop {
        loop_dart: darts[0],
        edges,
        pcurves,
    }
}

pub(crate) fn add_imprint_section_loop<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    source_face: FaceKey,
    surface: &Surface,
    imprint: &FaceImprint,
) -> SectionLoop {
    add_section_loop(edit, source_face, surface, std::slice::from_ref(imprint))
}

pub(crate) fn sew_section_loops<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    outside: &SectionLoop,
    island: &SectionLoop,
) -> Result<Vec<EdgeKey>, FaceImprintSplitError> {
    let pairs = outside
        .edges
        .iter()
        .map(|outside_edge| {
            let island_edge = matching_reversed_loop_edge(outside_edge, &island.edges).ok_or(
                FaceImprintSplitError::MissingPcurve {
                    face,
                    dart: outside_edge.dart,
                },
            )?;
            Ok((outside_edge, edit.alpha(Dim::Zero, island_edge.dart)))
        })
        .collect::<Result<Vec<_>, FaceImprintSplitError>>()?;
    // One pair is one section closing on itself, which `add_section_loop` left
    // without a vertex.
    let closes_on_itself = pairs.len() == 1;
    let mut edges = Vec::with_capacity(pairs.len());
    for (outside_edge, island_end) in pairs {
        edit.sew(Dim::Two, outside_edge.dart, island_end)
            .map_err(|source| FaceImprintSplitError::SectionLoopSewFailed { face, source })?;
        // The edge's default traversal follows the section, including its
        // directed source interval. Orient the support to that traversal before
        // discarding the span: a marked circle's corner cannot encode direction.
        let interval = outside_edge.curve.interval();
        let curve = if interval.end.value() < interval.start.value() {
            outside_edge.curve.curve().reversed()
        } else {
            outside_edge.curve.curve().clone()
        };
        let key = edit.add_edge_derived_from(
            vec![EditKey::Face(face)],
            EdgeAttr::new(outside_edge.dart, curve),
        );
        if closes_on_itself {
            edit.own_cell(Dim::Zero, outside_edge.dart, EntityOwner::Edge(key));
        }
        edges.push(key);
    }
    Ok(edges)
}

pub(crate) fn matching_reversed_loop_edge(
    edge: &SectionLoopEdge,
    candidates: &[SectionLoopEdge],
) -> Option<SectionLoopEdge> {
    candidates
        .iter()
        .find(|candidate| {
            (candidate.start_uv - edge.end_uv).norm() <= LINEAR_TOLERANCE
                && (candidate.end_uv - edge.start_uv).norm() <= LINEAR_TOLERANCE
        })
        .cloned()
}

/// Which way a face's boundary turns in its own parameter domain.
///
/// Read from the realized domain, never from the face's corners. A ring face's
/// loops each run exactly one period and carry no winding of their own, and
/// the corners they happen to have — the two ends of a chord imprinted in the
/// same pass, say — are not a polygon: an area taken over them has a sign
/// decided by how many corners some unrelated cut left on the rim, so one more
/// arriving flips it.
///
/// A face nothing encloses has no boundary to read and needs none: it covers
/// the support's whole domain as the support is parameterized, so
/// counter-clockwise is its material side, and the positive sense stands in.
pub(crate) fn boundary_winding<P: Payload>(edit: &ModelEdit<'_, P>, face: FaceKey) -> f64 {
    edit.face(face)
        .and_then(|view| view.boundary_signed_area())
        .filter(|area| area.abs() > LINEAR_TOLERANCE)
        .unwrap_or(1.0)
}

/// Winds an imprint loop so it cuts a hole rather than bounding one.
///
/// A face's material lies to the left of its boundary, so a hole runs the
/// opposite way round from whatever encloses the face, which is what
/// `boundary_area` states. Leaving that to whichever way the intersection
/// chain happened to be walked instead makes the face's winding depend on which
/// Boolean operand it belonged to — the same cut then sews up correctly one way
/// round and inside-out the other.
pub(crate) fn orient_imprint_loop_against_boundary(
    boundary_area: f64,
    imprints: &mut Vec<FaceImprint>,
) -> Result<bool, NurbsError> {
    let loop_uvs = imprints
        .iter()
        .flat_map(|imprint| imprint.pcurve.sample(16).into_iter().take(16))
        .collect::<Vec<_>>();
    let loop_area = signed_area(&loop_uvs);

    if loop_area.abs() <= LINEAR_TOLERANCE {
        return Ok(false);
    }
    if boundary_area.signum() == loop_area.signum() {
        *imprints = reversed_imprint_loop(imprints)?;
        return Ok(true);
    }
    Ok(false)
}

pub(crate) fn reversed_imprint_loop(
    imprints: &[FaceImprint],
) -> Result<Vec<FaceImprint>, NurbsError> {
    imprints.iter().rev().map(FaceImprint::reversed).collect()
}

pub(crate) fn signed_area(uvs: &[Point2]) -> f64 {
    if uvs.len() < 3 {
        return 0.0;
    }

    0.5 * uvs
        .iter()
        .zip(uvs.iter().cycle().skip(1))
        .take(uvs.len())
        .map(|(a, b)| a.x * b.y - b.x * a.y)
        .sum::<f64>()
}

/// Cuts the boundary where an imprint meets it, and says which corner that is.
///
/// The corner is returned rather than left to be found again: the caller has to
/// know which vertex this endpoint became, and asking the boundary afterwards
/// puts it back to comparing a projected corner with a traced endpoint.
pub(crate) fn split_boundary_at_uv<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    uv: Point2,
) -> Result<Option<VertexKey>, FaceImprintSplitError> {
    let boundary = face_boundary_edges(edit, face)?;
    let support = edit
        .face(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?
        .surface()
        .clone();
    if let Some(index) = snap_boundary_corner_on(&support, &boundary, uv) {
        return Ok(boundary[index].vertex);
    }

    let Some(target) = boundary_edge_at_uv(edit, face, uv)? else {
        return Ok(None);
    };

    let edge = Edge::new(edit, target.edge);
    let curve = edge_curve(&edge);
    let face_view = edit
        .face(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?;
    let surface = face_view.surface();
    let interval = edge.parameter_interval();
    let mut parameter = curve.parameter_at(surface.point_at(uv.x, uv.y));
    if let Periodicity::Periodic(period) = curve.periodicity() {
        let domain = interval.ordered();
        while parameter < NativeParam::new(domain.start.value() - LINEAR_TOLERANCE) {
            parameter += period;
        }
        while parameter > NativeParam::new(domain.end.value() + LINEAR_TOLERANCE) {
            parameter -= period;
        }
    }
    // The splitter cuts at a fraction of the edge's span, and the span that
    // fraction is of is the one the parameter was just brought onto.
    let parameter = interval.fraction_of(parameter);
    match split_face_edge_edit(edit, face, target.edge, parameter) {
        Ok(split) => Ok(Some(split.vertex())),
        // A cut that lands on an end of the edge adds no vertex because one is
        // already there, so the corner is the one the boundary now reports.
        Err(FaceEdgeSplitError::EdgeSplitFailed(EdgeSplitError::DegenerateSplit { .. })) => {
            let boundary = face_boundary_edges(edit, face)?;
            Ok(snap_boundary_corner_on(&support, &boundary, uv)
                .and_then(|index| boundary[index].vertex))
        }
        Err(error) => Err(error.into()),
    }
}

pub(crate) fn split_one_face_by_imprints<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    imprints: &[FaceImprint],
    placed: &[[Option<VertexKey>; 2]],
) -> Result<Option<FaceImprintSplit>, FaceImprintSplitError> {
    let face_attr = edit
        .face_attr(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?;

    let old_face = face_attr.clone();
    // A chord runs between two corners of one loop, so each bounding loop is
    // tried on its own: a ring has two, and the imprint chord lands on one.
    let bounding = bounding_loops(old_face.loops());
    for chorded in bounding {
        let boundary = loop_boundary_edges(edit, face, chorded.seed())?;
        let Some(cut) = FaceImprintCut::from_chain(imprints, &boundary, placed)? else {
            continue;
        };
        let split = apply_face_chord_split(edit, face, old_face, chorded, &cut)?;
        return Ok(Some(split));
    }
    Ok(None)
}

#[cfg(test)]
mod tests {
    use super::split_face_by_imprints_edit;
    use crate::builders::faces::{FaceImprint, add_face};
    use crate::builders::profiles::add_rectangle as add_rectangle_profile;
    use crate::builders::test_support::LineageRecorder;
    use crate::geometry::{Curve, Plane, Point2, Point3, TrimmedCurve2};
    use crate::model::Model;
    use crate::topology::ModelEditError;
    use crate::topology::edit::{EditKey, EditPolicy, Origin, PreservePayload};
    use crate::topology::payload::Payload;
    use crate::topology::shape_keys::{
        EdgeKey, FaceKey, ProfileKey, SheetKey, SolidKey, VertexKey,
    };

    #[derive(Clone, Default)]
    struct FacePayload;

    impl Payload for FacePayload {
        type V = ();
        type E = ();
        type Profile = String;
        type F = String;
        type Sheet = ();
        type S = ();

        type Policy = PreservePayload;
    }

    #[test]
    fn boundary_chord_split_preserves_source_face_and_applies_payload_policy() {
        let mut g = attributed_rectangle();
        let source = g.iter_faces().next().expect("face should exist").0;
        let source_profile = g
            .profile_key(g.face_unchecked(source).dart())
            .expect("source face should have a profile");
        let imprint = planar_line_imprint(Point2::new(0.0, 0.0), Point2::new(2.0, 2.0));
        let mut policy = LineageRecorder::<FacePayload>::default();

        let splits = g
            .transaction_with_policy(&mut policy, |edit| {
                split_face_by_imprints_edit(edit, source, &[imprint])
            })
            .expect("face imprint split should commit");

        assert_eq!(splits.len(), 1);
        assert_eq!(splits[0].first, source);
        assert_eq!(g.face_attr_unchecked(source).data(), "source");
        assert_eq!(g.face_attr_unchecked(splits[0].second).data(), "source");
        assert!(policy.created.contains(&(
            EditKey::Face(splits[0].second),
            Origin::Split(EditKey::Face(source)),
        )));
        let split_profile = policy
            .created
            .iter()
            .find_map(|(key, origin)| match (key, origin) {
                (EditKey::Profile(key), Origin::Split(EditKey::Profile(source)))
                    if *source == source_profile =>
                {
                    Some(*key)
                }
                _ => None,
            })
            .expect("profile split should be recorded");
        assert_eq!(
            g.profile_attr_unchecked(split_profile).data(),
            "source profile"
        );
    }

    #[test]
    fn closed_loop_split_declares_the_island_as_a_source_face_split() {
        let mut g = attributed_rectangle();
        let source = g.iter_faces().next().expect("face should exist").0;
        let points = [
            Point2::new(0.5, 0.5),
            Point2::new(1.5, 0.5),
            Point2::new(1.5, 1.5),
            Point2::new(0.5, 1.5),
            Point2::new(0.5, 0.5),
        ];
        let imprints = points
            .windows(2)
            .map(|pair| planar_line_imprint(pair[0], pair[1]))
            .collect::<Vec<_>>();
        let mut policy = LineageRecorder::<FacePayload>::default();

        let splits = g
            .transaction_with_policy(&mut policy, |edit| {
                split_face_by_imprints_edit(edit, source, &imprints)
            })
            .expect("closed face imprint split should commit");

        assert_eq!(splits.len(), 1);
        assert_eq!(splits[0].first, source);
        assert_eq!(g.face_attr_unchecked(source).data(), "source");
        assert_eq!(g.face_attr_unchecked(splits[0].second).data(), "source");
        assert!(policy.created.contains(&(
            EditKey::Face(splits[0].second),
            Origin::Split(EditKey::Face(source)),
        )));
    }

    #[test]
    fn late_face_policy_failure_restores_the_complete_source_face() {
        let mut g = attributed_rectangle();
        let source = g.iter_faces().next().expect("face should exist").0;
        let imprint = planar_line_imprint(Point2::new(0.0, 0.0), Point2::new(2.0, 2.0));
        let original_dart_count = g.dart_count();
        let mut policy = RejectFaceSplit;

        let result = g.transaction_with_policy(&mut policy, |edit| {
            split_face_by_imprints_edit(edit, source, &[imprint])
        });

        assert!(result.is_err());
        assert_eq!(g.dart_count(), original_dart_count);
        assert_eq!(g.iter_faces().count(), 1);
        assert_eq!(g.iter_edges().count(), 4);
        assert_eq!(g.face_attr_unchecked(source).data(), "source");
        assert_eq!(
            g.face_unchecked(source)
                .outer_loop()
                .expect("face should have an outer loop")
                .edges()
                .len(),
            4
        );
    }

    struct RejectFaceSplit;

    impl EditPolicy<FacePayload> for RejectFaceSplit {
        type Error = std::io::Error;

        fn vertex_created(
            &mut self,
            _key: VertexKey,
            _origin: Origin,
            _before: &Model<FacePayload>,
        ) -> Result<(), Self::Error> {
            Ok(())
        }

        fn edge_created(
            &mut self,
            _key: EdgeKey,
            _origin: Origin,
            _before: &Model<FacePayload>,
        ) -> Result<(), Self::Error> {
            Ok(())
        }

        fn profile_created(
            &mut self,
            _key: ProfileKey,
            _origin: Origin,
            _before: &Model<FacePayload>,
        ) -> Result<String, Self::Error> {
            Ok(String::new())
        }

        fn face_created(
            &mut self,
            _key: FaceKey,
            _origin: Origin,
            _before: &Model<FacePayload>,
        ) -> Result<String, Self::Error> {
            Err(std::io::Error::other("reject face split"))
        }

        fn sheet_created(
            &mut self,
            _key: SheetKey,
            _origin: Origin,
            _before: &Model<FacePayload>,
        ) -> Result<(), Self::Error> {
            Ok(())
        }

        fn solid_created(
            &mut self,
            _key: SolidKey,
            _origin: Origin,
            _before: &Model<FacePayload>,
        ) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    fn attributed_rectangle() -> Model<FacePayload> {
        let mut g = Model::new();
        let profile = add_rectangle_profile(&mut g, Plane::xy(), 2.0, 2.0)
            .expect("rectangle profile should build");
        let face = add_face(&mut g, profile).expect("rectangle face should build");
        g.transaction(|edit| {
            *edit.profile_attr_mut_unchecked(profile).data_mut() = "source profile".to_owned();
            *edit.face_attr_mut_unchecked(face).data_mut() = "source".to_owned();
            Ok::<_, ModelEditError>(())
        })
        .unwrap();
        g
    }

    fn planar_line_imprint(start: Point2, end: Point2) -> FaceImprint {
        FaceImprint::new(
            Curve::line(
                Point3::new(start.x, start.y, 0.0),
                Point3::new(end.x, end.y, 0.0),
            ),
            TrimmedCurve2::segment(start, end),
        )
    }
}
