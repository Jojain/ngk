//! Atomic deletion, canonical-span sewing, and result shell registration.

use super::{
    BooleanContext, BooleanError, BooleanSection, IntersectionSpanId, SolidBoolean,
    SolidBooleanLineage, arrangement::Arrangement, neighborhood::FragmentGraph,
    select::SelectionPlan, solid_domain::SolidDomain,
};
use crate::builders::edges::split_face_boundary_edge;
use crate::builders::faces::reverse_face_winding_edit;
use crate::builders::scaffold::cut_between_shells;
use crate::geometry::parameter::Fraction;
use crate::geometry::{Point3, PointCoincidence};
use crate::healing::{HealingOptions, HealingScope, remove_redundant_cells_edit};
use crate::model::Model;
use crate::topology::{
    EditKey, ModelEdit,
    attributes::{SheetAttr, SolidAttr},
    closed::Closed,
    edge::Edge,
    gmap::{Dart, Dim},
    payload::Payload,
    shape_keys::{EdgeKey, FaceKey, SolidKey, VertexKey},
    validation::{
        ModelValidationError, validate_gmap, validate_solid_manifold, validate_solid_orientation,
    },
};
use std::collections::{BTreeSet, HashMap, HashSet};

/// One section edge the result keeps, and the one kept face that runs along it.
pub(super) type SeamSide = (EdgeKey, FaceKey);

/// Pairs the two sides of every section the result keeps.
///
/// A section edge is on the result's boundary when exactly one kept face runs
/// along it; an edge between two kept faces is already interior to one
/// operand's surviving boundary, and one with none is gone. Both sides of a
/// section must keep the same number of edges, and each edge on one side is
/// sewn to the one on the other that covers the same piece of the section.
///
/// Partners are chosen within one section only, so geometry decides which
/// piece of one intersection curve meets which, never which curve: two
/// fragments are the same piece when their ends coincide, and when a closed
/// section was cut into pieces sharing both ends, when a point partway along
/// one lies on the other. An edge already paired by an earlier section, as
/// where three operands share one line, is not offered again.
///
/// The kept face is carried alongside its edge: it is what says which way the
/// fragment runs, and looking it up again after the sewing has started would
/// be asking a map that has moved on.
pub(super) fn section_pairs<P: Payload, F>(
    edit: &ModelEdit<'_, P>,
    sections: &[BooleanSection],
    kept: &HashSet<FaceKey>,
    tolerance: f64,
    mut pair: impl FnMut(IntersectionSpanId, SeamSide, SeamSide) -> F,
) -> Result<Vec<F>, BooleanError> {
    let mut claimed = HashSet::new();
    let mut pairs = Vec::new();
    for section in sections {
        let boundary = section.edges.each_ref().map(|side| {
            side.iter()
                .copied()
                .filter(|edge| !claimed.contains(edge))
                .filter_map(|edge| {
                    let faces = edit.edge(edge)?.faces();
                    let mut kept_faces = faces.iter().filter(|face| kept.contains(&face.key()));
                    let face = kept_faces.next()?.key();
                    kept_faces.next().is_none().then_some((edge, face))
                })
                .collect::<Vec<_>>()
        });
        let span = section.span;
        if boundary[0].len() != boundary[1].len() {
            return Err(BooleanError::NonIsomorphicSpanSubdivision {
                span,
                first: boundary[0].len(),
                second: boundary[1].len(),
            });
        }
        let mut partners = boundary[1].clone();
        for &a in &boundary[0] {
            let at = partner(edit, a.0, &partners, tolerance).ok_or(
                BooleanError::NonIsomorphicSpanSubdivision {
                    span,
                    first: boundary[0].len(),
                    second: boundary[1].len(),
                },
            )?;
            let b = partners.swap_remove(at);
            claimed.insert(a.0);
            claimed.insert(b.0);
            pairs.push(pair(span, a, b));
        }
    }
    Ok(pairs)
}

/// The index of the one candidate that covers the same piece of a section as
/// `edge`, if exactly one does.
fn partner<P: Payload>(
    edit: &ModelEdit<'_, P>,
    edge: EdgeKey,
    candidates: &[SeamSide],
    tolerance: f64,
) -> Option<usize> {
    let piece = edit.edge_unchecked(edge).trimmed_curve();
    let (start, end) = (piece.start(), piece.end());
    let closed = start.coincides(end, tolerance);
    let same_ends = |other: &crate::geometry::TrimmedCurve| {
        let (a, b) = (other.start(), other.end());
        if closed {
            a.coincides(b, tolerance)
        } else {
            (a.coincides(start, tolerance) && b.coincides(end, tolerance))
                || (a.coincides(end, tolerance) && b.coincides(start, tolerance))
        }
    };
    let matching = candidates
        .iter()
        .enumerate()
        .filter_map(|(index, &(candidate, _))| {
            let other = edit.edge_unchecked(candidate).trimmed_curve();
            same_ends(&other).then_some((index, other))
        })
        .collect::<Vec<_>>();
    match matching.as_slice() {
        [(index, _)] => Some(*index),
        [] => None,
        _ => {
            let middle = piece.point_at(Fraction::new(0.5));
            let mut on = matching
                .iter()
                .filter(|(_, other)| other.contains(middle, tolerance));
            let (index, _) = on.next()?;
            on.next().is_none().then_some(*index)
        }
    }
}

/// Forms result shells, requiring a complete pairing for every surviving section.
pub(crate) fn run<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    context: &BooleanContext,
    graph: &FragmentGraph<SolidDomain>,
    mut arrangement: Arrangement,
    selection: SelectionPlan<SolidDomain>,
) -> Result<SolidBoolean, BooleanError> {
    if selection.kept.is_empty() {
        return Err(BooleanError::EmptyResult);
    }
    let kept = selection.kept.iter().copied().collect::<HashSet<_>>();
    let pairs = section_pairs(
        edit,
        &arrangement.sections,
        &kept,
        context.tolerances.linear,
        |span, a, b| (span, a, b),
    )?;
    for &face in &selection.reversed {
        reverse_face_winding_edit(edit, face);
    }
    // Captured before any sheet is removed: a kept face still names the
    // original sheet it belonged to, and that lineage is what lets a result
    // sheet carry a payload forward instead of defaulting.
    let sheet_sources = selection
        .kept
        .iter()
        .filter_map(|&face| {
            let dart = edit.face_unchecked(face).dart();
            edit.sheet_key(dart).map(|sheet| (face, sheet))
        })
        .collect::<HashMap<_, _>>();
    let solids = context.operands.keys().collect::<Vec<_>>();
    let sheets = solids
        .iter()
        .flat_map(|&solid| {
            edit.solid_unchecked(solid)
                .shells()
                .into_iter()
                .map(|shell| shell.key())
        })
        .collect::<BTreeSet<_>>();
    // Every operand's shells are about to be torn apart and reassembled, so
    // none can stay staged until commit the way a `merge_*_into` survivor
    // would need to: `validate_required_domain_attributes` runs before
    // reconciliation and would reject a solid still pointing at shells the
    // rebuild below has stripped of their sheet registration. They are all
    // removed immediately instead, and the result solid names every one as
    // its `Derived` origin below, so a caller's policy sees where it came from.
    for &solid in &solids {
        edit.remove_solid(solid);
    }
    for sheet in sheets {
        edit.remove_sheet(sheet);
    }
    edit.remove_faces(&selection.dropped)?;
    let mut edge_merges = HashMap::new();
    let mut vertex_merges = HashMap::new();
    for (span, a, b) in pairs {
        sew_pair(
            edit,
            span,
            a,
            b,
            context.tolerances.linear,
            &mut edge_merges,
            &mut vertex_merges,
        )?;
    }
    let components = shell_components(edit, &selection.kept);
    let mut outer = Vec::new();
    let mut inner = Vec::new();
    for component in components {
        let root = edit.face_unchecked(component[0]).dart();
        // One of the shells that fed this component supplies the split
        // source when one is known; a component assembled purely from
        // healing or from a side with no prior sheet still gets a fresh one.
        let sheet = match sheet_sources.get(&component[0]) {
            Some(&source) => edit.add_sheet_split_from(source, SheetAttr::new(root)),
            None => edit.add_sheet(SheetAttr::new(root)),
        };
        if Closed::new(edit.sheet_unchecked(sheet)).is_none() {
            return Err(BooleanError::OpenResultShell { face: component[0] });
        }
        let volume = signed_volume(edit, &component);
        if volume.abs() <= context.tolerances.linear.powi(3) {
            return Err(BooleanError::DegenerateResultShell { face: component[0] });
        }
        if volume > 0.0 {
            outer.push(root);
        } else {
            inner.push(root);
        }
    }
    if outer.is_empty() {
        return Err(BooleanError::EmptyResult);
    }
    if outer.len() != 1 {
        return Err(BooleanError::DisconnectedResult {
            components: outer.len(),
        });
    }
    let mut shell_roots = Vec::with_capacity(1 + inner.len());
    shell_roots.push(outer[0]);
    shell_roots.extend(inner.iter().copied());
    // A same-kind, many-source `Derived` rather than a `Split`: the result is
    // not the target continuing, it is a new solid the whole operation
    // produced, and naming every operand is what lets a policy combine their
    // payloads instead of one arbitrarily standing in for all.
    let sources = solids.iter().map(|&solid| EditKey::Solid(solid)).collect();
    let solid = edit.add_solid_derived_from(sources, SolidAttr::new(outer[0], Some(inner)));
    cut_between_shells(edit, solid, &shell_roots)?;
    validate_gmap(edit.topology()).map_err(ModelValidationError::from)?;
    validate_solid_manifold(edit, solid)?;
    validate_solid_orientation(edit, solid)?;
    let lineages = &mut arrangement.lineages;
    for lineage in lineages.iter_mut() {
        for faces in lineage.faces.values_mut() {
            faces.clear();
        }
    }
    for fragment in &graph.fragments {
        if kept.contains(&fragment.fragment) {
            lineages[fragment.operand]
                .faces
                .entry(fragment.source)
                .or_default()
                .push(fragment.fragment);
        }
    }
    for lineage in lineages.iter_mut() {
        for edges in lineage.edges.values_mut() {
            remap_keys(edges, &edge_merges);
            edges.retain(|edge| edit.edge(*edge).is_some());
        }
        for vertices in lineage.vertices.values_mut() {
            remap_keys(vertices, &vertex_merges);
            vertices.retain(|vertex| edit.vertex(*vertex).is_some());
        }
    }
    if context.options.heal {
        heal_result(edit, context, solid, &mut arrangement)?;
    }
    let (target, tools) = context.operands.per_input(arrangement.lineages);
    Ok(SolidBoolean {
        operation: context.operation,
        solid,
        diagnostics: arrangement.diagnostics,
        lineage: SolidBooleanLineage {
            target,
            tools,
            sections: arrangement.sections,
            discarded_faces: selection.dropped,
        },
    })
}

/// Removes the redundant topology imprinting left in the result.
///
/// A contact that lands on geometry the result keeps splits an edge or a face
/// without changing its shape, so the fragments are fused back together here.
/// Healing runs on the Boolean's own tolerances, because sections fitted by the
/// intersection engine do not meet the kernel's default budget. Lineage is then
/// rewritten onto the surviving identities.
fn heal_result<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    context: &BooleanContext,
    solid: SolidKey,
    arrangement: &mut Arrangement,
) -> Result<(), BooleanError> {
    let report = remove_redundant_cells_edit(
        edit,
        &HealingOptions {
            scope: HealingScope::Solid(solid),
            linear_tolerance: context.tolerances.linear,
            angular_tolerance: context.tolerances.angular,
            ..HealingOptions::default()
        },
    )?;
    if report.is_empty() {
        return Ok(());
    }

    let faces = fusion_map(&report.fused_faces);
    let edges = fusion_map(&report.fused_edges);
    for lineage in &mut arrangement.lineages {
        for keys in lineage.faces.values_mut() {
            remap_keys(keys, &faces);
            keys.retain(|face| edit.face(*face).is_some());
        }
        for keys in lineage.edges.values_mut() {
            remap_keys(keys, &edges);
            keys.retain(|edge| edit.edge(*edge).is_some());
        }
        for keys in lineage.vertices.values_mut() {
            keys.retain(|vertex| edit.vertex(*vertex).is_some());
        }
    }
    for section in &mut arrangement.sections {
        for side in section.edges.iter_mut() {
            remap_keys(side, &edges);
            side.retain(|edge| edit.edge(*edge).is_some());
        }
    }

    validate_gmap(edit.topology()).map_err(ModelValidationError::from)?;
    validate_solid_manifold(edit, solid)?;
    validate_solid_orientation(edit, solid)?;
    Ok(())
}

/// Turns `(survivor, consumed)` fusions into the merge chain `remap_keys` wants.
fn fusion_map<K: Copy + Eq + std::hash::Hash>(fusions: &[(K, K)]) -> HashMap<K, K> {
    fusions
        .iter()
        .map(|&(survivor, consumed)| (consumed, survivor))
        .collect()
}

/// Applies merge chains to lineage; geometric proximity never chooses sewing partners.
fn remap_keys<K: Copy + Eq + std::hash::Hash + Ord>(keys: &mut Vec<K>, merges: &HashMap<K, K>) {
    for key in keys.iter_mut() {
        while let Some(&next) = merges.get(key) {
            *key = next;
        }
    }
    keys.sort_unstable();
    keys.dedup();
}

/// Aligns only the endpoints of an already identified canonical-span pair.
pub(super) fn sew_pair<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    span: IntersectionSpanId,
    first: (EdgeKey, FaceKey),
    second: (EdgeKey, FaceKey),
    tolerance: f64,
    edge_merges: &mut HashMap<EdgeKey, EdgeKey>,
    vertex_merges: &mut HashMap<VertexKey, VertexKey>,
) -> Result<(), BooleanError> {
    let ((first, first_face), (second, second_face)) = (first, second);
    mark_to_match(edit, first, second)?;
    // Which way each fragment runs is a question about the face that keeps it,
    // not about the edge: an edge's own reference dart is whichever one it was
    // built from, and a fragment copied in from the other operand's map need
    // not have been built from the same side as one made here. Reading the
    // direction off the edge instead of off the loop is then right exactly half
    // the time — the half where the two happen to agree — and sews the second
    // operand's faces on backwards whenever they disagree.
    let (da, av, [a0, a1, aq]) = loop_traversal(edit, first, first_face)
        .ok_or(BooleanError::SpanEndpointMismatch { span })?;
    let (mut db, mut bv, [b0, b1, bq]) = loop_traversal(edit, second, second_face)
        .ok_or(BooleanError::SpanEndpointMismatch { span })?;
    let reversed = if a0.coincides(a1, tolerance) {
        // A closed span: its ends coincide, so only a point partway along can
        // say whether the other side runs with it or against it.
        if !aq.coincides(b0, tolerance) && !b0.coincides(a0, tolerance) {
            return Err(BooleanError::SpanEndpointMismatch { span });
        }
        !aq.coincides(bq, tolerance)
    } else if a0.coincides(b1, tolerance) && a1.coincides(b0, tolerance) {
        true
    } else if a0.coincides(b0, tolerance) && a1.coincides(b1, tolerance) {
        false
    } else {
        return Err(BooleanError::SpanEndpointMismatch { span });
    };
    if reversed {
        db = edit.alpha(Dim::Zero, db);
        bv.swap(0, 1);
    }
    if !edit.is_free(da, Dim::Two) || !edit.is_free(db, Dim::Two) {
        return Err(BooleanError::SpanEndpointMismatch { span });
    }
    edit.sew(Dim::Two, da, db)?;
    edit.merge_edges_into(first, second);
    edge_merges.insert(second, first);
    for (a, b) in av.into_iter().zip(bv) {
        // Nothing to reconcile where an end carries no logical vertex: the two
        // sides meet along the edge, not at a corner either of them has.
        let (Some(mut a), Some(mut b)) = (a, b) else {
            continue;
        };
        while let Some(&next) = vertex_merges.get(&a) {
            a = next;
        }
        while let Some(&next) = vertex_merges.get(&b) {
            b = next;
        }
        if a != b {
            edit.merge_vertices_into(a, b);
            vertex_merges.insert(b, a);
        }
    }
    Ok(())
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
) -> Result<(), BooleanError> {
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

/// Discovers connected face sets using current typed incidence after all compaction/sewing.
fn shell_components<P: Payload>(map: &Model<P>, faces: &[FaceKey]) -> Vec<Vec<FaceKey>> {
    let mut remaining = faces.iter().copied().collect::<BTreeSet<_>>();
    let mut result = Vec::new();
    while let Some(&seed) = remaining.first() {
        let mut pending = vec![seed];
        let mut component = Vec::new();
        while let Some(face) = pending.pop() {
            if !remaining.remove(&face) {
                continue;
            }
            component.push(face);
            for edge in map.face_unchecked(face).edges() {
                pending.extend(edge.faces().into_iter().map(|face| face.key()));
            }
        }
        result.push(component);
    }
    result
}

/// Signed volume of an oriented face component during shell assembly.
fn signed_volume<P: Payload>(map: &Model<P>, faces: &[FaceKey]) -> f64 {
    crate::measure::signed_volume_for_faces(map, faces).unwrap_or(f64::NAN)
}
