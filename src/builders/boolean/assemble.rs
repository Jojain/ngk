//! Atomic deletion, canonical-span sewing, and result shell registration.

use super::{
    BooleanContext, BooleanError, BooleanPreparation, BooleanResult, BooleanResultLineage,
    BooleanSide, IntersectionSpanId, neighborhood::FragmentGraph, select::SelectionPlan,
};
use crate::builders::faces::reverse_face_winding;
use crate::builders::scaffold::cut_between_shells;
use crate::geometry::parameter::Fraction;
use crate::geometry::{Point3, PointCoincidence};
use crate::healing::{HealingOptions, HealingScope, remove_redundant_cells_staged};
use crate::model::Model;
use crate::topology::{
    ModelEdit,
    attributes::{SheetAttr, SolidAttr},
    closed::Closed,
    gmap::{Dart, Dim},
    payload::Payload,
    shape_keys::{EdgeKey, FaceKey, SolidKey, VertexKey},
    validation::{
        ModelValidationError, validate_gmap, validate_solid_manifold, validate_solid_orientation,
    },
};
use std::collections::{BTreeSet, HashMap, HashSet};

/// Forms result shells, requiring a complete positional pairing for every surviving span.
pub(crate) fn run<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    context: &BooleanContext,
    graph: &FragmentGraph,
    mut prepared: BooleanPreparation,
    selection: SelectionPlan,
) -> Result<BooleanResult, BooleanError> {
    if selection.kept.is_empty() {
        return Err(BooleanError::EmptyResult);
    }
    let kept = selection.kept.iter().copied().collect::<HashSet<_>>();
    let mut pairs = Vec::new();
    let mut spans = prepared.span_edges.iter().collect::<Vec<_>>();
    spans.sort_by_key(|(span, _)| span.0);
    for (&span, sides) in spans {
        // The one kept face is carried alongside its edge: it is what says
        // which way the fragment runs, and looking it up again after the
        // sewing has started would be asking a map that has moved on.
        let boundary = sides.each_ref().map(|side| {
            side.iter()
                .copied()
                .filter_map(|edge| {
                    let faces = edit.edge(edge)?.faces();
                    let mut kept_faces = faces.iter().filter(|face| kept.contains(&face.key()));
                    let face = kept_faces.next()?.key();
                    kept_faces.next().is_none().then_some((edge, face))
                })
                .collect::<Vec<_>>()
        });
        if boundary[0].len() != boundary[1].len() {
            return Err(BooleanError::NonIsomorphicSpanSubdivision {
                span,
                first: boundary[0].len(),
                second: boundary[1].len(),
            });
        }
        pairs.extend(
            boundary[0]
                .iter()
                .copied()
                .zip(boundary[1].iter().copied())
                .map(|(a, b)| (span, a, b)),
        );
    }
    for &face in &selection.reversed {
        reverse_face_winding(edit, face);
    }
    let sheets = [context.first, context.second]
        .into_iter()
        .flat_map(|solid| {
            edit.solid_unchecked(solid)
                .shells()
                .into_iter()
                .map(|shell| shell.key())
        })
        .collect::<BTreeSet<_>>();
    let data = edit.solid_unchecked(context.first).data().clone();
    for solid in [context.first, context.second] {
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
        let sheet = edit.add_sheet(SheetAttr::new(root, P::Sheet::default()));
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
    let solid =
        edit.add_solid_split_from(context.first, SolidAttr::new(data, outer[0], Some(inner)));
    cut_between_shells(edit, solid, &shell_roots)?;
    validate_gmap(edit.topology()).map_err(ModelValidationError::from)?;
    validate_solid_manifold(edit, solid)?;
    validate_solid_orientation(edit, solid)?;
    for lineage in [&mut prepared.first_lineage, &mut prepared.second_lineage] {
        for faces in lineage.faces.values_mut() {
            faces.clear();
        }
    }
    for fragment in &graph.fragments {
        if kept.contains(&fragment.face) {
            let lineage = match fragment.side {
                BooleanSide::First => &mut prepared.first_lineage,
                BooleanSide::Second => &mut prepared.second_lineage,
            };
            lineage
                .faces
                .entry(fragment.source_face)
                .or_default()
                .push(fragment.face);
        }
    }
    for lineage in [&mut prepared.first_lineage, &mut prepared.second_lineage] {
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
        heal_result(edit, context, solid, &mut prepared)?;
    }
    prepared.diagnostics.fragments = graph.fragments.len();
    prepared.diagnostics.components = graph.components.len();
    Ok(BooleanResult {
        operation: context.operation,
        solid,
        diagnostics: prepared.diagnostics,
        lineage: BooleanResultLineage {
            first: prepared.first_lineage,
            second: prepared.second_lineage,
            span_edges: prepared.span_edges,
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
    prepared: &mut BooleanPreparation,
) -> Result<(), BooleanError> {
    let report = remove_redundant_cells_staged(
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
    for lineage in [&mut prepared.first_lineage, &mut prepared.second_lineage] {
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
    for sides in prepared.span_edges.values_mut() {
        for side in sides.iter_mut() {
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
fn sew_pair<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    span: IntersectionSpanId,
    first: (EdgeKey, FaceKey),
    second: (EdgeKey, FaceKey),
    tolerance: f64,
    edge_merges: &mut HashMap<EdgeKey, EdgeKey>,
    vertex_merges: &mut HashMap<VertexKey, VertexKey>,
) -> Result<(), BooleanError> {
    let ((first, first_face), (second, second_face)) = (first, second);
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
    let keys = match traversed.bounded() {
        Some(bounded) => {
            let (start, end) = bounded.vertices();
            [Some(start.key()), Some(end.key())]
        }
        None => [None, None],
    };
    Some((dart, keys, samples))
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

/// Signed boundary integral for planar polygon loops, including concave loops and holes.
///
/// The integral is taken about an arbitrary reference point, which is read off
/// a boundary dart rather than off a corner: a disc bounded by one unmarked
/// circle has no vertex at all, and asking its vertices for one answers with
/// nothing.
fn signed_volume<P: Payload>(map: &Model<P>, faces: &[FaceKey]) -> f64 {
    let reference = map
        .point_at_dart(map.face_unchecked(faces[0]).dart())
        .expect("a face of a result shell sits on a boundary that has a position");
    let mut volume = 0.0;

    for &key in faces {
        let face = map.face_unchecked(key);

        if !matches!(face.surface(), crate::geometry::Surface::Plane(_))
            || face.edges().iter().any(|edge| {
                edge.curve()
                    .to_nurbs()
                    .is_ok_and(|curve| curve.degree().get() != 1)
            })
        {
            volume += face
                .signed_volume_contribution(reference)
                .unwrap_or(f64::NAN);
            continue;
        }

        for boundary in face.loops() {
            let points = boundary
                .edges()
                .iter()
                .map(|edge| edge.trimmed_curve().point_at(Fraction::new(0.0)))
                .collect::<Vec<Point3>>();

            for pair in points[1..].windows(2) {
                volume += (points[0] - reference)
                    .dot(&(pair[0] - reference).cross(&(pair[1] - reference)))
                    / 6.0;
            }
        }
    }

    volume
}
