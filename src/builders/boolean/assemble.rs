//! Atomic deletion, canonical-span sewing, and result shell registration.

use super::{
    BooleanContext, BooleanError, BooleanPreparation, BooleanResult, BooleanResultLineage,
    BooleanSide, IntersectionSpanId, neighborhood::FragmentGraph, select::SelectionPlan,
};
use crate::builders::faces::reverse_face_winding;
use crate::geometry::{Point3, PointCoincidence};
use crate::topology::{
    TopologyEdit,
    attributes::{SheetAttr, SolidAttr},
    closed::Closed,
    gmap::{Dim, GMap},
    payload::Payload,
    shape_keys::{EdgeKey, FaceKey, VertexKey},
    validation::{validate_gmap, validate_solid_manifold, validate_solid_orientation},
};
use std::collections::{BTreeSet, HashMap, HashSet};

/// Forms result shells, requiring a complete positional pairing for every surviving span.
pub(crate) fn run<P: Payload>(
    edit: &mut TopologyEdit<'_, P>,
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
        let boundary = sides.each_ref().map(|side| {
            side.iter()
                .copied()
                .filter(|edge| {
                    edit.edge(*edge).is_some_and(|edge| {
                        edge.faces()
                            .iter()
                            .filter(|face| kept.contains(&face.key()))
                            .count()
                            == 1
                    })
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
    let solid =
        edit.add_solid_split_from(context.first, SolidAttr::new(data, outer[0], Some(inner)));
    validate_gmap(edit)?;
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
    edit: &mut TopologyEdit<'_, P>,
    span: IntersectionSpanId,
    first: EdgeKey,
    second: EdgeKey,
    tolerance: f64,
    edge_merges: &mut HashMap<EdgeKey, EdgeKey>,
    vertex_merges: &mut HashMap<VertexKey, VertexKey>,
) -> Result<(), BooleanError> {
    let a = edit.edge_unchecked(first);
    let b = edit.edge_unchecked(second);
    let da = a.dart();
    let mut db = b.dart();
    let a0 = *a.start().point().expect("admitted geometry");
    let a1 = *a.end().point().expect("admitted geometry");
    let b0 = *b.start().point().expect("admitted geometry");
    let b1 = *b.end().point().expect("admitted geometry");
    let av = [a.start().key(), a.end().key()];
    let mut bv = [b.start().key(), b.end().key()];
    if a0.coincides(b1, tolerance) && a1.coincides(b0, tolerance) {
        db = edit.alpha(Dim::Zero, db);
        bv.swap(0, 1);
    } else if !a0.coincides(b0, tolerance) || !a1.coincides(b1, tolerance) {
        return Err(BooleanError::SpanEndpointMismatch { span });
    }
    if !edit.is_free(da, Dim::Two) || !edit.is_free(db, Dim::Two) {
        return Err(BooleanError::SpanEndpointMismatch { span });
    }
    edit.sew(Dim::Two, da, db)?;
    edit.merge_edges_into(first, second);
    edge_merges.insert(second, first);
    for (a, b) in av.into_iter().zip(bv) {
        let mut a = a;
        let mut b = b;
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

/// Discovers connected face sets using current typed incidence after all compaction/sewing.
fn shell_components<P: Payload>(map: &GMap<P>, faces: &[FaceKey]) -> Vec<Vec<FaceKey>> {
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
fn signed_volume<P: Payload>(map: &GMap<P>, faces: &[FaceKey]) -> f64 {
    let reference = *map.face_unchecked(faces[0]).vertices()[0]
        .point()
        .expect("admitted geometry");
    let mut volume = 0.0;
    for &key in faces {
        let face = map.face_unchecked(key);
        if !matches!(face.surface(), crate::geometry::Surface::Plane(_))
            || face.edges().iter().any(|edge| {
                edge.curve().is_some_and(|curve| {
                    curve
                        .to_nurbs()
                        .is_ok_and(|curve| curve.degree().get() != 1)
                })
            })
        {
            volume += face
                .signed_volume_contribution(reference)
                .unwrap_or(f64::NAN);
            continue;
        }
        for boundary in map.face_unchecked(key).loops() {
            let points = boundary
                .edges()
                .iter()
                .map(|edge| *edge.start().point().expect("admitted geometry"))
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
