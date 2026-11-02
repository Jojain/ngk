//! Atomic deletion, canonical-span sewing, and result shell registration.

use super::{
    BooleanContext, BooleanError, BooleanOperand, BooleanSection, IntersectionSpanId, SolidBoolean,
    SolidBooleanLineage, arrangement::Arrangement, neighborhood::FragmentGraph,
    select::SelectionPlan, solid_domain::SolidDomain,
};
use crate::builders::faces::{FaceSewError, SewMerges, reverse_face_winding_edit, sew_faces_along};
use crate::builders::scaffold::cut_between_shells;
use crate::geometry::PointCoincidence;
use crate::geometry::parameter::Fraction;
use crate::healing::{HealingOptions, HealingScope, remove_redundant_cells_edit};
use crate::model::Model;
use crate::topology::{
    EditKey, ModelEdit,
    attributes::{SheetAttr, SolidAttr},
    closed::Closed,
    payload::Payload,
    shape_keys::{EdgeKey, FaceKey, SolidKey},
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
    let solids = context
        .operands
        .keys()
        .filter_map(|operand| match operand {
            BooleanOperand::Solid(solid) => Some(solid),
            _ => None,
        })
        .collect::<Vec<_>>();
    let sheets = context
        .operands
        .keys()
        .flat_map(|operand| match operand {
            BooleanOperand::Solid(solid) => edit
                .solid_unchecked(solid)
                .shells()
                .into_iter()
                .map(|shell| shell.key())
                .collect(),
            BooleanOperand::Sheet(sheet) => vec![sheet],
            _ => Vec::new(),
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
    let mut merges = SewMerges::default();
    for (span, a, b) in pairs {
        sew_pair(edit, span, a, b, context.tolerances.linear, &mut merges)?;
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
            remap_keys(edges, &merges.edges);
            edges.retain(|edge| edit.edge(*edge).is_some());
        }
        for vertices in lineage.vertices.values_mut() {
            remap_keys(vertices, &merges.vertices);
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

/// Welds one canonical-span pair, its two sides already identified.
pub(super) fn sew_pair<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    span: IntersectionSpanId,
    first: (EdgeKey, FaceKey),
    second: (EdgeKey, FaceKey),
    tolerance: f64,
    merges: &mut SewMerges,
) -> Result<(), BooleanError> {
    match sew_faces_along(edit, first, second, tolerance, merges) {
        Ok(_) => Ok(()),
        Err(FaceSewError::Mismatch) => Err(BooleanError::SpanEndpointMismatch { span }),
        Err(FaceSewError::EdgeSplit(error)) => Err(error.into()),
        Err(FaceSewError::ModelEdit(error)) => Err(error.into()),
    }
}

/// Discovers connected face sets using current typed incidence after all compaction/sewing.
pub(super) fn shell_components<P: Payload>(map: &Model<P>, faces: &[FaceKey]) -> Vec<Vec<FaceKey>> {
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
