//! Imprints every operand of one Boolean on every other.
//!
//! A Boolean of a target and its tools is decided fragment by fragment, and a
//! fragment can only be kept or dropped whole if no operand's boundary crosses
//! it. So before anything is classified, every pair of operands -- tool against
//! tool included -- is intersected and split against each other, one pair at a
//! time, inside the one transaction the whole Boolean runs in.
//!
//! Splitting changes where an operand's cells are, never what they cover: each
//! operand is still a valid solid or face after a pass, and the next pass reads
//! it as one. A point where three operands meet is therefore cut into every
//! boundary through it, by the passes of the pairs that cross there.
//!
//! Each pass reports its section edges per span. A later pass may split those
//! edges again where a third operand crosses the section, so every recorded
//! section is refined through each later pass's lineage and ends up holding
//! the edges that are actually left.

use std::collections::{HashMap, HashSet};

use slotmap::Key;

use super::broad_phase::Bounds;
use super::operand::{Operand, OperandCells};
use super::operations::{apply_boolean_splits_edit, compute_cell_intersections, lineage_for};
use super::{
    BooleanDiagnostics, BooleanError, BooleanLineage, BooleanOperand, BooleanOptions,
    BooleanSection, BooleanTolerances, operand_cells, validate_solid_network,
};
use crate::topology::ModelEdit;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::EdgeKey;

/// Every operand split against every other, and what the splitting left.
pub(crate) struct Arrangement {
    /// Per operand, each source cell and the fragments it was split into.
    pub(crate) lineages: Vec<BooleanLineage>,
    /// Every section of every pass, in pass order then span order.
    pub(crate) sections: Vec<BooleanSection>,
    pub(crate) diagnostics: BooleanDiagnostics,
}

/// Intersects and splits every pair of `operands` against each other.
///
/// `operands` is in index order, the target first. A pair whose certified
/// boxes do not meet cannot touch and is not intersected. Any failure in a
/// pass is reported against the two inputs it was imprinting.
pub(crate) fn arrange<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    operands: &[Operand<BooleanOperand>],
    options: BooleanOptions,
    tolerances: BooleanTolerances,
) -> Result<Arrangement, BooleanError> {
    let mut lineages = Vec::with_capacity(operands.len());
    let mut bounds = Vec::with_capacity(operands.len());
    for operand in operands {
        let cells = operand_cells(edit, operand.key)?;
        bounds.push(Bounds::of_faces(
            edit,
            cells.faces.iter().copied(),
            tolerances.bbox.max(tolerances.linear),
        ));
        lineages.push(lineage_for(&cells, &HashMap::new(), &HashMap::new()));
    }
    let mut arrangement = Arrangement {
        lineages,
        sections: Vec::new(),
        diagnostics: BooleanDiagnostics {
            tolerances,
            ..Default::default()
        },
    };
    for first in 0..operands.len() {
        for second in first + 1..operands.len() {
            if let (Some(a), Some(b)) = (bounds[first], bounds[second])
                && !a.overlaps(b)
            {
                continue;
            }
            imprint(edit, &mut arrangement, operands, [first, second], options).map_err(
                |source| BooleanError::Imprint {
                    inputs: [operands[first].input, operands[second].input],
                    source: Box::new(source),
                },
            )?;
        }
    }
    Ok(arrangement)
}

/// One pass: splits the two operands at `pair`'s indices against each other
/// and folds what it did into the arrangement.
fn imprint<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    arrangement: &mut Arrangement,
    operands: &[Operand<BooleanOperand>],
    pair: [usize; 2],
    options: BooleanOptions,
) -> Result<(), BooleanError> {
    let inputs = pair.map(|index| operands[index].input);
    let keys = pair.map(|index| operands[index].key);
    let cells = [
        current_cells(edit, keys[0], &arrangement.lineages[pair[0]])?,
        current_cells(edit, keys[1], &arrangement.lineages[pair[1]])?,
    ];
    let plan = compute_cell_intersections(edit, keys, cells, options)?;
    if !plan.diagnostics.coverage.is_empty()
        || plan.diagnostics.branches_uncertified > 0
        || !plan.diagnostics.unresolved_overlaps.is_empty()
    {
        return Err(BooleanError::IncompleteIntersections {
            diagnostics: Box::new(plan.diagnostics),
        });
    }
    if matches!(keys[0], BooleanOperand::Solid(_)) {
        validate_solid_network(edit, &plan.network, arrangement.diagnostics.tolerances)?;
    }
    let prepared = apply_boolean_splits_edit(edit, plan, false)?;
    let passes = [&prepared.first_lineage, &prepared.second_lineage];

    for (operand, pass) in pair.into_iter().zip(passes) {
        refine_lineage(&mut arrangement.lineages[operand], pass);
    }
    for section in &mut arrangement.sections {
        for (side, input) in section.inputs.iter().enumerate() {
            let Some(at) = inputs.iter().position(|pass_input| pass_input == input) else {
                continue;
            };
            section.edges[side] = refine(&section.edges[side], &passes[at].edges);
        }
    }

    let mut spans = prepared.span_edges.into_iter().collect::<Vec<_>>();
    spans.sort_by_key(|(span, _)| span.0);
    arrangement
        .sections
        .extend(spans.into_iter().map(|(span, edges)| BooleanSection {
            inputs,
            span,
            edges,
        }));
    merge_diagnostics(&mut arrangement.diagnostics, prepared.diagnostics);
    Ok(())
}

/// Every cell an operand is made of now.
///
/// A solid's registration reaches all of its faces however often they were
/// split. A face operand has no such registration: once a pass split it, its
/// key names one fragment, and the others are known only to its lineage.
fn current_cells<P: Payload>(
    edit: &ModelEdit<'_, P>,
    operand: BooleanOperand,
    lineage: &BooleanLineage,
) -> Result<OperandCells, BooleanError> {
    let BooleanOperand::Face(_) = operand else {
        return operand_cells(edit, operand);
    };
    let mut cells = OperandCells::default();
    let (mut vertices, mut edges) = (HashSet::new(), HashSet::new());
    for &face in lineage.faces.values().flatten() {
        let fragment = operand_cells(edit, BooleanOperand::Face(face))?;
        cells.faces.push(face);
        cells.edges.extend(
            fragment
                .edges
                .into_iter()
                .filter(|edge| edges.insert(*edge)),
        );
        cells.vertices.extend(
            fragment
                .vertices
                .into_iter()
                .filter(|vertex| vertices.insert(*vertex)),
        );
    }
    Ok(cells)
}

/// Carries an operand's lineage through one more pass.
///
/// The pass knew the operand's cells as they stood, which are this lineage's
/// fragments; each one is replaced by whatever the pass split it into.
fn refine_lineage(lineage: &mut BooleanLineage, pass: &BooleanLineage) {
    for fragments in lineage.edges.values_mut() {
        *fragments = refine(fragments, &pass.edges);
    }
    for fragments in lineage.faces.values_mut() {
        *fragments = refine(fragments, &pass.faces);
        fragments.sort_by_key(|key| key.data().as_ffi());
    }
}

fn refine<K: Copy + Eq + std::hash::Hash>(keys: &[K], pass: &HashMap<K, Vec<K>>) -> Vec<K> {
    keys.iter()
        .flat_map(|key| pass.get(key).cloned().unwrap_or_else(|| vec![*key]))
        .collect()
}

fn merge_diagnostics(total: &mut BooleanDiagnostics, pass: BooleanDiagnostics) {
    total.candidate_pairs_tested += pass.candidate_pairs_tested;
    total.candidate_pairs_pruned += pass.candidate_pairs_pruned;
    total.edge_face_pairs_tested += pass.edge_face_pairs_tested;
    total.edge_face_pairs_pruned += pass.edge_face_pairs_pruned;
    total.branches_uncertified += pass.branches_uncertified;
    total.unresolved_overlaps.extend(pass.unresolved_overlaps);
    total.coverage.extend(pass.coverage);
}

/// Every section edge any pass left, deduplicated, in section order.
pub(crate) fn section_edges(sections: &[BooleanSection]) -> Vec<EdgeKey> {
    let mut seen = HashSet::new();
    sections
        .iter()
        .flat_map(|section| section.edges.iter().flatten())
        .copied()
        .filter(|edge| seen.insert(*edge))
        .collect()
}
