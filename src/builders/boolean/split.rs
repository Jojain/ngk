//! Cutting a solid along a sheet into the parts on either side of it.
//!
//! A sheet bounds no region, but inside the solid it cuts, it separates two:
//! the part its faces' normals point into, the **front**, and the part behind
//! them, the **back**. Call the region behind the sheet `H`. Then the back is
//! `target ∩ H` and the front is `target − H`, and both are ordinary Booleans
//! once every fragment says where it lies against `H`:
//!
//! - a fragment of the sheet is located against the solid by the usual ray
//!   cast, and its own operand's membership -- outside in front, inside behind
//!   -- is exactly `H`'s;
//! - a fragment of the solid's boundary cannot be ray cast against an open
//!   sheet. Its side is read off the sections instead: along a section edge, a
//!   back fragment and the sheet fragment inside the solid run the edge in
//!   opposite directions, as two faces of one consistently wound shell do,
//!   and a front fragment runs it the same way. The side then spreads across
//!   every edge the sheet did not cut.
//!
//! Selection and assembly are the solid Boolean's, run once per part on its
//! own copy of the operands.

use std::collections::{HashMap, HashSet};

use nalgebra::Vector3;

use super::arrangement::{self, Arrangement};
use super::domain::{BooleanDomain, OperandClassifier, RelativeLocation};
use super::neighborhood::FragmentGraph;
use super::operand::BooleanContext;
use super::solid_domain::SolidDomain;
use super::{
    BooleanError, BooleanOperation, BooleanOptions, BooleanTolerances, BooleanTrace, assemble,
    select,
};
use crate::geometry::parameter::Fraction;
use crate::geometry::{Point3, PointCoincidence};
use crate::model::Model;
use crate::topology::ModelEdit;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey, SheetKey, SolidKey};

/// The two parts a solid was cut into.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SolidSplit {
    /// The part on the side the sheet's normals point to.
    pub front: SolidKey,
    /// The part behind the sheet.
    pub back: SolidKey,
}

/// One side of the sheet a solid is split along.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SplitSide {
    Front,
    Back,
}

/// Cuts `target` along `tool` into the parts in front of and behind it, in
/// one transaction.
///
/// The sheet must reach past the solid everywhere it crosses it: a sheet that
/// ends inside the solid leaves an open section, which is refused. Each part
/// must be one connected solid; a side falling in several pieces is refused as
/// [`BooleanError::DisconnectedResult`]. Both inputs are consumed, and every
/// other entity of the map keeps its key.
pub fn split_solid<P: Payload>(
    map: &mut Model<P>,
    target: SolidKey,
    tool: SheetKey,
    options: BooleanOptions,
) -> Result<SolidSplit, BooleanError> {
    BooleanContext::admit_split(map, target, tool, BooleanOperation::Difference, options)?;
    // Each part consumes its operands, so the front is cut from a copy.
    let source = map.clone();
    map.transaction(|edit| {
        let target_copy = edit.merge(source.solid_unchecked(target));
        let target_copy = edit
            .solid_key(target_copy)
            .expect("a copied solid keeps its registration");
        let tool_copy = edit.merge(source.sheet_unchecked(tool));
        let tool_copy = edit
            .sheet_key(tool_copy)
            .expect("a copied sheet keeps its registration");
        let back = split_side(edit, target, tool, SplitSide::Back, options)?;
        let front = split_side(edit, target_copy, tool_copy, SplitSide::Front, options)?;
        Ok(SolidSplit { front, back })
    })
}

/// Builds the part of `target` on one `side` of `tool`, consuming both.
fn split_side<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    target: SolidKey,
    tool: SheetKey,
    side: SplitSide,
    options: BooleanOptions,
) -> Result<SolidKey, BooleanError> {
    let operation = match side {
        SplitSide::Back => BooleanOperation::Intersection,
        SplitSide::Front => BooleanOperation::Difference,
    };
    let context = BooleanContext::admit_split(edit, target, tool, operation, options)?;
    let operands = context.operands.operands.clone();
    let arrangement = arrangement::arrange(
        edit,
        &operands,
        context.options,
        context.tolerances,
        &mut BooleanTrace::disabled(),
    )?;
    if arrangement.sections.is_empty() {
        return Err(BooleanError::SplitMissesTarget);
    }
    let graph = FragmentGraph::<SolidDomain>::build(&arrangement.lineages);
    let classes = locate(
        edit,
        &arrangement,
        &graph,
        context.options,
        context.tolerances,
    )?;
    let selection = select::run(operation, operands.len(), &graph, &classes);
    match assemble::run(edit, &context, &graph, arrangement, selection) {
        Ok(result) => Ok(result.solid),
        Err(BooleanError::EmptyResult) => Err(BooleanError::SplitSideEmpty { side }),
        Err(error) => Err(error),
    }
}

/// Where every fragment lies against the other operand: a sheet fragment
/// against the solid, a solid fragment against the region behind the sheet.
fn locate<P: Payload>(
    model: &Model<P>,
    arrangement: &Arrangement,
    graph: &FragmentGraph<SolidDomain>,
    options: BooleanOptions,
    tolerances: BooleanTolerances,
) -> Result<Vec<Vec<(usize, RelativeLocation)>>, BooleanError> {
    const TARGET: usize = 0;
    const TOOL: usize = 1;
    let solid = SolidDomain::classifier(model, &arrangement.lineages[TARGET], options, tolerances)?;
    let mut classes = vec![Vec::new(); graph.fragments.len()];
    let mut inside = HashSet::new();
    for (class, fragment) in classes.iter_mut().zip(&graph.fragments) {
        if fragment.operand != TOOL {
            continue;
        }
        let (point, normal) = SolidDomain::probe(model, fragment.fragment, tolerances)?;
        let location = solid.locate(point, normal, fragment.fragment)?;
        if location == RelativeLocation::Inside {
            inside.insert(fragment.fragment);
        }
        *class = vec![(TARGET, location)];
    }
    let sides = target_sides(model, arrangement, graph, &inside, tolerances.linear)?;
    for (class, fragment) in classes.iter_mut().zip(&graph.fragments) {
        if fragment.operand != TARGET {
            continue;
        }
        let location = match sides[&fragment.fragment] {
            SplitSide::Back => RelativeLocation::Inside,
            SplitSide::Front => RelativeLocation::Outside,
        };
        *class = vec![(TOOL, location)];
    }
    Ok(classes)
}

/// Which side of the sheet every fragment of the solid's boundary is on.
fn target_sides<P: Payload>(
    model: &Model<P>,
    arrangement: &Arrangement,
    graph: &FragmentGraph<SolidDomain>,
    inside: &HashSet<FaceKey>,
    tolerance: f64,
) -> Result<HashMap<FaceKey, SplitSide>, BooleanError> {
    let fragments = graph
        .fragments
        .iter()
        .filter(|fragment| fragment.operand == 0)
        .map(|fragment| fragment.fragment)
        .collect::<HashSet<_>>();
    let cut = arrangement
        .sections
        .iter()
        .flat_map(|section| section.edges[0].iter().copied())
        .collect::<HashSet<_>>();

    let mut sides = HashMap::new();
    let mut pending = Vec::new();
    for section in &arrangement.sections {
        for &edge in &section.edges[0] {
            let middle = edge_middle(model, edge);
            let (sheet_face, sheet_edge) = section.edges[1]
                .iter()
                .find_map(|&candidate| {
                    if !passes_through(model, candidate, middle, tolerance) {
                        return None;
                    }
                    model
                        .edge_unchecked(candidate)
                        .faces()
                        .into_iter()
                        .map(|face| face.key())
                        .find(|face| inside.contains(face))
                        .map(|face| (face, candidate))
                })
                .ok_or(BooleanError::SplitSectionUnpaired { edge })?;
            let sheet = run_direction(model, sheet_face, sheet_edge, middle)
                .ok_or(BooleanError::SplitSectionUnpaired { edge })?;
            for face in model.edge_unchecked(edge).faces() {
                let face = face.key();
                if !fragments.contains(&face) {
                    continue;
                }
                let own = run_direction(model, face, edge, middle)
                    .ok_or(BooleanError::SplitSectionUnpaired { edge })?;
                let side = if own.dot(&sheet) < 0.0 {
                    SplitSide::Back
                } else {
                    SplitSide::Front
                };
                assign(&mut sides, &mut pending, face, side)?;
            }
        }
    }

    while let Some(face) = pending.pop() {
        let side = sides[&face];
        for edge in model.face_unchecked(face).edges() {
            if cut.contains(&edge.key()) {
                continue;
            }
            for neighbour in edge.faces() {
                let neighbour = neighbour.key();
                if neighbour != face && fragments.contains(&neighbour) {
                    assign(&mut sides, &mut pending, neighbour, side)?;
                }
            }
        }
    }
    if let Some(&face) = fragments.iter().find(|face| !sides.contains_key(face)) {
        return Err(BooleanError::SplitBoundaryUncut { face });
    }
    Ok(sides)
}

/// Records `face`'s side, refusing a face both sides reach.
fn assign(
    sides: &mut HashMap<FaceKey, SplitSide>,
    pending: &mut Vec<FaceKey>,
    face: FaceKey,
    side: SplitSide,
) -> Result<(), BooleanError> {
    match sides.insert(face, side) {
        None => {
            pending.push(face);
            Ok(())
        }
        Some(previous) if previous == side => Ok(()),
        Some(_) => Err(BooleanError::SplitSideAmbiguous { face }),
    }
}

fn edge_middle<P: Payload>(model: &Model<P>, edge: EdgeKey) -> Point3 {
    model
        .edge_unchecked(edge)
        .trimmed_curve()
        .point_at(Fraction::new(0.5))
}

fn passes_through<P: Payload>(
    model: &Model<P>,
    edge: EdgeKey,
    point: Point3,
    tolerance: f64,
) -> bool {
    let curve = model.edge_unchecked(edge).trimmed_curve();
    curve
        .point_at(curve.parameter_at(point))
        .coincides(point, tolerance)
}

/// The direction `face`'s boundary runs along `edge` at `point`.
fn run_direction<P: Payload>(
    model: &Model<P>,
    face: FaceKey,
    edge: EdgeKey,
    point: Point3,
) -> Option<Vector3<f64>> {
    const STEP: f64 = 1e-3;
    let curve = model
        .face_unchecked(face)
        .loops()
        .into_iter()
        .flat_map(|boundary| boundary.edges())
        .find(|candidate| candidate.key() == edge)?
        .trimmed_curve();
    let at = curve.parameter_at(point).value();
    let before = curve.point_at(Fraction::new((at - STEP).max(0.0)));
    let after = curve.point_at(Fraction::new((at + STEP).min(1.0)));
    Some(after - before)
}
