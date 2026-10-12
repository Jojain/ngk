//! Regularized Booleans on faces that share one plane.
//!
//! Splitting replaces each operand with the faces the other operands'
//! boundaries cut it into, so after preparation the two overlapping squares
//! A=[0,2]x[0,2] and B=[1,3]x[1,3] are held as four faces: A's [1,2]x[1,2] and
//! L-shape, and B's own [1,2]x[1,2] and the other L-shape. The shared area is
//! there twice, once per operand.
//!
//! Two of the three operations never keep a tool's faces at all:
//!
//! - `a & b` is the faces of `a` that lie inside `b`;
//! - `a - b` is the faces of `a` that lie outside `b`.
//!
//! Neither keeps one face of `b`'s family, so neither meets the duplicate and
//! neither has anything to sew. Both are `remove_faces` over what selection
//! dropped — which includes every face of `b`, so `b` is gone and `a` is what
//! is left, exactly as its lineage says.
//!
//! `a + b` is the one that needs `b`'s faces: it is all of `a` plus the parts
//! of `b` outside it, and those parts arrive as faces of `b` that are not sewn
//! to anything of `a`'s. Joining them is [`sew_union_plan`] and [`weld`], which
//! weld each face of `b` onto the face of `a` across the section, and healing
//! then fuses the coplanar neighbours the weld left sharing a shape-free edge.
//! With several tools, each one's surviving area is welded onto whichever
//! operand's face meets it across a section.

use std::collections::{BTreeSet, HashMap, HashSet};

use crate::healing::{HealingOptions, HealingScope, remove_redundant_cells_edit};
use crate::model::Model;
use crate::topology::ModelEdit;
use crate::topology::payload::Payload;
use crate::topology::planar::Planar;
use crate::topology::shape_keys::{EdgeKey, FaceKey, VertexKey};

use super::arrangement::{self, Arrangement, section_edges};
use super::assemble::{section_pairs, sew_pair};
use super::neighborhood::FragmentGraph;
use super::operand::{Operand, Operands, operands_cells, reject_shared_faces};
use super::planar_domain::PlanarDomain;
use super::select::SelectionPlan;
use super::{
    BooleanDiagnostics, BooleanError, BooleanInput, BooleanLineage, BooleanOperand,
    BooleanOperation, BooleanOptions, BooleanTolerancePolicy, BooleanTolerances,
    IntersectionSpanId, classify, select,
};

/// One committed face Boolean.
#[derive(Debug)]
pub struct FaceBoolean {
    pub operation: BooleanOperation,
    /// The faces the answer is made of, all of them descended from the
    /// operands. Several where the answer falls in disconnected parts.
    pub faces: Vec<FaceKey>,
    /// Which source face each survivor came from, per input.
    pub lineage: FaceBooleanLineage,
    pub diagnostics: BooleanDiagnostics,
}

/// Which source face of each input the surviving faces came from.
///
/// Only a union keeps area from the tools, so under the other two operations
/// every tool's lineage is empty.
#[derive(Debug, Clone, Default)]
pub struct FaceBooleanLineage {
    pub target: BooleanLineage,
    /// One lineage per tool, in the order the tools were passed.
    pub tools: Vec<BooleanLineage>,
}

/// Evaluates one regularized Boolean of `target` with every one of `tools`,
/// all on one support surface, in one transaction.
///
/// The sets are the solid Boolean's, read as areas: `target − (t₁ ∪ … ∪ tₙ)`,
/// `target ∪ t₁ ∪ … ∪ tₙ` or `target ∩ t₁ ∩ … ∩ tₙ`, whatever order the tools
/// come in. Tools may overlap each other, and a key passed twice is one
/// operand; with no tools, `target` is returned as it is.
///
/// Every tool is consumed: each face descended from one is removed or, under a
/// union, welded into the answer. The answer keeps `target`'s key on one of its
/// faces, and every other face of the map keeps its key. Anything the
/// preparation cannot certify rolls the whole transaction back.
pub fn face_boolean<P: Payload>(
    map: &mut Model<P>,
    target: FaceKey,
    tools: &[FaceKey],
    operation: BooleanOperation,
    options: BooleanOptions,
) -> Result<FaceBoolean, BooleanError> {
    let operands = Operands::new(target, tools);
    let tolerances = admit(map, &operands, options)?;
    if operation == BooleanOperation::Difference && operands.target_is_a_tool() {
        return Err(BooleanError::EmptyResult);
    }
    if operands.operands.len() == 1 {
        let mut lineage = BooleanLineage::default();
        lineage.faces.insert(target, vec![target]);
        return Ok(FaceBoolean {
            operation,
            faces: vec![target],
            lineage: FaceBooleanLineage {
                tools: vec![lineage.clone(); tools.len()],
                target: lineage,
            },
            diagnostics: BooleanDiagnostics {
                tolerances,
                ..Default::default()
            },
        });
    }
    map.transaction(|edit| run(edit, &operands, operation, options, tolerances))
}

/// Checks every operand exists and shares the target's plane, and fixes the
/// budget.
///
/// Coplanarity is what makes the operands comparable at all: containment is
/// decided in one surface's parameter space, and a point of one face has no
/// place in another's unless the supports agree.
fn admit<P: Payload>(
    map: &Model<P>,
    operands: &Operands<FaceKey>,
    options: BooleanOptions,
) -> Result<BooleanTolerances, BooleanError> {
    let cells = operands_cells(map, operands, BooleanOperand::Face)?;
    reject_shared_faces(operands, &cells)?;
    let tolerances = BooleanTolerances::from_operands(map, &cells, options.tolerances)?;
    // `Planar` is the type that says "this view lies on this plane", so the
    // support is taken from it rather than read off the surface: a face whose
    // stored surface is a plane but whose boundary wanders off it is not a
    // planar operand, and only the check knows the difference.
    let plane = |face| Planar::new(map.face_unchecked(face)).map(|planar| planar.plane().clone());
    let target = operands.target();
    let differs = |input, face| BooleanError::OperandSupportsDiffer {
        input,
        target,
        face,
    };
    let reference = plane(target).map_err(|_| differs(BooleanInput::Target, target))?;
    for &Operand { key: face, input } in &operands.operands[1..] {
        let other = plane(face).map_err(|_| differs(input, face))?;
        let parallel = reference.normal().cross(&other.normal()).norm() <= tolerances.angular;
        let coincident = (other.origin() - reference.origin())
            .dot(&reference.normal())
            .abs()
            <= tolerances.linear;
        if !parallel || !coincident {
            return Err(differs(input, face));
        }
    }
    Ok(tolerances)
}

/// Prepares, classifies, selects, and removes everything selection dropped.
fn run<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    operands: &Operands<FaceKey>,
    operation: BooleanOperation,
    options: BooleanOptions,
    tolerances: BooleanTolerances,
) -> Result<FaceBoolean, BooleanError> {
    let mut options = options;
    tolerances.apply(&mut options.intersections);
    options.tolerances = BooleanTolerancePolicy::Fixed(tolerances);

    let target = operands.target();
    let faces = operands.map(BooleanOperand::Face);
    let arrangement = arrangement::arrange(edit, &faces, options, tolerances)?;
    let graph = FragmentGraph::<PlanarDomain>::build(&arrangement.lineages);
    let classes = classify::run(
        edit,
        &arrangement.lineages,
        &graph,
        operation,
        options,
        tolerances,
    )?;
    let selection = select::run(operation, faces.len(), &graph, &classes);
    if selection.kept.is_empty() {
        return Err(BooleanError::EmptyResult);
    }

    // Every tool face but those a union keeps outside the target is dropped,
    // so this removes the tools themselves along with the target's discarded
    // faces. A face selection kept still holds its own darts on the edges it
    // shared with a removed neighbour, so those edges survive as free boundary.
    // The splitter retains the source key on whichever half it built first,
    // which need not be a half this operation keeps. Putting it back on a
    // surviving face before anything is removed is what lets the caller go on
    // using the key it passed in.
    let mut selection = selection;
    if let Some(&heir) = selection.kept.first()
        && selection.dropped.contains(&target)
    {
        edit.swap_face_identities(target, heir);
        for face in selection
            .kept
            .iter_mut()
            .chain(selection.dropped.iter_mut())
        {
            *face = match *face {
                key if key == target => heir,
                key if key == heir => target,
                key => key,
            };
        }
    }
    // The seam is read off the faces as they stand, so it is found before
    // anything is removed and welded after -- exactly the order the solid
    // assembly uses, and for the same reason.
    let union = operation == BooleanOperation::Union;
    let seam = union
        .then(|| sew_union_plan(edit, &arrangement, &selection, tolerances))
        .transpose()?
        .unwrap_or_default();
    edit.remove_faces(&selection.dropped)?;
    let merges = weld(edit, seam, tolerances)?;
    let fused = union
        .then(|| fuse_seams(edit, &arrangement, &merges, tolerances))
        .transpose()?
        .unwrap_or_default();

    // A fused face carries on from both of its halves, so it inherits the
    // sources of the one it consumed. Tracking that here rather than in the
    // lineage maps keeps one entry per surviving face while it is being
    // rewritten, and the maps are rebuilt from it once nothing moves again.
    // Sources are tagged with their operand, because a face can carry on from
    // several operands at once.
    let mut sources = HashMap::<FaceKey, BTreeSet<(usize, FaceKey)>>::new();
    for (operand, lineage) in arrangement.lineages.iter().enumerate() {
        if operand == 0 || union {
            for (face, origins) in source_index(lineage) {
                let tagged = origins.into_iter().map(|source| (operand, source));
                sources.entry(face).or_default().extend(tagged);
            }
        }
    }
    let mut kept = selection.kept.clone();
    for (survivor, consumed) in fused {
        let inherited = sources.remove(&consumed).unwrap_or_default();
        sources.entry(survivor).or_default().extend(inherited);
        kept.retain(|face| *face != consumed);
        if !kept.contains(&survivor) {
            kept.push(survivor);
        }
    }
    kept.retain(|face| edit.face(*face).is_some());
    if kept.is_empty() {
        return Err(BooleanError::EmptyResult);
    }
    kept.sort();
    let (target, tools) = operands.per_input(lineages_of(&sources, &kept, faces.len()));
    Ok(FaceBoolean {
        operation,
        faces: kept,
        lineage: FaceBooleanLineage { target, tools },
        diagnostics: arrangement.diagnostics,
    })
}

/// Inverts one operand's lineage into fragment-to-sources.
///
/// The lineage is written source-first because that is how splitting produces
/// it; fusing rewrites it fragment-first, because a fused face gains a source
/// rather than a source gaining a face.
fn source_index(lineage: &BooleanLineage) -> HashMap<FaceKey, BTreeSet<FaceKey>> {
    let mut index: HashMap<FaceKey, BTreeSet<FaceKey>> = HashMap::new();
    for (&source, faces) in &lineage.faces {
        for &face in faces {
            index.entry(face).or_default().insert(source);
        }
    }
    index
}

/// Rebuilds one source-first lineage per operand over the faces that survived.
fn lineages_of(
    sources: &HashMap<FaceKey, BTreeSet<(usize, FaceKey)>>,
    kept: &[FaceKey],
    operands: usize,
) -> Vec<BooleanLineage> {
    let mut lineages = vec![BooleanLineage::default(); operands];
    for &face in kept {
        for &(operand, source) in sources.get(&face).into_iter().flatten() {
            lineages[operand]
                .faces
                .entry(source)
                .or_default()
                .push(face);
        }
    }
    lineages
}

/// Pairs each section's two edges with the face that still keeps them.
///
/// A union is the only operation that keeps faces from several operands, and
/// they arrive unjoined: where two boundaries coincide, each operand carries
/// its own edge. This finds those pairs while the faces are all still present;
/// the welding itself happens after the discarded faces are gone.
///
/// An edge bounding two kept faces is already interior to the answer and is
/// skipped, so only the seams between operands are paired.
fn sew_union_plan<P: Payload>(
    edit: &ModelEdit<'_, P>,
    arrangement: &Arrangement,
    selection: &SelectionPlan<PlanarDomain>,
    tolerances: BooleanTolerances,
) -> Result<Vec<SeamPair>, BooleanError> {
    let kept = selection.kept.iter().copied().collect::<HashSet<_>>();
    section_pairs(
        edit,
        &arrangement.sections,
        &kept,
        tolerances.linear,
        |span, a, b| SeamPair { span, a, b },
    )
}

/// Sews every paired seam edge, one dimension up from what it joins.
///
/// Welding two faces across an edge is `sew(Dim::Two, ..)`, which is the same
/// call the solid assembly makes; only what it joins differs.
fn weld<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    pairs: Vec<SeamPair>,
    tolerances: BooleanTolerances,
) -> Result<SeamMerges, BooleanError> {
    let mut merges = SeamMerges::default();
    for pair in pairs {
        sew_pair(
            edit,
            pair.span,
            pair.a,
            pair.b,
            tolerances.linear,
            &mut merges.edges,
            &mut merges.vertices,
        )?;
    }
    Ok(merges)
}

/// One section span's two edges, each with the face that kept it.
struct SeamPair {
    span: IntersectionSpanId,
    a: (EdgeKey, FaceKey),
    b: (EdgeKey, FaceKey),
}

/// The cell identities a seam consumed, survivor keyed by consumed.
#[derive(Default)]
struct SeamMerges {
    edges: HashMap<EdgeKey, EdgeKey>,
    vertices: HashMap<VertexKey, VertexKey>,
}

/// Fuses the coplanar neighbours a seam left sharing a shape-free edge.
///
/// After welding, the answer is held as several faces meeting along the section
/// and along the cuts the split made inside each operand. Those edges separate
/// faces on one plane and carry no shape, so healing removes them and the faces
/// become one. Scoping the run to the section's own cells keeps it proportional
/// to the operation rather than to the model.
fn fuse_seams<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    arrangement: &Arrangement,
    merges: &SeamMerges,
    tolerances: BooleanTolerances,
) -> Result<Vec<(FaceKey, FaceKey)>, BooleanError> {
    // Healing removes edges in the order it is offered them, so the sections
    // are walked in the order the passes made them.
    let mut offered = HashSet::new();
    let edges = section_edges(&arrangement.sections)
        .into_iter()
        .map(|mut edge| {
            while let Some(&survivor) = merges.edges.get(&edge) {
                edge = survivor;
            }
            edge
        })
        .filter(|&edge| edit.edge(edge).is_some() && offered.insert(edge))
        .collect::<Vec<_>>();
    let vertices = edges
        .iter()
        .filter_map(|&edge| edit.edge(edge))
        .flat_map(|edge| edge.vertices())
        .map(|vertex| vertex.key())
        .collect::<Vec<_>>();
    let report = remove_redundant_cells_edit(
        edit,
        &HealingOptions {
            scope: HealingScope::Cells { vertices, edges },
            linear_tolerance: tolerances.linear,
            angular_tolerance: tolerances.angular,
            ..HealingOptions::default()
        },
    )?;
    Ok(report.fused_faces)
}
