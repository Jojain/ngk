//! Regularized Booleans on two faces that share one plane.
//!
//! Splitting replaces each operand with the faces the other operand's boundary
//! cut it into, so after preparation the two overlapping squares A=[0,2]x[0,2]
//! and B=[1,3]x[1,3] are held as four faces: A's [1,2]x[1,2] and L-shape, and
//! B's own [1,2]x[1,2] and the other L-shape. The shared area is there twice,
//! once per operand.
//!
//! Two of the three operations never look at the second operand's faces at all:
//!
//! - `a & b` is the faces of `a` that lie inside `b`;
//! - `a - b` is the faces of `a` that lie outside `b`.
//!
//! Neither reads one face of `b`'s family, so neither meets the duplicate and
//! neither has anything to sew. Both are `remove_faces` over what the table
//! dropped — which includes every face of `b`, so `b` is gone and `a` is what
//! is left, exactly as its lineage says.
//!
//! `a + b` is the one that needs `b`'s faces: it is all of `a` plus the parts
//! of `b` outside it, and those parts arrive as faces of `b` that are not sewn
//! to anything of `a`'s. Joining them is [`sew_union`], which welds each face
//! of `b` onto the face of `a` across the section, and healing then fuses the
//! coplanar neighbours the weld left sharing a shape-free edge.

use std::collections::{BTreeSet, HashMap, HashSet};

use crate::healing::{HealingOptions, HealingScope, remove_redundant_cells_edit};
use crate::model::Model;
use crate::topology::ModelEdit;
use crate::topology::payload::Payload;
use crate::topology::planar::Planar;
use crate::topology::shape_keys::{EdgeKey, FaceKey, VertexKey};

use super::assemble::sew_pair;
use super::neighborhood::FragmentGraph;
use super::operations::apply_boolean_splits_edit;
use super::planar_domain::PlanarDomain;
use super::select::SelectionPlan;
use super::{
    BooleanDiagnostics, BooleanError, BooleanLineage, BooleanOperand, BooleanOperandPreparation,
    BooleanOperation, BooleanOptions, BooleanTolerancePolicy, BooleanTolerances,
    IntersectionSpanId, classify, compute_boolean_intersections, operand_cells, select,
};

/// One committed face Boolean.
#[derive(Debug)]
pub struct FaceBoolean {
    pub operation: BooleanOperation,
    /// The faces the answer is made of, all of them descended from the first
    /// operand. Several where the answer falls in disconnected parts.
    pub faces: Vec<FaceKey>,
    /// Which source face each survivor came from.
    pub lineage: BooleanLineage,
    pub diagnostics: BooleanDiagnostics,
}

/// Evaluates one regularized Boolean on two coplanar faces, in one transaction.
///
/// The second operand is consumed: every face descended from it is removed, so
/// on success only the first operand's surviving faces remain. Anything the
/// preparation cannot certify rolls the whole transaction back.
///
pub fn face_boolean<P: Payload>(
    map: &mut Model<P>,
    first: FaceKey,
    second: FaceKey,
    operation: BooleanOperation,
    options: BooleanOptions,
) -> Result<FaceBoolean, BooleanError> {
    let tolerances = admit(map, first, second, options)?;
    map.transaction(|edit| run(edit, first, second, operation, options, tolerances))
}

/// Checks both operands exist and share one plane, and fixes the budget.
///
/// Coplanarity is what makes the two operands comparable at all: containment is
/// decided in one surface's parameter space, and a point of one face has no
/// place in the other's unless the supports agree.
fn admit<P: Payload>(
    map: &Model<P>,
    first: FaceKey,
    second: FaceKey,
    options: BooleanOptions,
) -> Result<BooleanTolerances, BooleanError> {
    if first == second {
        return Err(BooleanError::SharedOperandBoundary);
    }
    let first_cells = operand_cells(map, BooleanOperand::Face(first))?;
    let second_cells = operand_cells(map, BooleanOperand::Face(second))?;
    let tolerances =
        BooleanTolerances::from_cells(map, &first_cells, &second_cells, options.tolerances)?;
    // `Planar` is the type that says "this view lies on this plane", so the
    // support is taken from it rather than read off the surface: a face whose
    // stored surface is a plane but whose boundary wanders off it is not a
    // planar operand, and only the check knows the difference.
    let planes = [first, second]
        .map(|face| Planar::new(map.face_unchecked(face)).map(|planar| planar.plane().clone()));
    let (Ok(a), Ok(b)) = (&planes[0], &planes[1]) else {
        return Err(BooleanError::OperandSupportsDiffer { first, second });
    };
    let parallel = a.normal().cross(&b.normal()).norm() <= tolerances.angular;
    let coincident = (b.origin() - a.origin()).dot(&a.normal()).abs() <= tolerances.linear;
    if !parallel || !coincident {
        return Err(BooleanError::OperandSupportsDiffer { first, second });
    }
    Ok(tolerances)
}

/// Prepares, classifies, selects, and removes everything the table dropped.
fn run<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    first: FaceKey,
    second: FaceKey,
    operation: BooleanOperation,
    options: BooleanOptions,
    tolerances: BooleanTolerances,
) -> Result<FaceBoolean, BooleanError> {
    let mut options = options;
    tolerances.apply(&mut options.intersections);
    options.tolerances = BooleanTolerancePolicy::Fixed(tolerances);

    let plan = compute_boolean_intersections(
        edit,
        BooleanOperand::Face(first),
        BooleanOperand::Face(second),
        options,
    )?;
    if !plan.diagnostics.coverage.is_empty()
        || plan.diagnostics.branches_uncertified > 0
        || !plan.diagnostics.unresolved_overlaps.is_empty()
    {
        return Err(BooleanError::IncompleteIntersections {
            diagnostics: Box::new(plan.diagnostics),
        });
    }
    let prepared = apply_boolean_splits_edit(edit, plan, false)?;
    let graph = FragmentGraph::<PlanarDomain>::build(&prepared);
    let classes = classify::run(edit, &prepared, &graph, options, tolerances)?;
    let selection = select::run(operation, &graph, &classes);
    if selection.kept.is_empty() {
        return Err(BooleanError::EmptyResult);
    }

    // Every face of the second operand is dropped by both surviving rules, so
    // this removes the operand itself along with the first operand's discarded
    // faces. A face the table kept still holds its own darts on the edges it
    // shared with a removed neighbour, so those edges survive as free boundary.
    // The splitter retains the source key on whichever half it built first,
    // which need not be a half this operation keeps. Putting it back on a
    // surviving face before anything is removed is what lets the caller go on
    // using the key it passed in.
    let mut selection = selection;
    if let Some(&heir) = selection.kept.first()
        && selection.dropped.contains(&first)
    {
        edit.swap_face_identities(first, heir);
        for face in selection
            .kept
            .iter_mut()
            .chain(selection.dropped.iter_mut())
        {
            *face = match *face {
                key if key == first => heir,
                key if key == heir => first,
                key => key,
            };
        }
    }
    // The seam is read off the faces as they stand, so it is found before
    // anything is removed and welded after -- exactly the order the solid
    // assembly uses, and for the same reason.
    let union = operation == BooleanOperation::Union;
    let seam = union
        .then(|| sew_union_plan(edit, &prepared, &selection))
        .transpose()?
        .unwrap_or_default();
    edit.remove_faces(&selection.dropped)?;
    let merges = weld(edit, seam, tolerances)?;
    let fused = union
        .then(|| fuse_seams(edit, &prepared, &merges, tolerances))
        .transpose()?
        .unwrap_or_default();

    // A fused face carries on from both of its halves, so it inherits the
    // sources of the one it consumed. Tracking that here rather than in the
    // lineage maps keeps one entry per surviving face while it is being
    // rewritten, and the maps are rebuilt from it once nothing moves again.
    let mut sources = source_index(&prepared.first_lineage);
    if union {
        sources.extend(source_index(&prepared.second_lineage));
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
    let lineage = lineage_of(&sources, &kept);
    let faces = kept;
    Ok(FaceBoolean {
        operation,
        faces,
        lineage,
        diagnostics: prepared.diagnostics,
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

/// Rebuilds a source-first lineage over the faces that survived.
fn lineage_of(sources: &HashMap<FaceKey, BTreeSet<FaceKey>>, kept: &[FaceKey]) -> BooleanLineage {
    let mut lineage = BooleanLineage::default();
    for &face in kept {
        for &source in sources.get(&face).into_iter().flatten() {
            lineage.faces.entry(source).or_default().push(face);
        }
    }
    lineage
}

/// Pairs each section span's two edges with the face that still keeps them.
///
/// A union is the only operation that keeps faces from both operands, and they
/// arrive unjoined: where the two boundaries coincide, each operand carries its
/// own edge. This finds those pairs while the faces are all still present; the
/// welding itself happens after the discarded faces are gone.
///
/// An edge bounding two kept faces is already interior to the answer and is
/// skipped, so only the seam between the operands is paired.
fn sew_union_plan<P: Payload>(
    edit: &ModelEdit<'_, P>,
    prepared: &BooleanOperandPreparation,
    selection: &SelectionPlan<PlanarDomain>,
) -> Result<Vec<SeamPair>, BooleanError> {
    let kept = selection.kept.iter().copied().collect::<HashSet<_>>();
    let mut spans = prepared.span_edges.iter().collect::<Vec<_>>();
    spans.sort_by_key(|(span, _)| span.0);
    let mut pairs = Vec::new();
    for (&span, sides) in spans {
        let seam = sides.each_ref().map(|side| {
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
        if seam[0].len() != seam[1].len() {
            return Err(BooleanError::NonIsomorphicSpanSubdivision {
                span,
                first: seam[0].len(),
                second: seam[1].len(),
            });
        }
        pairs.extend(
            seam[0]
                .iter()
                .copied()
                .zip(seam[1].iter().copied())
                .map(|(a, b)| SeamPair { span, a, b }),
        );
    }
    Ok(pairs)
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
    prepared: &BooleanOperandPreparation,
    merges: &SeamMerges,
    tolerances: BooleanTolerances,
) -> Result<Vec<(FaceKey, FaceKey)>, BooleanError> {
    // Healing removes edges in the order it is offered them, so the spans are
    // walked by id rather than in the hash order the map happens to hold them.
    let mut spans = prepared.span_edges.iter().collect::<Vec<_>>();
    spans.sort_by_key(|(span, _)| span.0);
    let edges = spans
        .into_iter()
        .flat_map(|(_, sides)| sides.iter().flatten())
        .map(|edge| *merges.edges.get(edge).unwrap_or(edge))
        .filter(|&edge| edit.edge(edge).is_some())
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
