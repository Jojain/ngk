//! Cutting a solid, a sheet, a face, a profile or an edge along a sheet into
//! the parts on either side of it.
//!
//! Where a sheet cuts a target it separates two sides: the one its faces'
//! normals point into, the **front**, and the one behind them, the **back**.
//! Both parts start the same way: the target and the sheet are imprinted on
//! each other, and every fragment of the target is given a side. A fragment's
//! side is read where it meets the sheet -- along a section edge -- and then
//! spreads across every edge the sheet did not cut.
//!
//! A solid's part is then an ordinary Boolean. Call the region behind the
//! sheet `H`: the back is `target ∩ H` and the front `target − H`, once every
//! fragment says where it lies against `H`:
//!
//! - a fragment of the sheet is located against the solid by the usual ray
//!   cast, and its own operand's membership -- outside in front, inside behind
//!   -- is exactly `H`'s;
//! - a fragment of the solid's boundary cannot be ray cast against an open
//!   sheet. Along a section edge, a back fragment and the sheet fragment inside
//!   the solid run the edge in opposite directions, as two faces of one
//!   consistently wound shell do, and a front fragment runs it the same way.
//!
//! A sheet or a face bounds nothing, so its part is simply its fragments on
//! one side, the rest removed. Such a fragment leaves a section edge towards
//! its own interior, which lies on one side of the sheet's normal there. Only
//! the target is cut: on the sheet the section is an open slit, and nothing of
//! the sheet is kept.
//!
//! A profile or an edge meets the sheet at points instead: it is cut where it
//! crosses, each piece leaves a crossing towards one side of the sheet's
//! normal, and the side spreads across every vertex off the sheet.
//!
//! Each part is built on its own copy of the operands.

use std::collections::{BTreeSet, HashMap, HashSet};
use std::hash::Hash;

use nalgebra::Vector3;

use super::arrangement::{self, Arrangement, Imprinted};
use super::assemble::shell_components;
use super::domain::{BooleanDomain, OperandClassifier, RelativeLocation};
use super::neighborhood::FragmentGraph;
use super::operand::BooleanContext;
use super::solid_domain::SolidDomain;
use super::{
    BooleanCell, BooleanError, BooleanOperand, BooleanOperation, BooleanOptions, BooleanSection,
    BooleanTolerances, BooleanTrace, assemble, select,
};
use crate::geometry::parameter::Fraction;
use crate::geometry::{Point3, PointCoincidence};
use crate::model::{Cell1, Cell2, Model};
use crate::topology::ModelEdit;
use crate::topology::attributes::{ProfileAttr, SheetAttr};
use crate::topology::gmap::Dim;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey, ProfileKey, SheetKey, SolidKey, VertexKey};
use crate::topology::trim_domain::{FaceTrimDomain, TrimLocation};
use sealed::Splitter;

/// The two parts a target was cut into.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Split<K> {
    /// The part on the side the sheet's normals point to.
    pub front: K,
    /// The part behind the sheet.
    pub back: K,
}

/// One side of the sheet a target is split along.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SplitSide {
    Front,
    Back,
}

/// An entity [`split`] can cut: a solid, a sheet, a face, a profile or an
/// edge, by its key.
pub trait SplitTarget: sealed::Splitter {}

impl SplitTarget for SolidKey {}
impl SplitTarget for SheetKey {}
impl SplitTarget for FaceKey {}
impl SplitTarget for ProfileKey {}
impl SplitTarget for EdgeKey {}

/// Cuts `target` along `tool` into the parts in front of and behind it, in
/// one transaction. Each part is of the target's own kind.
///
/// The sheet must reach past the target everywhere it crosses it: a sheet that
/// ends inside a solid, a sheet or a face leaves an open section, which is
/// refused. Each part must be one connected piece; a side falling in several
/// is refused as [`BooleanError::SplitSideDisconnected`], and a face or an edge
/// must stay one face or one edge per side. A face, a profile or an edge must
/// be loose -- joined to nothing it is only part of -- since cutting it alone
/// would tear what holds it. Both inputs are consumed, and every other entity
/// of the map keeps its key.
pub fn split<P: Payload, K: SplitTarget>(
    map: &mut Model<P>,
    target: K,
    tool: SheetKey,
    options: BooleanOptions,
) -> Result<Split<K>, BooleanError> {
    K::admit(map, target)?;
    BooleanContext::admit_split(
        map,
        target.operand(),
        tool,
        BooleanOperation::Difference,
        options,
    )?;
    let source = map.clone();
    map.transaction(|edit| {
        let target_copy = K::copy(edit, &source, target);
        let tool_copy = SheetKey::copy(edit, &source, tool);
        let back = K::part(edit, target, tool, SplitSide::Back, options)?;
        let front = K::part(edit, target_copy, tool_copy, SplitSide::Front, options)?;
        Ok(Split { front, back })
    })
}

mod sealed {
    use super::*;

    /// How one kind of target is admitted, copied and cut into one part.
    pub trait Splitter: Copy {
        fn operand(self) -> BooleanOperand;

        /// Refuses a target that cannot be cut on its own.
        fn admit<P: Payload>(_model: &Model<P>, _key: Self) -> Result<(), BooleanError> {
            Ok(())
        }

        /// Copies `key` out of `source` into the edit.
        fn copy<P: Payload>(edit: &mut ModelEdit<'_, P>, source: &Model<P>, key: Self) -> Self;

        /// Builds the part of `target` on one `side` of `tool`, consuming both.
        fn part<P: Payload>(
            edit: &mut ModelEdit<'_, P>,
            target: Self,
            tool: SheetKey,
            side: SplitSide,
            options: BooleanOptions,
        ) -> Result<Self, BooleanError>;
    }

    impl Splitter for SolidKey {
        fn operand(self) -> BooleanOperand {
            BooleanOperand::Solid(self)
        }

        fn copy<P: Payload>(edit: &mut ModelEdit<'_, P>, source: &Model<P>, key: Self) -> Self {
            let dart = edit.merge(source.solid_unchecked(key));
            edit.solid_key(dart)
                .expect("a copied solid keeps its registration")
        }

        fn part<P: Payload>(
            edit: &mut ModelEdit<'_, P>,
            target: Self,
            tool: SheetKey,
            side: SplitSide,
            options: BooleanOptions,
        ) -> Result<Self, BooleanError> {
            solid_part(edit, target, tool, side, options)
        }
    }

    impl Splitter for SheetKey {
        fn operand(self) -> BooleanOperand {
            BooleanOperand::Sheet(self)
        }

        fn copy<P: Payload>(edit: &mut ModelEdit<'_, P>, source: &Model<P>, key: Self) -> Self {
            let dart = edit.merge(source.sheet_unchecked(key));
            edit.sheet_key(dart)
                .expect("a copied sheet keeps its registration")
        }

        fn part<P: Payload>(
            edit: &mut ModelEdit<'_, P>,
            target: Self,
            tool: SheetKey,
            side: SplitSide,
            options: BooleanOptions,
        ) -> Result<Self, BooleanError> {
            let faces = surface_part(edit, target.operand(), tool, side, options)?;
            let root = edit.face_unchecked(faces[0]).dart();
            Ok(edit.add_sheet_split_from(target, SheetAttr::new(root)))
        }
    }

    impl Splitter for FaceKey {
        fn operand(self) -> BooleanOperand {
            BooleanOperand::Face(self)
        }

        /// A face joined to a neighbour belongs to a sheet or a solid.
        fn admit<P: Payload>(model: &Model<P>, key: Self) -> Result<(), BooleanError> {
            let joined = model.face(key).is_some_and(|face| {
                face.edges()
                    .iter()
                    .any(|edge| !model.is_free(edge.dart(), Dim::Two))
            });
            match joined {
                true => Err(BooleanError::SplitTargetNotLoose {
                    cell: BooleanCell::Face(key),
                }),
                false => Ok(()),
            }
        }

        fn copy<P: Payload>(edit: &mut ModelEdit<'_, P>, source: &Model<P>, key: Self) -> Self {
            let dart = edit.merge(source.face_unchecked(key));
            edit.cell_key_unchecked::<Cell2>(dart)
        }

        fn part<P: Payload>(
            edit: &mut ModelEdit<'_, P>,
            target: Self,
            tool: SheetKey,
            side: SplitSide,
            options: BooleanOptions,
        ) -> Result<Self, BooleanError> {
            let faces = surface_part(edit, target.operand(), tool, side, options)?;
            single(faces, side)
        }
    }

    impl Splitter for ProfileKey {
        fn operand(self) -> BooleanOperand {
            BooleanOperand::Profile(self)
        }

        /// A profile a face runs along is that face's boundary.
        fn admit<P: Payload>(model: &Model<P>, key: Self) -> Result<(), BooleanError> {
            let bounding = model.profile(key).and_then(|profile| {
                profile
                    .edges()
                    .into_iter()
                    .find(|edge| !edge.faces().is_empty())
            });
            match bounding {
                Some(edge) => Err(BooleanError::SplitTargetNotLoose {
                    cell: BooleanCell::Edge(edge.key()),
                }),
                None => Ok(()),
            }
        }

        fn copy<P: Payload>(edit: &mut ModelEdit<'_, P>, source: &Model<P>, key: Self) -> Self {
            let dart = edit.merge(source.profile_unchecked(key));
            edit.profile_key_unchecked(dart)
        }

        fn part<P: Payload>(
            edit: &mut ModelEdit<'_, P>,
            target: Self,
            tool: SheetKey,
            side: SplitSide,
            options: BooleanOptions,
        ) -> Result<Self, BooleanError> {
            let edges = curve_part(edit, target.operand(), tool, side, options)?;
            // A profile is walked forward from its root, so an open one is
            // rooted at an end or the walk would miss what lies behind it.
            let darts = edges
                .iter()
                .flat_map(|&edge| {
                    let dart = edit.edge_unchecked(edge).dart();
                    [dart, edit.alpha(Dim::Zero, dart)]
                })
                .collect::<Vec<_>>();
            let root = darts
                .iter()
                .copied()
                .find(|&dart| edit.is_free(dart, Dim::One))
                .unwrap_or(darts[0]);
            Ok(edit.add_profile_split_from(target, ProfileAttr::new(root)))
        }
    }

    impl Splitter for EdgeKey {
        fn operand(self) -> BooleanOperand {
            BooleanOperand::Edge(self)
        }

        /// An edge joined at an end to another, or run along by a face, is
        /// part of a profile or a face. A closed edge joined to itself is not.
        fn admit<P: Payload>(model: &Model<P>, key: Self) -> Result<(), BooleanError> {
            let joined = model.edge(key).is_some_and(|edge| {
                let own = model
                    .orbit(edge.dart(), model.orbit_indices(Dim::One))
                    .collect::<HashSet<_>>();
                !edge.faces().is_empty()
                    || own
                        .iter()
                        .any(|&dart| !own.contains(&model.alpha(Dim::One, dart)))
            });
            match joined {
                true => Err(BooleanError::SplitTargetNotLoose {
                    cell: BooleanCell::Edge(key),
                }),
                false => Ok(()),
            }
        }

        fn copy<P: Payload>(edit: &mut ModelEdit<'_, P>, source: &Model<P>, key: Self) -> Self {
            let dart = edit.merge(source.edge_unchecked(key));
            edit.cell_key_unchecked::<Cell1>(dart)
        }

        fn part<P: Payload>(
            edit: &mut ModelEdit<'_, P>,
            target: Self,
            tool: SheetKey,
            side: SplitSide,
            options: BooleanOptions,
        ) -> Result<Self, BooleanError> {
            let edges = curve_part(edit, target.operand(), tool, side, options)?;
            single(edges, side)
        }
    }

    /// The one piece a side of a face or an edge must be.
    fn single<K: Copy>(pieces: Vec<K>, side: SplitSide) -> Result<K, BooleanError> {
        match pieces.as_slice() {
            &[piece] => Ok(piece),
            pieces => Err(BooleanError::SplitSideDisconnected {
                side,
                components: pieces.len(),
            }),
        }
    }
}

/// Imprints `tool` on `target`, and `target` on `tool` too unless it is to
/// stay whole.
fn arrange<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    target: BooleanOperand,
    tool: SheetKey,
    imprinted: Imprinted,
    operation: BooleanOperation,
    options: BooleanOptions,
) -> Result<(BooleanContext, Arrangement), BooleanError> {
    let context = BooleanContext::admit_split(edit, target, tool, operation, options)?;
    let arrangement = arrangement::arrange(
        edit,
        &context.operands.operands,
        imprinted,
        context.options,
        context.tolerances,
        &mut BooleanTrace::disabled(),
    )?;
    Ok((context, arrangement))
}

/// Refuses an arrangement in which the sheet cut nothing of a solid, a sheet
/// or a face.
fn crossed(arrangement: &Arrangement) -> Result<(), BooleanError> {
    match arrangement.sections.is_empty() {
        true => Err(BooleanError::SplitMissesTarget),
        false => Ok(()),
    }
}

/// Builds the part of the solid `target` on one `side` of `tool`.
fn solid_part<P: Payload>(
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
    let (context, arrangement) = arrange(
        edit,
        target.operand(),
        tool,
        Imprinted::Every,
        operation,
        options,
    )?;
    crossed(&arrangement)?;
    let graph = FragmentGraph::<SolidDomain>::build(&arrangement.lineages);
    let classes = locate(
        edit,
        &arrangement,
        &graph,
        context.options,
        context.tolerances,
    )?;
    let selection = select::run(operation, context.operands.operands.len(), &graph, &classes);
    match assemble::run(edit, &context, &graph, arrangement, selection) {
        Ok(result) => Ok(result.solid),
        Err(BooleanError::EmptyResult) => Err(BooleanError::SplitSideEmpty { side }),
        Err(BooleanError::DisconnectedResult { components }) => {
            Err(BooleanError::SplitSideDisconnected { side, components })
        }
        Err(error) => Err(error),
    }
}

/// Keeps the faces of the sheet or face `target` on one `side` of `tool` and
/// removes everything else of both, returning the faces kept.
///
/// Every sheet registered on either input is removed with it: the part is a
/// new set of faces, and registering it is the caller's.
fn surface_part<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    target: BooleanOperand,
    tool: SheetKey,
    side: SplitSide,
    options: BooleanOptions,
) -> Result<Vec<FaceKey>, BooleanError> {
    // On the tool the section is an open slit, which no face split makes, and
    // the tool is consumed whole anyway.
    let (context, arrangement) = arrange(
        edit,
        target,
        tool,
        Imprinted::TargetOnly,
        BooleanOperation::Difference,
        options,
    )?;
    crossed(&arrangement)?;
    let fragments = |operand: usize| {
        arrangement.lineages[operand]
            .faces
            .values()
            .flatten()
            .copied()
            .collect::<Vec<_>>()
    };
    let (target_faces, tool_faces) = (fragments(0), fragments(1));
    let tolerances = context.tolerances;
    let model: &Model<P> = edit;
    let sides = target_sides(
        model,
        &arrangement,
        &target_faces.iter().copied().collect(),
        |_, edge, middle, face| surface_side(model, &tool_faces, edge, middle, face, tolerances),
    )?;
    let (kept, mut dropped): (Vec<_>, Vec<_>) = target_faces
        .iter()
        .copied()
        .partition(|face| sides[face] == side);
    dropped.extend(&tool_faces);
    let registered = target_faces
        .iter()
        .chain(&tool_faces)
        .filter_map(|&face| edit.sheet_key(edit.face_unchecked(face).dart()))
        .collect::<BTreeSet<_>>();
    for sheet in registered {
        edit.remove_sheet(sheet);
    }
    edit.remove_faces(&dropped)?;
    if kept.is_empty() {
        return Err(BooleanError::SplitSideEmpty { side });
    }
    let components = shell_components(edit, &kept).len();
    if components > 1 {
        return Err(BooleanError::SplitSideDisconnected { side, components });
    }
    Ok(kept)
}

/// Keeps the edges of the profile or edge `target` on one `side` of `tool`
/// and removes everything else of both, returning the edges kept.
///
/// A curve meets the sheet at points, not along sections: each piece leaves a
/// point on the sheet towards one side of the sheet's normal there, and every
/// other piece takes the side of the pieces it shares a vertex off the sheet
/// with.
fn curve_part<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    target: BooleanOperand,
    tool: SheetKey,
    side: SplitSide,
    options: BooleanOptions,
) -> Result<Vec<EdgeKey>, BooleanError> {
    let (context, arrangement) = arrange(
        edit,
        target,
        tool,
        Imprinted::TargetOnly,
        BooleanOperation::Difference,
        options,
    )?;
    let tolerances = context.tolerances;
    let target_edges = arrangement.lineages[0]
        .edges
        .values()
        .flatten()
        .copied()
        .collect::<Vec<_>>();
    let tool_faces = arrangement.lineages[1]
        .faces
        .values()
        .flatten()
        .copied()
        .collect::<Vec<_>>();

    let model: &Model<P> = edit;
    let mut ends = HashMap::<VertexKey, Vec<EdgeKey>>::new();
    let mut on_sheet = HashSet::new();
    let mut sides = HashMap::new();
    let mut pending = Vec::new();
    for &edge in &target_edges {
        for vertex in model.edge_unchecked(edge).vertices() {
            ends.entry(vertex.key()).or_default().push(edge);
            let point = *vertex.point();
            let Some(sheet_face) = face_at(model, &tool_faces, point, tolerances)? else {
                continue;
            };
            on_sheet.insert(vertex.key());
            let ambiguous = || BooleanError::SplitSideAmbiguous {
                cell: BooleanCell::Edge(edge),
            };
            let leaving = leaving_direction(model, edge, point)
                .try_normalize(0.0)
                .ok_or_else(ambiguous)?;
            let reach = leaving.dot(&normal_at(model, sheet_face, point)?);
            if reach.abs() <= tolerances.angular {
                return Err(ambiguous());
            }
            let seen = if reach > 0.0 {
                SplitSide::Front
            } else {
                SplitSide::Back
            };
            assign(
                &mut sides,
                &mut pending,
                edge,
                seen,
                BooleanCell::Edge(edge),
            )?;
        }
    }
    if on_sheet.is_empty() {
        return Err(BooleanError::SplitMissesTarget);
    }
    while let Some(edge) = pending.pop() {
        let seen = sides[&edge];
        for vertex in model.edge_unchecked(edge).vertices() {
            if on_sheet.contains(&vertex.key()) {
                continue;
            }
            for &neighbour in &ends[&vertex.key()] {
                if neighbour != edge {
                    assign(
                        &mut sides,
                        &mut pending,
                        neighbour,
                        seen,
                        BooleanCell::Edge(neighbour),
                    )?;
                }
            }
        }
    }
    if let Some(&edge) = target_edges.iter().find(|edge| !sides.contains_key(edge)) {
        return Err(BooleanError::SplitBoundaryUncut {
            cell: BooleanCell::Edge(edge),
        });
    }

    let (kept, dropped): (Vec<_>, Vec<_>) = target_edges
        .iter()
        .copied()
        .partition(|edge| sides[edge] == side);
    let components = curve_components(model, &kept, &ends);
    let profiles = target_edges
        .iter()
        .filter_map(|&edge| edit.profile_key(edit.edge_unchecked(edge).dart()))
        .collect::<BTreeSet<_>>();
    let sheets = tool_faces
        .iter()
        .filter_map(|&face| edit.sheet_key(edit.face_unchecked(face).dart()))
        .collect::<BTreeSet<_>>();
    for profile in profiles {
        edit.remove_profile(profile);
    }
    for sheet in sheets {
        edit.remove_sheet(sheet);
    }
    edit.remove_faces(&tool_faces)?;
    edit.remove_edges(&dropped)?;
    if kept.is_empty() {
        return Err(BooleanError::SplitSideEmpty { side });
    }
    if components > 1 {
        return Err(BooleanError::SplitSideDisconnected { side, components });
    }
    Ok(kept)
}

/// How many pieces `edges` fall into, joined where they share an end.
fn curve_components<P: Payload>(
    model: &Model<P>,
    edges: &[EdgeKey],
    ends: &HashMap<VertexKey, Vec<EdgeKey>>,
) -> usize {
    let mut remaining = edges.iter().copied().collect::<BTreeSet<_>>();
    let mut components = 0;
    while let Some(&seed) = remaining.first() {
        components += 1;
        let mut pending = vec![seed];
        while let Some(edge) = pending.pop() {
            if !remaining.remove(&edge) {
                continue;
            }
            for vertex in model.edge_unchecked(edge).vertices() {
                pending.extend(&ends[&vertex.key()]);
            }
        }
    }
    components
}

/// The direction `edge` leaves `point`, one of its ends, along its curve.
fn leaving_direction<P: Payload>(model: &Model<P>, edge: EdgeKey, point: Point3) -> Vector3<f64> {
    const STEP: f64 = 1e-3;
    let curve = model.edge_unchecked(edge).trimmed_curve();
    let at = curve.parameter_at(point).value();
    let toward = if at < 0.5 { at + STEP } else { at - STEP };
    curve.point_at(Fraction::new(toward)) - point
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
    let target = graph
        .fragments
        .iter()
        .filter(|fragment| fragment.operand == TARGET)
        .map(|fragment| fragment.fragment)
        .collect();
    let sides = target_sides(
        model,
        arrangement,
        &target,
        |section, edge, middle, face| {
            solid_side(
                model,
                section,
                edge,
                middle,
                face,
                &inside,
                tolerances.linear,
            )
        },
    )?;
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

/// Which side of the sheet every one of the target's `fragments` is on.
///
/// `side_along` reads the side of a fragment running a section edge, given
/// the edge's section and middle; every other fragment takes the side of the
/// neighbours it shares an uncut edge with.
fn target_sides<P: Payload>(
    model: &Model<P>,
    arrangement: &Arrangement,
    fragments: &HashSet<FaceKey>,
    side_along: impl Fn(&BooleanSection, EdgeKey, Point3, FaceKey) -> Result<SplitSide, BooleanError>,
) -> Result<HashMap<FaceKey, SplitSide>, BooleanError> {
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
            for face in model.edge_unchecked(edge).faces() {
                let face = face.key();
                if fragments.contains(&face) {
                    let side = side_along(section, edge, middle, face)?;
                    assign(
                        &mut sides,
                        &mut pending,
                        face,
                        side,
                        BooleanCell::Face(face),
                    )?;
                }
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
                    let cell = BooleanCell::Face(neighbour);
                    assign(&mut sides, &mut pending, neighbour, side, cell)?;
                }
            }
        }
    }
    if let Some(&face) = fragments.iter().find(|face| !sides.contains_key(face)) {
        return Err(BooleanError::SplitBoundaryUncut {
            cell: BooleanCell::Face(face),
        });
    }
    Ok(sides)
}

/// The side of a solid's boundary fragment running a section edge: behind the
/// sheet when it runs the edge against the sheet fragment inside the solid.
fn solid_side<P: Payload>(
    model: &Model<P>,
    section: &BooleanSection,
    edge: EdgeKey,
    middle: Point3,
    face: FaceKey,
    inside: &HashSet<FaceKey>,
    tolerance: f64,
) -> Result<SplitSide, BooleanError> {
    let unpaired = || BooleanError::SplitSectionUnpaired { edge };
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
        .ok_or_else(unpaired)?;
    let sheet = run_direction(model, sheet_face, sheet_edge, middle).ok_or_else(unpaired)?;
    let own = run_direction(model, face, edge, middle).ok_or_else(unpaired)?;
    Ok(if own.dot(&sheet) < 0.0 {
        SplitSide::Back
    } else {
        SplitSide::Front
    })
}

/// The side of a sheet's or face's fragment running a section edge: the side
/// of the sheet's normal its interior leaves the edge towards.
///
/// A fragment lies to the left of its own boundary, seen from its normal, so
/// it leaves the edge along the normal crossed with the way it runs the edge.
/// A fragment leaving along the sheet itself, as where the two are tangent,
/// is on neither side.
fn surface_side<P: Payload>(
    model: &Model<P>,
    tool_faces: &[FaceKey],
    edge: EdgeKey,
    middle: Point3,
    face: FaceKey,
    tolerances: BooleanTolerances,
) -> Result<SplitSide, BooleanError> {
    let unpaired = || BooleanError::SplitSectionUnpaired { edge };
    let sheet_face = face_at(model, tool_faces, middle, tolerances)?.ok_or_else(unpaired)?;
    let sheet = normal_at(model, sheet_face, middle)?;
    let own = run_direction(model, face, edge, middle).ok_or_else(unpaired)?;
    let inward = normal_at(model, face, middle)?
        .cross(&own)
        .try_normalize(0.0)
        .ok_or(BooleanError::SplitSideAmbiguous {
            cell: BooleanCell::Face(face),
        })?;
    let reach = inward.dot(&sheet);
    if reach.abs() <= tolerances.angular {
        return Err(BooleanError::SplitSideAmbiguous {
            cell: BooleanCell::Face(face),
        });
    }
    Ok(if reach > 0.0 {
        SplitSide::Front
    } else {
        SplitSide::Back
    })
}

/// The first of `faces` that `point` lies on, its boundary included.
fn face_at<P: Payload>(
    model: &Model<P>,
    faces: &[FaceKey],
    point: Point3,
    tolerances: BooleanTolerances,
) -> Result<Option<FaceKey>, BooleanError> {
    for &face in faces {
        let view = model.face_unchecked(face);
        // A support that cannot place the point does not hold it.
        let Ok(uv) = view.surface().param_at(point) else {
            continue;
        };
        if !view
            .point_at(uv.x, uv.y)
            .coincides(point, tolerances.linear)
        {
            continue;
        }
        let trim = FaceTrimDomain::new(&view, tolerances.parameter)?;
        if !matches!(trim.classify(uv), TrimLocation::Outside { .. }) {
            return Ok(Some(face));
        }
    }
    Ok(None)
}

/// Records `key`'s side, refusing a piece both sides reach.
fn assign<K: Copy + Eq + Hash>(
    sides: &mut HashMap<K, SplitSide>,
    pending: &mut Vec<K>,
    key: K,
    side: SplitSide,
    cell: BooleanCell,
) -> Result<(), BooleanError> {
    match sides.insert(key, side) {
        None => {
            pending.push(key);
            Ok(())
        }
        Some(previous) if previous == side => Ok(()),
        Some(_) => Err(BooleanError::SplitSideAmbiguous { cell }),
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

/// `face`'s oriented normal at `point`, which lies on its support.
fn normal_at<P: Payload>(
    model: &Model<P>,
    face: FaceKey,
    point: Point3,
) -> Result<Vector3<f64>, BooleanError> {
    let view = model.face_unchecked(face);
    let uv = view.surface().param_at(point)?;
    Ok(view.normal_at(uv.x, uv.y).into_inner())
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
