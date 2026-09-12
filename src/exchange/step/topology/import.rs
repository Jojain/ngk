//! Sewing a `MANIFOLD_SOLID_BREP` into a map.
//!
//! **Stitching is the whole job.** STEP hands over faces whose loops name
//! shared `EDGE_CURVE`s by `#N`; NGK needs an α2-sewn 3-GMap. Nothing in the
//! kernel does that, because no builder ever receives topology as a heap of
//! independent faces — every other producer sews as it goes.
//!
//! **Two phases, in this order, because of the transaction rule.** A
//! transaction is atomic: any failure inside it restores the snapshot, so a
//! single unreadable face would cost the whole solid. Every face is therefore
//! *planned* first — geometry read, loops resolved, pcurves projected — with
//! the ones that fail recorded and dropped. Only a set of faces already known
//! to be constructible enters [`GMap::transaction`], one transaction per
//! solid, so one bad solid does not lose the file.
//!
//! **Orientation is not carried across; it is reproduced.** NGK derives a
//! face's normal from its boundary winding, so walking each bound in the
//! direction STEP composes and writing each pcurve in the surface's own chart
//! gets the normal right on its own. `ADVANCED_FACE.same_sense` is then not
//! state to store but a free consistency check: it is compared against the
//! winding we computed, and a disagreement is reported rather than silently
//! resolved one way or the other.

use std::collections::HashMap;

use crate::builders::profiles::curve_pcurve;
use crate::geometry::{Curve, NurbsError, Plane, Point2, Point3, Surface, TrimmedCurve2};
use crate::healing::{HealingOptions, remove_redundant_cells};
use crate::topology::attributes::{
    EdgeAttr, FaceAttr, ProfileAttr, SheetAttr, ShellRoot, SolidAttr, VertexAttr,
};
use crate::topology::gmap::{Dart, Dim, GMap};
use crate::topology::shape::{Shape, SolidTag};
use crate::topology::shape_keys::SolidKey;
use crate::topology::{StandardPayload, TopologyEdit, TopologyEditError};

use super::super::StepImport;
use super::super::convert::curves::read_curve;
use super::super::convert::placement::read_point;
use super::super::convert::surfaces::read_surface;
use super::super::error::{GeometryError, StepError, TopologyError};
use super::super::options::StepReadOptions;
use super::super::part21::{EntityId, StepExchange};
use super::super::report::{ImportReport, ImportSkip, ImportSkipReason};
use super::super::schema::entities;
use super::super::schema::resolver::{Located, Origin, Resolver, SchemaError};

/// How finely a pcurve is sampled when reading a loop's winding.
///
/// Only the sign of the enclosed area is wanted, and a pcurve that needs more
/// than this to get its *sign* right is degenerate for other reasons.
const WINDING_SAMPLES: usize = 8;

/// Reads every `MANIFOLD_SOLID_BREP` in a file into its own shape.
///
/// One map per B-Rep: a STEP file is a document holding several products, and
/// a `Shape` owns its map, so the solids cannot share one.
pub fn read_solids(
    exchange: &StepExchange,
    options: &StepReadOptions,
) -> Result<StepImport, StepError> {
    let resolver = Resolver::new(exchange, options.uncertainty)?;
    let mut import = StepImport::default();

    for instance in exchange.instances_of("MANIFOLD_SOLID_BREP") {
        let brep = resolver.decode::<entities::ManifoldSolidBrep>(instance)?;
        match read_solid(&resolver, &brep, options, &mut import.report) {
            Ok(Some(shape)) => import.shapes.push(shape),
            Ok(None) => {}
            Err(error) if options.strict => return Err(error),
            Err(error) => import.report.skipped.push(ImportSkip {
                entity: Some(brep.origin.id),
                line: brep.origin.line,
                reason: ImportSkipReason::SolidNotConstructible {
                    detail: error.to_string(),
                },
            }),
        }
    }

    Ok(import)
}

/// Reads one B-Rep, or `None` when nothing in it survived.
fn read_solid(
    resolver: &Resolver<'_>,
    brep: &Located<entities::ManifoldSolidBrep>,
    options: &StepReadOptions,
    report: &mut ImportReport,
) -> Result<Option<Shape<SolidTag, StandardPayload>>, StepError> {
    let shell = resolver.read::<entities::ClosedShell>(brep.origin, brep.outer)?;

    let mut planned = Vec::new();
    for id in &shell.cfs_faces {
        let face = resolver.read::<entities::AdvancedFace>(shell.origin, *id)?;
        match plan_face(resolver, &face, report) {
            Ok(plan) => planned.push(plan),
            Err(error) if options.strict => return Err(error),
            Err(error) => report.skipped.push(ImportSkip {
                entity: Some(face.origin.id),
                line: face.origin.line,
                reason: ImportSkipReason::FaceNotConstructible {
                    detail: error.to_string(),
                },
            }),
        }
    }

    if planned.is_empty() {
        return Ok(None);
    }

    check_edge_uses(&planned, brep.origin, options, report)?;

    let mut gmap = GMap::<StandardPayload>::new();
    let solid = gmap
        .transaction(|edit| sew_shell(edit, &planned))
        .map_err(|error| TopologyError::UnsewableShell {
            brep: brep.origin,
            detail: error.to_string(),
        })?;

    if options.heal_seams {
        // A seam is not part of the shape: STEP writes a periodic face with
        // its parameterization cut open, and that cut comes off as a step of
        // its own rather than as something the sewing above is allowed to
        // assume. A planar solid has none, so this changes nothing for one.
        remove_redundant_cells(&mut gmap, HealingOptions::seams_only()).map_err(|error| {
            TopologyError::UnsewableShell {
                brep: brep.origin,
                detail: error.to_string(),
            }
        })?;
    }

    Ok(Some(Shape::new(gmap, solid)))
}

/// One use of an `EDGE_CURVE` by one loop, in the direction the loop walks it.
#[derive(Debug, Clone)]
struct PlannedUse {
    /// The `EDGE_CURVE` instance, which is what two faces share.
    edge: EntityId,
    /// The `VERTEX_POINT` the walk leaves from.
    start: EntityId,
    /// The `VERTEX_POINT` it arrives at.
    end: EntityId,
    /// Whether the walk agrees with the `EDGE_CURVE`'s own start → end.
    forward: bool,
    /// The support the edge lies on.
    curve: Curve,
    /// Where `start` and `end` are, so the map can be filled without
    /// resolving the two `VERTEX_POINT`s a second time.
    start_point: Point3,
    end_point: Point3,
    /// The walk's image in the face's parameter space.
    pcurve: TrimmedCurve2,
}

/// One boundary of a face, already composed into its traversal direction.
#[derive(Debug, Clone)]
struct PlannedLoop {
    uses: Vec<PlannedUse>,
    /// The area the closed pcurve polygon encloses, signed by its winding.
    signed_area: f64,
}

/// One `ADVANCED_FACE` known to be constructible.
#[derive(Debug, Clone)]
struct PlannedFace {
    surface: Surface,
    loops: Vec<PlannedLoop>,
    /// Which of `loops` is the outer boundary.
    outer: usize,
}

/// Reads one `ADVANCED_FACE` without touching a map.
fn plan_face(
    resolver: &Resolver<'_>,
    face: &Located<entities::AdvancedFace>,
    report: &mut ImportReport,
) -> Result<PlannedFace, StepError> {
    let surface = read_surface(resolver, face.origin, face.face_geometry)?;

    // `read_surface` yields only a plane, so this cannot fail. The plane is
    // needed by value, because projecting a parameter curve needs its chart.
    let Surface::Plane(plane) = &surface else {
        unreachable!("read_surface yields only planar supports");
    };

    let mut loops = Vec::with_capacity(face.bounds.len());
    let mut declared_outer = None;
    for (index, id) in face.bounds.iter().enumerate() {
        let bound = resolver.read::<entities::FaceBound>(face.origin, *id)?;
        if bound.outer {
            declared_outer = Some(index);
        }
        loops.push(plan_loop(resolver, &bound, plane)?);
    }

    let outer = pick_outer(&loops, declared_outer, face.origin, report)?;
    check_sense(&loops[outer], face.same_sense, face.origin, report);

    Ok(PlannedFace {
        surface: surface.clone(),
        loops,
        outer,
    })
}

/// Reads one `FACE_BOUND` into the direction the face traverses it.
///
/// Composition is the whole subtlety. Both orientation flags mean *agrees*,
/// so the face walks an edge forwards exactly when the two are equal — and a
/// bound whose own flag is `.F.` also reverses the *order* of its oriented
/// edges, not just each one's direction.
fn plan_loop(
    resolver: &Resolver<'_>,
    bound: &Located<entities::FaceBound>,
    plane: &Plane,
) -> Result<PlannedLoop, StepError> {
    // `VERTEX_LOOP` and `POLY_LOOP` are refused by name rather than skipped:
    // NGK can represent neither, and a face silently missing a boundary is
    // worse than a face that says why it is missing.
    let edge_loop = resolver.read::<entities::EdgeLoop>(bound.origin, bound.bound)?;

    let mut walk = edge_loop.edge_list.clone();
    if !bound.orientation {
        walk.reverse();
    }

    let mut uses = Vec::with_capacity(walk.len());
    for id in walk {
        let oriented = resolver.read::<entities::OrientedEdge>(edge_loop.origin, id)?;
        let forward = oriented.orientation == bound.orientation;

        let edge = resolver.read::<entities::EdgeCurve>(oriented.origin, oriented.edge_element)?;
        let (start, end) = if forward {
            (edge.edge_start, edge.edge_end)
        } else {
            (edge.edge_end, edge.edge_start)
        };

        // `EDGE_CURVE.same_sense` is not read. Which corner the edge runs
        // from is `edge_start`, and the flag says only how the support runs
        // between the two — which, since an `EdgeAttr` stores no interval,
        // is re-derived from the corners themselves.
        let curve = read_curve(resolver, edge.origin, edge.edge_geometry)?;
        let start_point = read_vertex(resolver, edge.origin, start)?;
        let end_point = read_vertex(resolver, edge.origin, end)?;
        let pcurve = curve_pcurve(&curve, start_point, end_point, plane)
            .map_err(|error| curve_error(edge.origin, error))?;

        uses.push(PlannedUse {
            edge: oriented.edge_element,
            start,
            end,
            forward,
            curve,
            start_point,
            end_point,
            pcurve,
        });
    }

    if uses.is_empty() {
        return Err(SchemaError::UnreadableUnit {
            origin: edge_loop.origin,
            detail: "an edge loop with no edges bounds nothing".to_string(),
        }
        .into());
    }

    check_loop_closes(&uses, edge_loop.origin)?;
    let signed_area = signed_area(&uses);
    Ok(PlannedLoop { uses, signed_area })
}

/// Reads a `VERTEX_POINT`'s position.
fn read_vertex(resolver: &Resolver<'_>, from: Origin, id: EntityId) -> Result<Point3, StepError> {
    let vertex = resolver.read::<entities::VertexPoint>(from, id)?;
    Ok(read_point(resolver, vertex.origin, vertex.vertex_geometry)?)
}

/// Refuses a loop whose consecutive edges do not meet at one vertex.
///
/// Caught here rather than at commit because a map sewn from it would be
/// structurally valid and geometrically wrong — the darts would link, and the
/// corner would simply be in two places.
fn check_loop_closes(uses: &[PlannedUse], edge_loop: Origin) -> Result<(), StepError> {
    for (position, pair) in uses.windows(2).enumerate() {
        if pair[0].end != pair[1].start {
            return Err(SchemaError::UnreadableUnit {
                origin: edge_loop,
                detail: format!(
                    "edge {} starts at {}, but the one before it ended at {}",
                    position + 1,
                    pair[1].start,
                    pair[0].end
                ),
            }
            .into());
        }
    }
    if uses.len() > 1 && uses[uses.len() - 1].end != uses[0].start {
        return Err(SchemaError::UnreadableUnit {
            origin: edge_loop,
            detail: format!(
                "the loop ends at {} but began at {}",
                uses[uses.len() - 1].end,
                uses[0].start
            ),
        }
        .into());
    }
    Ok(())
}

/// Returns the area a loop's pcurves enclose, signed by their winding.
fn signed_area(uses: &[PlannedUse]) -> f64 {
    let mut points = Vec::with_capacity(uses.len() * WINDING_SAMPLES);
    for use_ in uses {
        for sample in 0..WINDING_SAMPLES {
            points.push(use_.pcurve.point_at(sample as f64 / WINDING_SAMPLES as f64));
        }
    }
    shoelace(&points)
}

fn shoelace(points: &[Point2]) -> f64 {
    let mut total = 0.0;
    for index in 0..points.len() {
        let current = points[index];
        let next = points[(index + 1) % points.len()];
        total += current.x * next.y - next.x * current.y;
    }
    total / 2.0
}

/// Decides which boundary of a face is its outer one.
///
/// `FACE_OUTER_BOUND` says so outright, but it is optional and OpenCascade
/// does not write it at all — every bound of a file it produces is a plain
/// `FACE_BOUND`. With one bound there is nothing to choose; with several, the
/// outer one is the one enclosing the most area, and having had to guess is
/// reported.
fn pick_outer(
    loops: &[PlannedLoop],
    declared: Option<usize>,
    face: Origin,
    report: &mut ImportReport,
) -> Result<usize, StepError> {
    if let Some(declared) = declared {
        return Ok(declared);
    }
    if loops.len() == 1 {
        return Ok(0);
    }
    let outer = loops
        .iter()
        .enumerate()
        .max_by(|(_, left), (_, right)| left.signed_area.abs().total_cmp(&right.signed_area.abs()))
        .map(|(index, _)| index)
        .ok_or_else(|| SchemaError::UnreadableUnit {
            origin: face,
            detail: "a face with no bounds encloses nothing".to_string(),
        })?;
    report.skipped.push(ImportSkip {
        entity: Some(face.id),
        line: face.line,
        reason: ImportSkipReason::GuessedOuterBound {
            bounds: loops.len(),
        },
    });
    Ok(outer)
}

/// Compares the winding we built against the sense the file declared.
///
/// Nothing is *done* with the answer: the face normal already follows from the
/// winding, so agreeing costs nothing and disagreeing means one of the two is
/// wrong. Saying which face raised it is the point — it catches a bad pcurve
/// at the face that caused it, rather than as a failed orientation validation
/// over the whole solid much later.
fn check_sense(outer: &PlannedLoop, same_sense: bool, face: Origin, report: &mut ImportReport) {
    if outer.signed_area == 0.0 {
        return;
    }
    if (outer.signed_area > 0.0) != same_sense {
        report.skipped.push(ImportSkip {
            entity: Some(face.id),
            line: face.line,
            reason: ImportSkipReason::SenseMismatch,
        });
    }
}

/// Reports edges whose use count NGK cannot hold, before anything is sewn.
///
/// Two uses is a closed manifold shell. One leaves the shell open, which is
/// worth knowing but still yields a map. More than two is non-manifold, which
/// a 3-GMap cannot represent at all, so the solid is refused by name.
fn check_edge_uses(
    planned: &[PlannedFace],
    brep: Origin,
    options: &StepReadOptions,
    report: &mut ImportReport,
) -> Result<(), StepError> {
    let mut counts: HashMap<EntityId, usize> = HashMap::new();
    for face in planned {
        for loop_ in &face.loops {
            for use_ in &loop_.uses {
                *counts.entry(use_.edge).or_default() += 1;
            }
        }
    }

    let mut edges: Vec<_> = counts.into_iter().collect();
    edges.sort_by_key(|(edge, _)| *edge);
    for (edge, uses) in edges {
        let reason = match uses {
            2 => continue,
            1 => ImportSkipReason::OpenShell,
            uses => ImportSkipReason::NonManifoldEdge { uses },
        };
        let non_manifold = matches!(reason, ImportSkipReason::NonManifoldEdge { .. });
        report.skipped.push(ImportSkip {
            entity: Some(edge),
            line: brep.line,
            reason,
        });
        if non_manifold {
            return Err(TopologyError::NonManifoldShell { brep, edge }.into());
        }
        if options.strict {
            return Err(TopologyError::OpenShell { brep, edge }.into());
        }
    }
    Ok(())
}

/// The darts one loop walks an edge with, in that walk's direction.
#[derive(Debug, Clone, Copy)]
struct DartUse {
    start: Dart,
    end: Dart,
    forward: bool,
}

/// Builds every planned face and sews them into one solid.
fn sew_shell(
    edit: &mut TopologyEdit<'_, StandardPayload>,
    planned: &[PlannedFace],
) -> Result<SolidKey, TopologyEditError> {
    // One entry per loop of every face, in the same order, so a face can find
    // the darts its own boundaries were built from.
    let mut loop_darts: Vec<Vec<Vec<Dart>>> = Vec::with_capacity(planned.len());
    let mut uses_by_edge: HashMap<EntityId, Vec<DartUse>> = HashMap::new();

    for face in planned {
        let mut face_darts = Vec::with_capacity(face.loops.len());
        for loop_ in &face.loops {
            let count = loop_.uses.len();
            let darts: Vec<Dart> = (0..2 * count).map(|_| edit.add_dart()).collect();
            for pair in 0..count {
                edit.link(Dim::Zero, darts[2 * pair], darts[2 * pair + 1])?;
            }
            for pair in 0..count {
                edit.link(
                    Dim::One,
                    darts[2 * pair + 1],
                    darts[(2 * pair + 2) % (2 * count)],
                )?;
            }
            for (index, use_) in loop_.uses.iter().enumerate() {
                uses_by_edge.entry(use_.edge).or_default().push(DartUse {
                    start: darts[2 * index],
                    end: darts[2 * index + 1],
                    forward: use_.forward,
                });
            }
            face_darts.push(darts);
        }
        loop_darts.push(face_darts);
    }

    // α2 pairs the two uses of an edge so that both darts sit at the same
    // corner — which for two loops walking the edge in opposite directions,
    // as an orientable shell always does, means one's start against the
    // other's end. That pairing is exactly what makes `α0 ∘ α2` land back in
    // the boundary walk, which is the consistent-winding invariant
    // `validate_all_solid_orientations` checks.
    for uses in uses_by_edge.values() {
        let [first, second] = uses.as_slice() else {
            continue;
        };
        if first.forward == second.forward {
            edit.sew(Dim::Two, first.start, second.start)?;
        } else {
            edit.sew(Dim::Two, first.start, second.end)?;
        }
    }

    let mut vertices: HashMap<EntityId, ()> = HashMap::new();
    let mut edges: HashMap<EntityId, ()> = HashMap::new();
    let mut shell_root = None;

    for (face, face_darts) in planned.iter().zip(&loop_darts) {
        for (loop_, darts) in face.loops.iter().zip(face_darts) {
            for (index, use_) in loop_.uses.iter().enumerate() {
                let (start, end) = (darts[2 * index], darts[2 * index + 1]);

                if vertices.insert(use_.start, ()).is_none() {
                    edit.add_vertex(VertexAttr::new(start, use_.start_point, ()));
                }
                if vertices.insert(use_.end, ()).is_none() {
                    edit.add_vertex(VertexAttr::new(end, use_.end_point, ()));
                }

                if edges.insert(use_.edge, ()).is_none() {
                    // The reference dart must run the way the `EDGE_CURVE`
                    // itself does, so that the default orientation NGK derives
                    // for the edge is the one the file declared — and export
                    // writes the same `same_sense` back out.
                    let reference = if use_.forward { start } else { end };
                    edit.add_edge(EdgeAttr::new(reference, use_.curve.clone(), ()));
                }
            }
            edit.add_profile(ProfileAttr::new(darts[0], ()));
        }

        let outer_seed = face_darts[face.outer][0];
        let inner_seeds: Vec<Dart> = face_darts
            .iter()
            .enumerate()
            .filter(|(index, _)| *index != face.outer)
            .map(|(_, darts)| darts[0])
            .collect();
        let pcurves: HashMap<Dart, TrimmedCurve2> = face
            .loops
            .iter()
            .zip(face_darts)
            .flat_map(|(loop_, darts)| {
                loop_
                    .uses
                    .iter()
                    .enumerate()
                    .map(move |(index, use_)| (darts[2 * index], use_.pcurve.clone()))
            })
            .collect();

        edit.add_face(FaceAttr::with_pcurves(
            face.surface.clone(),
            (),
            outer_seed,
            inner_seeds,
            pcurves,
        ));
        shell_root.get_or_insert(outer_seed);
    }

    // Any boundary dart names the whole shell, and every face was built in the
    // direction STEP composed, so the first face's outer seed already carries
    // the outward side.
    let root = ShellRoot::Dart(shell_root.expect("a planned solid has at least one face"));
    edit.add_sheet(SheetAttr::new(root, ()));
    Ok(edit.add_solid(SolidAttr::new((), root, None)))
}

/// Reports a curve that would not project into the face's plane.
///
/// Only a NURBS conversion can fail here, and only on geometry too degenerate
/// to carry a control polygon — so the message names the attribute rather than
/// re-spelling the conversion's own wording, which says nothing about STEP.
fn curve_error(edge: Origin, error: NurbsError) -> StepError {
    GeometryError::UnprojectableCurve {
        origin: edge,
        detail: error.to_string(),
    }
    .into()
}
