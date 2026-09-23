//! Sewing a B-Rep solid into a map.
//!
//! **Stitching is the whole job.** STEP hands over faces whose loops name
//! shared `EDGE_CURVE`s by `#N`; NGK needs an α2-sewn 3-gmap. Nothing in the
//! kernel does that, because no builder ever receives topology as a heap of
//! independent faces — every other producer sews as it goes.
//!
//! **Two phases, in this order, because of the transaction rule.** A
//! transaction is atomic: any failure inside it restores the snapshot, so a
//! single unreadable face would cost the whole solid. Every face is therefore
//! *planned* first — geometry read, loops resolved, pcurves rebuilt — with
//! the ones that fail recorded and dropped. Only a set of faces already known
//! to be constructible enters [`Model::transaction`], one transaction per
//! solid, so one bad solid does not lose the file.
//!
//! **Orientation is not carried across; it is reproduced.** NGK derives a
//! face's normal from its boundary winding, so walking each bound in the
//! direction STEP composes and writing each pcurve in the surface's own chart
//! gets the normal right on its own. `ADVANCED_FACE.same_sense` is then not
//! state to store but a free consistency check: it is compared against the
//! winding we computed, and a disagreement is reported rather than silently
//! resolved one way or the other.
//!
//! The one exception is a boundary that encloses no area — a whole sphere's
//! cut, walked out and back along one meridian — where there is no winding to
//! compare with and the flag is the only statement of the sense there is. See
//! [`unfold_cut_walk`], which is where it is read rather than checked.

use std::collections::HashMap;

use thiserror::Error;

use crate::builders::errors::ClosedFaceCellError;
use crate::builders::scaffold::{add_closed_face_cell, cut_between_loops, cut_between_shells};
use crate::geometry::{
    Curve, LINEAR_TOLERANCE, NurbsError, Periodicity, Point2, Point3, PointCoincidence, Surface,
    SurfacePeriodicity, TrimmedCurve2, Vector2,
};
use crate::healing::{HealingError, HealingOptions, remove_redundant_cells_edit};
use crate::model::Model;
use crate::topology::attributes::{
    EdgeAttr, FaceAttr, ProfileAttr, SheetAttr, SolidAttr, VertexAttr,
};
use crate::topology::edge::Edge;
use crate::topology::embedding::EntityOwner;
use crate::topology::gmap::{Dart, Dim};
use crate::topology::orientation::Orientation;
use crate::topology::shape::{Shape, SolidTag};
use crate::topology::shape_keys::{EdgeKey, SolidKey, VertexKey};
use crate::topology::{ModelEdit, ModelEditError, StandardPayload};

use super::super::StepImport;
use super::super::convert::curves::read_curve;
use super::super::convert::pcurve::lift_pcurve;
use super::super::convert::placement::read_point;
use super::super::convert::surfaces::read_surface;
use super::super::error::{GeometryError, StepError, TopologyError};
use super::super::options::StepReadOptions;
use super::super::part21::{EntityId, StepExchange};
use super::super::report::{ImportReport, ImportSkip, ImportSkipReason};
use super::super::schema::entities;
use super::super::schema::resolver::{Located, Origin, Resolver, SchemaError};
use crate::geometry::parameter::Fraction;

/// How finely a pcurve is sampled when reading a loop's winding.
///
/// Only the sign of the enclosed area is wanted, and a pcurve that needs more
/// than this to get its *sign* right is degenerate for other reasons.
const WINDING_SAMPLES: usize = 8;

/// Reads every B-Rep solid in a file into its own shape.
///
/// One map per B-Rep: a STEP file is a document holding several products, and
/// a `Shape` owns its map, so the solids cannot share one.
///
/// Both spellings are read. `BREP_WITH_VOIDS` is the `MANIFOLD_SOLID_BREP`
/// subtype that adds cavities, and Part 21 writes a subtype under its own
/// keyword, so a file whose solids are hollow has no instance of the supertype
/// in it at all.
pub fn read_solids(
    exchange: &StepExchange,
    options: &StepReadOptions,
) -> Result<StepImport, StepError> {
    let resolver = Resolver::new(exchange, options.uncertainty)?;
    let mut import = StepImport::default();

    for instance in exchange.instances_of("MANIFOLD_SOLID_BREP") {
        let brep = resolver.decode::<entities::ManifoldSolidBrep>(instance)?;
        let read = read_solid(
            &resolver,
            brep.origin,
            brep.outer,
            &[],
            options,
            &mut import.report,
        );
        collect(read, brep.origin, options, &mut import)?;
    }

    for instance in exchange.instances_of("BREP_WITH_VOIDS") {
        let brep = resolver.decode::<entities::BrepWithVoids>(instance)?;
        let read = read_solid(
            &resolver,
            brep.origin,
            brep.outer,
            &brep.voids,
            options,
            &mut import.report,
        );
        collect(read, brep.origin, options, &mut import)?;
    }

    Ok(import)
}

/// Files one B-Rep's outcome into the import, or reports why it was given up.
fn collect(
    read: Result<Option<Shape<SolidTag, StandardPayload>>, StepError>,
    origin: Origin,
    options: &StepReadOptions,
    import: &mut StepImport,
) -> Result<(), StepError> {
    match read {
        Ok(Some(shape)) => import.shapes.push(shape),
        Ok(None) => {}
        Err(error) if options.strict => return Err(error),
        Err(error) => import.report.skipped.push(ImportSkip {
            entity: Some(origin.id),
            line: origin.line,
            reason: ImportSkipReason::SolidNotConstructible {
                detail: error.to_string(),
            },
        }),
    }
    Ok(())
}

/// Reads one B-Rep, or `None` when nothing in it survived.
fn read_solid(
    resolver: &Resolver<'_>,
    origin: Origin,
    outer: EntityId,
    voids: &[EntityId],
    options: &StepReadOptions,
    report: &mut ImportReport,
) -> Result<Option<Shape<SolidTag, StandardPayload>>, StepError> {
    let mut shells = vec![plan_shell(resolver, origin, outer, options, report)?];
    for &id in voids {
        // A void is named through an `ORIENTED_CLOSED_SHELL`, which may state
        // that the shell is to be read the other way round. NGK stores every
        // shell facing away from the material, so a shell declared reversed is
        // turned here rather than carried as a flag no later reader would
        // consult.
        let oriented = resolver.read::<entities::OrientedClosedShell>(origin, id)?;
        let mut shell = plan_shell(
            resolver,
            oriented.origin,
            oriented.closed_shell_element,
            options,
            report,
        )?;
        if !oriented.orientation {
            shell = shell.iter().map(PlannedFace::reversed).collect();
        }
        shells.push(shell);
    }

    shells.retain(|shell| !shell.is_empty());
    if shells.is_empty() {
        return Ok(None);
    }

    for shell in &shells {
        // A face with no boundary covers a support closed in every direction,
        // so it is a whole shell on its own: anything else in the same shell
        // would have to meet it along an edge it does not have.
        if shell.len() > 1
            && shell
                .iter()
                .any(|face| matches!(face, PlannedFace::Boundaryless { .. }))
        {
            return Err(TopologyError::UnsewableShell {
                brep: origin,
                detail: "a shell holds a boundaryless face beside others".to_string(),
            }
            .into());
        }
        check_edge_uses(shell, origin, options, report)?;
    }

    let mut gmap = Model::<StandardPayload>::new();
    let solid = gmap
        .transaction(|edit| sew_solid(edit, &shells, options.heal_seams))
        .map_err(|error| TopologyError::UnsewableShell {
            brep: origin,
            detail: error.to_string(),
        })?;

    demote_closure_vertices(&mut gmap).map_err(|error| TopologyError::UnsewableShell {
        brep: origin,
        detail: error.to_string(),
    })?;

    Ok(Some(Shape::new(gmap, solid)))
}

/// Classifies the corners STEP demanded but the shape does not have.
///
/// `EDGE_CURVE` names two ends, so a closed curve is written leaving from and
/// arriving at a vertex even where nothing meets there -- a cylinder's rim is
/// one circle, and the file has to give it somewhere to start. Such a vertex
/// ends up alone on a single closed edge, and that is the tell: the point is
/// where the edge's parameterization closes, not a corner of the shape, so it
/// is classified inside the edge rather than standing as a logical vertex the
/// shape never had. A vertex any second edge reaches is a real junction and is
/// left alone, which is what keeps a circle someone deliberately marked marked.
fn demote_closure_vertices(gmap: &mut Model<StandardPayload>) -> Result<(), ModelEditError> {
    let closures: Vec<(VertexKey, Dart, EdgeKey)> = gmap
        .iter_vertices()
        .filter_map(|(key, attr)| {
            let edges = gmap.vertex(key)?.edges();
            let [edge] = edges.as_slice() else {
                return None;
            };
            matches!(edge, Edge::Marked(_) | Edge::Unmarked(_))
                .then(|| (key, attr.dart, edge.key()))
        })
        .collect();
    if closures.is_empty() {
        return Ok(());
    }

    gmap.transaction(|edit| {
        for (vertex, dart, edge) in &closures {
            // Both in one breath: the label says the cell is interior to the
            // edge, which contradicts a logical vertex sitting on it.
            edit.remove_vertex(*vertex);
            edit.own_cell(Dim::Zero, *dart, EntityOwner::Edge(*edge));
        }
        Ok(())
    })
}

/// Plans every `ADVANCED_FACE` of one `CLOSED_SHELL`, dropping the ones that
/// cannot be built.
fn plan_shell(
    resolver: &Resolver<'_>,
    origin: Origin,
    id: EntityId,
    options: &StepReadOptions,
    report: &mut ImportReport,
) -> Result<Vec<PlannedFace>, StepError> {
    let shell = resolver.read::<entities::ClosedShell>(origin, id)?;

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
    Ok(planned)
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
    /// Whether the walk runs the way the support itself does.
    ///
    /// Not the same question as whether it agrees with the `EDGE_CURVE`'s own
    /// start → end: `same_sense` sits between the two. This is the composed
    /// answer, which is what both the edge's reference dart and the α2 pairing
    /// need.
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
enum PlannedFace {
    /// A face STEP bounded with loops, which is nearly all of them.
    Bounded {
        surface: Surface,
        loops: Vec<PlannedLoop>,
        /// Which of `loops` is the outer boundary.
        outer: usize,
    },
    /// A face covering its whole support, bounded by nothing that bounds.
    ///
    /// STEP has no boundaryless face, so a writer spells one as a `VERTEX_LOOP`
    /// naming a point on it. With no boundary there is no winding, and the
    /// sense the file declared is the only statement of which way it points.
    Boundaryless {
        surface: Surface,
        sense: Orientation,
    },
}

impl PlannedUse {
    /// The same use walked the other way.
    fn reversed(&self) -> Self {
        Self {
            edge: self.edge,
            start: self.end,
            end: self.start,
            forward: !self.forward,
            curve: self.curve.clone(),
            start_point: self.end_point,
            end_point: self.start_point,
            pcurve: self.pcurve.reversed(),
        }
    }
}

impl PlannedFace {
    /// The same face turned over, which is how a reversed shell is read.
    ///
    /// Turning a face over means walking every one of its boundaries the other
    /// way: NGK derives a face's normal from that winding, so reversing the
    /// walk is the whole of it and no flag is left behind. The loops keep their
    /// positions, so whichever of them bounds the face from outside still does.
    /// A face with no boundary carries its sense instead, and that is what
    /// flips.
    fn reversed(&self) -> Self {
        match self {
            Self::Bounded {
                surface,
                loops,
                outer,
            } => Self::Bounded {
                surface: surface.clone(),
                loops: loops
                    .iter()
                    .map(|loop_| PlannedLoop {
                        uses: loop_.uses.iter().rev().map(PlannedUse::reversed).collect(),
                        signed_area: -loop_.signed_area,
                    })
                    .collect(),
                outer: *outer,
            },
            Self::Boundaryless { surface, sense } => Self::Boundaryless {
                surface: surface.clone(),
                sense: sense.flip(),
            },
        }
    }

    /// The boundaries the face carries, which a face covering its support has
    /// none of.
    fn loops(&self) -> &[PlannedLoop] {
        match self {
            Self::Bounded { loops, .. } => loops,
            Self::Boundaryless { .. } => &[],
        }
    }
}

/// Reads one `ADVANCED_FACE` without touching a map.
fn plan_face(
    resolver: &Resolver<'_>,
    face: &Located<entities::AdvancedFace>,
    report: &mut ImportReport,
) -> Result<PlannedFace, StepError> {
    let mapped = read_surface(resolver, face.origin, face.face_geometry)?;

    let mut loops = Vec::with_capacity(face.bounds.len());
    let mut declared_outer = None;
    for id in &face.bounds {
        let bound = resolver.read::<entities::FaceBound>(face.origin, *id)?;
        // A `VERTEX_LOOP` bounds nothing: it names one point of a face that
        // covers its whole support, which is how OpenCascade writes a sphere.
        // NGK holds such a face with no boundary at all, so the bound is
        // dropped rather than turned into topology that has no shape.
        if resolver
            .attributes(bound.origin, bound.bound)?
            .is("VERTEX_LOOP")
        {
            continue;
        }
        if bound.outer {
            declared_outer = Some(loops.len());
        }
        loops.push(plan_loop(resolver, &bound, &mapped.surface, report)?);
    }

    if loops.is_empty() {
        return boundaryless_face(&mapped.surface, face);
    }

    let outer = pick_outer(&loops, declared_outer, face.origin, report)?;
    unfold_cut_walk(&mut loops[outer], &mapped.surface, face.same_sense);
    check_sense(&loops[outer], face.same_sense, face.origin, report);

    Ok(PlannedFace::Bounded {
        surface: mapped.surface,
        loops,
        outer,
    })
}

/// Plans a face whose every bound turned out to bound nothing.
///
/// The support has to close in both directions for such a face to be a face
/// at all, and `same_sense` is the only statement of which way it points:
/// there is no winding, and a closed support's two sides are alike until
/// something says otherwise.
fn boundaryless_face(
    surface: &Surface,
    face: &Located<entities::AdvancedFace>,
) -> Result<PlannedFace, StepError> {
    if !surface.is_closed() {
        return Err(SchemaError::UnreadableUnit {
            origin: face.origin,
            detail: "a face bounded by nothing needs a support that closes".to_string(),
        }
        .into());
    }
    Ok(PlannedFace::Boundaryless {
        surface: surface.clone(),
        sense: match face.same_sense {
            true => Orientation::Same,
            false => Orientation::Reversed,
        },
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
    surface: &Surface,
    report: &mut ImportReport,
) -> Result<PlannedLoop, StepError> {
    // `POLY_LOOP` is refused by name rather than skipped: NGK has no
    // representation for it, and a face silently missing a boundary is worse
    // than a face that says why it is missing.
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

        // `EDGE_CURVE.same_sense` says whether the support runs the way the
        // edge does, and on a closed support that is not something the corners
        // can re-derive: two arcs of one circle share both of them. So the
        // walk's direction along the *curve* composes the two flags, and the
        // edge is later rooted on a dart that runs that way — which is what
        // makes NGK's derived span the arc the file meant rather than its
        // complement.
        let along_curve = forward == edge.same_sense;
        let curve = read_curve(resolver, edge.origin, edge.edge_geometry)?;
        let start_point = read_vertex(resolver, edge.origin, start)?;
        let end_point = read_vertex(resolver, edge.origin, end)?;

        // A closed edge on a support with no period -- a circle written as a
        // rational B-spline -- is the whole curve, and only from where its two
        // ends meet: that is the one place such a curve can start a turn.
        // Asked for the span between its corner and itself, `interval_between`
        // answers the empty one, which reads a hole's rim as nothing at all.
        let closed_without_period =
            start == end && matches!(curve.periodicity(), Periodicity::None);
        let span = if closed_without_period {
            if !curve.closes_at(start_point) {
                return Err(SchemaError::UnreadableUnit {
                    origin: edge.origin,
                    detail: "a closed edge on a curve with no period has its corner                              somewhere other than where the curve closes"
                        .to_string(),
                }
                .into());
            }
            match along_curve {
                true => curve.domain(),
                false => curve.domain().reversed(),
            }
        } else if along_curve {
            curve.interval_between(start_point, end_point)
        } else {
            curve.interval_between(end_point, start_point).reversed()
        };
        let lifted = lift_pcurve(surface, &curve, span, resolver.units().uncertainty)
            .map_err(|error| curve_error(edge.origin, error))?;
        if let Some(deviation) = lifted.residual {
            report.skipped.push(ImportSkip {
                entity: Some(edge.origin.id),
                line: edge.origin.line,
                reason: ImportSkipReason::ApproximatedPcurve { deviation },
            });
        }

        uses.push(PlannedUse {
            edge: oriented.edge_element,
            start,
            end,
            forward: along_curve,
            curve,
            start_point,
            end_point,
            pcurve: lifted.pcurve,
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
    join_across_closing_edges(&mut uses, surface);
    let signed_area = signed_area(&uses, surface);
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
///
/// **Each pcurve is placed before it is sampled.** A pcurve is rebuilt by
/// inverting its curve onto the support, and inversion answers within one
/// period — so on a periodic support the two sides of a seam come back at the
/// *same* parameter rather than a period apart, and the loop that was a
/// rectangle in the file shoelaces as a triangle. Shifting each pcurve by whole
/// periods so it continues from the one before is what closes the polygon
/// again, and only then does its sign mean the winding.
///
/// Aligning across a collapsed row is skipped, because a pole is a genuine gap
/// in the walk rather than a seam crossing: shifting there would carry the rest
/// of the loop onto the wrong branch.
fn signed_area(uses: &[PlannedUse], surface: &Surface) -> f64 {
    let periods = periods_of(surface);
    let mut points = Vec::with_capacity(uses.len() * WINDING_SAMPLES);
    let mut offset = Vector2::zeros();
    let mut previous: Option<Point2> = None;

    for use_ in uses {
        let start = use_.pcurve.start();
        if let Some(previous) = previous
            && !(is_degenerate(surface, previous) && is_degenerate(surface, start + offset))
        {
            for (axis, period) in periods.iter().enumerate() {
                let Some(period) = *period else {
                    continue;
                };
                let gap = start[axis] + offset[axis] - previous[axis];
                offset[axis] -= (gap / period).round() * period;
            }
        }
        for sample in 0..WINDING_SAMPLES {
            let fraction = sample as f64 / WINDING_SAMPLES as f64;
            points.push(use_.pcurve.point_at(Fraction::new(fraction)) + offset);
        }
        previous = Some(use_.pcurve.end() + offset);
    }

    shoelace(&points)
}

/// The support's periods, in parameter order.
fn periods_of(surface: &Surface) -> [Option<f64>; 2] {
    match surface.periodicity() {
        SurfacePeriodicity::None => [None, None],
        SurfacePeriodicity::UPeriodic(u) => [Some(u), None],
        SurfacePeriodicity::VPeriodic(v) => [None, Some(v)],
        SurfacePeriodicity::UVPeriodic(u, v) => [Some(u), Some(v)],
    }
}

fn is_degenerate(surface: &Surface, point: Point2) -> bool {
    surface.is_degenerate_at(point.x, point.y)
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

/// Places the return walk of a cut that encloses nothing one period away.
///
/// A boundary that shoelaces to zero states no winding, and a winding is what
/// a face's normal is read from — so a face bounded that way arrives with its
/// sense said nowhere but `ADVANCED_FACE.same_sense`. It happens where a cut
/// is walked out and straight back along one parameter line: a whole sphere,
/// cut open along a meridian and turning through a pole at either end, whose
/// two walks invert to the same longitude because inversion answers within one
/// period and a pole names every longitude at once.
///
/// Putting the return walk one period along the transverse axis makes the
/// boundary the rectangle the domain really is. Which way that period runs is
/// the one thing left for the file to say, and it is the one case where
/// `same_sense` is read rather than merely checked — a rectangle and its
/// mirror describe the same sphere and opposite normals, so there is nothing
/// else to derive the choice from.
fn unfold_cut_walk(loop_: &mut PlannedLoop, surface: &Surface, same_sense: bool) {
    if loop_.signed_area.abs() > LINEAR_TOLERANCE {
        return;
    }
    // The walk back is the second use of an edge the loop already walked, and
    // everything after it belongs on the far side of the cut with it.
    let Some(repeat) = loop_.uses.iter().enumerate().position(|(index, use_)| {
        loop_.uses[..index]
            .iter()
            .any(|other| other.edge == use_.edge)
    }) else {
        return;
    };

    for (axis, period) in periods_of(surface).into_iter().enumerate() {
        let Some(period) = period else {
            continue;
        };
        for step in [period, -period] {
            let mut shift = Vector2::zeros();
            shift[axis] = step;
            let Some(uses) = shifted(&loop_.uses, repeat, shift) else {
                continue;
            };
            let area = signed_area(&uses, surface);
            if area.abs() > LINEAR_TOLERANCE && (area > 0.0) == same_sense {
                loop_.uses = uses;
                loop_.signed_area = area;
                return;
            }
        }
    }
}

/// Moves a pcurve that inverted onto the wrong side of a closing edge.
///
/// A support can close on itself without a period -- a swept spline whose
/// first and last columns are one row of points -- and a curve lying along
/// that row inverts onto either column. A cut walked there both ways then
/// comes back as two walks on one column, and the loop leaves a gap a whole
/// domain wide at each end of one of them. Moving that one across the domain
/// is free, because the two columns are one place on the surface, and it is
/// taken only when it closes the gap on *both* sides: that is what says the
/// walk was on the wrong column rather than somewhere else entirely.
fn join_across_closing_edges(uses: &mut [PlannedUse], surface: &Surface) {
    let count = uses.len();
    if count < 2 {
        return;
    }
    let spans = closing_spans(surface);
    let meets = |a: Point2, b: Point2| (a - b).norm() <= LINEAR_TOLERANCE;
    for index in 0..count {
        let previous = uses[(index + count - 1) % count].pcurve.end();
        let next = uses[(index + 1) % count].pcurve.start();
        let current = &uses[index].pcurve;
        if meets(previous, current.start()) && meets(current.end(), next) {
            continue;
        }
        let mut shifts = Vec::new();
        for (axis, span) in spans.iter().enumerate() {
            let Some(span) = *span else {
                continue;
            };
            for step in [span, -span] {
                let mut shift = Vector2::zeros();
                shift[axis] = step;
                shifts.push(shift);
            }
        }
        for shift in shifts {
            let (start, end) = (current.start() + shift, current.end() + shift);
            if meets(previous, start)
                && meets(end, next)
                && let Ok(moved) = current.translated(shift)
            {
                uses[index].pcurve = moved;
                break;
            }
        }
    }
}

/// How wide the domain is along each axis the support closes on without a
/// period.
///
/// Closing is read off the surface: the two ends of the axis are one row of
/// points, sampled across the other axis.
fn closing_spans(surface: &Surface) -> [Option<f64>; 2] {
    const SAMPLES: usize = 5;
    let periods = periods_of(surface);
    let (u, v) = surface.domain();
    let domains = [u, v];
    [0, 1].map(|axis| {
        let (along, across) = (domains[axis], domains[1 - axis]);
        if periods[axis].is_some() || !along.is_finite() || !across.is_finite() {
            return None;
        }
        let (low, high) = (along.ordered().start.value(), along.ordered().end.value());
        let closes = (0..=SAMPLES).all(|step| {
            let other = across
                .ordered()
                .at(Fraction::new(step as f64 / SAMPLES as f64))
                .value();
            let at = |value: f64| {
                let mut point = Point2::new(other, other);
                point[axis] = value;
                point[1 - axis] = other;
                surface.point_at(point.x, point.y)
            };
            at(low).coincides(at(high), LINEAR_TOLERANCE)
        });
        closes.then_some(high - low)
    })
}

/// The loop's uses with everything from `from` onward moved by `shift`.
fn shifted(uses: &[PlannedUse], from: usize, shift: Vector2) -> Option<Vec<PlannedUse>> {
    uses.iter()
        .enumerate()
        .map(|(index, use_)| {
            if index < from {
                return Some(use_.clone());
            }
            Some(PlannedUse {
                pcurve: use_.pcurve.translated(shift).ok()?,
                ..use_.clone()
            })
        })
        .collect()
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
/// a 3-gmap cannot represent at all, so the solid is refused by name.
fn check_edge_uses(
    planned: &[PlannedFace],
    brep: Origin,
    options: &StepReadOptions,
    report: &mut ImportReport,
) -> Result<(), StepError> {
    let mut counts: HashMap<EntityId, usize> = HashMap::new();
    for face in planned {
        for loop_ in face.loops() {
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

/// Builds every shell of one B-Rep and registers the solid they bound.
///
/// The outer shell comes first and the voids after it, each sewn on its own:
/// a void is disjoint from the material's outside, so nothing is shared
/// between them and an `EDGE_CURVE` naming both would be an edge with four
/// uses rather than a join.
fn sew_solid(
    edit: &mut ModelEdit<'_, StandardPayload>,
    shells: &[Vec<PlannedFace>],
    heal_seams: bool,
) -> Result<SolidKey, SolidSewError> {
    let mut roots = Vec::with_capacity(shells.len());
    for planned in shells {
        let root = sew_shell(edit, planned)?;
        edit.add_sheet(SheetAttr::new(root));
        roots.push(root);
    }

    let [outer, voids @ ..] = roots.as_slice() else {
        unreachable!("a planned solid has at least one shell");
    };
    let inner = (!voids.is_empty()).then(|| voids.to_vec());
    let solid = edit.add_solid(SolidAttr::new(*outer, inner));
    if heal_seams {
        // STEP's synthetic periodic cuts have to come off before the cavity
        // cut is attached. Once that solid-owned face turns through the same
        // edge, the edge is no longer merely a parameterization seam.
        remove_redundant_cells_edit(edit, &HealingOptions::seams_only())?;
    }
    let roots = edit
        .solid_attr_unchecked(solid)
        .shells()
        .collect::<Vec<_>>();
    cut_between_shells(edit, solid, &roots)?;
    Ok(solid)
}

#[derive(Debug, Error)]
enum SolidSewError {
    #[error(transparent)]
    ClosedFace(#[from] ClosedFaceCellError),
    #[error(transparent)]
    Healing(#[from] HealingError),
    #[error(transparent)]
    ModelEdit(#[from] ModelEditError),
}

/// Builds every planned face of one shell and sews them together.
///
/// Returns the dart the shell is anchored at, which carries its direction.
///
/// A shell that is one face covering a closed support gets the polygon schema
/// of that surface underneath it — the 2-cell the face is required to occupy —
/// and anchors there. The file's `same_sense` flag is the only statement of
/// which way such a face points, since it has no boundary whose winding could
/// say, so a reversed one anchors at the `alpha0` partner instead.
fn sew_shell(
    edit: &mut ModelEdit<'_, StandardPayload>,
    planned: &[PlannedFace],
) -> Result<Dart, ClosedFaceCellError> {
    if let [PlannedFace::Boundaryless { surface, sense }] = planned {
        let cell = add_closed_face_cell(edit, surface)?;
        let face = edit.add_face(FaceAttr::closed(
            surface.clone(),
            cell.anchor(),
            HashMap::new(),
        ));
        cell.own(edit, face);
        return Ok(match sense {
            Orientation::Same => cell.anchor(),
            Orientation::Reversed => edit.alpha(Dim::Zero, cell.anchor()),
        });
    }

    // One entry per loop of every face, in the same order, so a face can find
    // the darts its own boundaries were built from.
    let mut loop_darts: Vec<Vec<Vec<Dart>>> = Vec::with_capacity(planned.len());
    let mut uses_by_edge: HashMap<EntityId, Vec<DartUse>> = HashMap::new();

    for face in planned {
        let mut face_darts = Vec::with_capacity(face.loops().len());
        for loop_ in face.loops() {
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
        let PlannedFace::Bounded {
            surface,
            loops,
            outer,
        } = face
        else {
            continue;
        };
        for (loop_, darts) in loops.iter().zip(face_darts) {
            for (index, use_) in loop_.uses.iter().enumerate() {
                let (start, end) = (darts[2 * index], darts[2 * index + 1]);

                if vertices.insert(use_.start, ()).is_none() {
                    edit.add_vertex(VertexAttr::new(start, use_.start_point));
                }
                if vertices.insert(use_.end, ()).is_none() {
                    edit.add_vertex(VertexAttr::new(end, use_.end_point));
                }

                if edges.insert(use_.edge, ()).is_none() {
                    // The reference dart must run the way the *support* does.
                    // An `EdgeAttr` stores no interval, so NGK re-derives the
                    // span as the one running forward along the curve from the
                    // reference dart's corner — which is the arc the file named
                    // only if the dart leaves from where the curve starts. On a
                    // circle the other choice is the complementary arc, which
                    // is a different edge of a different shape.
                    let reference = if use_.forward { start } else { end };
                    edit.add_edge(EdgeAttr::new(reference, use_.curve.clone()));
                }
            }
            edit.add_profile(ProfileAttr::new(darts[0]));
        }

        let outer_seed = face_darts[*outer][0];
        let inner_seeds: Vec<Dart> = face_darts
            .iter()
            .enumerate()
            .filter(|(index, _)| index != outer)
            .map(|(_, darts)| darts[0])
            .collect();
        let pcurves: HashMap<Dart, TrimmedCurve2> = loops
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

        let face = edit.add_face(FaceAttr::with_pcurves(
            surface.clone(),
            outer_seed,
            inner_seeds.clone(),
            pcurves,
        ));
        // STEP hands a face its bounds as a list and says nothing about how they
        // connect; a face is one 2-cell only once something joins them. Each
        // further bound is reached from the outer one along a cut the face owns,
        // so the map carries what the list only asserted.
        for inner_seed in inner_seeds {
            cut_between_loops(edit, face, outer_seed, inner_seed)?;
        }
        shell_root.get_or_insert(outer_seed);
    }

    // Any boundary dart names the whole shell, and every face was built in the
    // direction STEP composed, so the first face's outer seed already carries
    // the side facing away from the material.
    Ok(shell_root.expect("a planned shell has at least one bounded face"))
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
