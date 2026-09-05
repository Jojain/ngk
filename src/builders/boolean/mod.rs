//! Boolean preparation: contact computation and two-sided B-Rep splitting.

mod assemble;
mod broad_phase;
mod classify;
mod clip;
mod contacts;
mod diagnostics;
mod errors;
mod neighborhood;
mod select;
pub use diagnostics::BooleanDiagnostics;
mod graph;
mod imprint;
mod operand;
mod result;
mod tolerance;
mod trim;
pub use tolerance::{BooleanTolerancePolicy, BooleanTolerances};

pub use classify::solid_contains_point;
use contacts::{
    compute_edge_contacts, compute_edge_face_contacts, compute_face_contacts,
    compute_vertex_contacts, normalize_face_imprint_chains, reroute_boundary_imprints,
};
pub use errors::BooleanError;
pub use graph::{
    IntersectionEvent, IntersectionEventId, IntersectionEventLocation, IntersectionEventUse,
    IntersectionNetwork, IntersectionNetworkValidationError, IntersectionOrientation,
    IntersectionRegion, IntersectionSpan, IntersectionSpanId, IntersectionSpanKind,
    IntersectionSpanUse, validate_solid_network,
};
use graph::{IntersectionNetworkBuilder, edge_use, face_use, vertex_use};
use operand::{BooleanContext, OperandCells, import_operand, operand_cells};
pub use result::{
    BooleanCell, BooleanLineage, BooleanOperand, BooleanOperation, BooleanPreparation,
    BooleanResult, BooleanResultLineage, BooleanSide, PointContactKind,
};

use std::collections::{BTreeSet, HashMap, HashSet};

use crate::builders::edges::split_edge_staged;
use crate::builders::faces::{FaceImprint, split_face_by_imprints_staged, split_face_edge_staged};
use crate::geometry::{
    ControlPolygon, ControlPolygon2, Curve, Curve2, CurveCurveIntersection,
    CurveSurfaceIntersection, Degree, HPoint, HPoint2, IntersectionOptions, Interval, KnotVector,
    Line2, NurbsCurve, NurbsCurve2, NurbsError, Point2, Point3, PointCoincidence, PreparedCurve,
    PreparedSurface, Surface, SurfaceSurfaceIntersection, intersect_prepared_curve_surface,
};
use crate::topology::TopologyEdit;
use crate::topology::gmap::GMap;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey, SolidKey, VertexKey};
use nalgebra::Vector2;
use slotmap::Key;

/// Raw narrow-phase observation consumed during network canonicalization.
#[derive(Clone)]
enum RawIntersection {
    Point {
        point: Point3,
        first: BooleanCell,
        second: BooleanCell,
        kind: PointContactKind,
    },
    Overlap {
        first_edge: EdgeKey,
        second_edge: EdgeKey,
        first_interval: Interval,
        second_interval: Interval,
    },
    Region {
        first_face: FaceKey,
        second_face: FaceKey,
    },
    /// A contact section one operand's existing edge already realizes.
    EdgeSection {
        side: BooleanSide,
        edge: EdgeKey,
        curve: Curve,
        interval: Interval,
    },
}

/// Tunables used by Boolean intersection and splitting.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BooleanOptions {
    pub intersections: IntersectionOptions,
    pub tolerances: BooleanTolerancePolicy,
    pub max_classification_rays: usize,
    pub strict: bool,
}

impl Default for BooleanOptions {
    fn default() -> Self {
        Self {
            intersections: IntersectionOptions::default(),
            tolerances: BooleanTolerancePolicy::default(),
            max_classification_rays: 16,
            strict: true,
        }
    }
}

/// Evaluates one regularized Boolean inside a single transaction.
/// Consumes the operand boundary registrations on success. Empty and disconnected
/// results, ambiguous classification, or incomplete geometric coverage roll back.
/// The current certified classification path admits planar polygonal boundaries.
pub fn boolean<P: Payload>(
    map: &mut GMap<P>,
    first: SolidKey,
    second: SolidKey,
    operation: BooleanOperation,
    options: BooleanOptions,
) -> Result<BooleanResult, BooleanError> {
    let context = BooleanContext::admit(map, first, second, operation, options)?;
    map.transaction(|edit| {
        if first == second {
            if operation == BooleanOperation::Difference {
                return Err(BooleanError::EmptyResult);
            }
            let cells = operand_cells(edit, BooleanOperand::Solid(first))?;
            return Ok(BooleanResult {
                operation,
                solid: first,
                lineage: BooleanResultLineage {
                    first: lineage_for(&cells, &HashMap::new(), &HashMap::new()),
                    second: BooleanLineage::default(),
                    span_edges: HashMap::new(),
                    discarded_faces: Vec::new(),
                },
                diagnostics: BooleanDiagnostics {
                    tolerances: context.tolerances,
                    ..Default::default()
                },
            });
        }
        let plan = compute_boolean_intersections(
            edit,
            BooleanOperand::Solid(first),
            BooleanOperand::Solid(second),
            context.options,
        )?;
        if !plan.diagnostics.coverage.is_empty()
            || plan.diagnostics.branches_uncertified > 0
            || !plan.diagnostics.unresolved_overlaps.is_empty()
        {
            return Err(BooleanError::IncompleteIntersections {
                diagnostics: Box::new(plan.diagnostics),
            });
        }
        validate_solid_network(edit, &plan.network, context.tolerances)?;
        let mut prepared = apply_boolean_splits_staged(edit, plan, false)?;
        let graph = neighborhood::FragmentGraph::build(edit, &prepared);
        let (classes, rays) = classify::run(edit, &graph, context.options, context.tolerances)?;
        prepared.diagnostics.classification_rays = rays;
        let selection = select::run(operation, &graph, &classes);
        assemble::run(edit, &context, &graph, prepared, selection)
    })
}
/// A non-mutating contact plan for two operands already in one map.
#[derive(Clone)]
pub struct BooleanIntersectionPlan {
    pub first: BooleanOperand,
    pub second: BooleanOperand,
    pub network: IntersectionNetwork,
    pub diagnostics: BooleanDiagnostics,
    options: BooleanOptions,
    face_imprints: HashMap<FaceKey, Vec<imprint::SpanImprint>>,
    first_cells: OperandCells,
    second_cells: OperandCells,
}

/// Mutable narrow-phase observations discarded after network canonicalization.
struct IntersectionAccumulator {
    diagnostics: BooleanDiagnostics,
    contacts: Vec<RawIntersection>,
    first_cells: OperandCells,
    second_cells: OperandCells,
    edge_points: HashMap<EdgeKey, Vec<Point3>>,
    face_imprints: HashMap<FaceKey, Vec<FaceImprint>>,
}

/// Computes all contacts between two operands without modifying the map.
pub fn compute_boolean_intersections<P: Payload>(
    g: &GMap<P>,
    first: BooleanOperand,
    second: BooleanOperand,
    options: BooleanOptions,
) -> Result<BooleanIntersectionPlan, BooleanError> {
    if !options.intersections.validate() {
        return Err(crate::geometry::IntersectionError::InvalidOptions.into());
    }
    let first_cells = operand_cells(g, first)?;
    let second_cells = operand_cells(g, second)?;
    let tolerances =
        BooleanTolerances::from_cells(g, &first_cells, &second_cells, options.tolerances)?;
    let mut options = options;
    tolerances.apply(&mut options.intersections);
    let mut observations = IntersectionAccumulator {
        diagnostics: BooleanDiagnostics {
            tolerances,
            ..Default::default()
        },
        contacts: Vec::new(),
        first_cells,
        second_cells,
        edge_points: HashMap::new(),
        face_imprints: HashMap::new(),
    };

    compute_vertex_contacts(g, &mut observations, options)?;
    compute_edge_contacts(g, &mut observations, options)?;
    compute_edge_face_contacts(g, &mut observations, options)?;
    compute_face_contacts(g, &mut observations, options)?;
    reroute_boundary_imprints(g, &mut observations, options);
    normalize_face_imprint_chains(g, &mut observations, options)?;
    let observed_network = build_intersection_network(g, &observations, options)?;
    let mut face_imprints = imprint::face_imprints(&observed_network);
    let (mut network, subdivision) =
        graph::finalize_network(&observed_network, tolerances.linear, tolerances.parameter)?;
    graph::close_regions(&mut network, g)?;
    for imprint in face_imprints.values_mut().flatten() {
        imprint.pieces = subdivision[imprint.span.0].clone();
        if imprint.orientation == IntersectionOrientation::Reversed {
            for piece in &mut imprint.pieces {
                piece.interval =
                    Interval::new(1.0 - piece.interval.end, 1.0 - piece.interval.start);
                piece.reversed = !piece.reversed;
            }
        }
    }
    observations.diagnostics.events = network.events().len();
    observations.diagnostics.spans = network.spans().len();
    observations.diagnostics.regions = network.regions().len();
    Ok(BooleanIntersectionPlan {
        diagnostics: observations.diagnostics,
        face_imprints,
        options,
        first,
        second,
        network,
        first_cells: observations.first_cells,
        second_cells: observations.second_cells,
    })
}

/// Canonicalizes the raw narrow-phase observations into the shared network.
fn build_intersection_network<P: Payload>(
    g: &GMap<P>,
    plan: &IntersectionAccumulator,
    options: BooleanOptions,
) -> Result<IntersectionNetwork, BooleanError> {
    let mut builder = IntersectionNetworkBuilder::new(options.intersections.linear_tolerance);

    for contact in &plan.contacts {
        match contact {
            RawIntersection::Point {
                point,
                first,
                second,
                kind,
            } => {
                let first_use = event_use_for_cell(g, BooleanSide::First, *first, *point);
                let second_use = event_use_for_cell(g, BooleanSide::Second, *second, *point);
                builder.record_event(*point, *kind, [first_use, second_use]);
            }
            RawIntersection::Overlap {
                first_edge,
                second_edge,
                first_interval,
                second_interval,
            } => {
                let first_edge_view = g.edge_unchecked(*first_edge);
                let first_curve = first_edge_view.curve().expect("registered edge geometry");
                let start = first_curve.point_at(first_interval.start);
                let end = first_curve.point_at(first_interval.end);
                let curve = Curve::line(start, end);
                builder.record_span(
                    curve,
                    IntersectionSpanKind::Overlap,
                    [
                        edge_use(BooleanSide::First, *first_edge, first_interval.start),
                        edge_use(BooleanSide::Second, *second_edge, second_interval.start),
                    ],
                    [
                        edge_use(BooleanSide::First, *first_edge, first_interval.end),
                        edge_use(BooleanSide::Second, *second_edge, second_interval.end),
                    ],
                    [
                        IntersectionSpanUse::Edge {
                            side: BooleanSide::First,
                            edge: *first_edge,
                            interval: *first_interval,
                        },
                        IntersectionSpanUse::Edge {
                            side: BooleanSide::Second,
                            edge: *second_edge,
                            interval: *second_interval,
                        },
                    ],
                );
            }
            RawIntersection::Region {
                first_face,
                second_face,
            } => builder.record_region(*first_face, *second_face),
            RawIntersection::EdgeSection {
                side,
                edge,
                curve,
                interval,
            } => {
                builder.record_span(
                    curve.clone(),
                    IntersectionSpanKind::Overlap,
                    [edge_use(*side, *edge, interval.start)],
                    [edge_use(*side, *edge, interval.end)],
                    [IntersectionSpanUse::Edge {
                        side: *side,
                        edge: *edge,
                        interval: *interval,
                    }],
                );
            }
        }
    }

    let mut faces = plan.face_imprints.iter().collect::<Vec<_>>();
    faces.sort_by_key(|(face, _)| face.data().as_ffi());
    for (face, imprints) in faces {
        let side = if plan.first_cells.faces.contains(face) {
            BooleanSide::First
        } else {
            BooleanSide::Second
        };
        for imprint in imprints {
            let start_uv = imprint.pcurve.point_at(0.0);
            let end_uv = imprint.pcurve.point_at(1.0);
            builder.record_span(
                imprint.curve.clone(),
                IntersectionSpanKind::Transverse,
                [face_use(side, *face, start_uv)],
                [face_use(side, *face, end_uv)],
                [IntersectionSpanUse::Face {
                    side,
                    face: *face,
                    pcurve: Box::new(imprint.pcurve.clone()),
                    orientation: IntersectionOrientation::Forward,
                }],
            );
        }
    }

    Ok(builder.finish()?)
}

fn event_use_for_cell<P: Payload>(
    g: &GMap<P>,
    side: BooleanSide,
    cell: BooleanCell,
    point: Point3,
) -> IntersectionEventUse {
    match cell {
        BooleanCell::Vertex(_) => vertex_use(side, cell),
        BooleanCell::Edge(edge) => {
            let parameter = g
                .edge_unchecked(edge)
                .curve()
                .expect("registered edge geometry")
                .param_at(point);
            edge_use(side, edge, parameter)
        }
        BooleanCell::Face(face) => {
            let uv = g
                .face_unchecked(face)
                .surface()
                .closest_parameter(point)
                .expect("recorded face contact must project onto its face");
            face_use(side, face, uv)
        }
    }
}

/// Applies a previously computed plan in one topology transaction.
pub fn apply_boolean_splits<P: Payload>(
    g: &mut GMap<P>,
    plan: BooleanIntersectionPlan,
) -> Result<BooleanPreparation, BooleanError> {
    g.transaction(|edit| apply_boolean_splits_staged(edit, plan, false))
}

/// Computes contacts and splits two operands already stored in the same map.
pub fn prepare_boolean<P: Payload>(
    g: &mut GMap<P>,
    first: BooleanOperand,
    second: BooleanOperand,
    options: BooleanOptions,
) -> Result<BooleanPreparation, BooleanError> {
    let plan = compute_boolean_intersections(g, first, second, options)?;
    apply_boolean_splits(g, plan)
}

/// Copies an external tool into `target_map`, then splits both working operands.
///
/// The source `tool_map` is only read. Import, contact computation, and all
/// splits share one target-map transaction, so any failure removes the copy.
pub fn prepare_boolean_with_external_tool<P: Payload>(
    target_map: &mut GMap<P>,
    target: BooleanOperand,
    tool_map: &GMap<P>,
    tool: BooleanOperand,
    options: BooleanOptions,
) -> Result<BooleanPreparation, BooleanError> {
    operand_cells(target_map, target)?;
    operand_cells(tool_map, tool)?;
    target_map.transaction(|edit| {
        let imported = import_operand(edit, tool_map, tool)?;
        let plan = compute_boolean_intersections(edit, target, imported, options)?;
        apply_boolean_splits_staged(edit, plan, true)
    })
}

fn apply_boolean_splits_staged<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    plan: BooleanIntersectionPlan,
    imported_second: bool,
) -> Result<BooleanPreparation, BooleanError> {
    // Revalidate source handles so applying an old plan fails atomically.
    revalidate_plan_operand(g, plan.first)?;
    revalidate_plan_operand(g, plan.second)?;

    let edge_points = imprint::edge_points(&plan.network);
    let mut edge_lineage = HashMap::new();
    for source in plan
        .first_cells
        .edges
        .iter()
        .chain(&plan.second_cells.edges)
        .copied()
        .collect::<BTreeSet<_>>()
    {
        let points = edge_points.get(&source).cloned().unwrap_or_default();
        edge_lineage.insert(
            source,
            split_edge_at_points(g, source, points, plan.options.intersections)?,
        );
    }

    let face_imprints = plan.face_imprints;
    let mut face_lineage = HashMap::new();
    let mut span_sections = HashMap::<IntersectionSpanId, [Vec<(f64, EdgeKey)>; 2]>::new();
    for (span, side, edge) in imprint::realize_edge_spans(
        g,
        &plan.network,
        &edge_lineage,
        plan.options.intersections.linear_tolerance,
    ) {
        let index = match side {
            BooleanSide::First => 0,
            BooleanSide::Second => 1,
        };
        span_sections.entry(span).or_default()[index].push((0.0, edge));
    }
    for source in plan
        .first_cells
        .faces
        .iter()
        .chain(&plan.second_cells.faces)
        .copied()
        .collect::<BTreeSet<_>>()
    {
        let imprints = face_imprints.get(&source).cloned().unwrap_or_default();
        let curves = imprints
            .iter()
            .map(|imprint| imprint.imprint.clone())
            .collect::<Vec<_>>();
        let splits = split_face_by_imprints_staged(g, source, &curves)?;
        for section in splits.iter().flat_map(|split| &split.sections) {
            let imprint = &imprints[section.imprint];
            let side = match imprint.side {
                BooleanSide::First => 0,
                BooleanSide::Second => 1,
            };
            for (span, parameter, edge) in imprint::realize_section(
                g,
                imprint,
                section,
                plan.options.intersections.parameter_tolerance,
            )? {
                span_sections.entry(span).or_default()[side].push((parameter, edge));
            }
        }
        let mut fragments = vec![source];
        fragments.extend(splits.into_iter().map(|split| split.second));
        fragments.sort_by_key(|key| key.data().as_ffi());
        fragments.dedup();
        face_lineage.insert(source, fragments);
    }

    let first_lineage = lineage_for(&plan.first_cells, &edge_lineage, &face_lineage);
    let second_lineage = lineage_for(&plan.second_cells, &edge_lineage, &face_lineage);
    Ok(BooleanPreparation {
        first: plan.first,
        second: plan.second,
        imported_tool: imported_second.then_some(plan.second),
        imported_second,
        network: plan.network,
        diagnostics: plan.diagnostics,
        span_edges: span_sections
            .into_iter()
            .map(|(span, sides)| {
                let edges = sides.map(|mut sections| {
                    sections.sort_by(|a, b| {
                        a.0.total_cmp(&b.0)
                            .then_with(|| a.1.data().as_ffi().cmp(&b.1.data().as_ffi()))
                    });
                    sections.dedup_by_key(|section| section.1);
                    sections.into_iter().map(|(_, edge)| edge).collect()
                });
                (span, edges)
            })
            .collect(),
        first_lineage,
        second_lineage,
    })
}

fn revalidate_plan_operand<P: Payload>(
    g: &GMap<P>,
    operand: BooleanOperand,
) -> Result<(), BooleanError> {
    operand_cells(g, operand)
        .map(|_| ())
        .map_err(|error| match error {
            BooleanError::MissingOperand { .. } => BooleanError::StalePlan { operand },
            other => other,
        })
}

fn lineage_for(
    cells: &OperandCells,
    edges: &HashMap<EdgeKey, Vec<EdgeKey>>,
    faces: &HashMap<FaceKey, Vec<FaceKey>>,
) -> BooleanLineage {
    BooleanLineage {
        vertices: cells
            .vertices
            .iter()
            .copied()
            .map(|key| (key, vec![key]))
            .collect(),
        edges: cells
            .edges
            .iter()
            .copied()
            .map(|key| (key, edges.get(&key).cloned().unwrap_or_else(|| vec![key])))
            .collect(),
        faces: cells
            .faces
            .iter()
            .copied()
            .map(|key| (key, faces.get(&key).cloned().unwrap_or_else(|| vec![key])))
            .collect(),
    }
}

fn split_edge_at_points<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    source: EdgeKey,
    mut points: Vec<Point3>,
    options: IntersectionOptions,
) -> Result<Vec<EdgeKey>, BooleanError> {
    let source_curve = g
        .edge(source)
        .and_then(|edge| edge.curve().cloned())
        .ok_or(BooleanError::MissingOperand {
            operand: BooleanOperand::Edge(source),
        })?;
    points.sort_by(|a, b| {
        source_curve
            .param_at(*a)
            .total_cmp(&source_curve.param_at(*b))
    });
    points.dedup_by(|a, b| a.coincides(*b, options.linear_tolerance));

    let mut fragments = vec![source];
    for point in points {
        let Some(fragment) = fragments.iter().copied().find(|edge| {
            let view = g.edge_unchecked(*edge);
            let Some(curve) = view.curve() else {
                return false;
            };
            let start = *view.start().point().expect("edge start geometry");
            let end = *view.end().point().expect("edge end geometry");
            let parameter = curve.param_at(point);
            let domain = curve.parameters_between(start, end).ordered();
            domain.contains(parameter, options.parameter_tolerance)
                && (parameter - domain.start).abs() > options.parameter_tolerance
                && (parameter - domain.end).abs() > options.parameter_tolerance
        }) else {
            continue;
        };

        let view = g.edge_unchecked(fragment);
        let parameter = view
            .curve()
            .expect("registered edge geometry")
            .param_at(point);
        let incident_face = view.faces().first().map(|face| face.key());
        let split = if let Some(face) = incident_face {
            split_face_edge_staged(g, face, fragment, parameter)?
        } else {
            split_edge_staged(g, fragment, parameter)?
        };
        fragments.push(split.second);
    }
    Ok(fragments)
}
