//! Boolean operation orchestration.

use super::*;

#[derive(Clone)]
pub(crate) enum RawIntersection {
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
        /// The section in its own support's parameters, which need not be the
        /// edge's: a clipped arc is carried as the exact NURBS its pcurve was
        /// fitted to. Where the section sits *on the edge* is therefore
        /// recovered from its points, the same rule that locates every other
        /// event on that edge.
        curve: TrimmedCurve,
    },
}

/// Tunables used by Boolean intersection and splitting.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BooleanOptions {
    pub intersections: IntersectionOptions,
    pub tolerances: BooleanTolerancePolicy,
    pub max_classification_rays: usize,
    pub strict: bool,
    /// Remove the redundant topology imprinting leaves behind.
    ///
    /// Splitting one operand against the other creates edges and vertices that
    /// carry no shape wherever a contact falls on geometry the result keeps.
    /// When set, [`crate::healing`] runs over the result solid inside the same
    /// transaction, using the Boolean's own tolerances rather than the
    /// kernel's default ones.
    pub heal: bool,
}

impl Default for BooleanOptions {
    fn default() -> Self {
        Self {
            intersections: IntersectionOptions::default(),
            tolerances: BooleanTolerancePolicy::default(),
            max_classification_rays: 16,
            strict: true,
            heal: true,
        }
    }
}

/// Evaluates one regularized Boolean inside a single transaction.
/// Consumes the operand boundary registrations on success. Empty and disconnected
/// results, ambiguous classification, or incomplete geometric coverage roll back.
/// The current certified classification path admits planar polygonal boundaries.
pub fn boolean<P: Payload>(
    map: &mut Model<P>,
    first: SolidKey,
    second: SolidKey,
    operation: BooleanOperation,
    options: BooleanOptions,
) -> Result<SolidBoolean, BooleanError> {
    let context = BooleanContext::admit(map, first, second, operation, options)?;
    map.transaction(|edit| {
        if first == second {
            if operation == BooleanOperation::Difference {
                return Err(BooleanError::EmptyResult);
            }
            let cells = operand_cells(edit, BooleanOperand::Solid(first))?;
            return Ok(SolidBoolean {
                operation,
                solid: first,
                lineage: SolidBooleanLineage {
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
        let prepared = apply_boolean_splits_edit(edit, plan, false)?;
        let graph = neighborhood::FragmentGraph::<SolidDomain>::build(&prepared);
        let classes = classify::run(edit, &prepared, &graph, context.options, context.tolerances)?;
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
pub(crate) struct IntersectionAccumulator {
    pub(crate) diagnostics: BooleanDiagnostics,
    pub(crate) contacts: Vec<RawIntersection>,
    pub(crate) first_cells: OperandCells,
    pub(crate) second_cells: OperandCells,
    pub(crate) edge_points: HashMap<EdgeKey, Vec<Point3>>,
    pub(crate) face_imprints: HashMap<FaceKey, Vec<FaceImprint>>,
    pub(crate) tangent_face_imprints: HashMap<FaceKey, Vec<FaceImprint>>,
}

/// Computes all contacts between two operands without modifying the map.
pub fn compute_boolean_intersections<P: Payload>(
    g: &Model<P>,
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
        tangent_face_imprints: HashMap::new(),
    };

    compute_contacts(g, &mut observations, options)?;
    reroute_boundary_imprints(g, &mut observations, options);
    normalize_face_imprint_chains(g, &mut observations, options)?;
    let observed_network = build_intersection_network(g, &observations, options)?;
    let mut face_imprints = imprint::face_imprints(&observed_network);
    let (mut network, embedding) = graph::finalize_network(
        g,
        &observed_network,
        tolerances.linear,
        tolerances.parameter,
    )?;
    graph::close_regions(&mut network, g)?;
    for imprint in face_imprints.values_mut().flatten() {
        imprint.pieces = embedding[imprint.span.0].clone();
        if imprint.orientation == IntersectionOrientation::Reversed {
            for piece in &mut imprint.pieces {
                piece.interval = Interval::new(
                    1.0 - piece.interval.end.value(),
                    1.0 - piece.interval.start.value(),
                );
                piece.reversed = !piece.reversed;
            }
        }
    }
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
    g: &Model<P>,
    plan: &IntersectionAccumulator,
    options: BooleanOptions,
) -> Result<IntersectionNetwork, BooleanError> {
    let mut builder = IntersectionNetworkBuilder::new(g, options.intersections.linear_tolerance);

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
                let first_curve = first_edge_view.curve();
                let start = first_curve.point_at(first_interval.start);
                let end = first_curve.point_at(first_interval.end);
                let curve = TrimmedCurve::segment(start, end);
                builder.record_span(
                    curve,
                    IntersectionSpanKind::Overlap,
                    [
                        edge_use(
                            BooleanSide::First,
                            *first_edge,
                            first_interval.start.value(),
                        ),
                        edge_use(
                            BooleanSide::Second,
                            *second_edge,
                            second_interval.start.value(),
                        ),
                    ],
                    [
                        edge_use(BooleanSide::First, *first_edge, first_interval.end.value()),
                        edge_use(
                            BooleanSide::Second,
                            *second_edge,
                            second_interval.end.value(),
                        ),
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
            RawIntersection::EdgeSection { side, edge, curve } => {
                let edge_view = g.edge_unchecked(*edge);
                let edge_curve = edge_view.curve();
                let edge_interval = edge_section_parameters(edge_curve, curve);
                builder.record_span(
                    curve.clone(),
                    IntersectionSpanKind::Overlap,
                    [edge_use(*side, *edge, edge_interval.start.value())],
                    [edge_use(*side, *edge, edge_interval.end.value())],
                    [IntersectionSpanUse::Edge {
                        side: *side,
                        edge: *edge,
                        interval: edge_interval,
                    }],
                );
            }
        }
    }

    for (face_imprints, kind) in [
        (&plan.face_imprints, IntersectionSpanKind::Transverse),
        (&plan.tangent_face_imprints, IntersectionSpanKind::Tangent),
    ] {
        let mut faces = face_imprints.iter().collect::<Vec<_>>();
        faces.sort_by_key(|(face, _)| face.data().as_ffi());
        for (face, imprints) in faces {
            let side = if plan.first_cells.faces.contains(face) {
                BooleanSide::First
            } else {
                BooleanSide::Second
            };
            for imprint in imprints {
                let start_uv = imprint.pcurve.point_at(Fraction::new(0.0));
                let end_uv = imprint.pcurve.point_at(Fraction::new(1.0));
                builder.record_span(
                    imprint.curve.clone(),
                    kind,
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
    }

    Ok(builder.finish()?)
}

/// Locates a section on the edge that already realizes it, in that edge curve's
/// own native parameters.
///
/// The section carries its own support and parameterization — a clipped arc is
/// an exact NURBS, not the edge's circle — so its interval says nothing about
/// where the section sits on the edge. Only the points are shared, and
/// [`event_use_for_cell`] locates every other event on that edge from its point
/// by the same rule, so the two agree by construction.
///
/// A periodic edge reports its parameter on one fixed branch — a circle uses
/// `atan2`, so `(-pi, pi]` — which the section may cross. Unwrapping through the
/// section's midpoint keeps the returned interval monotone along the section's
/// own sweep instead of folding it back over the branch cut.
fn edge_section_parameters(edge_curve: &Curve, section: &TrimmedCurve) -> Interval {
    let point_at = |parameter: f64| section.point_at(Fraction::new(parameter));
    let start = edge_curve.parameter_at(point_at(0.0));
    let Periodicity::Periodic(period) = edge_curve.periodicity() else {
        return Interval::new(start, edge_curve.parameter_at(point_at(1.0)));
    };
    let continued = |previous: f64, point: Point3| {
        let offset = (edge_curve.parameter_at(point).value() - previous).rem_euclid(period);
        let offset = if offset > 0.5 * period {
            offset - period
        } else {
            offset
        };
        previous + offset
    };
    let middle = continued(start.value(), point_at(0.5));
    Interval::new(start, continued(middle, point_at(1.0)))
}

fn event_use_for_cell<P: Payload>(
    g: &Model<P>,
    side: BooleanSide,
    cell: BooleanCell,
    point: Point3,
) -> IntersectionEventUse {
    match cell {
        BooleanCell::Vertex(_) => vertex_use(side, cell),
        BooleanCell::Edge(edge) => {
            let parameter = g.edge_unchecked(edge).curve().parameter_at(point);
            edge_use(side, edge, parameter.value())
        }
        BooleanCell::Face(face) => {
            let uv = g
                .face_unchecked(face)
                .surface()
                .param_at(point)
                .expect("recorded face contact must project onto its face");
            face_use(side, face, uv)
        }
    }
}

/// Applies a previously computed plan in one topology transaction.
pub fn apply_boolean_splits<P: Payload>(
    g: &mut Model<P>,
    plan: BooleanIntersectionPlan,
) -> Result<BooleanOperandPreparation, BooleanError> {
    g.transaction(|edit| apply_boolean_splits_edit(edit, plan, false))
}

/// Computes contacts and splits two operands already stored in the same map.
pub fn prepare_boolean<P: Payload>(
    g: &mut Model<P>,
    first: BooleanOperand,
    second: BooleanOperand,
    options: BooleanOptions,
) -> Result<BooleanOperandPreparation, BooleanError> {
    let plan = compute_boolean_intersections(g, first, second, options)?;
    apply_boolean_splits(g, plan)
}

/// Copies an external tool into `target_map`, then splits both working operands.
///
/// The source `tool_map` is only read. Import, contact computation, and all
/// splits share one target-map transaction, so any failure removes the copy.
pub fn prepare_boolean_with_external_tool<P: Payload>(
    target_map: &mut Model<P>,
    target: BooleanOperand,
    tool_map: &Model<P>,
    tool: BooleanOperand,
    options: BooleanOptions,
) -> Result<BooleanOperandPreparation, BooleanError> {
    operand_cells(target_map, target)?;
    operand_cells(tool_map, tool)?;
    target_map.transaction(|edit| {
        let imported = import_operand(edit, tool_map, tool)?;
        let plan = compute_boolean_intersections(edit, target, imported, options)?;
        apply_boolean_splits_edit(edit, plan, true)
    })
}

pub(super) fn apply_boolean_splits_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    plan: BooleanIntersectionPlan,
    imported_second: bool,
) -> Result<BooleanOperandPreparation, BooleanError> {
    // Revalidate source handles so applying an old plan fails atomically.
    revalidate_plan_operand(edit, plan.first)?;
    revalidate_plan_operand(edit, plan.second)?;

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
            split_edge_at_points(edit, source, points, plan.options.intersections)?,
        );
    }

    let face_imprints = plan.face_imprints;
    let mut face_lineage = HashMap::new();
    let mut span_sections = HashMap::<IntersectionSpanId, [Vec<(f64, EdgeKey)>; 2]>::new();
    for (span, side, edge) in imprint::realize_edge_spans(
        edit,
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
        let splits = split_face_by_imprints_edit(edit, source, &curves)?;
        for section in splits.iter().flat_map(|split| &split.sections) {
            let imprint = &imprints[section.imprint];
            let side = match imprint.side {
                BooleanSide::First => 0,
                BooleanSide::Second => 1,
            };
            for (span, parameter, edge) in imprint::realize_section(
                edit,
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
    Ok(BooleanOperandPreparation {
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
    g: &Model<P>,
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
    edit: &mut ModelEdit<'_, P>,
    source: EdgeKey,
    mut points: Vec<Point3>,
    options: IntersectionOptions,
) -> Result<Vec<EdgeKey>, BooleanError> {
    let source_curve = edit
        .edge(source)
        .and_then(|edge| Some(edge.curve()))
        .ok_or(BooleanError::MissingOperand {
            operand: BooleanOperand::Edge(source),
        })?;
    let source_domain = edit.edge_unchecked(source).parameter_interval().ordered();
    points.sort_by(|a, b| {
        periodic_parameter_in_domain(&source_curve, *a, source_domain).total_cmp(
            &periodic_parameter_in_domain(&source_curve, *b, source_domain),
        )
    });
    points.dedup_by(|a, b| a.coincides(*b, options.linear_tolerance));

    let mut fragments = vec![source];
    for point in points {
        let Some(fragment) = fragments.iter().copied().find(|edge| {
            let view = edit.edge_unchecked(*edge);

            let domain = view.parameter_interval().ordered();
            let parameter = periodic_parameter_in_domain(view.curve(), point, domain);
            // Asked of the corners, not of the span's ends: on an unmarked edge
            // those ends are where the curve closes, and a contact landing there
            // is a corner to add rather than one already taken.
            domain.contains(parameter, options.parameter_tolerance)
                && !view.has_corner_at(parameter, options.linear_tolerance)
        }) else {
            continue;
        };

        let view = edit.edge_unchecked(fragment);
        let curve = view.curve();
        let domain = view.parameter_interval().ordered();
        let parameter = periodic_parameter_in_domain(curve, point, domain);
        let parameter = view.parameter_interval().fraction_of(parameter);
        let incident_face = view.faces().first().map(|face| face.key());
        let split = if let Some(face) = incident_face {
            split_face_edge_edit(edit, face, fragment, parameter)?
        } else {
            split_edge_edit(edit, fragment, parameter)?
        };
        // A cut that only marked an unmarked edge created nothing: the edge
        // still covers the whole of what it covered, and the next point cuts
        // that same edge into the two arcs.
        if let Some(created) = split.created() {
            fragments.push(created);
        }
    }
    Ok(fragments)
}

fn periodic_parameter_in_domain(curve: &Curve, point: Point3, domain: Interval) -> NativeParam {
    let mut parameter = curve.parameter_at(point);
    if let Periodicity::Periodic(period) = curve.periodicity() {
        while parameter < NativeParam::new(domain.start.value()) {
            parameter += period;
        }
        while parameter > NativeParam::new(domain.end.value()) {
            parameter -= period;
        }
    }
    parameter
}
