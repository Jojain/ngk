//! Narrow-phase geometric contact computation.

use super::*;
use super::{
    clip::clip_branch,
    trim::{FaceTrimDomain, TrimLocation, boundary_edge_for},
};
use crate::geometry::{CurveIntersectionOptions, IntersectionCoverage};
use std::collections::hash_map::Entry;

pub(super) fn compute_vertex_contacts<P: Payload>(
    g: &GMap<P>,
    plan: &mut IntersectionAccumulator,
    options: BooleanOptions,
) -> Result<(), BooleanError> {
    let tolerance = options.intersections.linear_tolerance;
    let first_vertices = plan.first_cells.vertices.clone();
    let second_vertices = plan.second_cells.vertices.clone();

    for first in first_vertices.iter().copied() {
        let first_point = *g
            .vertex_unchecked(first)
            .point()
            .expect("registered vertex geometry");
        for second in second_vertices.iter().copied() {
            if first == second {
                continue;
            }
            let second_point = *g
                .vertex_unchecked(second)
                .point()
                .expect("registered vertex geometry");
            if first_point.coincides(second_point, tolerance) {
                push_point_contact(
                    &mut plan.contacts,
                    first_point,
                    BooleanCell::Vertex(first),
                    BooleanCell::Vertex(second),
                );
            }
        }
    }

    let second_edges = plan.second_cells.edges.clone();
    for vertex in first_vertices.iter().copied() {
        for edge in second_edges.iter().copied() {
            intersect_vertex_edge(g, plan, vertex, edge, true, tolerance);
        }
    }
    let first_edges = plan.first_cells.edges.clone();
    for vertex in second_vertices.iter().copied() {
        for edge in first_edges.iter().copied() {
            intersect_vertex_edge(g, plan, vertex, edge, false, tolerance);
        }
    }

    let second_faces = plan.second_cells.faces.clone();
    for face in second_faces.iter().copied() {
        intersect_vertices_face(g, plan, &first_vertices, face, true, options)?;
    }
    let first_faces = plan.first_cells.faces.clone();
    for face in first_faces.iter().copied() {
        intersect_vertices_face(g, plan, &second_vertices, face, false, options)?;
    }
    Ok(())
}

fn intersect_vertex_edge<P: Payload>(
    g: &GMap<P>,
    plan: &mut IntersectionAccumulator,
    vertex_key: VertexKey,
    edge_key: EdgeKey,
    vertex_is_first: bool,
    tolerance: f64,
) {
    let point = *g
        .vertex_unchecked(vertex_key)
        .point()
        .expect("registered vertex geometry");
    let edge = g.edge_unchecked(edge_key);
    let curve = edge.curve().expect("registered edge geometry");
    let parameter = curve.param_at(point);
    if !curve.point_at(parameter).coincides(point, tolerance) {
        return;
    }
    let start = *edge
        .start()
        .point()
        .expect("registered edge start geometry");
    let end = *edge.end().point().expect("registered edge end geometry");
    if !curve
        .parameters_between(start, end)
        .ordered()
        .contains(parameter, tolerance)
    {
        return;
    }

    push_edge_point(plan, edge_key, point);
    let (first, second) = if vertex_is_first {
        (BooleanCell::Vertex(vertex_key), BooleanCell::Edge(edge_key))
    } else {
        (BooleanCell::Edge(edge_key), BooleanCell::Vertex(vertex_key))
    };
    push_point_contact(&mut plan.contacts, point, first, second);
}

/// Records every vertex of one operand lying inside one trimmed face of the other.
///
/// The face's trim domain is flattened at most once here, and only once a
/// vertex has actually reached the face's surface. A vertex that misses the
/// surface -- the common case, since most face pairs never touch -- therefore
/// costs one closest point and one surface evaluation.
fn intersect_vertices_face<P: Payload>(
    g: &GMap<P>,
    plan: &mut IntersectionAccumulator,
    vertices: &[VertexKey],
    face_key: FaceKey,
    vertices_are_first: bool,
    options: BooleanOptions,
) -> Result<(), BooleanError> {
    let face = g.face_unchecked(face_key);
    let mut trim: Option<FaceTrimDomain> = None;
    for vertex_key in vertices.iter().copied() {
        let point = *g
            .vertex_unchecked(vertex_key)
            .point()
            .expect("registered vertex geometry");
        let Ok(uv) = face.surface().closest_parameter(point) else {
            continue;
        };
        if !face
            .surface()
            .point_at(uv.x, uv.y)
            .coincides(point, options.intersections.linear_tolerance)
        {
            continue;
        }
        let trim = match trim {
            Some(ref trim) => trim,
            None => trim.insert(FaceTrimDomain::new(
                &face,
                options.intersections.parameter_tolerance,
            )?),
        };
        if !trim.contains(uv) {
            continue;
        }

        let (first, second) = if vertices_are_first {
            (BooleanCell::Vertex(vertex_key), BooleanCell::Face(face_key))
        } else {
            (BooleanCell::Face(face_key), BooleanCell::Vertex(vertex_key))
        };
        push_point_contact(&mut plan.contacts, point, first, second);
    }
    Ok(())
}

pub(super) fn compute_edge_contacts<P: Payload>(
    g: &GMap<P>,
    plan: &mut IntersectionAccumulator,
    options: BooleanOptions,
) -> Result<(), BooleanError> {
    let first_edges = plan.first_cells.edges.clone();
    let second_edges = plan.second_cells.edges.clone();
    for first in first_edges {
        for second in second_edges.iter().copied() {
            if first == second {
                continue;
            }
            let first_edge = g.edge_unchecked(first);
            let first_curve = first_edge.curve().expect("registered edge geometry");
            let second_edge = g.edge_unchecked(second);
            let second_curve = second_edge.curve().expect("registered edge geometry");
            for contact in
                first_curve.intersect_curve_with_options(second_curve, options.intersections)?
            {
                match contact {
                    CurveCurveIntersection::Point { point, .. } => {
                        let point = grazed_vertex(
                            [&first_edge, &second_edge],
                            |candidate| {
                                [first_curve, second_curve].iter().all(|curve| {
                                    lies_on_curve(
                                        curve,
                                        candidate,
                                        options.intersections.linear_tolerance,
                                    )
                                })
                            },
                            point,
                            options.intersections.linear_tolerance,
                            plan.diagnostics.tolerances.graze,
                        )
                        .unwrap_or(point);
                        push_edge_point(plan, first, point);
                        push_edge_point(plan, second, point);
                        push_point_contact(
                            &mut plan.contacts,
                            point,
                            BooleanCell::Edge(first),
                            BooleanCell::Edge(second),
                        );
                    }
                    CurveCurveIntersection::Overlap {
                        interval_a,
                        interval_b,
                    } => {
                        push_edge_point(plan, first, first_curve.point_at(interval_a.start));
                        push_edge_point(plan, first, first_curve.point_at(interval_a.end));
                        push_edge_point(plan, second, second_curve.point_at(interval_b.start));
                        push_edge_point(plan, second, second_curve.point_at(interval_b.end));
                        plan.contacts.push(RawIntersection::Overlap {
                            first_edge: first,
                            second_edge: second,
                            first_interval: interval_a,
                            second_interval: interval_b,
                        });
                    }
                }
            }
        }
    }
    Ok(())
}

fn push_edge_point(plan: &mut IntersectionAccumulator, edge: EdgeKey, point: Point3) {
    plan.edge_points.entry(edge).or_default().push(point);
}

/// Records every distinct reason a narrow-phase result was not certified.
fn merge_coverage(plan: &mut IntersectionAccumulator, coverage: &IntersectionCoverage) {
    let IntersectionCoverage::Incomplete(reasons) = coverage else {
        return;
    };
    for reason in reasons {
        if !plan.diagnostics.coverage.contains(reason) {
            plan.diagnostics.coverage.push(*reason);
        }
    }
}

fn push_point_contact(
    contacts: &mut Vec<RawIntersection>,
    point: Point3,
    first: BooleanCell,
    second: BooleanCell,
) {
    contacts.push(RawIntersection::Point {
        point,
        first,
        second,
        kind: PointContactKind::Transverse,
    });
}

/// Decomposed operand geometry reused across every candidate pair.
///
/// Bézier decomposition dominates a single curve/surface query, so an operand
/// touched by many pairs must be decomposed once rather than once per pair.
#[derive(Default)]
struct PreparedGeometry {
    edges: HashMap<EdgeKey, PreparedCurve>,
    faces: HashMap<FaceKey, PreparedSurface>,
}

impl PreparedGeometry {
    fn edge<P: Payload>(
        &mut self,
        g: &GMap<P>,
        key: EdgeKey,
    ) -> Result<&PreparedCurve, BooleanError> {
        match self.edges.entry(key) {
            Entry::Occupied(entry) => Ok(entry.into_mut()),
            Entry::Vacant(entry) => {
                let edge = g.edge_unchecked(key);
                let curve = edge.curve().expect("registered edge geometry");
                Ok(entry.insert(PreparedCurve::new(curve)?))
            }
        }
    }

    /// Prepares a face's surface over its own trim domain.
    ///
    /// An unbounded analytic surface otherwise converts to an arbitrary unit
    /// patch, which drops every contact outside it.
    fn face<P: Payload>(
        &mut self,
        g: &GMap<P>,
        key: FaceKey,
        tolerance: f64,
    ) -> Result<&PreparedSurface, BooleanError> {
        match self.faces.entry(key) {
            Entry::Occupied(entry) => Ok(entry.into_mut()),
            Entry::Vacant(entry) => {
                let face = g.face_unchecked(key);
                let prepared = prepare_face_surface(&face, tolerance)?;
                Ok(entry.insert(prepared))
            }
        }
    }
}

pub(super) fn compute_edge_face_contacts<P: Payload>(
    g: &GMap<P>,
    plan: &mut IntersectionAccumulator,
    options: BooleanOptions,
) -> Result<(), BooleanError> {
    let mut geometry = PreparedGeometry::default();
    let sides = [
        (
            plan.first_cells.edges.clone(),
            plan.second_cells.faces.clone(),
            true,
        ),
        (
            plan.second_cells.edges.clone(),
            plan.first_cells.faces.clone(),
            false,
        ),
    ];
    for (edges, faces, edge_is_first) in sides {
        let candidates = broad_phase::candidate_edge_face_pairs(
            g,
            &edges,
            &faces,
            options.intersections.bbox_tolerance,
        );
        plan.diagnostics.edge_face_pairs_tested += candidates.pairs.len();
        plan.diagnostics.edge_face_pairs_pruned += candidates.pruned;
        for (edge, face) in candidates.pairs {
            intersect_edge_face(g, plan, &mut geometry, edge, face, edge_is_first, options)?;
        }
    }
    Ok(())
}

fn intersect_edge_face<P: Payload>(
    g: &GMap<P>,
    plan: &mut IntersectionAccumulator,
    geometry: &mut PreparedGeometry,
    edge_key: EdgeKey,
    face_key: FaceKey,
    edge_is_first: bool,
    options: BooleanOptions,
) -> Result<(), BooleanError> {
    let edge = g.edge_unchecked(edge_key);
    let face = g.face_unchecked(face_key);
    let curve = edge.curve().expect("registered edge geometry");
    let trim = FaceTrimDomain::new(&face, options.intersections.parameter_tolerance)?;
    // Points both operands have already located exactly.
    let anchors = plan
        .contacts
        .iter()
        .filter_map(|contact| match contact {
            RawIntersection::Point { point, .. } => Some(*point),
            _ => None,
        })
        .collect::<Vec<_>>();
    let contacts = {
        let prepared_curve = geometry.edge(g, edge_key)?.clone();
        let prepared_face = geometry.face(g, face_key, options.intersections.linear_tolerance)?;
        intersect_prepared_curve_surface(&prepared_curve, prepared_face, options.intersections)?
    };
    merge_coverage(plan, contacts.coverage());
    for contact in contacts {
        match contact {
            CurveSurfaceIntersection::Point {
                point,
                curve_u,
                surface_u,
                surface_v,
            } => {
                if !trim.contains(Point2::new(surface_u, surface_v)) {
                    continue;
                }
                let point = grazed_vertex(
                    [&edge],
                    |candidate| {
                        let surface = face.surface();
                        surface.closest_parameter(candidate).is_ok_and(|uv| {
                            surface
                                .point_at(uv.x, uv.y)
                                .coincides(candidate, options.intersections.linear_tolerance)
                        })
                    },
                    point,
                    options.intersections.linear_tolerance,
                    plan.diagnostics.tolerances.graze,
                )
                .unwrap_or(point);
                push_edge_point(plan, edge_key, point);
                let tangent = curve.derivative_at(curve_u, 1);
                let normal = face.surface().normal_at(surface_u, surface_v);
                let kind = if tangent.dot(&normal).abs() <= options.intersections.linear_tolerance {
                    PointContactKind::Tangent
                } else {
                    PointContactKind::Transverse
                };
                let (first, second) = if edge_is_first {
                    (BooleanCell::Edge(edge_key), BooleanCell::Face(face_key))
                } else {
                    (BooleanCell::Face(face_key), BooleanCell::Edge(edge_key))
                };
                plan.contacts.push(RawIntersection::Point {
                    point,
                    first,
                    second,
                    kind,
                });
            }
            CurveSurfaceIntersection::Overlap { curve_interval } => {
                // The edge rests on the surface over this interval, but only the
                // part the face's own trim keeps is a contact of the two cells.
                // The solver reports the interval in the curve's own NURBS
                // parameters; subcurves are taken over normalized ones.
                let native = curve.to_nurbs()?.domain();
                let extent = native.end - native.start;
                let section = graph::normalized_subcurve(
                    curve,
                    Interval::new(
                        ((curve_interval.start - native.start) / extent).clamp(0.0, 1.0),
                        ((curve_interval.end - native.start) / extent).clamp(0.0, 1.0),
                    ),
                )?;
                let Some(imprint) = section_imprint(&face, &section, options)? else {
                    continue;
                };
                let side = if edge_is_first {
                    BooleanSide::First
                } else {
                    BooleanSide::Second
                };
                for piece in clip_imprint_to_trim(
                    &trim,
                    &imprint,
                    &anchors,
                    plan.diagnostics.tolerances.graze,
                    options,
                )? {
                    let start = piece.curve.point_at(0.0);
                    let end = piece.curve.point_at(1.0);
                    push_edge_point(plan, edge_key, start);
                    push_edge_point(plan, edge_key, end);
                    // The section belongs to both cells: the edge carries it
                    // already, the face has to be split along it.
                    plan.contacts.push(RawIntersection::EdgeSection {
                        side,
                        edge: edge_key,
                        curve: piece.curve.clone(),
                        interval: Interval::new(curve.param_at(start), curve.param_at(end)),
                    });
                    plan.face_imprints.entry(face_key).or_default().push(piece);
                }
            }
        }
    }
    Ok(())
}

/// Relative margin added to a face's own parameter box before intersection.
///
/// The box exists to cover the face; trimming happens afterwards against the
/// exact pcurves. A branch that leaves the box exactly where the face's trim
/// ends is stopped by a boundary correction that is a double root at a
/// tangential exit, which both misplaces the branch end and lets the same
/// section be traced twice from opposite seeds. Overshooting the trim keeps
/// every branch end transversal.
const DOMAIN_MARGIN: f64 = 1.0e-2;

/// Prepares a face's surface over a parameter box that overshoots its trim.
///
/// A direction the surface already closes in is left alone: extending it would
/// wrap the surface over itself.
fn prepare_face_surface<P: Payload>(
    face: &crate::topology::face::Face<'_, P>,
    tolerance: f64,
) -> Result<PreparedSurface, BooleanError> {
    let Some((domain_u, domain_v)) = broad_phase::face_uv_bounds(face) else {
        return Ok(PreparedSurface::new(face.surface())?);
    };
    let surface = face.surface();
    let middle = |domain: Interval| 0.5 * (domain.start + domain.end);
    let closed_in_u = surface
        .point_at(domain_u.start, middle(domain_v))
        .coincides(surface.point_at(domain_u.end, middle(domain_v)), tolerance);
    let closed_in_v = surface
        .point_at(middle(domain_u), domain_v.start)
        .coincides(surface.point_at(middle(domain_u), domain_v.end), tolerance);
    let grown = |domain: Interval, closed: bool| {
        if closed {
            return domain;
        }
        let margin = (domain.end - domain.start) * DOMAIN_MARGIN;
        Interval::new(domain.start - margin, domain.end + margin)
    };
    let domain_u = grown(domain_u, closed_in_u);
    let domain_v = grown(domain_v, closed_in_v);
    match PreparedSurface::over(surface, domain_u, domain_v) {
        Ok(prepared) => Ok(prepared),
        // A surface that cannot be realized past its own extent keeps the box
        // the face gave it.
        Err(_) => Ok(PreparedSurface::over(
            surface,
            broad_phase::face_uv_bounds(face)
                .expect("bounds computed above")
                .0,
            broad_phase::face_uv_bounds(face)
                .expect("bounds computed above")
                .1,
        )?),
    }
}

/// Returns the edge vertex a grazing curve/surface contact actually is.
///
/// Where an edge only grazes a surface the crossing is a double root, and the
/// solver locates it to the square root of its distance tolerance rather than
/// to the tolerance itself. When such a contact lands beside an endpoint of the
/// edge that is itself on the surface, that vertex is the exact answer the
/// solver was approximating, and is used instead.
fn grazed_vertex<P: Payload, const N: usize>(
    edges: [&crate::topology::edge::Edge<'_, P>; N],
    on_other: impl Fn(Point3) -> bool,
    point: Point3,
    tolerance: f64,
    graze: f64,
) -> Option<Point3> {
    edges
        .into_iter()
        .flat_map(|edge| [edge.start(), edge.end()])
        .filter_map(|vertex| vertex.point().copied())
        .find(|vertex| {
            !vertex.coincides(point, tolerance)
                && (vertex - point).norm() <= graze
                && on_other(*vertex)
        })
}

/// Whether `point` sits on `curve` within `tolerance`.
fn lies_on_curve(curve: &Curve, point: Point3, tolerance: f64) -> bool {
    curve
        .point_at(curve.param_at(point))
        .coincides(point, tolerance)
}
/// Samples used to carry a section whose parameter image is not a straight line.
const SECTION_SAMPLE_COUNT: usize = 32;

/// Builds the parameter-space image of a section already resting on `face`.
///
/// A planar face maps model space to its parameter space affinely, so the
/// section's own control points carry over exactly. On a curved surface a
/// section whose projection stays collinear — a ruling, say — is still exact;
/// anything else is carried by the interpolating polyline the face splitter
/// consumes. `None` means the section does not project onto the surface at all.
fn section_imprint<P: Payload>(
    face: &crate::topology::face::Face<'_, P>,
    section: &Curve,
    options: BooleanOptions,
) -> Result<Option<FaceImprint>, BooleanError> {
    if let Surface::Plane(plane) = face.surface() {
        let pcurve = crate::builders::profiles::curve_pcurve(
            section,
            section.point_at(0.0),
            section.point_at(1.0),
            plane,
        )?;
        return Ok(Some(FaceImprint::new(section.clone(), pcurve)));
    }
    let mut points = Vec::with_capacity(SECTION_SAMPLE_COUNT + 1);
    let mut uv_points = Vec::with_capacity(SECTION_SAMPLE_COUNT + 1);
    for index in 0..=SECTION_SAMPLE_COUNT {
        let point = section.point_at(index as f64 / SECTION_SAMPLE_COUNT as f64);
        let Ok(uv) = face.surface().closest_parameter(point) else {
            return Ok(None);
        };
        points.push(point);
        uv_points.push(uv);
    }
    let tolerance = options.intersections.parameter_tolerance;
    let chord = uv_points[SECTION_SAMPLE_COUNT] - uv_points[0];
    let collinear = chord.norm() > tolerance
        && uv_points
            .iter()
            .all(|uv| cross2(chord, uv - uv_points[0]).abs() <= tolerance * chord.norm());
    if collinear {
        return Ok(Some(FaceImprint::new(
            section.clone(),
            Curve2::Line(Line2::new(uv_points[0], uv_points[SECTION_SAMPLE_COUNT])),
        )));
    }
    Ok(Some(polyline_imprint(&points, &uv_points)?))
}

/// Splits an imprint at its exact trim crossings and keeps the interior pieces.
///
/// A piece running along a trim loop is dropped: that section is already
/// carried by the boundary edge it rests on, and imprinting it again would
/// split the face along its own boundary.
fn clip_imprint_to_trim(
    trim: &FaceTrimDomain,
    imprint: &FaceImprint,
    anchors: &[Point3],
    graze: f64,
    options: BooleanOptions,
) -> Result<Vec<FaceImprint>, BooleanError> {
    let tolerance = options.intersections.parameter_tolerance;
    let curve_options = CurveIntersectionOptions {
        linear_tolerance: tolerance,
        parameter_tolerance: tolerance,
        bbox_tolerance: tolerance,
        max_subdivision_depth: options.intersections.max_subdivision_depth,
        leaf_diagonal_tolerance: tolerance * 10.0,
        newton_max_iterations: options.intersections.newton_max_iterations,
    };
    let mut crossings = Vec::new();
    trim.crossings(&imprint.pcurve, curve_options, &mut crossings)?;
    // A crossing where the section grazes the trim is a double root, located
    // only to `graze`. An exact point that close to it is what it approximates,
    // and cutting there instead keeps the piece ending on a known event.
    let nodes = crossings
        .iter()
        .filter_map(|crossing| {
            let point = imprint.curve.point_at(*crossing);
            anchors
                .iter()
                .find(|anchor| (point - **anchor).norm() <= graze)
        })
        .map(|anchor| imprint.curve.param_at(*anchor).clamp(0.0, 1.0))
        .collect::<Vec<_>>();
    crossings.retain(|crossing| {
        let point = imprint.curve.point_at(*crossing);
        !nodes
            .iter()
            .any(|node| (point - imprint.curve.point_at(*node)).norm() <= graze)
    });
    let mut parameters = vec![0.0, 1.0];
    parameters.append(&mut crossings);
    parameters.extend(nodes);
    parameters.sort_by(f64::total_cmp);
    parameters.dedup_by(|a, b| (*a - *b).abs() <= tolerance);
    let mut pieces = Vec::new();
    for pair in parameters.windows(2) {
        let midpoint = 0.5 * (pair[0] + pair[1]);
        if !matches!(
            trim.classify(imprint.pcurve.point_at(midpoint)),
            TrimLocation::Inside { .. }
        ) {
            continue;
        }
        pieces.push(imprint.trimmed(Interval::new(pair[0], pair[1]))?);
    }
    Ok(pieces)
}

pub(super) fn compute_face_contacts<P: Payload>(
    g: &GMap<P>,
    plan: &mut IntersectionAccumulator,
    options: BooleanOptions,
) -> Result<(), BooleanError> {
    let candidates = broad_phase::candidate_face_pairs(
        g,
        &plan.first_cells.faces,
        &plan.second_cells.faces,
        options.intersections.bbox_tolerance,
    );
    plan.diagnostics.candidate_pairs_tested = candidates.pairs.len();
    plan.diagnostics.candidate_pairs_pruned = candidates.pruned;
    for (first, second) in candidates.pairs {
        intersect_face_pair(g, plan, first, second, options)?;
    }
    Ok(())
}

/// Moves contact sections lying on a face's own trim loop onto that edge.
///
/// Imprinting such a section would split the face along its own boundary and
/// leave a degenerate fragment with no interior probe. The edge already carries
/// the geometry, so the section is recorded as an edge contact and later
/// realized on the fragment the edge split pass produces. Running this before
/// chain normalization keeps boundary sections out of the chained polylines.
pub(super) fn reroute_boundary_imprints<P: Payload>(
    g: &GMap<P>,
    plan: &mut IntersectionAccumulator,
    options: BooleanOptions,
) {
    // Sorted, because the recorded order fixes canonical span identity.
    let mut face_keys = plan.face_imprints.keys().copied().collect::<Vec<_>>();
    face_keys.sort_by_key(|face| face.data().as_ffi());
    for face_key in face_keys {
        let side = if plan.first_cells.faces.contains(&face_key) {
            BooleanSide::First
        } else {
            BooleanSide::Second
        };
        let face = g.face_unchecked(face_key);
        let imprints = plan.face_imprints.remove(&face_key).unwrap_or_default();
        let mut retained = Vec::new();
        for imprint in imprints {
            let Some(edge) = boundary_edge_for(
                &face,
                &imprint.pcurve,
                options.intersections.parameter_tolerance,
            ) else {
                retained.push(imprint);
                continue;
            };
            let edge_view = g.edge_unchecked(edge);
            let curve = edge_view.curve().expect("registered edge geometry");
            let start = curve.param_at(imprint.curve.point_at(0.0));
            let end = curve.param_at(imprint.curve.point_at(1.0));
            plan.contacts.push(RawIntersection::EdgeSection {
                side,
                edge,
                curve: imprint.curve.clone(),
                interval: Interval::new(start, end),
            });
        }
        if !retained.is_empty() {
            plan.face_imprints.insert(face_key, retained);
        }
    }
}

/// Discards face imprints that repeat a section already recorded on that face.
///
/// A solid contact is observed by several face pairs at once — a coplanar
/// overlap and the transverse pairs bounding it report the same section — and a
/// repeated section would give the chain graph a doubled edge, hiding the open
/// chain the face splitter needs.
fn dedup_face_imprints(imprints: &mut Vec<FaceImprint>, tolerance: f64) {
    let mut kept = Vec::<FaceImprint>::new();
    for imprint in imprints.drain(..) {
        if !kept
            .iter()
            .any(|existing| same_section(&existing.pcurve, &imprint.pcurve, tolerance))
        {
            kept.push(imprint);
        }
    }
    *imprints = kept;
}

/// Whether two pcurves trace the same section, in either direction.
fn same_section(left: &Curve2, right: &Curve2, tolerance: f64) -> bool {
    let samples = [0.0, 0.25, 0.5, 0.75, 1.0];
    let forward = samples
        .iter()
        .all(|t| (left.point_at(*t) - right.point_at(*t)).norm() <= tolerance);
    let reversed = samples
        .iter()
        .all(|t| (left.point_at(*t) - right.point_at(1.0 - *t)).norm() <= tolerance);
    forward || reversed
}

/// Replaces connected open segment chains with one exact polyline NURBS.
///
/// The face splitter consumes one boundary-to-boundary curve at a time. Solid
/// intersections naturally produce that curve as several face-pair branches,
/// so normalization happens on the complete contact graph before mutation.
pub(super) fn normalize_face_imprint_chains<P: Payload>(
    g: &GMap<P>,
    plan: &mut IntersectionAccumulator,
    options: BooleanOptions,
) -> Result<(), BooleanError> {
    let mut face_keys = plan.face_imprints.keys().copied().collect::<Vec<_>>();
    face_keys.sort_by_key(|face| face.data().as_ffi());
    for face_key in face_keys {
        let mut imprints = plan.face_imprints.remove(&face_key).unwrap_or_default();
        dedup_face_imprints(&mut imprints, options.intersections.parameter_tolerance);
        if imprints.len() < 2
            || imprints
                .iter()
                .any(|imprint| !matches!(imprint.curve.base(), Curve::Line(_)))
        {
            plan.face_imprints.insert(face_key, imprints);
            continue;
        }
        let face = g.face_unchecked(face_key);
        let mut nodes = Vec::<Point2>::new();
        let mut edges = Vec::<(usize, usize, usize)>::new();
        for (index, imprint) in imprints.iter().enumerate() {
            let start = graph_node(
                &mut nodes,
                imprint.pcurve.point_at(0.0),
                options.intersections.parameter_tolerance,
            );
            let end = graph_node(
                &mut nodes,
                imprint.pcurve.point_at(1.0),
                options.intersections.parameter_tolerance,
            );
            edges.push((start, end, index));
        }

        let mut seen_edges = HashSet::new();
        let mut normalized = Vec::new();
        for seed in 0..edges.len() {
            if seen_edges.contains(&seed) {
                continue;
            }
            let component = edge_component(seed, &edges);
            seen_edges.extend(component.iter().copied());
            let mut degree = HashMap::<usize, usize>::new();
            for edge_index in &component {
                let (start, end, _) = edges[*edge_index];
                *degree.entry(start).or_default() += 1;
                *degree.entry(end).or_default() += 1;
            }
            let mut endpoints = degree
                .iter()
                .filter_map(|(node, degree)| (*degree == 1).then_some(*node))
                .collect::<Vec<_>>();
            endpoints.sort_unstable();
            let is_open_chain = endpoints.len() == 2 && degree.values().all(|value| *value <= 2);
            if !is_open_chain
                || !point_on_face_boundary(
                    &face,
                    nodes[endpoints[0]],
                    options.intersections.parameter_tolerance,
                )
                || !point_on_face_boundary(
                    &face,
                    nodes[endpoints[1]],
                    options.intersections.parameter_tolerance,
                )
            {
                normalized.extend(
                    component
                        .into_iter()
                        .map(|edge_index| imprints[edges[edge_index].2].clone()),
                );
                continue;
            }

            let ordered_nodes = order_chain(endpoints[0], &component, &edges);
            if ordered_nodes.len() <= 2 {
                normalized.push(imprints[edges[component[0]].2].clone());
                continue;
            }
            let uv_points = ordered_nodes
                .iter()
                .map(|node| nodes[*node])
                .collect::<Vec<_>>();
            let points = uv_points
                .iter()
                .map(|uv| face.point_at(uv.x, uv.y))
                .collect::<Vec<_>>();
            normalized.push(polyline_imprint(&points, &uv_points)?);
        }
        plan.face_imprints.insert(face_key, normalized);
    }
    Ok(())
}

fn graph_node(nodes: &mut Vec<Point2>, point: Point2, tolerance: f64) -> usize {
    if let Some(index) = nodes
        .iter()
        .position(|existing| (*existing - point).norm() <= tolerance)
    {
        index
    } else {
        nodes.push(point);
        nodes.len() - 1
    }
}

fn edge_component(seed: usize, edges: &[(usize, usize, usize)]) -> Vec<usize> {
    let mut component = Vec::new();
    let mut pending = vec![seed];
    let mut seen = HashSet::new();
    while let Some(edge_index) = pending.pop() {
        if !seen.insert(edge_index) {
            continue;
        }
        component.push(edge_index);
        let (start, end, _) = edges[edge_index];
        pending.extend(edges.iter().enumerate().filter_map(|(index, edge)| {
            (edge.0 == start || edge.0 == end || edge.1 == start || edge.1 == end).then_some(index)
        }));
    }
    component
}

fn order_chain(start: usize, component: &[usize], edges: &[(usize, usize, usize)]) -> Vec<usize> {
    let mut ordered = vec![start];
    let mut current = start;
    let mut unused = component.iter().copied().collect::<HashSet<_>>();
    while let Some(edge_index) = unused
        .iter()
        .copied()
        .filter(|index| edges[*index].0 == current || edges[*index].1 == current)
        .min()
    {
        unused.remove(&edge_index);
        let edge = edges[edge_index];
        current = if edge.0 == current { edge.1 } else { edge.0 };
        ordered.push(current);
    }
    ordered
}

fn point_on_face_boundary<P: Payload>(
    face: &crate::topology::face::Face<'_, P>,
    point: Point2,
    tolerance: f64,
) -> bool {
    face.edges().into_iter().any(|edge| {
        face.pcurve(edge.dart())
            .and_then(|pcurve| pcurve.parameter_at(point, tolerance))
            .is_some()
    })
}

fn polyline_imprint(points: &[Point3], uv_points: &[Point2]) -> Result<FaceImprint, NurbsError> {
    let parameters = chord_parameters(points);
    let mut knots = vec![0.0, 0.0];
    knots.extend(parameters.iter().copied().skip(1).take(points.len() - 2));
    knots.extend([1.0, 1.0]);
    let knots = KnotVector::new(knots)?;
    let curve = NurbsCurve::new(
        Degree::new(1)?,
        ControlPolygon::new(
            points
                .iter()
                .copied()
                .map(|point| HPoint::from_cartesian(point, 1.0))
                .collect(),
        )?,
        knots.clone(),
    )?;
    let pcurve = NurbsCurve2::new(
        Degree::new(1)?,
        ControlPolygon2::new(
            uv_points
                .iter()
                .copied()
                .map(|point| HPoint2::from_cartesian(point, 1.0))
                .collect(),
        )?,
        knots,
    )?;
    Ok(FaceImprint::new(Curve::Nurbs(curve), Curve2::Nurbs(pcurve)))
}

fn chord_parameters(points: &[Point3]) -> Vec<f64> {
    let lengths = points
        .windows(2)
        .map(|pair| (pair[1] - pair[0]).norm())
        .collect::<Vec<_>>();
    let total = lengths.iter().sum::<f64>();
    let mut parameters = vec![0.0];
    let mut accumulated = 0.0;
    for length in lengths {
        accumulated += length;
        parameters.push(accumulated / total);
    }
    *parameters.last_mut().expect("polyline has endpoints") = 1.0;
    parameters
}

fn intersect_face_pair<P: Payload>(
    g: &GMap<P>,
    plan: &mut IntersectionAccumulator,
    first_key: FaceKey,
    second_key: FaceKey,
    options: BooleanOptions,
) -> Result<(), BooleanError> {
    let first = g.face_unchecked(first_key);
    let second = g.face_unchecked(second_key);
    let (Surface::Plane(first_plane), Surface::Plane(second_plane)) =
        (first.surface(), second.surface())
    else {
        return intersect_general_face_pair(g, plan, first_key, second_key, options);
    };

    let first_normal = *first_plane.normal();
    let second_normal = *second_plane.normal();
    let cross = first_normal.cross(&second_normal);
    let cross_squared = cross.norm_squared();
    if cross_squared <= options.intersections.angular_tolerance.powi(2) {
        // The section each face cuts out of the other is its partner's boundary
        // clipped to its own trim, which the edge/face pass already imprinted.
        // Only the region record is left to make here.
        let distance = (second_plane.origin() - first_plane.origin()).dot(&first_normal);
        if distance.abs() <= options.intersections.linear_tolerance
            && coplanar_faces_share_area(&first, &second, options)?
        {
            plan.contacts.push(RawIntersection::Region {
                first_face: first_key,
                second_face: second_key,
            });
        }
        return Ok(());
    }

    let direction = cross.normalize();
    let first_offset = first_normal.dot(&first_plane.origin().coords);
    let second_offset = second_normal.dot(&second_plane.origin().coords);
    let line_point = Point3::from(
        (first_offset * second_normal.cross(&cross) + second_offset * cross.cross(&first_normal))
            / cross_squared,
    );
    let first_intervals =
        line_intervals_in_face(&first, line_point, direction, options.intersections)?;
    let second_intervals =
        line_intervals_in_face(&second, line_point, direction, options.intersections)?;

    for first_interval in first_intervals {
        for second_interval in &second_intervals {
            let start_t = first_interval.start.max(second_interval.start);
            let end_t = first_interval.end.min(second_interval.end);
            if end_t - start_t <= options.intersections.linear_tolerance {
                continue;
            }
            let start = line_point + direction * start_t;
            let end = line_point + direction * end_t;
            let curve = Curve::line(start, end);
            let first_start = first_plane.parameter_at(start);
            let first_end = first_plane.parameter_at(end);
            let second_start = second_plane.parameter_at(start);
            let second_end = second_plane.parameter_at(end);
            let first_pcurve = Curve2::Line(Line2::new(first_start, first_end));
            let second_pcurve = Curve2::Line(Line2::new(second_start, second_end));
            plan.face_imprints
                .entry(first_key)
                .or_default()
                .push(FaceImprint::new(curve.clone(), first_pcurve.clone()));
            plan.face_imprints
                .entry(second_key)
                .or_default()
                .push(FaceImprint::new(curve.clone(), second_pcurve.clone()));
        }
    }
    Ok(())
}

fn intersect_general_face_pair<P: Payload>(
    g: &GMap<P>,
    plan: &mut IntersectionAccumulator,
    first_key: FaceKey,
    second_key: FaceKey,
    options: BooleanOptions,
) -> Result<(), BooleanError> {
    let first = g.face_unchecked(first_key);
    let second = g.face_unchecked(second_key);
    let first_trim = FaceTrimDomain::new(&first, options.intersections.parameter_tolerance)?;
    let second_trim = FaceTrimDomain::new(&second, options.intersections.parameter_tolerance)?;
    let prepared_first = prepare_face_surface(&first, options.intersections.linear_tolerance)?;
    let prepared_second = prepare_face_surface(&second, options.intersections.linear_tolerance)?;
    let contacts = crate::geometry::intersect_prepared_surfaces(
        &prepared_first,
        &prepared_second,
        options.intersections,
    )?;
    merge_coverage(plan, contacts.coverage());
    // Points both operands already located exactly. A branch is a fit, so where
    // it runs into one of these the exact point is the node, not the crossing
    // the fit reports for itself.
    let anchors = plan
        .contacts
        .iter()
        .filter_map(|contact| match contact {
            RawIntersection::Point { point, .. } => Some(*point),
            _ => None,
        })
        .collect::<Vec<_>>();
    for contact in contacts.intersections() {
        match contact {
            SurfaceSurfaceIntersection::Point(contact) => {
                let point = contact.point;
                let (surface_a_u, surface_a_v) = (contact.uv_a.x, contact.uv_a.y);
                let (surface_b_u, surface_b_v) = (contact.uv_b.x, contact.uv_b.y);
                if first_trim.contains(Point2::new(surface_a_u, surface_a_v))
                    && second_trim.contains(Point2::new(surface_b_u, surface_b_v))
                {
                    push_point_contact(
                        &mut plan.contacts,
                        point,
                        BooleanCell::Face(first_key),
                        BooleanCell::Face(second_key),
                    );
                }
            }
            SurfaceSurfaceIntersection::Branch(branch) => {
                plan.diagnostics.branches_found += 1;
                plan.diagnostics.branches_uncertified += usize::from(!branch.quality.certified);
                for [first_imprint, second_imprint] in clip_branch(
                    branch,
                    (first.surface(), &first_trim),
                    (second.surface(), &second_trim),
                    &anchors,
                    plan.diagnostics.tolerances.graze,
                    options.intersections,
                )? {
                    plan.face_imprints
                        .entry(first_key)
                        .or_default()
                        .push(first_imprint);
                    plan.face_imprints
                        .entry(second_key)
                        .or_default()
                        .push(second_imprint);
                }
            }
            SurfaceSurfaceIntersection::OverlapCandidate(_) => {
                plan.diagnostics
                    .unresolved_overlaps
                    .push((first_key, second_key));
            }
        }
    }
    Ok(())
}

/// Whether two coplanar faces share area rather than only touching.
///
/// Faces that meet along an edge or at a corner have every shared point on both
/// boundaries, so no probe reaches the interior of the other. Each face is
/// probed at its own boundary samples — which catches a partial overlap — and at
/// their centroid, which catches a face contained in the other.
fn coplanar_faces_share_area<P: Payload>(
    first: &crate::topology::face::Face<'_, P>,
    second: &crate::topology::face::Face<'_, P>,
    options: BooleanOptions,
) -> Result<bool, BooleanError> {
    let tolerance = options.intersections.parameter_tolerance;
    let trims = [
        FaceTrimDomain::new(first, tolerance)?,
        FaceTrimDomain::new(second, tolerance)?,
    ];
    for (index, face) in [first, second].into_iter().enumerate() {
        let other = [second, first][index];
        let other_trim = &trims[1 - index];
        let mut samples = Vec::new();
        for boundary in face.loops() {
            for edge in boundary.edges() {
                let Some(pcurve) = face.pcurve(edge.dart()) else {
                    continue;
                };
                for fraction in [0.0, 0.5] {
                    let uv = pcurve.point_at(fraction);
                    samples.push(face.point_at(uv.x, uv.y));
                }
            }
        }
        let Some(centroid) = centroid_of(&samples) else {
            continue;
        };
        samples.push(centroid);
        if samples.iter().any(|point| {
            other
                .surface()
                .closest_parameter(*point)
                .is_ok_and(|uv| other_trim.contains(uv))
        }) {
            return Ok(true);
        }
    }
    Ok(false)
}

/// Averages points, or `None` when there are none.
fn centroid_of(points: &[Point3]) -> Option<Point3> {
    let count = points.len();
    (count > 0).then(|| {
        Point3::from(
            points
                .iter()
                .fold(nalgebra::Vector3::zeros(), |sum, point| sum + point.coords)
                / count as f64,
        )
    })
}

fn line_intervals_in_face<P: Payload>(
    face: &crate::topology::face::Face<'_, P>,
    line_point: Point3,
    direction: nalgebra::Vector3<f64>,
    options: IntersectionOptions,
) -> Result<Vec<Interval>, BooleanError> {
    let origin_uv = face
        .surface()
        .closest_parameter(line_point)
        .expect("planar parameter projection");
    let direction_uv = face
        .surface()
        .closest_parameter(line_point + direction)
        .expect("planar parameter projection")
        - origin_uv;
    Ok(
        FaceTrimDomain::new(face, options.parameter_tolerance)?.line_intervals(
            origin_uv,
            direction_uv,
            options,
        )?,
    )
}

fn cross2(a: Vector2<f64>, b: Vector2<f64>) -> f64 {
    a.x * b.y - a.y * b.x
}
