//! Narrow-phase geometric contact computation.

use super::*;
use super::{clip::clip_branch, trim::FaceTrimDomain};
use crate::geometry::IntersectionCoverage;
use std::collections::hash_map::Entry;

pub(super) fn compute_vertex_contacts<P: Payload>(
    g: &GMap<P>,
    plan: &mut IntersectionAccumulator,
    options: BooleanOptions,
) {
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
    for vertex in first_vertices {
        for face in second_faces.iter().copied() {
            intersect_vertex_face(g, plan, vertex, face, true, tolerance);
        }
    }
    let first_faces = plan.first_cells.faces.clone();
    for vertex in second_vertices {
        for face in first_faces.iter().copied() {
            intersect_vertex_face(g, plan, vertex, face, false, tolerance);
        }
    }
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

fn intersect_vertex_face<P: Payload>(
    g: &GMap<P>,
    plan: &mut IntersectionAccumulator,
    vertex_key: VertexKey,
    face_key: FaceKey,
    vertex_is_first: bool,
    tolerance: f64,
) {
    let point = *g
        .vertex_unchecked(vertex_key)
        .point()
        .expect("registered vertex geometry");
    let face = g.face_unchecked(face_key);
    let Ok(uv) = face.surface().closest_parameter(point) else {
        return;
    };
    if !face
        .surface()
        .point_at(uv.x, uv.y)
        .coincides(point, tolerance)
        || !face_contains_uv(&face_uv_loops(&face), uv)
    {
        return;
    }

    let (first, second) = if vertex_is_first {
        (BooleanCell::Vertex(vertex_key), BooleanCell::Face(face_key))
    } else {
        (BooleanCell::Face(face_key), BooleanCell::Vertex(vertex_key))
    };
    push_point_contact(&mut plan.contacts, point, first, second);
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
    ) -> Result<&PreparedSurface, BooleanError> {
        match self.faces.entry(key) {
            Entry::Occupied(entry) => Ok(entry.into_mut()),
            Entry::Vacant(entry) => {
                let face = g.face_unchecked(key);
                let prepared = match broad_phase::face_uv_bounds(&face) {
                    Some((domain_u, domain_v)) => {
                        PreparedSurface::over(face.surface(), domain_u, domain_v)?
                    }
                    None => PreparedSurface::new(face.surface())?,
                };
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
    let loops = face_uv_loops(&face);
    let contacts = {
        let prepared_curve = geometry.edge(g, edge_key)?.clone();
        let prepared_face = geometry.face(g, face_key)?;
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
                if !face_contains_uv(&loops, Point2::new(surface_u, surface_v)) {
                    continue;
                }
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
                // Boundary-edge contacts already appear in edge/edge dispatch.
                // An edge through the face interior becomes a face imprint when
                // both overlap endpoints belong to the trimmed region.
                let start = curve.point_at(curve_interval.start);
                let end = curve.point_at(curve_interval.end);
                let start_uv = face.surface().closest_parameter(start).ok();
                let end_uv = face.surface().closest_parameter(end).ok();
                let Some((start_uv, end_uv)) = start_uv.zip(end_uv) else {
                    continue;
                };
                if !face_contains_uv(&loops, start_uv) || !face_contains_uv(&loops, end_uv) {
                    continue;
                }
                let imprint_curve = Curve::line(start, end);
                let pcurve = Curve2::Line(Line2::new(start_uv, end_uv));
                plan.face_imprints
                    .entry(face_key)
                    .or_default()
                    .push(FaceImprint::new(imprint_curve, pcurve));
                push_edge_point(plan, edge_key, start);
                push_edge_point(plan, edge_key, end);
            }
        }
    }
    Ok(())
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
    let face_keys = plan.face_imprints.keys().copied().collect::<Vec<_>>();
    for face_key in face_keys {
        let imprints = plan.face_imprints.remove(&face_key).unwrap_or_default();
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
        let distance = (second_plane.origin() - first_plane.origin()).dot(&first_normal);
        if distance.abs() <= options.intersections.linear_tolerance
            && add_planar_overlap_imprints(
                g,
                plan,
                first_key,
                second_key,
                options.intersections.parameter_tolerance,
            )
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
    let first_intervals = line_intervals_in_face(
        &first,
        line_point,
        direction,
        options.intersections.parameter_tolerance,
    );
    let second_intervals = line_intervals_in_face(
        &second,
        line_point,
        direction,
        options.intersections.parameter_tolerance,
    );

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
    let first_loops = face_uv_loops(&first);
    let second_loops = face_uv_loops(&second);
    let contacts = crate::geometry::dim3::intersections::intersect_surfaces_with_options(
        first.surface(),
        second.surface(),
        options.intersections,
    )?;
    merge_coverage(plan, contacts.coverage());
    for contact in contacts.intersections() {
        match contact {
            SurfaceSurfaceIntersection::Point(contact) => {
                let point = contact.point;
                let (surface_a_u, surface_a_v) = (contact.uv_a.x, contact.uv_a.y);
                let (surface_b_u, surface_b_v) = (contact.uv_b.x, contact.uv_b.y);
                if face_contains_uv(&first_loops, Point2::new(surface_a_u, surface_a_v))
                    && face_contains_uv(&second_loops, Point2::new(surface_b_u, surface_b_v))
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
                let first_trim =
                    FaceTrimDomain::new(&first, options.intersections.parameter_tolerance)?;
                let second_trim =
                    FaceTrimDomain::new(&second, options.intersections.parameter_tolerance)?;
                for [first_imprint, second_imprint] in
                    clip_branch(branch, &first_trim, &second_trim, options.intersections)?
                {
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

/// Overlays two coplanar convex outer loops and imprints the non-boundary part
/// of the overlap polygon on each face.
fn add_planar_overlap_imprints<P: Payload>(
    g: &GMap<P>,
    plan: &mut IntersectionAccumulator,
    first_key: FaceKey,
    second_key: FaceKey,
    tolerance: f64,
) -> bool {
    let first = g.face_unchecked(first_key);
    let second = g.face_unchecked(second_key);
    let (Surface::Plane(first_plane), Surface::Plane(second_plane)) =
        (first.surface(), second.surface())
    else {
        return false;
    };
    let first_polygon = face_outer_corners_uv(&first);
    let second_in_first = face_outer_corners_uv(&second)
        .into_iter()
        .map(|uv| first_plane.parameter_at(second.point_at(uv.x, uv.y)))
        .collect::<Vec<_>>();
    let overlap = clip_convex_polygon(second_in_first, &first_polygon, tolerance);
    if overlap.len() < 3 || signed_area2(&overlap).abs() <= tolerance {
        return false;
    }

    let first_imprints = overlap_segments_for_face(&first, &overlap, tolerance);
    let overlap_second = overlap
        .iter()
        .map(|uv| {
            let point = first.point_at(uv.x, uv.y);
            second_plane.parameter_at(point)
        })
        .collect::<Vec<_>>();
    let second_imprints = overlap_segments_for_face(&second, &overlap_second, tolerance);
    plan.face_imprints
        .entry(first_key)
        .or_default()
        .extend(first_imprints);
    plan.face_imprints
        .entry(second_key)
        .or_default()
        .extend(second_imprints);
    true
}

fn face_outer_corners_uv<P: Payload>(face: &crate::topology::face::Face<'_, P>) -> Vec<Point2> {
    face.outer_loop()
        .edges()
        .into_iter()
        .filter_map(|edge| face.pcurve(edge.dart()).map(|pcurve| pcurve.point_at(0.0)))
        .collect()
}

fn overlap_segments_for_face<P: Payload>(
    face: &crate::topology::face::Face<'_, P>,
    polygon: &[Point2],
    tolerance: f64,
) -> Vec<FaceImprint> {
    polygon
        .iter()
        .copied()
        .zip(polygon.iter().copied().cycle().skip(1))
        .take(polygon.len())
        .filter_map(|(start_uv, end_uv)| {
            if (end_uv - start_uv).norm() <= tolerance {
                return None;
            }
            let midpoint = Point2::from((start_uv.coords + end_uv.coords) * 0.5);
            if point_on_face_boundary(face, midpoint, tolerance) {
                return None;
            }
            let start = face.point_at(start_uv.x, start_uv.y);
            let end = face.point_at(end_uv.x, end_uv.y);
            Some(FaceImprint::new(
                Curve::line(start, end),
                Curve2::Line(Line2::new(start_uv, end_uv)),
            ))
        })
        .collect()
}

fn clip_convex_polygon(mut subject: Vec<Point2>, clip: &[Point2], tolerance: f64) -> Vec<Point2> {
    if subject.len() < 3 || clip.len() < 3 {
        return Vec::new();
    }
    let orientation = signed_area2(clip).signum();
    for (clip_start, clip_end) in clip
        .iter()
        .copied()
        .zip(clip.iter().copied().cycle().skip(1))
        .take(clip.len())
    {
        let input = subject;
        subject = Vec::new();
        let Some(mut previous) = input.last().copied() else {
            break;
        };
        let mut previous_inside =
            polygon_half_plane_contains(clip_start, clip_end, previous, orientation, tolerance);
        for current in input {
            let current_inside =
                polygon_half_plane_contains(clip_start, clip_end, current, orientation, tolerance);
            if current_inside != previous_inside
                && let Some(intersection) =
                    segment_line_intersection(previous, current, clip_start, clip_end, tolerance)
            {
                subject.push(intersection);
            }
            if current_inside {
                subject.push(current);
            }
            previous = current;
            previous_inside = current_inside;
        }
    }
    dedup_polygon(subject, tolerance)
}

fn polygon_half_plane_contains(
    edge_start: Point2,
    edge_end: Point2,
    point: Point2,
    orientation: f64,
    tolerance: f64,
) -> bool {
    cross2(edge_end - edge_start, point - edge_start) * orientation >= -tolerance
}

fn segment_line_intersection(
    segment_start: Point2,
    segment_end: Point2,
    line_start: Point2,
    line_end: Point2,
    tolerance: f64,
) -> Option<Point2> {
    let segment = segment_end - segment_start;
    let line = line_end - line_start;
    let denominator = cross2(segment, line);
    if denominator.abs() <= tolerance {
        return None;
    }
    let parameter = cross2(line_start - segment_start, line) / denominator;
    Some(segment_start + segment * parameter)
}

fn dedup_polygon(points: Vec<Point2>, tolerance: f64) -> Vec<Point2> {
    let mut deduped = Vec::<Point2>::new();
    for point in points {
        if deduped
            .last()
            .is_none_or(|previous| (*previous - point).norm() > tolerance)
        {
            deduped.push(point);
        }
    }
    if deduped.len() > 1
        && (deduped[0] - *deduped.last().expect("non-empty polygon")).norm() <= tolerance
    {
        deduped.pop();
    }
    deduped
}

fn signed_area2(points: &[Point2]) -> f64 {
    0.5 * points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .take(points.len())
        .map(|(a, b)| a.x * b.y - b.x * a.y)
        .sum::<f64>()
}

fn line_intervals_in_face<P: Payload>(
    face: &crate::topology::face::Face<'_, P>,
    line_point: Point3,
    direction: nalgebra::Vector3<f64>,
    tolerance: f64,
) -> Vec<Interval> {
    let origin_uv = face
        .surface()
        .closest_parameter(line_point)
        .expect("planar parameter projection");
    let direction_uv = face
        .surface()
        .closest_parameter(line_point + direction)
        .expect("planar parameter projection")
        - origin_uv;
    let loops = face_uv_loops(face);
    let mut parameters = Vec::new();
    for loop_ in &loops {
        for segment in loop_
            .iter()
            .zip(loop_.iter().cycle().skip(1))
            .take(loop_.len())
        {
            if let Some(parameter) =
                line_segment_parameter(origin_uv, direction_uv, *segment.0, *segment.1, tolerance)
            {
                parameters.push(parameter);
            }
        }
    }
    parameters.sort_by(f64::total_cmp);
    parameters.dedup_by(|a, b| (*a - *b).abs() <= tolerance);
    parameters
        .windows(2)
        .filter_map(|pair| {
            let midpoint = 0.5 * (pair[0] + pair[1]);
            let uv = origin_uv + direction_uv * midpoint;
            face_contains_uv(&loops, uv).then_some(Interval::new(pair[0], pair[1]))
        })
        .collect()
}

fn face_uv_loops<P: Payload>(face: &crate::topology::face::Face<'_, P>) -> Vec<Vec<Point2>> {
    face.loops()
        .into_iter()
        .map(|loop_| {
            loop_
                .edges()
                .into_iter()
                .filter_map(|edge| face.pcurve(edge.dart()))
                .flat_map(|pcurve| {
                    let samples = pcurve.sample(16);
                    let count = samples.len().saturating_sub(1);
                    samples.into_iter().take(count)
                })
                .collect()
        })
        .collect()
}

fn line_segment_parameter(
    line_origin: Point2,
    line_direction: Vector2<f64>,
    segment_start: Point2,
    segment_end: Point2,
    tolerance: f64,
) -> Option<f64> {
    let segment_direction = segment_end - segment_start;
    let denominator = cross2(line_direction, segment_direction);
    if denominator.abs() <= tolerance {
        return None;
    }
    let delta = segment_start - line_origin;
    let line_t = cross2(delta, segment_direction) / denominator;
    let segment_t = cross2(delta, line_direction) / denominator;
    (-tolerance..=1.0 + tolerance)
        .contains(&segment_t)
        .then_some(line_t)
}

fn cross2(a: Vector2<f64>, b: Vector2<f64>) -> f64 {
    a.x * b.y - a.y * b.x
}

fn face_contains_uv(loops: &[Vec<Point2>], point: Point2) -> bool {
    let Some(outer) = loops.first() else {
        return false;
    };
    point_in_polygon(outer, point)
        && loops
            .iter()
            .skip(1)
            .all(|hole| !point_in_polygon(hole, point))
}

fn point_in_polygon(polygon: &[Point2], point: Point2) -> bool {
    if polygon.len() < 3 {
        return false;
    }
    let mut inside = false;
    for (a, b) in polygon
        .iter()
        .zip(polygon.iter().cycle().skip(1))
        .take(polygon.len())
    {
        let crosses = (a.y > point.y) != (b.y > point.y)
            && point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x;
        if crosses {
            inside = !inside;
        }
    }
    inside
}
