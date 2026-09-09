//! Narrow-phase geometric contact computation.
//!
//! Every probe here is a pure function of two cells' geometry. None of them
//! learns which operand a cell came from, and none of them writes to the
//! accumulator: a probe answers what it saw about the pair's edge and face and
//! returns. The driver walks one stream of candidate pairs, dispatches on the
//! pair's kinds, and hands each answer to [`record`] -- the single place
//! operand order is applied.
//!
//! That shape is what removes the mirrored passes. A heterogeneous pair such as
//! edge/face used to be probed twice, once per assignment of the operands to
//! its halves, with a flag threaded down to say which was which; that mirror is
//! now a second [`PairKind`] variant that the same probe answers.

use std::collections::hash_map::Entry;
use std::rc::Rc;

use super::pair::{Contact, ContactCell, PairKind, record};
use super::*;
use super::{
    clip::clip_branch,
    trim::{FaceTrimDomain, TrimLocation, boundary_edge_for},
};
use crate::geometry::{
    CurveIntersectionOptions, IntersectionCoverage, IntersectionIncompleteReason,
    SurfaceIntersectionBranch,
};

/// Decomposed operand geometry reused across every candidate pair.
///
/// Bézier decomposition dominates a single curve/surface query, so an operand
/// touched by many pairs must be decomposed once rather than once per pair.
/// Handed out as `Rc`s so a probe can hold two at once, which a surface pair
/// needs, without copying a decomposition.
#[derive(Default)]
struct PreparedGeometry {
    edges: HashMap<EdgeKey, Rc<PreparedCurve>>,
    faces: HashMap<FaceKey, Rc<PreparedSurface>>,
}

impl PreparedGeometry {
    fn edge<P: Payload>(
        &mut self,
        g: &GMap<P>,
        key: EdgeKey,
    ) -> Result<Rc<PreparedCurve>, BooleanError> {
        match self.edges.entry(key) {
            Entry::Occupied(entry) => Ok(Rc::clone(entry.get())),
            Entry::Vacant(entry) => {
                let edge = g.edge_unchecked(key);
                let curve = edge.curve().expect("registered edge geometry");
                Ok(Rc::clone(entry.insert(Rc::new(PreparedCurve::new(curve)?))))
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
    ) -> Result<Rc<PreparedSurface>, BooleanError> {
        match self.faces.entry(key) {
            Entry::Occupied(entry) => Ok(Rc::clone(entry.get())),
            Entry::Vacant(entry) => {
                let face = g.face_unchecked(key);
                let prepared = prepare_face_surface(&face, tolerance)?;
                Ok(Rc::clone(entry.insert(Rc::new(prepared))))
            }
        }
    }
}

/// Trim domains kept for the life of one contact computation.
///
/// Flattening a face's loops is one of the costlier things a pair does, and a
/// face of a solid is reached by many pairs -- once per candidate edge, plus
/// once per candidate face. Handing out `Rc`s rather than references lets a
/// probe hold two domains at once, which a surface pair needs.
#[derive(Default)]
struct TrimCache {
    domains: HashMap<FaceKey, Rc<FaceTrimDomain>>,
}

impl TrimCache {
    fn get<P: Payload>(
        &mut self,
        g: &GMap<P>,
        key: FaceKey,
        tolerance: f64,
    ) -> Result<Rc<FaceTrimDomain>, BooleanError> {
        if let Some(domain) = self.domains.get(&key) {
            return Ok(Rc::clone(domain));
        }
        let face = g.face_unchecked(key);
        let domain = Rc::new(FaceTrimDomain::new(&face, tolerance)?);
        self.domains.insert(key, Rc::clone(&domain));
        Ok(domain)
    }
}

/// A section found, but not yet cut down to the region it really covers.
///
/// A section is a fit. Where one runs into a point both operands located
/// exactly, that point -- not the crossing the fit reports for itself -- is the
/// node it was approximating. Judging that against only the points found so far
/// would make a pair's answer depend on which pairs happened to run before it,
/// so clipping waits until finding is complete and every point is known.
enum Deferred {
    /// An edge resting on a face, before the face's trim clips the section.
    EdgeOnFace { imprint: FaceImprint },
    /// A surface/surface branch, before either face's trim clips it.
    Branch {
        branch: Box<SurfaceIntersectionBranch>,
        tangent: bool,
    },
}

/// Everything one pair's probe observed, in the pair's own geometric roles.
///
/// Diagnostics travel back with the contacts rather than being written in
/// place, so a probe stays a function of its two cells.
#[derive(Default)]
struct Probed {
    contacts: Vec<Contact>,
    /// Sections awaiting the complete point set.
    deferred: Vec<Deferred>,
    /// Reasons this pair's answer was not certified.
    coverage: Vec<IntersectionIncompleteReason>,
    branches_found: usize,
    branches_uncertified: usize,
    /// Overlaps this surface pair reported that the solver could not resolve.
    unresolved_overlaps: usize,
}

impl Probed {
    fn of(contacts: Vec<Contact>) -> Self {
        Self {
            contacts,
            ..Default::default()
        }
    }

    /// Records every distinct reason a narrow-phase result was not certified.
    fn observe(&mut self, coverage: &IntersectionCoverage) {
        let IntersectionCoverage::Incomplete(reasons) = coverage else {
            return;
        };
        for reason in reasons {
            if !self.coverage.contains(reason) {
                self.coverage.push(*reason);
            }
        }
    }
}

/// Computes every contact between the two operands.
///
/// Two phases. Finding probes each pair independently -- no pair can see
/// another's result -- and yields every point, plus the sections it found
/// uncut. Clipping then cuts those sections against the complete point set.
///
/// The split is what makes the answer independent of the order pairs are
/// probed in: a section's nodes are decided against all the exact points that
/// exist, not against the ones that happened to be found first.
pub(super) fn compute_contacts<P: Payload>(
    g: &GMap<P>,
    plan: &mut IntersectionAccumulator,
    options: BooleanOptions,
) -> Result<(), BooleanError> {
    let pairs = enumerate_pairs(g, plan, options);
    let mut prepared = PreparedGeometry::default();
    let mut trims = TrimCache::default();
    let graze = plan.diagnostics.tolerances.graze;
    let mut timer = StageTimer::default();
    let mut deferred = Vec::new();

    for pair in pairs {
        timer.enter(&mut plan.diagnostics.stages, timing_group(pair));
        let probed = probe(g, &mut prepared, &mut trims, pair, graze, options)?;
        for reason in probed.coverage {
            if !plan.diagnostics.coverage.contains(&reason) {
                plan.diagnostics.coverage.push(reason);
            }
        }
        plan.diagnostics.branches_found += probed.branches_found;
        plan.diagnostics.branches_uncertified += probed.branches_uncertified;
        if probed.unresolved_overlaps > 0 {
            if let (BooleanCell::Face(first), BooleanCell::Face(second)) = pair.ordered_cells() {
                plan.diagnostics
                    .unresolved_overlaps
                    .extend(std::iter::repeat_n(
                        (first, second),
                        probed.unresolved_overlaps,
                    ));
            }
        }
        deferred.extend(probed.deferred.into_iter().map(|work| (pair, work)));
        record(plan, pair, probed.contacts);
    }

    // Every point both operands located exactly, now that finding is done.
    let anchors = plan
        .contacts
        .iter()
        .filter_map(|contact| match contact {
            RawIntersection::Point { point, .. } => Some(*point),
            _ => None,
        })
        .collect::<Vec<_>>();
    for (pair, work) in deferred {
        timer.enter(&mut plan.diagnostics.stages, timing_group(pair));
        let contacts = clip_deferred(g, &mut trims, pair, work, &anchors, graze, options)?;
        record(plan, pair, contacts);
    }
    timer.finish(&mut plan.diagnostics.stages);
    Ok(())
}

/// Attributes elapsed time to whichever stage bucket is currently open.
///
/// Both phases walk pairs grouped by kind, so one lap per group change reports
/// the same per-kind stages the four separate passes used to, without paying
/// for a clock read on every pair.
#[derive(Default)]
struct StageTimer {
    clock: Option<Instant>,
    group: Option<usize>,
}

impl StageTimer {
    fn enter(&mut self, stages: &mut BooleanStageTimings, group: usize) {
        if self.group == Some(group) {
            return;
        }
        self.finish(stages);
        self.clock = Some(Instant::now());
        self.group = Some(group);
    }

    fn finish(&mut self, stages: &mut BooleanStageTimings) {
        if let (Some(clock), Some(group)) = (self.clock.take(), self.group.take()) {
            *timing_slot(stages, group) += clock.elapsed();
        }
    }
}

/// Cuts one deferred section down to the region it really covers.
fn clip_deferred<P: Payload>(
    g: &GMap<P>,
    trims: &mut TrimCache,
    pair: PairKind,
    work: Deferred,
    anchors: &[Point3],
    graze: f64,
    options: BooleanOptions,
) -> Result<Vec<Contact>, BooleanError> {
    let tolerance = options.intersections.parameter_tolerance;
    let mut contacts = Vec::new();
    match work {
        Deferred::EdgeOnFace { imprint } => {
            let edge_key = pair.edge(ContactCell::Edge);
            let face_key = pair.face(ContactCell::Face);
            let edge = g.edge_unchecked(edge_key);
            let curve = edge.curve().expect("registered edge geometry");
            let trim = trims.get(g, face_key, tolerance)?;
            for piece in clip_imprint_to_trim(&trim, &imprint, anchors, graze, options)? {
                let start = piece.curve.point_at(0.0);
                let end = piece.curve.point_at(1.0);
                contacts.push(Contact::EdgePoint {
                    cell: ContactCell::Edge,
                    point: start,
                });
                contacts.push(Contact::EdgePoint {
                    cell: ContactCell::Edge,
                    point: end,
                });
                // The section belongs to both cells: the edge carries it
                // already, the face has to be split along it.
                contacts.push(Contact::EdgeSection {
                    cell: ContactCell::Edge,
                    curve: piece.curve.clone(),
                    interval: Interval::new(curve.param_at(start), curve.param_at(end)),
                });
                contacts.push(Contact::Imprint {
                    cell: ContactCell::Face,
                    imprint: piece,
                    tangent: false,
                });
            }
        }
        Deferred::Branch { branch, tangent } => {
            let a_key = pair.face(ContactCell::First);
            let b_key = pair.face(ContactCell::Second);
            let a = g.face_unchecked(a_key);
            let b = g.face_unchecked(b_key);
            let a_trim = trims.get(g, a_key, tolerance)?;
            let b_trim = trims.get(g, b_key, tolerance)?;
            let clipped = clip_branch(
                &branch,
                (a.surface(), &a_trim),
                (b.surface(), &b_trim),
                anchors,
                graze,
                options.intersections,
            )?;
            for [a_imprint, b_imprint] in clipped {
                contacts.push(Contact::Imprint {
                    cell: ContactCell::First,
                    imprint: a_imprint,
                    tangent,
                });
                contacts.push(Contact::Imprint {
                    cell: ContactCell::Second,
                    imprint: b_imprint,
                    tangent,
                });
            }
        }
    }
    Ok(contacts)
}

/// Which reported stage a pair's work is attributed to.
///
/// The three vertex kinds share one bucket: they are one sweep over the same
/// cheap coincidence tests, and splitting them would report noise.
fn timing_group(kind: PairKind) -> usize {
    match kind {
        PairKind::VertexVertex(..)
        | PairKind::VertexEdge(..)
        | PairKind::EdgeVertex(..)
        | PairKind::VertexFace(..)
        | PairKind::FaceVertex(..) => 0,
        PairKind::EdgeEdge(..) => 1,
        PairKind::EdgeFace(..) | PairKind::FaceEdge(..) => 2,
        PairKind::FaceFace(..) => 3,
    }
}

fn timing_slot(stages: &mut BooleanStageTimings, group: usize) -> &mut Duration {
    match group {
        0 => &mut stages.vertex_contacts,
        1 => &mut stages.edge_contacts,
        2 => &mut stages.edge_face_contacts,
        _ => &mut stages.face_contacts,
    }
}

/// Selects and runs the probe the pair's cell kinds call for.
///
/// Nothing reachable from here can see another pair's result, which is what
/// makes finding order-independent.
fn probe<P: Payload>(
    g: &GMap<P>,
    prepared: &mut PreparedGeometry,
    trims: &mut TrimCache,
    pair: PairKind,
    graze: f64,
    options: BooleanOptions,
) -> Result<Probed, BooleanError> {
    let tolerance = options.intersections.linear_tolerance;
    // The mirrored variants ask the same geometric question of operands in the
    // opposite order, so each pattern binds the two roles and one probe answers
    // both.
    match pair {
        PairKind::VertexVertex(a, b) => Ok(Probed::of(probe_vertex_vertex(g, a, b, tolerance))),
        PairKind::VertexEdge(vertex, edge) | PairKind::EdgeVertex(edge, vertex) => {
            Ok(Probed::of(probe_vertex_edge(g, vertex, edge, tolerance)))
        }
        PairKind::VertexFace(vertex, face) | PairKind::FaceVertex(face, vertex) => Ok(Probed::of(
            probe_vertex_face(g, trims, vertex, face, options)?,
        )),
        PairKind::EdgeEdge(a, b) => Ok(Probed::of(probe_edge_edge(g, a, b, graze, options)?)),
        PairKind::EdgeFace(edge, face) | PairKind::FaceEdge(face, edge) => {
            probe_edge_face(g, prepared, trims, edge, face, graze, options)
        }
        PairKind::FaceFace(a, b) => probe_face_face(g, prepared, trims, a, b, options),
    }
}

/// Enumerates every pair worth probing, and counts what broad phase rejected.
///
/// Vertex pairs are enumerated exhaustively: there are few of them, and a
/// bounds test on a point costs about as much as the coincidence test it would
/// be replacing.
fn enumerate_pairs<P: Payload>(
    g: &GMap<P>,
    plan: &mut IntersectionAccumulator,
    options: BooleanOptions,
) -> Vec<PairKind> {
    let first = &plan.first_cells;
    let second = &plan.second_cells;
    let mut pairs = Vec::new();

    for a in first.vertices.iter().copied() {
        for b in second.vertices.iter().copied() {
            if a != b {
                pairs.push(PairKind::VertexVertex(a, b));
            }
        }
    }
    for vertex in first.vertices.iter().copied() {
        for edge in second.edges.iter().copied() {
            pairs.push(PairKind::VertexEdge(vertex, edge));
        }
    }
    for edge in first.edges.iter().copied() {
        for vertex in second.vertices.iter().copied() {
            pairs.push(PairKind::EdgeVertex(edge, vertex));
        }
    }
    // Face-major, so a face's trim domain is reached by consecutive pairs.
    for face in second.faces.iter().copied() {
        for vertex in first.vertices.iter().copied() {
            pairs.push(PairKind::VertexFace(vertex, face));
        }
    }
    for face in first.faces.iter().copied() {
        for vertex in second.vertices.iter().copied() {
            pairs.push(PairKind::FaceVertex(face, vertex));
        }
    }
    for a in first.edges.iter().copied() {
        for b in second.edges.iter().copied() {
            if a != b {
                pairs.push(PairKind::EdgeEdge(a, b));
            }
        }
    }
    // Both directions share the broad phase; only which operand supplied the
    // edge differs, and that is the variant.
    for (edges, faces, pair_of) in [
        (
            &first.edges,
            &second.faces,
            PairKind::EdgeFace as fn(EdgeKey, FaceKey) -> PairKind,
        ),
        (&second.edges, &first.faces, |edge, face| {
            PairKind::FaceEdge(face, edge)
        }),
    ] {
        let candidates = broad_phase::candidate_edge_face_pairs(
            g,
            edges,
            faces,
            options.intersections.bbox_tolerance,
        );
        plan.diagnostics.edge_face_pairs_tested += candidates.pairs.len();
        plan.diagnostics.edge_face_pairs_pruned += candidates.pruned;
        pairs.extend(
            candidates
                .pairs
                .into_iter()
                .map(|(edge, face)| pair_of(edge, face)),
        );
    }
    let candidates = broad_phase::candidate_face_pairs(
        g,
        &first.faces,
        &second.faces,
        options.intersections.bbox_tolerance,
    );
    plan.diagnostics.candidate_pairs_tested = candidates.pairs.len();
    plan.diagnostics.candidate_pairs_pruned = candidates.pruned;
    pairs.extend(
        candidates
            .pairs
            .into_iter()
            .map(|(a, b)| PairKind::FaceFace(a, b)),
    );
    pairs
}

fn vertex_point<P: Payload>(g: &GMap<P>, key: VertexKey) -> Point3 {
    *g.vertex_unchecked(key)
        .point()
        .expect("registered vertex geometry")
}

fn edge_curve<P: Payload>(g: &GMap<P>, key: EdgeKey) -> &Curve {
    g.edge_unchecked(key)
        .curve()
        .expect("registered edge geometry")
}

fn probe_vertex_vertex<P: Payload>(
    g: &GMap<P>,
    a: VertexKey,
    b: VertexKey,
    tolerance: f64,
) -> Vec<Contact> {
    let point = vertex_point(g, a);
    if point.coincides(vertex_point(g, b), tolerance) {
        vec![Contact::Point {
            point,
            kind: PointContactKind::Transverse,
        }]
    } else {
        Vec::new()
    }
}

fn probe_vertex_edge<P: Payload>(
    g: &GMap<P>,
    vertex_key: VertexKey,
    edge_key: EdgeKey,
    tolerance: f64,
) -> Vec<Contact> {
    let point = vertex_point(g, vertex_key);
    let edge = g.edge_unchecked(edge_key);
    let curve = edge_curve(g, edge_key);
    let parameter = curve.param_at(point);
    if !curve.point_at(parameter).coincides(point, tolerance) {
        return Vec::new();
    }
    let start = *edge
        .start()
        .point()
        .expect("registered edge start geometry");
    let end = *edge.end().point().expect("registered edge end geometry");
    if !curve
        .interval_between(start, end)
        .ordered()
        .contains(parameter, tolerance)
    {
        return Vec::new();
    }
    vec![
        Contact::EdgePoint {
            cell: ContactCell::Edge,
            point,
        },
        Contact::Point {
            point,
            kind: PointContactKind::Transverse,
        },
    ]
}

/// Records a vertex lying inside one trimmed face.
///
/// The trim domain is consulted only once the vertex has actually reached the
/// face's surface. A vertex that misses the surface -- the common case, since
/// most pairs never touch -- therefore costs one closest point and one surface
/// evaluation, and never builds a domain.
fn probe_vertex_face<P: Payload>(
    g: &GMap<P>,
    trims: &mut TrimCache,
    vertex_key: VertexKey,
    face_key: FaceKey,
    options: BooleanOptions,
) -> Result<Vec<Contact>, BooleanError> {
    let face = g.face_unchecked(face_key);
    let point = vertex_point(g, vertex_key);
    let Ok(uv) = face.surface().param_at(point) else {
        return Ok(Vec::new());
    };
    if !face
        .surface()
        .point_at(uv.x, uv.y)
        .coincides(point, options.intersections.linear_tolerance)
    {
        return Ok(Vec::new());
    }
    let trim = trims.get(g, face_key, options.intersections.parameter_tolerance)?;
    if !trim.contains(uv) {
        return Ok(Vec::new());
    }
    Ok(vec![Contact::Point {
        point,
        kind: PointContactKind::Transverse,
    }])
}

fn probe_edge_edge<P: Payload>(
    g: &GMap<P>,
    a_key: EdgeKey,
    b_key: EdgeKey,
    graze: f64,
    options: BooleanOptions,
) -> Result<Vec<Contact>, BooleanError> {
    let a_edge = g.edge_unchecked(a_key);
    let a_curve = edge_curve(g, a_key);
    let b_edge = g.edge_unchecked(b_key);
    let b_curve = edge_curve(g, b_key);
    let mut contacts = Vec::new();
    for intersection in a_curve.intersect_curve_with_options(b_curve, options.intersections)? {
        match intersection {
            CurveCurveIntersection::Point { point, .. } => {
                let point = grazed_vertex(
                    [&a_edge, &b_edge],
                    |candidate| {
                        [a_curve, b_curve].iter().all(|curve| {
                            lies_on_curve(curve, candidate, options.intersections.linear_tolerance)
                        })
                    },
                    point,
                    options.intersections.linear_tolerance,
                    graze,
                )
                .unwrap_or(point);
                contacts.push(Contact::EdgePoint {
                    cell: ContactCell::First,
                    point,
                });
                contacts.push(Contact::EdgePoint {
                    cell: ContactCell::Second,
                    point,
                });
                contacts.push(Contact::Point {
                    point,
                    kind: PointContactKind::Transverse,
                });
            }
            CurveCurveIntersection::Overlap {
                interval_a,
                interval_b,
            } => {
                for (cell, curve, interval) in [
                    (ContactCell::First, a_curve, interval_a),
                    (ContactCell::Second, b_curve, interval_b),
                ] {
                    contacts.push(Contact::EdgePoint {
                        cell,
                        point: curve.point_at(interval.start),
                    });
                    contacts.push(Contact::EdgePoint {
                        cell,
                        point: curve.point_at(interval.end),
                    });
                }
                contacts.push(Contact::EdgeOverlap {
                    a: interval_a,
                    b: interval_b,
                });
            }
        }
    }
    Ok(contacts)
}

fn probe_edge_face<P: Payload>(
    g: &GMap<P>,
    prepared: &mut PreparedGeometry,
    trims: &mut TrimCache,
    edge_key: EdgeKey,
    face_key: FaceKey,
    graze: f64,
    options: BooleanOptions,
) -> Result<Probed, BooleanError> {
    let edge = g.edge_unchecked(edge_key);
    let face = g.face_unchecked(face_key);
    let curve = edge_curve(g, edge_key);
    let trim = trims.get(g, face_key, options.intersections.parameter_tolerance)?;
    // A trimmed edge is the only shape whose closed-form answer arrives in the
    // parameter the overlap handling below expects; anything else keeps the
    // searched path, which decomposes both operands to say the same thing.
    let analytic = matches!(curve, Curve::Bounded(_))
        .then(|| {
            crate::geometry::intersect_analytic_curve_surface(
                curve,
                face.surface(),
                options.intersections,
            )
        })
        .flatten();
    let normalized = analytic.is_some();
    let intersections = match analytic {
        Some(contacts) => contacts?,
        None => {
            let prepared_curve = prepared.edge(g, edge_key)?;
            let prepared_face =
                prepared.face(g, face_key, options.intersections.linear_tolerance)?;
            intersect_prepared_curve_surface(
                &prepared_curve,
                &prepared_face,
                options.intersections,
            )?
        }
    };
    let mut probed = Probed::default();
    probed.observe(intersections.coverage());
    for intersection in intersections {
        match intersection {
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
                        surface.param_at(candidate).is_ok_and(|uv| {
                            surface
                                .point_at(uv.x, uv.y)
                                .coincides(candidate, options.intersections.linear_tolerance)
                        })
                    },
                    point,
                    options.intersections.linear_tolerance,
                    graze,
                )
                .unwrap_or(point);
                let tangent = curve.derivative_at(curve_u, 1);
                let normal = face.surface().normal_at(surface_u, surface_v);
                let kind = if tangent.dot(&normal).abs()
                    <= options.intersections.angular_tolerance * tangent.norm() * normal.norm()
                {
                    PointContactKind::Tangent
                } else {
                    PointContactKind::Transverse
                };
                probed.contacts.push(Contact::EdgePoint {
                    cell: ContactCell::Edge,
                    point,
                });
                probed.contacts.push(Contact::Point { point, kind });
            }
            CurveSurfaceIntersection::Overlap { curve_interval } => {
                // The edge rests on the surface over this interval, but only the
                // part the face's own trim keeps is a contact of the two cells.
                // The searched path reports the interval in the curve's own
                // NURBS parameters and the closed-form path in normalized ones;
                // subcurves are taken over normalized ones.
                let interval = if normalized {
                    Interval::new(
                        curve_interval.start.clamp(0.0, 1.0),
                        curve_interval.end.clamp(0.0, 1.0),
                    )
                } else {
                    let native = curve.to_nurbs()?.domain();
                    let extent = native.end - native.start;
                    Interval::new(
                        ((curve_interval.start - native.start) / extent).clamp(0.0, 1.0),
                        ((curve_interval.end - native.start) / extent).clamp(0.0, 1.0),
                    )
                };
                let section = graph::normalized_subcurve(curve, interval)?;
                let Some(imprint) = section_imprint(&face, &section, options)? else {
                    continue;
                };
                probed.deferred.push(Deferred::EdgeOnFace { imprint });
            }
        }
    }
    Ok(probed)
}

fn probe_face_face<P: Payload>(
    g: &GMap<P>,
    prepared: &mut PreparedGeometry,
    trims: &mut TrimCache,
    a_key: FaceKey,
    b_key: FaceKey,
    options: BooleanOptions,
) -> Result<Probed, BooleanError> {
    let a = g.face_unchecked(a_key);
    let b = g.face_unchecked(b_key);
    let (Surface::Plane(a_plane), Surface::Plane(b_plane)) = (a.surface(), b.surface()) else {
        return probe_general_face_pair(g, prepared, trims, a_key, b_key, options);
    };

    let a_normal = *a_plane.normal();
    let b_normal = *b_plane.normal();
    let cross = a_normal.cross(&b_normal);
    let cross_squared = cross.norm_squared();
    if cross_squared <= options.intersections.angular_tolerance.powi(2) {
        // The section each face cuts out of the other is its partner's boundary
        // clipped to its own trim, which the edge/face pass already imprinted.
        // Only the region record is left to make here.
        let distance = (b_plane.origin() - a_plane.origin()).dot(&a_normal);
        if distance.abs() <= options.intersections.linear_tolerance
            && coplanar_faces_share_area(&a, &b, trims, g, [a_key, b_key], options)?
        {
            return Ok(Probed::of(vec![Contact::Region]));
        }
        return Ok(Probed::default());
    }

    let direction = cross.normalize();
    let a_offset = a_normal.dot(&a_plane.origin().coords);
    let b_offset = b_normal.dot(&b_plane.origin().coords);
    let line_point = Point3::from(
        (a_offset * b_normal.cross(&cross) + b_offset * cross.cross(&a_normal)) / cross_squared,
    );
    let a_intervals = line_intervals_in_face(
        &a,
        trims.get(g, a_key, options.intersections.parameter_tolerance)?,
        line_point,
        direction,
        options.intersections,
    )?;
    let b_intervals = line_intervals_in_face(
        &b,
        trims.get(g, b_key, options.intersections.parameter_tolerance)?,
        line_point,
        direction,
        options.intersections,
    )?;

    let mut contacts = Vec::new();
    for a_interval in a_intervals {
        for b_interval in &b_intervals {
            let start_t = a_interval.start.max(b_interval.start);
            let end_t = a_interval.end.min(b_interval.end);
            if end_t - start_t <= options.intersections.linear_tolerance {
                continue;
            }
            let start = line_point + direction * start_t;
            let end = line_point + direction * end_t;
            let curve = Curve::line(start, end);
            for (cell, plane) in [
                (ContactCell::First, a_plane),
                (ContactCell::Second, b_plane),
            ] {
                let pcurve = Curve2::Line(Line2::new(
                    plane.parameter_at(start),
                    plane.parameter_at(end),
                ));
                contacts.push(Contact::Imprint {
                    cell,
                    imprint: FaceImprint::new(curve.clone(), pcurve),
                    tangent: false,
                });
            }
        }
    }
    Ok(Probed::of(contacts))
}

fn probe_general_face_pair<P: Payload>(
    g: &GMap<P>,
    prepared: &mut PreparedGeometry,
    trims: &mut TrimCache,
    a_key: FaceKey,
    b_key: FaceKey,
    options: BooleanOptions,
) -> Result<Probed, BooleanError> {
    let a = g.face_unchecked(a_key);
    let b = g.face_unchecked(b_key);
    // The table is asked before the trim domains and the Bezier decompositions
    // are built, because building those is itself most of the cost of a face
    // pair -- skipping only the search would leave that cost in place.
    let analytic = match crate::geometry::intersect_analytic_surfaces(
        a.surface(),
        b.surface(),
        options.intersections,
    ) {
        Some(analytic) => crate::geometry::analytic_surface_intersections(
            analytic?,
            a.surface(),
            b.surface(),
            options.intersections,
        ),
        None => None,
    };
    let intersections = match analytic {
        Some(contacts) => contacts,
        None => {
            let prepared_a = prepared.face(g, a_key, options.intersections.linear_tolerance)?;
            let prepared_b = prepared.face(g, b_key, options.intersections.linear_tolerance)?;
            crate::geometry::intersect_prepared_surfaces(
                &prepared_a,
                &prepared_b,
                options.intersections,
            )?
        }
    };
    if intersections.is_empty()
        && matches!(intersections.coverage(), IntersectionCoverage::Complete)
    {
        // A certified empty answer needs no trim domain at all, which is the
        // common case for a face pair whose supports simply do not meet.
        return Ok(Probed::default());
    }
    let a_trim = trims.get(g, a_key, options.intersections.parameter_tolerance)?;
    let b_trim = trims.get(g, b_key, options.intersections.parameter_tolerance)?;
    let mut probed = Probed::default();
    probed.observe(intersections.coverage());
    for contact in intersections.intersections() {
        match contact {
            SurfaceSurfaceIntersection::Point(contact) => {
                if a_trim.contains(contact.uv_a) && b_trim.contains(contact.uv_b) {
                    probed.contacts.push(Contact::Point {
                        point: contact.point,
                        kind: PointContactKind::Transverse,
                    });
                }
            }
            SurfaceSurfaceIntersection::Branch(branch) => {
                probed.branches_found += 1;
                probed.branches_uncertified += usize::from(!branch.quality.certified);
                probed.deferred.push(Deferred::Branch {
                    branch: Box::new(branch.clone()),
                    tangent: branch.kind == crate::geometry::SurfaceIntersectionBranchKind::Tangent,
                });
            }
            SurfaceSurfaceIntersection::OverlapCandidate(_) => probed.unresolved_overlaps += 1,
        }
    }
    Ok(probed)
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
    match PreparedSurface::over(
        surface,
        grown(domain_u, closed_in_u),
        grown(domain_v, closed_in_v),
    ) {
        Ok(prepared) => Ok(prepared),
        // A surface that cannot be realized past its own extent keeps the box
        // the face gave it.
        Err(_) => Ok(PreparedSurface::over(surface, domain_u, domain_v)?),
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
        let Ok(uv) = face.surface().param_at(point) else {
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
        // Crossings are merged at the caller's tolerance, which is finer than
        // a curve can be trimmed to; a window below that carries no piece.
        if pair[1] - pair[0] <= crate::geometry::LINEAR_TOLERANCE {
            continue;
        }
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

/// Whether two coplanar faces share area rather than only touching.
///
/// Faces that meet along an edge or at a corner have every shared point on both
/// boundaries, so no probe reaches the interior of the other. Each face is
/// probed at its own boundary samples — which catches a partial overlap — and at
/// their centroid, which catches a face contained in the other.
fn coplanar_faces_share_area<P: Payload>(
    a: &crate::topology::face::Face<'_, P>,
    b: &crate::topology::face::Face<'_, P>,
    trims: &mut TrimCache,
    g: &GMap<P>,
    keys: [FaceKey; 2],
    options: BooleanOptions,
) -> Result<bool, BooleanError> {
    let tolerance = options.intersections.parameter_tolerance;
    let domains = [
        trims.get(g, keys[0], tolerance)?,
        trims.get(g, keys[1], tolerance)?,
    ];
    for (index, face) in [a, b].into_iter().enumerate() {
        let other = [b, a][index];
        let other_trim = &domains[1 - index];
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
                .param_at(*point)
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
    trim: Rc<FaceTrimDomain>,
    line_point: Point3,
    direction: nalgebra::Vector3<f64>,
    options: IntersectionOptions,
) -> Result<Vec<Interval>, BooleanError> {
    let origin_uv = face
        .surface()
        .param_at(line_point)
        .expect("planar parameter projection");
    let direction_uv = face
        .surface()
        .param_at(line_point + direction)
        .expect("planar parameter projection")
        - origin_uv;
    Ok(trim.line_intervals(origin_uv, direction_uv, options)?)
}

fn cross2(a: Vector2<f64>, b: Vector2<f64>) -> f64 {
    a.x * b.y - a.y * b.x
}
