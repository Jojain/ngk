//! Offsetting by re-embedding: the topology stays, every support moves.
//!
//! An offset that loses no edge and no face has the topology of its source,
//! so it is computed the way a rigid motion is: as new geometry for the same
//! entities. Planning reads the model and computes every new support, point
//! and pcurve; applying writes them and touches no dart.
//!
//! **A face moves along its own normal.** Its surface is offset by the
//! distance signed by which way the face faces its support, so a positive
//! distance moves every face of a solid outward.
//!
//! **An edge moves one of two ways**, decided by its faces:
//!
//! - *preserved*: an edge of a wire, a free boundary, or an edge its two faces
//!   continue each other across. It moves along its one normal and keeps its
//!   parameterization, so its pcurves stay valid as they are.
//! - *sharp*: two faces meet at an angle along it. Their offsets no longer
//!   meet along the old edge's offset; the edge becomes their intersection,
//!   extended or trimmed to where they meet. This is the intersection join
//!   every kernel offers; a rolling-ball join would add faces, which a
//!   re-embedding cannot.
//!
//! **A vertex goes where its moved neighbours meet**: along the normal where
//! everything around it is preserved, at the meeting point of its faces'
//! offsets (or of its two edges' offsets on a wire) where something is sharp.
//!
//! What would need new topology is refused: two moved neighbours that no
//! longer reach each other, an edge whose moved neighbours meet past its far
//! end, a corner where the moved faces do not meet at one point.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use nalgebra::{Matrix3, UnitVector3, Vector2, Vector3};

use super::errors::OffsetBuildError;
use super::on_surface::offset_on_surface;
use crate::builders::blend::pcurve::{closed_pcurve_on, onto_branch, pcurve_on};
use crate::geometry::parameter::{Fraction, NativeParam};
use crate::geometry::{
    Circle, Curve, IntersectionOptions, Interval, LINEAR_TOLERANCE, Periodicity, Plane, Point2,
    Point3, PointCoincidence, Surface, SurfaceSurfaceIntersection, TrimmedCurve, TrimmedCurve2,
    intersect_analytic_surfaces,
};
use crate::model::{Cell1, Model};
use crate::topology::ModelEdit;
use crate::topology::face::Face;
use crate::topology::gmap::{Dart, Dim};
use crate::topology::orientation::Orientation;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey, VertexKey};
use crate::topology::vertex::Vertex;

/// How far past its span a fitted edge offset is computed, as a fraction of
/// the span, so a corner that moves along the extended support still lands
/// on it.
const SPAN_MARGIN: f64 = 0.5;
/// How far past a face's pcurves a fitted surface offset is computed, as a
/// fraction of their extent in each direction.
const FACE_MARGIN: f64 = 0.5;
/// How close two moved neighbours must come to be said to meet.
const MEET_TOLERANCE: f64 = 1.0e-7;
/// Normals closer than this along an edge make its faces continue each other.
const TANGENT_TOLERANCE: f64 = 1.0e-6;
/// Points along an edge its faces' normals are compared at.
const TANGENCY_PROBES: usize = 5;
/// Newton steps a meeting point is refined with.
const MEET_ITERATIONS: usize = 50;

/// New geometry for every entity an offset moves.
#[derive(Default)]
pub(crate) struct Moves {
    surfaces: Vec<(FaceKey, Surface)>,
    curves: BTreeMap<EdgeKey, Curve>,
    points: BTreeMap<VertexKey, Point3>,
    pcurves: Vec<(FaceKey, Dart, TrimmedCurve2)>,
}

impl Moves {
    /// Writes every move into the model. Nothing here can fail: the plan
    /// already holds every value, and no dart changes.
    pub(crate) fn apply<P: Payload>(self, edit: &mut ModelEdit<'_, P>) {
        for (face, surface) in self.surfaces {
            edit.face_attr_mut_unchecked(face).surface = surface;
        }
        for (edge, curve) in self.curves {
            edit.edge_attr_mut_unchecked(edge).curve = curve;
        }
        for (vertex, point) in self.points {
            edit.vertex_attr_mut_unchecked(vertex).point = point;
        }
        for (face, dart, pcurve) in self.pcurves {
            edit.face_attr_mut_unchecked(face)
                .pcurves
                .insert(dart, pcurve);
        }
    }
}

/// One end of an edge, as the corner sees it.
#[derive(Clone, Copy)]
struct Incidence {
    edge: EdgeKey,
    /// The support parameter at the corner.
    parameter: f64,
    /// The unit direction the edge leaves the corner in.
    leaving: Vector3<f64>,
}

/// An edge's two corners in its stored direction, or `None` for an unmarked
/// edge.
fn corners<P: Payload>(model: &Model<P>, edge: EdgeKey) -> Option<(VertexKey, VertexKey)> {
    let dart = model.edge_attr_unchecked(edge).dart;
    Vertex::from_dart(model, dart)
        .zip(Vertex::from_dart(model, model.alpha(Dim::Zero, dart)))
        .map(|(start, end)| (start.key(), end.key()))
}

/// The span of `curve` between two corner points, read the way an edge
/// derives its own, or the whole support for an unmarked edge.
fn span_between(curve: &Curve, ends: Option<(Point3, Point3)>) -> Interval {
    match ends {
        Some((start, end))
            if start.coincides(end, LINEAR_TOLERANCE)
                && matches!(curve.periodicity(), Periodicity::None)
                && curve.closes_at(start) =>
        {
            curve.domain()
        }
        Some((start, end)) => curve.interval_between(start, end),
        None => curve.domain(),
    }
}

/// `span` widened by [`SPAN_MARGIN`] at both ends, clipped to the support's
/// own domain.
fn widened(curve: &Curve, span: Interval) -> Interval {
    let (start, end) = (span.start.value(), span.end.value());
    let (low, high) = (start.min(end), start.max(end));
    let margin = SPAN_MARGIN * (high - low);
    let domain = curve.domain();
    let (domain_low, domain_high) = (
        domain.start.value().min(domain.end.value()),
        domain.start.value().max(domain.end.value()),
    );
    if matches!(curve.periodicity(), Periodicity::Periodic(_)) {
        return Interval::new(low - margin, high + margin);
    }
    Interval::new(
        (low - margin).max(domain_low),
        (high + margin).min(domain_high),
    )
}

/// The incidences of `edges` at their corners, the edge's stored direction
/// giving which end is which.
fn incidences<P: Payload>(
    model: &Model<P>,
    edges: &[EdgeKey],
) -> BTreeMap<VertexKey, Vec<Incidence>> {
    let mut map: BTreeMap<VertexKey, Vec<Incidence>> = BTreeMap::new();
    for &edge in edges {
        let Some((start, end)) = corners(model, edge) else {
            continue;
        };
        let curve = &model.edge_attr_unchecked(edge).curve;
        let span = span_between(
            curve,
            Some((
                model.vertex_attr_unchecked(start).point,
                model.vertex_attr_unchecked(end).point,
            )),
        );
        let (t0, t1) = (span.start.value(), span.end.value());
        let forward = (t1 - t0).signum();
        let tangent = |t: f64| curve.derivative_at(NativeParam::new(t), 1).normalize();
        map.entry(start).or_default().push(Incidence {
            edge,
            parameter: t0,
            leaving: tangent(t0) * forward,
        });
        map.entry(end).or_default().push(Incidence {
            edge,
            parameter: t1,
            leaving: -tangent(t1) * forward,
        });
    }
    map
}

/// Whether two incidences at one corner continue each other.
fn smooth(a: &Incidence, b: &Incidence) -> bool {
    a.leaving.dot(&b.leaving) <= -1.0 + TANGENT_TOLERANCE
}

/// Plans the offset of a wire: every edge moves along `T × direction`, and
/// every corner goes where its moved edges meet.
pub(crate) fn plan_wire<P: Payload>(
    model: &Model<P>,
    edges: &[EdgeKey],
    direction: UnitVector3<f64>,
    distance: f64,
) -> Result<Moves, OffsetBuildError> {
    let mut moves = Moves::default();
    for &edge in edges {
        let curve = &model.edge_attr_unchecked(edge).curve;
        let span = model
            .edge(edge)
            .ok_or(OffsetBuildError::MissingEdge { edge })?
            .parameter_interval();
        let moved = curve
            .offset_over(direction, distance, widened(curve, span))
            .map_err(|source| OffsetBuildError::EdgeSupport { edge, source })?;
        moves.curves.insert(edge, moved);
    }
    for (vertex, around) in incidences(model, edges) {
        let point = match around.as_slice() {
            [only] => moves.curves[&only.edge].point_at(NativeParam::new(only.parameter)),
            [a, b] if smooth(a, b) => moves.curves[&a.edge].point_at(NativeParam::new(a.parameter)),
            [a, b] => meet_curves(
                vertex,
                &moves.curves[&a.edge],
                a.parameter,
                &moves.curves[&b.edge],
                b.parameter,
            )?,
            _ => {
                return Err(OffsetBuildError::UndecidedVertex {
                    vertex,
                    reason: "more than two edges meet there",
                });
            }
        };
        moves.points.insert(vertex, point);
    }
    check_spans(model, edges, &moves)?;
    Ok(moves)
}

/// The point where two curves meet, refined from their parameters at a
/// corner they used to share.
fn meet_curves(
    vertex: VertexKey,
    a: &Curve,
    mut s: f64,
    b: &Curve,
    mut t: f64,
) -> Result<Point3, OffsetBuildError> {
    for _ in 0..MEET_ITERATIONS {
        let (pa, pb) = (
            a.point_at(NativeParam::new(s)),
            b.point_at(NativeParam::new(t)),
        );
        let (da, db) = (
            a.derivative_at(NativeParam::new(s), 1),
            b.derivative_at(NativeParam::new(t), 1),
        );
        let residual = pa - pb;
        // Gauss-Newton on |A(s) - B(t)|²: the normal equations of
        // [A' -B'] [ds dt]ᵀ = -(A - B).
        let (aa, ab, bb) = (da.dot(&da), -da.dot(&db), db.dot(&db));
        let rhs = Vector2::new(-da.dot(&residual), db.dot(&residual));
        let determinant = aa * bb - ab * ab;
        if determinant.abs() <= f64::EPSILON * aa * bb {
            break;
        }
        let ds = (rhs.x * bb - ab * rhs.y) / determinant;
        let dt = (aa * rhs.y - ab * rhs.x) / determinant;
        s += ds;
        t += dt;
        if ds.abs() + dt.abs() <= 1.0e-14 * (1.0 + s.abs() + t.abs()) {
            break;
        }
    }
    let (pa, pb) = (
        a.point_at(NativeParam::new(s)),
        b.point_at(NativeParam::new(t)),
    );
    let gap = (pa - pb).norm();
    if gap > MEET_TOLERANCE {
        return Err(OffsetBuildError::CornerGap { vertex, gap });
    }
    Ok(nalgebra::center(&pa, &pb))
}

/// Refuses an edge whose moved corners have crossed: the old span's middle,
/// projected onto the new support, must still lie inside the new span, and
/// the span must still run the old one's way.
fn check_spans<P: Payload>(
    model: &Model<P>,
    edges: &[EdgeKey],
    moves: &Moves,
) -> Result<(), OffsetBuildError> {
    for &edge in edges {
        let Some((start, end)) = corners(model, edge) else {
            continue;
        };
        if start == end {
            continue;
        }
        let point = |vertex| model.vertex_attr_unchecked(vertex).point;
        let old = span_between(
            &model.edge_attr_unchecked(edge).curve,
            Some((point(start), point(end))),
        );
        let curve = &moves.curves[&edge];
        let new = span_between(curve, Some((moves.points[&start], moves.points[&end])));
        // The old middle, read on the new support: a sharp edge's support
        // comes from an intersection and has its own parameterization.
        let old_middle = model
            .edge_attr_unchecked(edge)
            .curve
            .point_at(NativeParam::new(
                0.5 * (old.start.value() + old.end.value()),
            ));
        let middle = curve.parameter_at(old_middle).value();
        let (s, e) = (new.start.value(), new.end.value());
        let inside = match curve.periodicity() {
            Periodicity::Periodic(period) => (middle - s).rem_euclid(period) <= e - s,
            Periodicity::None => (middle - s) * (e - middle) >= 0.0,
        };
        let same_way =
            (e - s).signum() == (old.end.value() - old.start.value()).signum() && (e - s) != 0.0;
        if !inside || !same_way {
            return Err(OffsetBuildError::EdgeVanishes { edge });
        }
    }
    Ok(())
}

/// How one edge of a face set moves.
#[derive(Clone, Copy, PartialEq)]
enum EdgeMove {
    /// Along the normal of this face, keeping its parameterization.
    Preserved(FaceKey),
    /// Onto the meeting of these two faces' offsets.
    Sharp(FaceKey, FaceKey),
}

/// Plans the offset of a set of faces: every face moves `distance` along its
/// own normal, signed per face by `signs`, and the edges and corners between
/// them follow.
///
/// The set must be closed under incidence: every edge and corner of a face
/// in it bounds only faces in it, as a copied face, sheet or solid does.
pub(crate) fn plan_faces<P: Payload>(
    model: &Model<P>,
    faces: &[(FaceKey, f64)],
    distance: f64,
) -> Result<Moves, OffsetBuildError> {
    let sign: HashMap<FaceKey, f64> = faces.iter().copied().collect();
    let mut moves = Moves::default();

    let mut surfaces: HashMap<FaceKey, Surface> = HashMap::new();
    for &(face, face_sign) in faces {
        let attr = model.face_attr_unchecked(face);
        let (u, v) = fit_window(&attr.surface, attr.pcurves.values());
        let moved = attr
            .surface
            .offset_over(face_sign * distance, u, v)
            .map_err(|source| OffsetBuildError::FaceSupport { face, source })?;
        surfaces.insert(face, moved);
    }

    // Each edge's uses, face by face.
    let mut uses: BTreeMap<EdgeKey, Vec<(FaceKey, Dart)>> = BTreeMap::new();
    for &(face, _) in faces {
        for &dart in model.face_attr_unchecked(face).pcurves.keys() {
            if let Some(edge) = model.cell_key::<Cell1>(dart) {
                uses.entry(edge).or_default().push((face, dart));
            }
        }
    }
    let edges: Vec<EdgeKey> = uses.keys().copied().collect();

    let mut kinds: BTreeMap<EdgeKey, EdgeMove> = BTreeMap::new();
    for (&edge, edge_uses) in &uses {
        let distinct: BTreeSet<FaceKey> = edge_uses.iter().map(|&(face, _)| face).collect();
        let kind = match distinct.into_iter().collect::<Vec<_>>().as_slice() {
            [face] => EdgeMove::Preserved(*face),
            [f, g] if continues(model, edge, (*f, sign[f]), (*g, sign[g]))? => {
                EdgeMove::Preserved(*f)
            }
            [f, g] => EdgeMove::Sharp(*f, *g),
            _ => {
                return Err(OffsetBuildError::SharpEdge {
                    edge,
                    reason: "more than two faces meet along it",
                });
            }
        };
        kinds.insert(edge, kind);
    }

    for (&edge, &kind) in &kinds {
        let attr = model.edge_attr_unchecked(edge);
        let span = model
            .edge(edge)
            .ok_or(OffsetBuildError::MissingEdge { edge })?
            .parameter_interval();
        let curve = match kind {
            EdgeMove::Preserved(face) => offset_on_surface(
                &attr.curve,
                widened(&attr.curve, span),
                &model.face_attr_unchecked(face).surface,
                sign[&face] * distance,
            )
            .map_err(|source| OffsetBuildError::EdgeSupport { edge, source })?,
            EdgeMove::Sharp(f, g) => sharp_edge_curve(
                model,
                edge,
                span,
                [(f, sign[&f]), (g, sign[&g])],
                [&surfaces[&f], &surfaces[&g]],
                distance,
            )?,
        };
        moves.curves.insert(edge, curve);
    }

    let around = incidences(model, &edges);
    let mut preserved_points: BTreeSet<VertexKey> = BTreeSet::new();
    for (&vertex, incident) in &around {
        let old = model.vertex_attr_unchecked(vertex).point;
        let sharp: Vec<(FaceKey, FaceKey, EdgeKey)> = incident
            .iter()
            .filter_map(|incidence| match kinds[&incidence.edge] {
                EdgeMove::Sharp(f, g) => Some((f, g, incidence.edge)),
                EdgeMove::Preserved(_) => None,
            })
            .collect();
        let point = if sharp.is_empty() {
            preserved_points.insert(vertex);
            let EdgeMove::Preserved(face) = kinds[&incident[0].edge] else {
                unreachable!("no incident edge is sharp");
            };
            moved_along_normal(model, face, sign[&face] * distance, old)
                .map_err(|source| OffsetBuildError::FaceSupport { face, source })?
        } else {
            let mut around_faces: BTreeSet<FaceKey> = BTreeSet::new();
            for incidence in incident {
                match kinds[&incidence.edge] {
                    EdgeMove::Preserved(face) => {
                        around_faces.insert(face);
                    }
                    EdgeMove::Sharp(f, g) => {
                        around_faces.insert(f);
                        around_faces.insert(g);
                    }
                }
            }
            let moved_faces = around_faces
                .iter()
                .map(|face| {
                    (
                        &model.face_attr_unchecked(*face).surface,
                        sign[face],
                        &surfaces[face],
                    )
                })
                .collect::<Vec<_>>();
            let smooth_edge = incident
                .iter()
                .find(|incidence| matches!(kinds[&incidence.edge], EdgeMove::Preserved(_)));
            let pairs: BTreeSet<(FaceKey, FaceKey)> =
                sharp.iter().map(|&(f, g, _)| (f, g)).collect();
            if let Some(smooth) = smooth_edge {
                // An edge that moves with its faces (a smooth edge, or a free
                // boundary) carries its corner along its own moved support,
                // to where that meets the sharp edges' faces: at the end of a
                // round running into a face, the round's tangent line slides
                // to the moved face.
                let point = slide_onto_surfaces(
                    &moves.curves[&smooth.edge],
                    smooth.parameter,
                    &moved_faces,
                );
                let on_every_smooth_edge = incident.iter().all(|incidence| {
                    matches!(kinds[&incidence.edge], EdgeMove::Sharp(..)) || {
                        let curve = &moves.curves[&incidence.edge];
                        (curve.project(point) - point).norm() <= MEET_TOLERANCE
                    }
                });
                if !on_every_smooth_edge || !on_every_surface(point, &moved_faces) {
                    return Err(OffsetBuildError::UndecidedVertex {
                        vertex,
                        reason: "the edges that move with their faces (free boundaries, or \
                                 smooth edges) and the sharp edges' faces do not meet at one \
                                 point there",
                    });
                }
                point
            } else if pairs.len() == 1 && around_faces.len() == 2 {
                // Two faces meet at the corner, along one sharp edge (a rim
                // with its corner on it): the corner stays on that edge's new
                // support, nearest where it was.
                let (_, _, edge) = sharp[0];
                let curve = &moves.curves[&edge];
                curve.project(old)
            } else {
                meet_surfaces(vertex, old, &moved_faces, distance)?
            }
        };
        moves.points.insert(vertex, point);
    }

    check_spans(model, &edges, &moves)?;

    for (&edge, edge_uses) in &uses {
        let kept = match kinds[&edge] {
            EdgeMove::Preserved(_) => match corners(model, edge) {
                Some((start, end)) => {
                    preserved_points.contains(&start) && preserved_points.contains(&end)
                }
                None => true,
            },
            EdgeMove::Sharp(..) => false,
        };
        if kept {
            continue;
        }
        let curve = &moves.curves[&edge];
        let span = span_between(
            curve,
            corners(model, edge).map(|(start, end)| (moves.points[&start], moves.points[&end])),
        );
        for &(face, dart) in edge_uses {
            let directed = match model.edge_orientation_at_dart(edge, dart) {
                Orientation::Same => span,
                Orientation::Reversed => span.reversed(),
            };
            let section = TrimmedCurve::new(curve.clone(), directed);
            let old = &model.face_attr_unchecked(face).pcurves[&dart];
            let pcurve = rewritten_pcurve(&surfaces[&face], &section, old, corners(model, edge))
                .map_err(|reason| OffsetBuildError::Pcurve { face, edge, reason })?;
            moves.pcurves.push((face, dart, pcurve));
        }
    }

    moves.surfaces = surfaces.into_iter().collect();
    Ok(moves)
}

/// The parameter window a fitted face offset spans: the box of its pcurves,
/// widened by [`FACE_MARGIN`], or the surface's own domain for a face with
/// none.
fn fit_window<'a>(
    surface: &Surface,
    pcurves: impl Iterator<Item = &'a TrimmedCurve2>,
) -> (Interval, Interval) {
    let mut low = Point2::new(f64::INFINITY, f64::INFINITY);
    let mut high = Point2::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
    for pcurve in pcurves {
        for index in 0..=16 {
            let point = pcurve.point_at(Fraction::new(index as f64 / 16.0));
            low = Point2::new(low.x.min(point.x), low.y.min(point.y));
            high = Point2::new(high.x.max(point.x), high.y.max(point.y));
        }
    }
    let (u, v) = surface.domain();
    if !low.x.is_finite() {
        return (u, v);
    }
    let clip = |domain: Interval, low: f64, high: f64| {
        let margin = FACE_MARGIN * (high - low).max(LINEAR_TOLERANCE.sqrt());
        let (start, end) = (domain.start.value(), domain.end.value());
        Interval::new((low - margin).max(start), (high + margin).min(end))
    };
    (clip(u, low.x, high.x), clip(v, low.y, high.y))
}

/// Whether two faces continue each other along `edge`: their outward normals
/// agree all along it.
fn continues<P: Payload>(
    model: &Model<P>,
    edge: EdgeKey,
    (f, f_sign): (FaceKey, f64),
    (g, g_sign): (FaceKey, f64),
) -> Result<bool, OffsetBuildError> {
    let curve = model.edge_attr_unchecked(edge).curve.clone();
    let span = model
        .edge(edge)
        .ok_or(OffsetBuildError::MissingEdge { edge })?
        .parameter_interval();
    let (sf, sg) = (
        &model.face_attr_unchecked(f).surface,
        &model.face_attr_unchecked(g).surface,
    );
    for index in 0..TANGENCY_PROBES {
        let fraction = (index as f64 + 0.5) / TANGENCY_PROBES as f64;
        let t = span.start.value() + (span.end.value() - span.start.value()) * fraction;
        let point = curve.point_at(NativeParam::new(t));
        let normal = |surface: &Surface, face: FaceKey| -> Result<Vector3<f64>, OffsetBuildError> {
            let uv = surface
                .param_at(point)
                .map_err(|error| OffsetBuildError::Pcurve {
                    face,
                    edge,
                    reason: error.to_string(),
                })?;
            Ok(surface.normal_at(uv.x, uv.y).into_inner())
        };
        let (nf, ng) = (normal(sf, f)? * f_sign, normal(sg, g)? * g_sign);
        if (nf - ng).norm() > TANGENT_TOLERANCE {
            return Ok(false);
        }
    }
    Ok(true)
}

/// `point`, which lies on `face`'s surface, moved `distance` along it.
fn moved_along_normal<P: Payload>(
    model: &Model<P>,
    face: FaceKey,
    distance: f64,
    point: Point3,
) -> Result<Point3, crate::geometry::OffsetError> {
    let surface = &model.face_attr_unchecked(face).surface;
    let uv = surface.param_at(point)?;
    Ok(point + *surface.normal_at(uv.x, uv.y) * distance)
}

/// The point where every listed face's offset passes, refined from where
/// their tangent planes at the old corner, moved, meet.
///
/// Each entry is a face's old surface, its sign and its moved surface. Three
/// or more faces fix a point; where they do not all pass through one, the
/// corner is refused.
fn meet_surfaces(
    vertex: VertexKey,
    old: Point3,
    faces: &[(&Surface, f64, &Surface)],
    distance: f64,
) -> Result<Point3, OffsetBuildError> {
    let undecided = |reason| OffsetBuildError::UndecidedVertex { vertex, reason };
    let mut planes = Vec::with_capacity(faces.len());
    for &(surface, sign, _) in faces {
        let uv = surface
            .param_at(old)
            .map_err(|_| undecided("a face's surface cannot be inverted there"))?;
        let normal = surface.normal_at(uv.x, uv.y).into_inner();
        planes.push((old + normal * (sign * distance), normal));
    }
    let mut point = least_squares_point(&planes).ok_or(undecided(
        "the faces around it do not fix a point: their normals are coplanar",
    ))?;
    for _ in 0..MEET_ITERATIONS {
        planes.clear();
        for &(_, _, moved) in faces {
            let uv = moved
                .param_at(point)
                .map_err(|_| undecided("a moved face's surface cannot be inverted there"))?;
            planes.push((
                moved.point_at(uv.x, uv.y),
                moved.normal_at(uv.x, uv.y).into_inner(),
            ));
        }
        let next = least_squares_point(&planes)
            .ok_or(undecided("the moved faces around it do not fix a point"))?;
        let step = (next - point).norm();
        point = next;
        if step <= 1.0e-14 * (1.0 + point.coords.norm()) {
            break;
        }
    }
    for &(_, _, moved) in faces {
        let uv = moved
            .param_at(point)
            .map_err(|_| undecided("a moved face's surface cannot be inverted there"))?;
        if (moved.point_at(uv.x, uv.y) - point).norm() > MEET_TOLERANCE {
            return Err(undecided(
                "the moved faces around it do not pass through one point",
            ));
        }
    }
    Ok(point)
}

/// The point of `curve` nearest every listed face's moved surface, refined
/// from `parameter` by Gauss-Newton on the distances to their tangent planes.
fn slide_onto_surfaces(
    curve: &Curve,
    mut parameter: f64,
    faces: &[(&Surface, f64, &Surface)],
) -> Point3 {
    for _ in 0..MEET_ITERATIONS {
        let point = curve.point_at(NativeParam::new(parameter));
        let tangent = curve.derivative_at(NativeParam::new(parameter), 1);
        let (mut numerator, mut denominator) = (0.0, 0.0);
        for &(_, _, moved) in faces {
            let Ok(uv) = moved.param_at(point) else {
                continue;
            };
            let normal = moved.normal_at(uv.x, uv.y);
            let residual = normal.dot(&(point - moved.point_at(uv.x, uv.y)));
            let slope = normal.dot(&tangent);
            numerator += residual * slope;
            denominator += slope * slope;
        }
        if denominator <= f64::EPSILON {
            break;
        }
        let step = numerator / denominator;
        parameter -= step;
        if step.abs() <= 1.0e-14 * (1.0 + parameter.abs()) {
            break;
        }
    }
    curve.point_at(NativeParam::new(parameter))
}

/// Whether `point` lies on every listed face's moved surface.
fn on_every_surface(point: Point3, faces: &[(&Surface, f64, &Surface)]) -> bool {
    faces.iter().all(|&(_, _, moved)| {
        moved
            .param_at(point)
            .is_ok_and(|uv| (moved.point_at(uv.x, uv.y) - point).norm() <= MEET_TOLERANCE)
    })
}

/// The point nearest every plane `(point, normal)` in the least-squares
/// sense, or `None` when the normals do not span space.
fn least_squares_point(planes: &[(Point3, Vector3<f64>)]) -> Option<Point3> {
    let mut matrix = Matrix3::zeros();
    let mut rhs = Vector3::zeros();
    for (point, normal) in planes {
        matrix += normal * normal.transpose();
        rhs += normal * normal.dot(&point.coords);
    }
    let (min, max) = matrix
        .symmetric_eigenvalues()
        .iter()
        .fold((f64::INFINITY, 0.0_f64), |(lo, hi), &value| {
            (lo.min(value), hi.max(value))
        });
    if min <= 1.0e-6 * max {
        return None;
    }
    matrix.lu().solve(&rhs).map(Point3::from)
}

/// The support of a sharp edge between two moved faces: the branch of their
/// intersection nearest where the edge's middle goes, running the old edge's
/// way.
fn sharp_edge_curve<P: Payload>(
    model: &Model<P>,
    edge: EdgeKey,
    span: Interval,
    signed: [(FaceKey, f64); 2],
    moved: [&Surface; 2],
    distance: f64,
) -> Result<Curve, OffsetBuildError> {
    let old_curve = &model.edge_attr_unchecked(edge).curve;
    let middle = NativeParam::new(0.5 * (span.start.value() + span.end.value()));
    let point = old_curve.point_at(middle);
    let tangent = old_curve.derivative_at(middle, 1).normalize();

    // Where the middle goes: on both moved tangent planes, in the plane
    // normal to the edge through its old middle.
    let mut planes = vec![(point, tangent)];
    for (face, sign) in signed {
        let surface = &model.face_attr_unchecked(face).surface;
        let uv = surface
            .param_at(point)
            .map_err(|error| OffsetBuildError::Pcurve {
                face,
                edge,
                reason: error.to_string(),
            })?;
        let normal = surface.normal_at(uv.x, uv.y).into_inner();
        planes.push((point + normal * (sign * distance), normal));
    }
    let target = least_squares_point(&planes).ok_or(OffsetBuildError::SharpEdge {
        edge,
        reason: "its faces' normals are parallel",
    })?;

    let supports: Vec<Curve> =
        match intersect_analytic_surfaces(moved[0], moved[1], IntersectionOptions::default()) {
            Some(Ok(analytic)) => analytic
                .sections()
                .iter()
                .map(|section| section.curve.curve().clone())
                .collect(),
            _ => moved[0]
                .intersect_surface(moved[1])
                .map_err(|error| OffsetBuildError::SharpEdgeIntersection {
                    edge,
                    reason: error.to_string(),
                })?
                .as_slice()
                .iter()
                .filter_map(|found| match found {
                    SurfaceSurfaceIntersection::Branch(branch) => {
                        Some(branch.curve_3d.curve().clone())
                    }
                    _ => None,
                })
                .collect(),
        };
    let branch = supports
        .into_iter()
        .map(|curve| {
            let near = curve.project(target);
            ((near - target).norm(), curve)
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .ok_or(OffsetBuildError::SharpEdge {
            edge,
            reason: "their offsets do not intersect",
        })?;
    let (gap, mut curve) = branch;
    let scale = 1.0 + distance.abs() + (point - Point3::origin()).norm();
    if gap > MEET_TOLERANCE * scale.max(1.0) * 1.0e3 {
        return Err(OffsetBuildError::SharpEdge {
            edge,
            reason: "their offsets do not intersect near the edge",
        });
    }
    let at = curve.parameter_at(target);
    if curve.derivative_at(at, 1).dot(&tangent) < 0.0 {
        curve = curve.reversed();
    }
    // A circle keeps the old one's phase, so the corner where an unmarked
    // rim closes stays where it was relative to the rest of the face.
    if let (Curve::Circle(old), Curve::Circle(new)) = (old_curve, &curve) {
        let axis = old.plane().normal();
        let offset = new.plane().origin() - old.plane().origin();
        let coaxial = (offset - *axis * offset.dot(&axis)).norm() <= LINEAR_TOLERANCE;
        if coaxial && new.plane().normal().dot(&axis) > 0.0 {
            let frame = old
                .plane()
                .frame
                .moved(&crate::geometry::Rigid::translation(offset));
            curve = Curve::Circle(Circle::new(Plane::from_frame(frame), new.radius()));
        }
    }
    Ok(curve)
}

/// The pcurve of a moved edge on a moved face, on the branch the old pcurve
/// was written on.
fn rewritten_pcurve(
    surface: &Surface,
    section: &TrimmedCurve,
    old: &TrimmedCurve2,
    corners: Option<(VertexKey, VertexKey)>,
) -> Result<TrimmedCurve2, String> {
    let closed = match corners {
        None => true,
        Some((start, end)) => start == end,
    };
    let pcurve = if closed {
        closed_pcurve_on(surface, section, Some(old.point_at(Fraction::new(0.0))))
            .map_err(|error| error.to_string())?
    } else {
        pcurve_on(surface, section).map_err(|error| error.to_string())?
    };
    // Shift the whole pcurve onto the branch the old one lived on, so the
    // face's loop stays one closed walk in parameter space.
    let middle = pcurve.point_at(Fraction::new(0.5));
    let shifted = onto_branch(surface, middle, old.point_at(Fraction::new(0.5)));
    let shift = shifted - middle;
    if shift.norm() <= LINEAR_TOLERANCE {
        return Ok(pcurve);
    }
    pcurve.translated(shift).map_err(|error| error.to_string())
}

/// Every face's sign: `1` where its own normal is its surface's, `-1` where
/// it faces the other way.
pub(crate) fn face_sign<P: Payload>(model: &Model<P>, face: FaceKey) -> f64 {
    let view = Face::new(model, face);
    let attr = model.face_attr_unchecked(face);
    let uv = attr
        .pcurves
        .values()
        .next()
        .map(|pcurve| pcurve.point_at(Fraction::new(0.5)))
        .unwrap_or_else(|| {
            let (u, v) = attr.surface.domain();
            let (u, v) = (u.or_extent(1.0), v.or_extent(1.0));
            Point2::new(
                0.5 * (u.start.value() + u.end.value()),
                0.5 * (v.start.value() + v.end.value()),
            )
        });
    let face_normal = view.normal_at(uv.x, uv.y);
    let surface_normal = attr.surface.normal_at(uv.x, uv.y);
    if face_normal.dot(&surface_normal) < 0.0 {
        -1.0
    } else {
        1.0
    }
}
