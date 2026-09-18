use crate::geometry::TrimmedCurve2;
use crate::geometry::parameter::NativeParam;
use std::collections::HashMap;

use nalgebra::Vector3;

use crate::geometry::{
    Circle2, ControlPolygon2, Curve, Curve2, Ellipse2, HPoint2, LINEAR_TOLERANCE, Line2,
    NurbsCurve2, NurbsError, Plane, Point2, Point3, PointCoincidence, TrimmedCurve, Vector2,
};
use crate::model::{Cell0, Model};
use crate::topology::ModelEdit;
use crate::topology::attributes::{EdgeAttr, ProfileAttr, VertexAttr};
use crate::topology::closed::Closeable;
use crate::topology::edit::ModelEditError;
use crate::topology::gmap::{Dart, Dim};
use crate::topology::payload::Payload;
use crate::topology::profile::Profile;
use crate::topology::shape_keys::{EdgeKey, ProfileKey, VertexKey};

pub use crate::builders::errors::PolylineError;

/// Adds a profile made of straight segments through `points` in order.
///
/// Consecutive points define the profile edges. When the last point coincides
/// with the first within [`LINEAR_TOLERANCE`], the final edge closes the profile;
/// otherwise both ends remain open.
///
/// At least two points are required.
pub fn add_polyline<P: Payload>(
    g: &mut Model<P>,
    points: &[Point3],
) -> Result<ProfileKey, PolylineError> {
    g.transaction(|edit| add_polyline_staged(edit, points))
}

/// Adds one profile from existing edges, regardless of their supplied order.
///
/// The edges must form exactly one non-branching open chain or closed cycle.
/// Coincident endpoints are joined within [`LINEAR_TOLERANCE`], and their
/// logical vertices are merged as the profile is sewn. Disconnected input and
/// branches are rejected without changing the model.
pub fn add_profile_from_edges<P: Payload>(
    g: &mut Model<P>,
    edges: &[EdgeKey],
) -> Result<ProfileKey, PolylineError> {
    g.transaction(|edit| add_profile_from_edges_staged(edit, edges))
}

/// Orders and joins existing edges inside the caller's transaction.
pub fn add_profile_from_edges_staged<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    edges: &[EdgeKey],
) -> Result<ProfileKey, PolylineError> {
    let mut edges = edges
        .iter()
        .copied()
        .map(|key| profile_edge_endpoints(edit, key))
        .collect::<Result<Vec<_>, _>>()?;
    let first = edges.first().ok_or(PolylineError::EmptyPolyline)?;

    if edges.len() == 1 {
        return Ok(edit.add_profile(ProfileAttr::new(first.start, P::Profile::default())));
    }

    let endpoint_degrees = edges
        .iter()
        .flat_map(|edge| [edge.start_point, edge.end_point])
        .map(|point| {
            let degree = edges
                .iter()
                .flat_map(|edge| [edge.start_point, edge.end_point])
                .filter(|candidate| candidate.coincides(point, LINEAR_TOLERANCE))
                .count();
            (point, degree)
        })
        .collect::<Vec<_>>();
    if let Some((point, _)) = endpoint_degrees.iter().find(|(_, degree)| *degree > 2) {
        return Err(PolylineError::NonManifoldEdgeConnection { point: *point });
    }
    if endpoint_degrees.iter().any(|(_, degree)| *degree == 1) {
        if endpoint_degrees
            .iter()
            .filter(|(_, degree)| *degree == 1)
            .count()
            != 2
        {
            return Err(PolylineError::DisconnectedEdges);
        }
    } else if endpoint_degrees.iter().any(|(_, degree)| *degree != 2) {
        return Err(PolylineError::DisconnectedEdges);
    }

    let start = endpoint_degrees
        .iter()
        .find(|(_, degree)| *degree == 1)
        .map(|(point, _)| *point)
        .unwrap_or(first.start_point);
    let mut ordered = Vec::with_capacity(edges.len());
    let mut current = start;

    while !edges.is_empty() {
        let Some(index) = edges.iter().position(|edge| edge.touches(current)) else {
            return Err(PolylineError::DisconnectedEdges);
        };
        let edge = edges.swap_remove(index);
        current = edge.other_end(current);
        ordered.push(edge);
    }

    let profile = edit.add_profile(ProfileAttr::new(
        ordered[0].dart_at(start),
        P::Profile::default(),
    ));
    for edge in ordered.iter().skip(1) {
        append_edge_staged(edit, profile, edge.key)?;
    }
    Ok(profile)
}

/// Builds all polyline edges and joins them into one staged profile.
pub fn add_polyline_staged<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    points: &[Point3],
) -> Result<ProfileKey, PolylineError> {
    if points.len() < 2 {
        return Err(PolylineError::EmptyPolyline);
    }

    let segments = points
        .windows(2)
        .map(|pair| (pair[0], pair[1], Curve::line(pair[0], pair[1])))
        .collect::<Vec<_>>();
    add_segments(edit, &segments)
}

/// Appends an existing edge to the open end of a profile.
///
/// The edge orientation is chosen from endpoint geometry: either stored edge
/// direction may be appended as long as one endpoint coincides with the profile
/// end. If the appended edge's other endpoint coincides with the profile start,
/// the profile is closed.
pub fn append_edge<P: Payload>(
    g: &mut Model<P>,
    profile_key: ProfileKey,
    edge_key: EdgeKey,
) -> Result<(), PolylineError> {
    g.transaction(|edit| append_edge_staged(edit, profile_key, edge_key))
}

/// Connects an edge to a profile and records any resulting vertex merge lineage.
pub(crate) fn append_edge_staged<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    profile_key: ProfileKey,
    edge_key: EdgeKey,
) -> Result<(), PolylineError> {
    let profile = edit
        .profile(profile_key)
        .ok_or(PolylineError::MissingProfile {
            profile: profile_key,
        })?;
    if profile.is_closed() {
        return Err(PolylineError::ClosedProfile { dart: profile.dart });
    }

    let profile_start = profile.dart;
    let profile_end = profile
        .darts()
        .last()
        .expect("non-empty profile should have an end dart");
    let profile_start_point = vertex_point(edit, profile_start)?;
    let profile_end_point = vertex_point(edit, profile_end)?;
    let default_edge_start = edit
        .edge_attr(edge_key)
        .ok_or(PolylineError::MissingEdge { edge: edge_key })?
        .dart;
    let default_edge_end = edit.alpha(Dim::Zero, default_edge_start);
    let default_edge_start_point = vertex_point(edit, default_edge_start)?;
    let default_edge_end_point = vertex_point(edit, default_edge_end)?;

    let Some((edge_dart, edge_end, edge_end_point)) = append_orientation(
        profile_end_point,
        default_edge_start,
        default_edge_start_point,
        default_edge_end,
        default_edge_end_point,
    ) else {
        return Err(PolylineError::NonContiguousEdge {
            profile_end: profile_end_point,
            edge_start: default_edge_start_point,
        });
    };

    let append_merge = VertexMerge {
        survivor: vertex_key(edit, profile_end)?,
        removed: vertex_key(edit, edge_dart)?,
    };
    let close_merge = edge_end_point
        .coincides(profile_start_point, LINEAR_TOLERANCE)
        .then(|| {
            Ok::<_, PolylineError>(VertexMerge {
                survivor: vertex_key(edit, profile_start)?,
                removed: vertex_key(edit, edge_end)?,
            })
        })
        .transpose()?;

    edit.sew(Dim::One, profile_end, edge_dart)
        .map_err(polyline_edit_error)?;
    edit.merge_vertices_into(append_merge.survivor, append_merge.removed);
    if let Some(close_merge) = close_merge {
        edit.sew(Dim::One, edge_end, profile_start)
            .map_err(polyline_edit_error)?;
        edit.merge_vertices_into(close_merge.survivor, close_merge.removed);
    }
    Ok(())
}

fn append_orientation(
    profile_end_point: Point3,
    edge_start: Dart,
    edge_start_point: Point3,
    edge_end: Dart,
    edge_end_point: Point3,
) -> Option<(Dart, Dart, Point3)> {
    if profile_end_point.coincides(edge_start_point, LINEAR_TOLERANCE) {
        Some((edge_start, edge_end, edge_end_point))
    } else if profile_end_point.coincides(edge_end_point, LINEAR_TOLERANCE) {
        Some((edge_end, edge_start, edge_start_point))
    } else {
        None
    }
}

/// Builds the face-parameter curves for every edge of `profile` on `plane`.
///
/// Each result is keyed by the oriented profile-edge dart and carries the
/// edge's own trimmed span, so the parameter curve covers the edge rather than
/// its whole support. The profile is required to lie on `plane`; see
/// [`curve_pcurve`].
pub fn profile_pcurves<P: Payload>(
    profile: &Profile<'_, P>,
    plane: &Plane,
) -> Result<HashMap<Dart, TrimmedCurve2>, PolylineError> {
    let edges = profile.edges();
    let mut pcurves = HashMap::with_capacity(edges.len());

    for edge in edges.iter() {
        let dart = edge.dart();
        let section = edge
            .trimmed_curve()
            .ok_or(PolylineError::MissingVertexPoint { dart })?;
        pcurves.insert(dart, curve_pcurve(&section, plane)?);
    }

    Ok(pcurves)
}

/// Projects one oriented 3D section into a plane's parameter space.
///
/// The section is required to lie on `plane`. That is what lets the span carry
/// over untouched: a plane's parameters are Cartesian coordinates in it, so
/// projection moves the support's representation and leaves its
/// parameterization where it was. A circle stays a [`Circle2`], an ellipse an
/// [`Ellipse2`], a line a [`Line2`], and a NURBS keeps its degree, weights and
/// knots. Nothing is degraded to NURBS to be projected.
pub(crate) fn curve_pcurve(
    section: &TrimmedCurve,
    plane: &Plane,
) -> Result<TrimmedCurve2, NurbsError> {
    let point = |point: Point3| plane_uv(plane, point);
    let direction =
        |vector: Vector3<f64>| Vector2::new(vector.dot(&plane.x_dir()), vector.dot(&plane.y_dir()));

    let support = match section.curve() {
        Curve::Line(line) => Curve2::Line(Line2::new(
            point(line.point_at(NativeParam::new(0.0))),
            direction(line.point_at(NativeParam::new(1.0)) - line.point_at(NativeParam::new(0.0))),
        )),
        Curve::Circle(circle) => {
            let x = direction(circle.plane().x_dir().into_inner());
            let y = direction(circle.plane().y_dir().into_inner());
            let flat = Circle2::new(point(circle.plane().origin()), x, circle.radius());
            Curve2::Circle(oriented(flat, x, y, Circle2::reversed))
        }
        Curve::Ellipse(ellipse) => {
            let frame = ellipse.frame();
            let x = direction(frame.x_dir.into_inner());
            let y = direction(frame.y_dir.into_inner());
            let flat = Ellipse2::new(
                point(frame.origin),
                x,
                ellipse.major_radius(),
                ellipse.minor_radius(),
            );
            Curve2::Ellipse(oriented(flat, x, y, Ellipse2::reversed))
        }
        // Projecting the homogeneous control polygon is exact because the
        // projection is affine, and it leaves the degree, the weights and the
        // knots alone — so the curve keeps its own parameter as well as its
        // point set.
        Curve::Nurbs(nurbs) => {
            let control_points = ControlPolygon2::new(
                nurbs
                    .control_points()
                    .iter()
                    .map(|control| {
                        HPoint2::from_cartesian(point(control.to_cartesian()), control.weight())
                    })
                    .collect(),
            )?;
            Curve2::Nurbs(NurbsCurve2::new(
                nurbs.degree(),
                control_points,
                nurbs.knots().clone(),
            )?)
        }
    };

    Ok(TrimmedCurve2::new(support, section.interval()))
}

/// Restores a conic's sense when the plane it lay in faces the other way.
///
/// A conic built from a centre, a start direction and a radius turns
/// counter-clockwise by construction, so its quarter-turn direction is
/// `perp(x)`. A circle whose own plane is the face's plane seen from behind
/// projects with the opposite one, and saying so is what keeps its angular
/// parameter — and so every span written on it — running the way it did in
/// space.
fn oriented<T>(conic: T, x: Vector2, y: Vector2, reverse: impl Fn(&T) -> T) -> T {
    let turns_counter_clockwise = x.x * y.y - x.y * y.x;
    if turns_counter_clockwise < 0.0 {
        reverse(&conic)
    } else {
        conic
    }
}

/// Returns the local `(u, v)` coordinates of `point` in `plane`.
///
/// The coordinates are the projections of `point - plane.origin()` onto the
/// plane's x and y directions.
pub fn plane_uv(plane: &Plane, point: Point3) -> Point2 {
    let v = point - plane.origin();
    Point2::new(v.dot(&plane.x_dir()), v.dot(&plane.y_dir()))
}

/// Adds a rectangular profile to the given model.
///
/// The corners are built on `plane` in the following order:
/// 0-----1
/// |     |
/// |     |
/// 3-----2
///
/// Returns the profile key whose stored dart starts at the first corner.
pub fn add_rectangle<P: Payload>(
    g: &mut Model<P>,
    plane: Plane,
    x_size: f64,
    y_size: f64,
) -> Result<ProfileKey, PolylineError> {
    g.transaction(|edit| add_rectangle_staged(edit, plane, x_size, y_size))
}

/// Builds the four rectangle edges and profile inside one transaction.
pub(crate) fn add_rectangle_staged<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    plane: Plane,
    x_size: f64,
    y_size: f64,
) -> Result<ProfileKey, PolylineError> {
    validate_rectangle_size("x", x_size)?;
    validate_rectangle_size("y", y_size)?;

    let corners = [
        plane.point_at(0.0, 0.0),
        plane.point_at(x_size, 0.0),
        plane.point_at(x_size, y_size),
        plane.point_at(0.0, y_size),
        plane.point_at(0.0, 0.0),
    ];
    add_polyline_staged(edit, &corners)
}

/// Adds a closed square profile on `plane`.
///
/// The first corner is the plane origin and the sides follow its positive x and
/// y directions. `size` must be positive and finite.
pub fn add_square<P: Payload>(
    g: &mut Model<P>,
    plane: Plane,
    size: f64,
) -> Result<ProfileKey, PolylineError> {
    g.transaction(|edit| add_rectangle_staged(edit, plane, size, size))
}

fn validate_rectangle_size(axis: &'static str, value: f64) -> Result<(), PolylineError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(PolylineError::InvalidRectangleSize { axis, value })
    }
}

/// Adds the given number of darts and sews them together in a profile, the profile is closed if the given closed is true.
pub fn add_profile_darts<P: Payload>(g: &mut Model<P>, count: usize, closed: bool) -> ProfileKey {
    g.transaction(|edit| {
        let darts: Vec<Dart> = (0..count).map(|_| edit.add_dart()).collect();
        for i in 0..count {
            edit.sew(Dim::Zero, darts[i], darts[(i + 1) % count])?;
        }
        for i in 0..count {
            edit.sew(Dim::One, darts[i], darts[(i + 1) % count])?;
        }
        if closed {
            edit.sew(Dim::Zero, darts[count - 1], darts[0])?;
        }
        Ok::<_, ModelEditError>(edit.add_profile(ProfileAttr::new(darts[0], P::Profile::default())))
    })
    .expect("fresh profile topology must commit")
}

fn add_segments<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    segments: &[(Point3, Point3, Curve)],
) -> Result<ProfileKey, PolylineError> {
    let first_segment = segments.first().ok_or(PolylineError::EmptyPolyline)?;
    let last_segment = segments.last().ok_or(PolylineError::EmptyPolyline)?;
    let closed = first_segment.0.coincides(&last_segment.1, LINEAR_TOLERANCE);

    let mut segment_topology = Vec::with_capacity(segments.len());
    for (start_point, _, curve) in segments {
        let start_dart = edit.add_dart();
        let end_dart = edit.add_dart();
        edit.link(Dim::Zero, start_dart, end_dart)
            .map_err(polyline_edit_error)?;
        edit.add_vertex(VertexAttr::new(start_dart, *start_point, P::V::default()));
        edit.add_edge(EdgeAttr::new(start_dart, curve.clone(), P::E::default()));
        segment_topology.push(SegmentTopology {
            start: start_dart,
            end: end_dart,
        });
    }

    for pair in segment_topology.windows(2) {
        edit.sew(Dim::One, pair[0].end, pair[1].start)
            .map_err(polyline_edit_error)?;
    }

    if closed {
        let first = segment_topology
            .first()
            .expect("non-empty segment list should have a first segment");
        let last = segment_topology
            .last()
            .expect("non-empty segment list should have a last segment");
        edit.sew(Dim::One, last.end, first.start)
            .map_err(polyline_edit_error)?;
    } else {
        let last_segment = segments
            .last()
            .expect("non-empty segment list should have a last segment");
        let last_topology = segment_topology
            .last()
            .expect("non-empty segment list should have a last segment");
        edit.add_vertex(VertexAttr::new(
            last_topology.end,
            last_segment.1,
            P::V::default(),
        ));
    }

    let first_start = segment_topology[0].start;
    Ok(edit.add_profile(ProfileAttr::new(first_start, P::Profile::default())))
}

#[derive(Clone, Copy)]
struct SegmentTopology {
    start: Dart,
    end: Dart,
}

#[derive(Clone, Copy)]
struct ProfileEdgeEndpoints {
    key: EdgeKey,
    start: Dart,
    end: Dart,
    start_point: Point3,
    end_point: Point3,
}

impl ProfileEdgeEndpoints {
    fn touches(self, point: Point3) -> bool {
        self.start_point.coincides(point, LINEAR_TOLERANCE)
            || self.end_point.coincides(point, LINEAR_TOLERANCE)
    }

    fn other_end(self, point: Point3) -> Point3 {
        if self.start_point.coincides(point, LINEAR_TOLERANCE) {
            self.end_point
        } else {
            self.start_point
        }
    }

    fn dart_at(self, point: Point3) -> Dart {
        if self.start_point.coincides(point, LINEAR_TOLERANCE) {
            self.start
        } else {
            self.end
        }
    }
}

#[derive(Clone, Copy)]
struct VertexMerge {
    survivor: VertexKey,
    removed: VertexKey,
}

fn polyline_edit_error(error: ModelEditError) -> PolylineError {
    error.into()
}

fn vertex_point<P: Payload>(g: &Model<P>, dart: Dart) -> Result<Point3, PolylineError> {
    g.attribute::<Cell0>(dart)
        .map(|attr| attr.point)
        .ok_or(PolylineError::MissingVertexPoint { dart })
}

fn vertex_key<P: Payload>(g: &Model<P>, dart: Dart) -> Result<VertexKey, PolylineError> {
    g.cell_key::<Cell0>(dart)
        .ok_or(PolylineError::MissingVertexPoint { dart })
}

fn profile_edge_endpoints<P: Payload>(
    g: &Model<P>,
    key: EdgeKey,
) -> Result<ProfileEdgeEndpoints, PolylineError> {
    let start = g
        .edge_attr(key)
        .ok_or(PolylineError::MissingEdge { edge: key })?
        .dart;
    let end = g.alpha(Dim::Zero, start);
    Ok(ProfileEdgeEndpoints {
        key,
        start,
        end,
        start_point: vertex_point(g, start)?,
        end_point: vertex_point(g, end)?,
    })
}
