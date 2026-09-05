use std::collections::HashMap;

use crate::geometry::{
    ControlPolygon2, Curve, Curve2, HPoint2, LINEAR_TOLERANCE, Line2, NurbsCurve2, NurbsError,
    Plane, Point2, Point3, PointCoincidence,
};
use crate::topology::TopologyEdit;
use crate::topology::attributes::{EdgeAttr, ProfileAttr, VertexAttr};
use crate::topology::closed::Closeable;
use crate::topology::gmap::{Cell0, Dart, Dim, GMap, TopologyEditError};
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
    g: &mut GMap<P>,
    points: &[Point3],
) -> Result<ProfileKey, PolylineError> {
    g.transaction(|g| add_polyline_staged(g, points))
}

/// Builds all polyline edges and joins them into one staged profile.
pub fn add_polyline_staged<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    points: &[Point3],
) -> Result<ProfileKey, PolylineError> {
    if points.len() < 2 {
        return Err(PolylineError::EmptyPolyline);
    }

    let segments = points
        .windows(2)
        .map(|pair| (pair[0], pair[1], Curve::line(pair[0], pair[1])))
        .collect::<Vec<_>>();
    add_segments(g, &segments)
}

/// Appends an existing edge to the open end of a profile.
///
/// The edge orientation is chosen from endpoint geometry: either stored edge
/// direction may be appended as long as one endpoint coincides with the profile
/// end. If the appended edge's other endpoint coincides with the profile start,
/// the profile is closed.
pub fn append_edge<P: Payload>(
    g: &mut GMap<P>,
    profile_key: ProfileKey,
    edge_key: EdgeKey,
) -> Result<(), PolylineError> {
    g.transaction(|g| append_edge_staged(g, profile_key, edge_key))
}

/// Connects an edge to a profile and records any resulting vertex merge lineage.
pub(crate) fn append_edge_staged<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    profile_key: ProfileKey,
    edge_key: EdgeKey,
) -> Result<(), PolylineError> {
    let profile = g
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
    let profile_start_point = vertex_point(g, profile_start)?;
    let profile_end_point = vertex_point(g, profile_end)?;
    let default_edge_start = g
        .edge_attr(edge_key)
        .ok_or(PolylineError::MissingEdge { edge: edge_key })?
        .dart;
    let default_edge_end = g.alpha(Dim::Zero, default_edge_start);
    let default_edge_start_point = vertex_point(g, default_edge_start)?;
    let default_edge_end_point = vertex_point(g, default_edge_end)?;

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
        survivor: vertex_key(g, profile_end)?,
        removed: vertex_key(g, edge_dart)?,
    };
    let close_merge = edge_end_point
        .coincides(profile_start_point, LINEAR_TOLERANCE)
        .then(|| {
            Ok::<_, PolylineError>(VertexMerge {
                survivor: vertex_key(g, profile_start)?,
                removed: vertex_key(g, edge_end)?,
            })
        })
        .transpose()?;

    g.sew(Dim::One, profile_end, edge_dart)
        .map_err(polyline_edit_error)?;
    g.merge_vertices_into(append_merge.survivor, append_merge.removed);
    if let Some(close_merge) = close_merge {
        g.sew(Dim::One, edge_end, profile_start)
            .map_err(polyline_edit_error)?;
        g.merge_vertices_into(close_merge.survivor, close_merge.removed);
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
/// Each result is keyed by the oriented profile-edge dart. Straight edges
/// become UV-space lines; other supported curves are converted to NURBS and
/// their control points are projected into the plane's local coordinates.
/// This function does not require the source geometry to lie on `plane`.
pub fn profile_pcurves<P: Payload>(
    profile: &Profile<'_, P>,
    plane: &Plane,
) -> Result<HashMap<Dart, Curve2>, PolylineError> {
    let edges = profile.edges();
    let mut pcurves = HashMap::with_capacity(edges.len());

    for edge in edges.iter() {
        let dart = edge.dart();
        let start = *edge
            .start()
            .point()
            .ok_or(PolylineError::MissingVertexPoint { dart })?;
        let end = *edge
            .end()
            .point()
            .ok_or(PolylineError::MissingVertexPoint { dart })?;
        let curve = edge
            .curve()
            .ok_or(PolylineError::MissingEdgeCurve { dart })?;

        pcurves.insert(dart, curve_pcurve(curve, start, end, plane)?);
    }

    Ok(pcurves)
}

/// Projects one oriented 3D curve boundary into a plane's parameter space.
///
/// Lines stay analytical. Other curve variants preserve their NURBS degree,
/// weights, and knots while projecting the homogeneous control polygon.
pub(crate) fn curve_pcurve(
    curve: &Curve,
    start: Point3,
    end: Point3,
    plane: &Plane,
) -> Result<Curve2, NurbsError> {
    match curve {
        Curve::Line(_) => Ok(Curve2::Line(Line2::new(
            plane_uv(plane, start),
            plane_uv(plane, end),
        ))),
        Curve::Bounded(bounded) if matches!(bounded.inner(), Curve::Line(_)) => Ok(Curve2::Line(
            Line2::new(plane_uv(plane, start), plane_uv(plane, end)),
        )),
        Curve::Circle(_) | Curve::Nurbs(_) | Curve::Bounded(_) => {
            let nurbs = curve.to_nurbs()?;
            let control_points = ControlPolygon2::new(
                nurbs
                    .control_points()
                    .iter()
                    .map(|point| {
                        HPoint2::from_cartesian(
                            plane_uv(plane, point.to_cartesian()),
                            point.weight(),
                        )
                    })
                    .collect(),
            )?;
            Ok(Curve2::Nurbs(NurbsCurve2::new(
                nurbs.degree(),
                control_points,
                nurbs.knots().clone(),
            )?))
        }
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

/// Adds a rectangular profile to the given GMap.
///
/// The corners are built on `plane` in the following order:
/// 0-----1
/// |     |
/// |     |
/// 3-----2
///
/// Returns the profile key whose stored dart starts at the first corner.
pub fn add_rectangle<P: Payload>(
    g: &mut GMap<P>,
    plane: Plane,
    x_size: f64,
    y_size: f64,
) -> Result<ProfileKey, PolylineError> {
    g.transaction(|g| add_rectangle_staged(g, plane, x_size, y_size))
}

/// Builds the four rectangle edges and profile inside one transaction.
pub(crate) fn add_rectangle_staged<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
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
    add_polyline_staged(g, &corners)
}

/// Adds a closed square profile on `plane`.
///
/// The first corner is the plane origin and the sides follow its positive x and
/// y directions. `size` must be positive and finite.
pub fn add_square<P: Payload>(
    g: &mut GMap<P>,
    plane: Plane,
    size: f64,
) -> Result<ProfileKey, PolylineError> {
    g.transaction(|g| add_rectangle_staged(g, plane, size, size))
}

fn validate_rectangle_size(axis: &'static str, value: f64) -> Result<(), PolylineError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(PolylineError::InvalidRectangleSize { axis, value })
    }
}

/// Adds the given number of darts and sews them together in a profile, the profile is closed if the given closed is true.
pub fn add_profile_darts<P: Payload>(g: &mut GMap<P>, count: usize, closed: bool) -> ProfileKey {
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
        Ok::<_, TopologyEditError>(
            edit.add_profile(ProfileAttr::new(darts[0], P::Profile::default())),
        )
    })
    .expect("fresh profile topology must commit")
}

fn add_segments<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    segments: &[(Point3, Point3, Curve)],
) -> Result<ProfileKey, PolylineError> {
    let first_segment = segments.first().ok_or(PolylineError::EmptyPolyline)?;
    let last_segment = segments.last().ok_or(PolylineError::EmptyPolyline)?;
    let closed = first_segment.0.coincides(&last_segment.1, LINEAR_TOLERANCE);

    let mut segment_topology = Vec::with_capacity(segments.len());
    for (start_point, _, curve) in segments {
        let start_dart = g.add_dart();
        let end_dart = g.add_dart();
        g.link(Dim::Zero, start_dart, end_dart)
            .map_err(polyline_edit_error)?;
        g.add_vertex(VertexAttr::new(start_dart, *start_point, P::V::default()));
        g.add_edge(EdgeAttr::new(start_dart, curve.clone(), P::E::default()));
        segment_topology.push(SegmentTopology {
            start: start_dart,
            end: end_dart,
        });
    }

    for pair in segment_topology.windows(2) {
        g.sew(Dim::One, pair[0].end, pair[1].start)
            .map_err(polyline_edit_error)?;
    }

    if closed {
        let first = segment_topology
            .first()
            .expect("non-empty segment list should have a first segment");
        let last = segment_topology
            .last()
            .expect("non-empty segment list should have a last segment");
        g.sew(Dim::One, last.end, first.start)
            .map_err(polyline_edit_error)?;
    } else {
        let last_segment = segments
            .last()
            .expect("non-empty segment list should have a last segment");
        let last_topology = segment_topology
            .last()
            .expect("non-empty segment list should have a last segment");
        g.add_vertex(VertexAttr::new(
            last_topology.end,
            last_segment.1,
            P::V::default(),
        ));
    }

    let first_start = segment_topology[0].start;
    Ok(g.add_profile(ProfileAttr::new(first_start, P::Profile::default())))
}

#[derive(Clone, Copy)]
struct SegmentTopology {
    start: Dart,
    end: Dart,
}

#[derive(Clone, Copy)]
struct VertexMerge {
    survivor: VertexKey,
    removed: VertexKey,
}

fn polyline_edit_error(error: TopologyEditError) -> PolylineError {
    error.into()
}

fn vertex_point<P: Payload>(g: &GMap<P>, dart: Dart) -> Result<Point3, PolylineError> {
    g.attribute::<Cell0>(dart)
        .map(|attr| attr.point)
        .ok_or(PolylineError::MissingVertexPoint { dart })
}

fn vertex_key<P: Payload>(g: &GMap<P>, dart: Dart) -> Result<VertexKey, PolylineError> {
    g.cell_key::<Cell0>(dart)
        .ok_or(PolylineError::MissingVertexPoint { dart })
}
