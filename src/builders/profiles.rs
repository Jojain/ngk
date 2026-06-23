use std::collections::HashMap;

use crate::geometry::{
    ControlPolygon2, Curve, Curve2, HPoint2, LINEAR_TOLERANCE, Line2, NurbsCurve2, Plane, Point2,
    Point3, PointCoincidence,
};
use crate::topology::attributes::{EdgeAttr, ProfileAttr, VertexAttr};
use crate::topology::closed::Closeable;
use crate::topology::gmap::{Cell0, Dart, Dim, GMap};
use crate::topology::payload::Payload;
use crate::topology::profile::Profile;
use crate::topology::shape_keys::ProfileKey;

pub use crate::builders::errors::PolylineError;

pub fn add_polyline<P: Payload>(
    g: &mut GMap<P>,
    points: &[Point3],
) -> Result<ProfileKey, PolylineError> {
    if points.len() < 2 {
        return Err(PolylineError::EmptyPolyline);
    }

    let segments = points
        .windows(2)
        .map(|pair| (pair[0], pair[1], Curve::line(pair[0], pair[1])))
        .collect::<Vec<_>>();
    let dart = add_segments(g, &segments)?;
    Ok(g.add_profile(ProfileAttr::new(dart, P::Profile::default())))
}

pub fn add_edge_to_profile<P: Payload>(
    g: &mut GMap<P>,
    profile_dart: Dart,
    edge_dart: Dart,
) -> Result<(), PolylineError> {
    g.ensure_profile(profile_dart);
    let profile = Profile::from_dart(g, profile_dart)
        .expect("profile dart must belong to a registered profile");
    if profile.is_closed() {
        return Err(PolylineError::ClosedProfile { dart: profile_dart });
    }

    let profile_start = profile_dart;
    let profile_end = profile
        .darts()
        .last()
        .expect("non-empty profile should have an end dart");
    let profile_start_point = vertex_point(g, profile_start)?;
    let profile_end_point = vertex_point(g, profile_end)?;
    let edge_start_point = vertex_point(g, edge_dart)?;
    let edge_end = g.alpha(Dim::Zero, edge_dart);
    let edge_end_point = vertex_point(g, edge_end)?;

    if !profile_end_point.coincides(edge_start_point, LINEAR_TOLERANCE) {
        return Err(PolylineError::NonContiguousEdge {
            profile_end: profile_end_point,
            edge_start: edge_start_point,
        });
    }

    sew(g, Dim::One, profile_end, edge_dart)?;
    if edge_end_point.coincides(profile_start_point, LINEAR_TOLERANCE) {
        sew(g, Dim::One, edge_end, profile_start)?;
    }

    Ok(())
}

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

fn curve_pcurve(
    curve: &Curve,
    start: Point3,
    end: Point3,
    plane: &Plane,
) -> Result<Curve2, PolylineError> {
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
    validate_rectangle_size("x", x_size)?;
    validate_rectangle_size("y", y_size)?;

    let corners = [
        plane.point_at(0.0, 0.0),
        plane.point_at(x_size, 0.0),
        plane.point_at(x_size, y_size),
        plane.point_at(0.0, y_size),
        plane.point_at(0.0, 0.0),
    ];
    add_polyline(g, &corners)
}

pub fn add_square<P: Payload>(
    g: &mut GMap<P>,
    plane: Plane,
    size: f64,
) -> Result<ProfileKey, PolylineError> {
    add_rectangle(g, plane, size, size)
}

fn validate_rectangle_size(axis: &'static str, value: f64) -> Result<(), PolylineError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(PolylineError::InvalidRectangleSize { axis, value })
    }
}

fn sew<P: Payload>(
    g: &mut GMap<P>,
    dim: Dim,
    first: Dart,
    second: Dart,
) -> Result<(), PolylineError> {
    g.edit_preserving(|edit| {
        edit.sew(dim, first, second)
            .map_err(|_| PolylineError::SewFailed { dim, first, second })
    })
    .map_err(|_| PolylineError::SewFailed { dim, first, second })
}

/// Adds the given number of darts and sews them together in a profile, the profile is closed if the given closed is true.
pub fn add_profile_darts<P: Payload>(g: &mut GMap<P>, count: usize, closed: bool) -> ProfileKey {
    g.edit_preserving(|edit| {
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
        Ok::<_, crate::topology::TopologyEditError>(
            edit.add_profile(ProfileAttr::new(darts[0], P::Profile::default())),
        )
    })
    .expect("fresh profile topology must commit")
}

fn add_segments<P: Payload>(
    g: &mut GMap<P>,
    segments: &[(Point3, Point3, Curve)],
) -> Result<Dart, PolylineError> {
    let first_segment = segments.first().ok_or(PolylineError::EmptyPolyline)?;
    let last_segment = segments.last().ok_or(PolylineError::EmptyPolyline)?;
    let closed = first_segment.0.coincides(&last_segment.1, LINEAR_TOLERANCE);

    let segment_darts = add_segment_topology(g, segments.len())?;
    let first_start = segment_darts[0].start;

    segment_darts.windows(2).try_for_each(|pair| {
        let previous_end = pair[0].end;
        let next_start = pair[1].start;
        sew(g, Dim::One, previous_end, next_start)
    })?;

    if closed {
        let last_end = segment_darts
            .last()
            .expect("non-empty polyline has a last segment")
            .end;
        sew(g, Dim::One, last_end, first_start)?;
    }

    add_segment_attributes(g, segments, &segment_darts, closed);
    Ok(first_start)
}

#[derive(Clone, Copy)]
struct SegmentTopology {
    start: Dart,
    end: Dart,
}

fn add_segment_topology<P: Payload>(
    g: &mut GMap<P>,
    count: usize,
) -> Result<Vec<SegmentTopology>, PolylineError> {
    let mut segments = Vec::with_capacity(count);
    for _ in 0..count {
        let (start, end) = g
            .edit_preserving(|edit| {
                Ok::<_, crate::topology::TopologyEditError>((edit.add_dart(), edit.add_dart()))
            })
            .expect("fresh test darts must commit");
        sew(g, Dim::Zero, start, end)?;
        segments.push(SegmentTopology { start, end });
    }
    Ok(segments)
}

fn add_segment_attributes<P: Payload>(
    g: &mut GMap<P>,
    segments: &[(Point3, Point3, Curve)],
    segment_darts: &[SegmentTopology],
    closed: bool,
) {
    for (segment, darts) in segments.iter().zip(segment_darts) {
        g.add_edge(EdgeAttr::new(
            darts.start,
            segment.2.clone(),
            P::E::default(),
        ));
    }

    for (segment, darts) in segments.iter().zip(segment_darts) {
        let vertex_dart = g.cell_representative(darts.start, Dim::Zero);
        g.add_vertex(VertexAttr::new(vertex_dart, segment.0, P::V::default()));
    }

    if !closed {
        let last = segments
            .last()
            .zip(segment_darts.last())
            .expect("non-empty segment list should have a last dart");
        let vertex_dart = g.cell_representative(last.1.end, Dim::Zero);
        g.add_vertex(VertexAttr::new(vertex_dart, last.0.1, P::V::default()));
    }
}

fn vertex_point<P: Payload>(g: &GMap<P>, dart: Dart) -> Result<Point3, PolylineError> {
    g.attribute::<Cell0>(dart)
        .map(|attr| attr.point)
        .ok_or(PolylineError::MissingVertexPoint { dart })
}
