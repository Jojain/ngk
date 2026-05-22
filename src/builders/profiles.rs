use std::collections::HashMap;

use crate::builders::edges::add_edge;
use crate::builders::errors::EdgeCreationError;
use crate::geometry::{Curve, Curve2, Line, Line2, Plane, Point2, Point3, Polyline2};
use crate::topology::gmap::{Cell1, Dart, Dim, GMap};
use crate::topology::payload::{Payload, StandardPayload};
use crate::topology::planar::PlanarityError;
use crate::topology::profile::Profile;
use thiserror::Error;

#[derive(Debug, Clone, Error, PartialEq)]
pub enum PolylineError {
    #[error("polyline is empty")]
    EmptyPolyline,

    #[error("created edge is missing")]
    CreatedEdgeMissing,
    #[error("profile starting at dart {dart:?} is open")]
    OpenProfile { dart: Dart },
    #[error("profile is not planar: {0}")]
    NonPlanarProfile(#[from] PlanarityError),
    #[error("missing vertex point for dart {dart:?}")]
    MissingVertexPoint { dart: Dart },
    #[error("missing edge curve for dart {dart:?}")]
    MissingEdgeCurve { dart: Dart },
    #[error("rectangle {axis} size must be greater than 0, got {value}")]
    InvalidRectangleSize { axis: &'static str, value: f64 },
    #[error("darts {first:?} and {second:?} are not sewable in dimension {dim:?}")]
    SewFailed { dim: Dim, first: Dart, second: Dart },
    #[error("failed to create polyline edge")]
    EdgeCreationFailed(#[from] EdgeCreationError),
}

pub fn add_polyline(
    g: &mut GMap<StandardPayload>,
    segments: &[(Point3, Point3, Curve)],
) -> Result<Dart, PolylineError> {
    let (first_segment, remaining_segments) =
        segments.split_first().ok_or(PolylineError::EmptyPolyline)?;
    let last_segment = segments.last().ok_or(PolylineError::EmptyPolyline)?;
    let closed = first_segment.0 == last_segment.1;

    let (first_start, mut previous_end) = add_polyline_segment(g, first_segment)?;

    for segment in remaining_segments {
        let (start_dart, end_dart) = add_polyline_segment(g, segment)?;
        sew(g, Dim::One, previous_end, start_dart)?;
        previous_end = end_dart;
    }

    if closed {
        sew(g, Dim::One, previous_end, first_start)?;
    }

    Ok(first_start)
}

pub fn profile_pcurves<P: Payload>(
    g: &GMap<P>,
    profile: &Profile<'_, P>,
    plane: &Plane,
) -> Result<HashMap<Dart, Curve2>, PolylineError> {
    let darts = profile.darts().step_by(2).collect::<Vec<_>>();
    let vertices = profile.vertices();
    let edges = profile.edges();
    let mut pcurves = HashMap::with_capacity(darts.len());

    for (i, dart) in darts.into_iter().enumerate() {
        let start = *vertices[i]
            .point()
            .ok_or(PolylineError::MissingVertexPoint { dart })?;
        let end = *vertices[(i + 1) % vertices.len()]
            .point()
            .ok_or(PolylineError::MissingVertexPoint { dart })?;
        let curve = g
            .attribute::<Cell1>(dart)
            .map(|attr| &attr.curve)
            .or_else(|| edges[i].curve())
            .ok_or(PolylineError::MissingEdgeCurve { dart })?;

        pcurves.insert(dart, curve_pcurve(curve, start, end, plane));
    }

    Ok(pcurves)
}

fn curve_pcurve(curve: &Curve, start: Point3, end: Point3, plane: &Plane) -> Curve2 {
    match curve {
        Curve::Line(_) => Curve2::Line(Line2::new(plane_uv(plane, start), plane_uv(plane, end))),
        Curve::Circle(_) | Curve::Nurbs(_) => {
            let (t0, t1) = curve.parameters_between(start, end);
            let segments = 32usize;
            let points = (0..=segments)
                .map(|i| {
                    let t = t0 + (t1 - t0) * (i as f64 / segments as f64);
                    plane_uv(plane, curve.point_at(t))
                })
                .collect();
            Curve2::Polyline(Polyline2::new(points))
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
/// Returns the dart of the first corner.
pub fn add_rectangle(
    g: &mut GMap<StandardPayload>,
    plane: Plane,
    x_size: f64,
    y_size: f64,
) -> Result<Dart, PolylineError> {
    validate_rectangle_size("x", x_size)?;
    validate_rectangle_size("y", y_size)?;

    let corners = [
        plane.point_at(0.0, 0.0),
        plane.point_at(x_size, 0.0),
        plane.point_at(x_size, y_size),
        plane.point_at(0.0, y_size),
    ];
    let segments = [
        (
            corners[0],
            corners[1],
            Curve::Line(Line::new(corners[0], corners[1])),
        ),
        (
            corners[1],
            corners[2],
            Curve::Line(Line::new(corners[1], corners[2])),
        ),
        (
            corners[2],
            corners[3],
            Curve::Line(Line::new(corners[2], corners[3])),
        ),
        (
            corners[3],
            corners[0],
            Curve::Line(Line::new(corners[3], corners[0])),
        ),
    ];
    add_polyline(g, &segments)
}

fn validate_rectangle_size(axis: &'static str, value: f64) -> Result<(), PolylineError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(PolylineError::InvalidRectangleSize { axis, value })
    }
}

fn add_polyline_segment(
    g: &mut GMap<StandardPayload>,
    (start, end, curve): &(Point3, Point3, Curve),
) -> Result<(Dart, Dart), PolylineError> {
    let (_, edge_key) = add_edge(g, *start, *end, curve.clone())?;
    let edge = g.edge(edge_key).ok_or(PolylineError::CreatedEdgeMissing)?;
    let start_dart = edge.dart;
    let end_dart = g.alpha(Dim::Zero, start_dart);
    Ok((start_dart, end_dart))
}

fn sew(
    g: &mut GMap<StandardPayload>,
    dim: Dim,
    first: Dart,
    second: Dart,
) -> Result<(), PolylineError> {
    g.sew(dim, first, second)
        .map_err(|_| PolylineError::SewFailed { dim, first, second })
}

/// Adds the given number of darts and sews them together in a profile, the profile is closed if the given closed is true.
pub fn add_profile_darts<P: Payload>(g: &mut GMap<P>, count: usize, closed: bool) -> Dart {
    let darts: Vec<Dart> = (0..count).map(|_| g.add_dart()).collect();
    for i in 0..count {
        g.sew(Dim::Zero, darts[i], darts[(i + 1) % count])
            .expect("fresh dart pair should be alpha0-sewable");
    }
    for i in 0..count {
        g.sew(Dim::One, darts[i], darts[(i + 1) % count])
            .expect("fresh dart pair should be alpha1-sewable");
    }
    if closed {
        g.sew(Dim::Zero, darts[count - 1], darts[0])
            .expect("fresh dart pair should be alpha0-sewable");
    }
    darts[0]
}
