use std::collections::HashMap;

use crate::builders::edges::add_edge;
use crate::geometry::{
    Curve, Curve2, LINEAR_TOLERANCE, Line, Line2, Plane, Point2, Point3, PointCoincidence,
    Polyline2,
};
use crate::topology::gmap::{Cell1, Dart, Dim, GMap};
use crate::topology::payload::{Payload, StandardPayload};
use crate::topology::profile::Profile;

pub use crate::builders::errors::PolylineError;

pub fn add_polyline(
    g: &mut GMap<StandardPayload>,
    segments: &[(Point3, Point3, Curve)],
) -> Result<Dart, PolylineError> {
    let first_segment = segments.first().ok_or(PolylineError::EmptyPolyline)?;
    let last_segment = segments.last().ok_or(PolylineError::EmptyPolyline)?;
    let closed = first_segment.0.coincides(&last_segment.1, LINEAR_TOLERANCE);

    let segment_darts = segments
        .iter()
        .map(|segment| add_polyline_segment(g, segment))
        .collect::<Result<Vec<_>, _>>()?;
    let first_start = segment_darts[0].0;

    segment_darts.windows(2).try_for_each(|pair| {
        let previous_end = pair[0].1;
        let next_start = pair[1].0;
        sew(g, Dim::One, previous_end, next_start)
    })?;

    if closed {
        let last_end = segment_darts
            .last()
            .expect("non-empty polyline has a last segment")
            .1;
        sew(g, Dim::One, last_end, first_start)?;
    }

    Ok(first_start)
}

pub fn profile_pcurves<P: Payload>(
    g: &GMap<P>,
    profile: &Profile<'_, P>,
    plane: &Plane,
) -> Result<HashMap<Dart, Curve2>, PolylineError> {
    let edges = profile.edges();
    let mut pcurves = HashMap::with_capacity(edges.len());

    for edge in edges.iter() {
        let dart = edge.dart;
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
        (corners[0], corners[1], Curve::line(corners[0], corners[1])),
        (corners[1], corners[2], Curve::line(corners[1], corners[2])),
        (corners[2], corners[3], Curve::line(corners[2], corners[3])),
        (corners[3], corners[0], Curve::line(corners[3], corners[0])),
    ];
    add_polyline(g, &segments)
}

pub fn add_square(
    g: &mut GMap<StandardPayload>,
    plane: Plane,
    size: f64,
) -> Result<Dart, PolylineError> {
    add_rectangle(g, plane, size, size)
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
    let (start_dart, _) = add_edge(g, *start, *end, curve.clone())?;
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
