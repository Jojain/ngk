use crate::builders::add_edge;
use crate::geometry::{Curve, Line, Point3};
use crate::topology::gmap::{Dart, Dim, GMap};
use crate::topology::payload::StandardPayload;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PolylineError {
    EmptyPolyline,
    CreatedEdgeMissing,
    SewFailed { dim: Dim, first: Dart, second: Dart },
}

pub fn add_polyline(
    g: &mut GMap<StandardPayload>,
    segments: &[(Point3, Point3, Curve)],
) -> Result<Dart, PolylineError> {
    let (first_segment, remaining_segments) = segments
        .split_first()
        .ok_or(PolylineError::EmptyPolyline)?;
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

/// Adds a square profile to the given GMap.
///
/// The corners are expected to be in the following order:
/// 0-----1
/// |     |
/// |     |
/// 3-----2
///
/// Returns the dart of the first corner.
pub fn add_square(g: &mut GMap<StandardPayload>, corners: &[Point3; 4]) -> Result<Dart, PolylineError> {
    let segments = [
        (corners[0], corners[1], Curve::Line(Line::new(corners[0], corners[1]))),
        (corners[1], corners[2], Curve::Line(Line::new(corners[1], corners[2]))),
        (corners[2], corners[3], Curve::Line(Line::new(corners[2], corners[3]))),
        (corners[3], corners[0], Curve::Line(Line::new(corners[3], corners[0]))),
    ];
    add_polyline(g, &segments)
}

fn add_polyline_segment(
    g: &mut GMap<StandardPayload>,
    (start, end, curve): &(Point3, Point3, Curve),
) -> Result<(Dart, Dart), PolylineError> {
    let edge_key = add_edge(g, *start, *end, curve.clone());
    let edge = g
        .edge(edge_key)
        .ok_or(PolylineError::CreatedEdgeMissing)?;
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
