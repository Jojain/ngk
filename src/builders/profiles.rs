use std::collections::HashMap;

use crate::geometry::{Curve, Curve2, Line, Line2, Plane, Point2, Point3, Polyline2, Surface};
use crate::topology::attributes::{EdgeAttr, FaceAttr, VertexAttr};
use crate::topology::gmap::{Cell1, Dart, Dim, GMap};
use crate::topology::payload::Payload;
use crate::topology::planar::PlanarLoop;
use crate::topology::profile::Profile;
use crate::topology::shape_keys::{EdgeKey, FaceKey};

pub fn add_edge<P: Payload>(
    g: &mut GMap<P>,
    start: Point3,
    end: Point3,
    curve: Curve,
) -> (Dart, EdgeKey) {
    let d1 = g.add_dart();
    let d2 = g.add_dart();
    g.sew_unchecked(Dim::Zero, d1, d2);
    g.add_vertex(VertexAttr::new(d1, start, P::V::default()));
    g.add_vertex(VertexAttr::new(d2, end, P::V::default()));
    let e = g.add_edge(EdgeAttr::new(d1, curve, P::E::default()));
    (d1, e)
}

use crate::topology::payload::StandardPayload;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PolylineError {
    EmptyPolyline,

    CreatedEdgeMissing,
    MissingVertexPoint { dart: Dart },
    MissingEdgeCurve { dart: Dart },
    SewFailed { dim: Dim, first: Dart, second: Dart },
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

pub fn add_face<P: Payload>(
    g: &mut GMap<P>,
    loop_: PlanarLoop<'_, P>,
) -> Result<FaceKey, PolylineError> {
    let (loop_, plane) = loop_.into_parts();
    let loop_dart = loop_.dart;
    let pcurves = profile_pcurves(g, &loop_, &plane)?;

    let face_key = g.add_face(FaceAttr::with_pcurves(
        Surface::Plane(plane),
        P::F::default(),
        loop_dart,
        Vec::new(),
        pcurves,
    ));
    Ok(face_key)
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
/// Adds a square profile to the given GMap.
///
/// The corners are expected to be in the following order:
/// 0-----1
/// |     |
/// |     |
/// 3-----2
///
/// Returns the dart of the first corner.
pub fn add_square(
    g: &mut GMap<StandardPayload>,
    corners: &[Point3; 4],
) -> Result<Dart, PolylineError> {
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

fn add_polyline_segment(
    g: &mut GMap<StandardPayload>,
    (start, end, curve): &(Point3, Point3, Curve),
) -> Result<(Dart, Dart), PolylineError> {
    let edge_key = add_edge(g, *start, *end, curve.clone());
    let edge = g
        .edge(edge_key.1)
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

/// Adds a single polygon face to `g` with the given corner points (in order).
///
/// Sews α0 and α1 to form a closed `n`-gon, stamps the vertex positions on
/// every dart of each corner's vertex orbit, and attaches a straight
/// [`Curve::Line`] on every 1-cell so downstream consumers (edge tessellation,
/// dart geometry) have a curve to follow. Does not touch α2 — the face is
/// returned with free boundary, ready to be stitched to neighbors.
///
/// Returns a dart on the outer ⟨α₀, α₁⟩ loop (same as the first corner dart).
pub fn add_polygon<P: Payload>(g: &mut GMap<P>, corners: &[Point3]) -> Dart {
    assert!(
        corners.len() >= 3,
        "add_polygon requires at least 3 corners, got {}",
        corners.len()
    );
    let n = corners.len();
    let darts: Vec<Dart> = (0..2 * n).map(|_| g.add_dart()).collect();

    for i in 0..n {
        g.sew(Dim::Zero, darts[2 * i], darts[2 * i + 1])
            .expect("fresh dart pair should be alpha0-sewable");
    }
    for i in 0..n {
        let a = darts[2 * i + 1];
        let b = darts[(2 * i + 2) % (2 * n)];
        g.sew(Dim::One, a, b)
            .expect("fresh dart pair should be alpha1-sewable");
    }

    for i in 0..n {
        let dart = g.cell_representative(darts[2 * i], Dim::Zero);
        g.add_vertex(VertexAttr::new(dart, corners[i], P::V::default()));
    }

    for i in 0..n {
        let edge_dart = g.cell_representative(darts[2 * i], Dim::One);
        let curve = Curve::Line(Line::new(corners[i], corners[(i + 1) % n]));
        g.add_edge(EdgeAttr::new(edge_dart, curve, P::E::default()));
    }
    darts[0]
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
