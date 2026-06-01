use std::collections::HashSet;

use crate::StandardPayload;
use crate::builders::edges::add_circle as add_circle_edge;
use crate::builders::edges::{EdgeSplit, EdgeSplitError, split_face_boundary_edge};
use crate::builders::errors::FaceCreationError;
use crate::builders::profiles::{
    add_rectangle as add_rectangle_profile, add_square as add_square_profile, profile_pcurves,
};
use crate::geometry::axis::Axis3;
use crate::geometry::{Curve, Curve2, LINEAR_TOLERANCE, Plane, Point3, Surface};
use crate::topology::attributes::{EdgeAttr, FaceAttr, VertexAttr};
use crate::topology::closed::Closed;
use crate::topology::gmap::{Cell0, Cell1, Cell2, Dart, Dim, GMap};
use crate::topology::payload::Payload;
use crate::topology::planar::Planar;
use crate::topology::profile::Profile;
use crate::topology::shape_keys::{EdgeKey, FaceKey};
use thiserror::Error;

#[derive(Debug, Clone, Error, PartialEq)]
pub enum FaceEdgeSplitError {
    #[error("missing face for key {face:?}")]
    MissingFace { face: FaceKey },
    #[error("edge {edge:?} is not on face {face:?}")]
    EdgeNotOnFace { face: FaceKey, edge: EdgeKey },
    #[error("face {face:?} has no pcurve for boundary dart {dart:?}")]
    MissingPcurve { face: FaceKey, dart: Dart },
    #[error("failed to split boundary edge")]
    EdgeSplitFailed(#[from] EdgeSplitError),
    #[error("edge at dart {dart:?} has missing endpoint geometry")]
    MissingEndpointGeometry { dart: Dart },
    #[error("edge at dart {dart:?} has no attached curve")]
    MissingEdgeCurve { dart: Dart },
    #[error("split parameter {parameter} is too close to an edge boundary")]
    DegenerateSplit { parameter: f64 },
}

struct IncidentFacePcurve {
    face: FaceKey,
    dart: Dart,
    pcurve: Curve2,
    fraction: f64,
}

pub fn add_face<P: Payload>(
    g: &mut GMap<P>,
    loop_dart: Dart,
) -> Result<FaceKey, FaceCreationError> {
    let (plane, pcurves) = {
        let profile = Profile::new(g, loop_dart);
        let closed =
            Closed::new(profile).ok_or(FaceCreationError::OpenProfile { dart: loop_dart })?;
        let planar = Planar::new(closed)?;
        let (closed, plane) = planar.into_parts();
        let pcurves = profile_pcurves(closed.inner(), &plane)?;
        (plane, pcurves)
    };

    let face_key = g.add_face(FaceAttr::with_pcurves(
        Surface::Plane(plane),
        P::F::default(),
        loop_dart,
        Vec::new(),
        pcurves,
    ));
    Ok(face_key)
}

pub fn add_rectangle(
    g: &mut GMap<StandardPayload>,
    plane: Plane,
    x_size: f64,
    y_size: f64,
) -> Result<FaceKey, FaceCreationError> {
    let loop_dart = add_rectangle_profile(g, plane, x_size, y_size)?;
    add_face(g, loop_dart)
}

pub fn add_square(
    g: &mut GMap<StandardPayload>,
    plane: Plane,
    size: f64,
) -> Result<FaceKey, FaceCreationError> {
    let loop_dart = add_square_profile(g, plane, size)?;
    add_face(g, loop_dart)
}

pub fn split_face_edge<P: Payload>(
    g: &mut GMap<P>,
    face: FaceKey,
    edge: EdgeKey,
    parameter: f64,
) -> Result<EdgeSplit, FaceEdgeSplitError> {
    face_edge_dart(g, face, edge)?;
    let pcurves = incident_face_pcurves(g, edge, parameter)?;

    let split = split_face_boundary_edge(g, edge, parameter)?;
    for pcurve in pcurves {
        assign_split_pcurves(g, pcurve)?;
    }
    Ok(split)
}

pub fn add_circle(
    g: &mut GMap<StandardPayload>,
    plane: Plane,
    radius: f64,
) -> Result<FaceKey, FaceCreationError> {
    let (loop_dart, _) = add_circle_edge(g, plane.clone(), radius)?;
    let pcurves = profile_pcurves(&Profile::new(g, loop_dart), &plane)?;
    let face_key = g.add_face(FaceAttr::with_pcurves(
        Surface::Plane(plane),
        (),
        loop_dart,
        Vec::new(),
        pcurves,
    ));
    Ok(face_key)
}

pub fn add_annulus(
    g: &mut GMap<StandardPayload>,
    plane: Plane,
    outer_radius: f64,
    inner_radius: f64,
) -> Result<FaceKey, FaceCreationError> {
    if inner_radius >= outer_radius {
        return Err(FaceCreationError::InvalidAnnulusRadii {
            outer_radius,
            inner_radius,
        });
    }

    let inner_plane = Plane::new(plane.origin(), plane.x_dir(), -plane.normal());
    let (outer_loop, _) = add_circle_edge(g, plane.clone(), outer_radius)?;
    let (inner_loop, _) = add_circle_edge(g, inner_plane, inner_radius)?;

    let mut pcurves = profile_pcurves(&Profile::new(g, outer_loop), &plane)?;
    pcurves.extend(profile_pcurves(&Profile::new(g, inner_loop), &plane)?);

    let face_key = g.add_face(FaceAttr::with_pcurves(
        Surface::Plane(plane),
        (),
        outer_loop,
        vec![inner_loop],
        pcurves,
    ));
    Ok(face_key)
}

fn face_edge_dart<P: Payload>(
    g: &GMap<P>,
    face: FaceKey,
    edge: EdgeKey,
) -> Result<Dart, FaceEdgeSplitError> {
    let face_attr = g
        .face(face)
        .ok_or(FaceEdgeSplitError::MissingFace { face })?;
    let edge_attr = g.edge(edge).ok_or(FaceEdgeSplitError::EdgeSplitFailed(
        EdgeSplitError::MissingEdge { edge },
    ))?;
    let edge_dart = g.cell_representative(edge_attr.dart, Dim::One);
    std::iter::once(face_attr.outer_loop)
        .chain(face_attr.inner_loops.iter().copied())
        .flat_map(|loop_dart| Profile::new(g, loop_dart).edges())
        .find_map(|candidate| {
            (g.cell_representative(candidate.dart, Dim::One) == edge_dart).then_some(candidate.dart)
        })
        .ok_or(FaceEdgeSplitError::EdgeNotOnFace { face, edge })
}

fn face_pcurve<P: Payload>(
    g: &GMap<P>,
    face: FaceKey,
    dart: Dart,
) -> Result<Curve2, FaceEdgeSplitError> {
    let face_attr = g
        .face(face)
        .ok_or(FaceEdgeSplitError::MissingFace { face })?;
    face_attr
        .pcurves
        .get(&dart)
        .cloned()
        .ok_or(FaceEdgeSplitError::MissingPcurve { face, dart })
}

fn incident_face_pcurves<P: Payload>(
    g: &GMap<P>,
    edge: EdgeKey,
    parameter: f64,
) -> Result<Vec<IncidentFacePcurve>, FaceEdgeSplitError> {
    let edge_attr = g.edge(edge).ok_or(FaceEdgeSplitError::EdgeSplitFailed(
        EdgeSplitError::MissingEdge { edge },
    ))?;
    let mut seen = HashSet::new();
    g.orbit(edge_attr.dart, g.orbit_indices(Dim::One))
        .filter_map(|dart| g.attribute::<Cell2>(dart).copied())
        .filter(|face| seen.insert(*face))
        .map(|face| {
            let dart = face_edge_dart(g, face, edge)?;
            let pcurve = face_pcurve(g, face, dart)?;
            let fraction = pcurve_split_fraction(g, dart, parameter)?;
            Ok(IncidentFacePcurve {
                face,
                dart,
                pcurve,
                fraction,
            })
        })
        .collect()
}

fn pcurve_split_fraction<P: Payload>(
    g: &GMap<P>,
    boundary_dart: Dart,
    parameter: f64,
) -> Result<f64, FaceEdgeSplitError> {
    let start = g
        .attribute::<Cell0>(boundary_dart)
        .map(|vertex| vertex.point)
        .ok_or(FaceEdgeSplitError::MissingEndpointGeometry {
            dart: boundary_dart,
        })?;
    let end_dart = g.alpha(Dim::Zero, boundary_dart);
    let end = g
        .attribute::<Cell0>(end_dart)
        .map(|vertex| vertex.point)
        .ok_or(FaceEdgeSplitError::MissingEndpointGeometry { dart: end_dart })?;
    let curve = g
        .attribute::<Cell1>(boundary_dart)
        .map(|edge| &edge.curve)
        .ok_or(FaceEdgeSplitError::MissingEdgeCurve {
            dart: boundary_dart,
        })?;
    let interval = curve.parameters_between(start, end);
    let length = interval.end - interval.start;
    if length.abs() <= LINEAR_TOLERANCE {
        return Err(FaceEdgeSplitError::DegenerateSplit { parameter });
    }
    Ok(((parameter - interval.start) / length).clamp(0.0, 1.0))
}

fn assign_split_pcurves<P: Payload>(
    g: &mut GMap<P>,
    pcurve: IncidentFacePcurve,
) -> Result<(), FaceEdgeSplitError> {
    let second_dart = g.alpha(Dim::One, g.alpha(Dim::Zero, pcurve.dart));
    let (first_pcurve, second_pcurve) = pcurve.pcurve.split_at(pcurve.fraction);
    let face_attr = g
        .face_mut(pcurve.face)
        .ok_or(FaceEdgeSplitError::MissingFace { face: pcurve.face })?;
    face_attr.pcurves.remove(&pcurve.dart);
    face_attr.pcurves.insert(pcurve.dart, first_pcurve);
    face_attr.pcurves.insert(second_dart, second_pcurve);
    Ok(())
}

pub fn add_polygon_with_holes(
    g: &mut GMap<StandardPayload>,
    plane: Plane,
    outer: &[Point3],
    holes: &[&[Point3]],
) -> Result<FaceKey, FaceCreationError> {
    validate_polygon(outer)?;
    for hole in holes {
        validate_polygon(hole)?;
    }

    let outer_loop = add_polygon(g, outer);
    let mut inner_loops = Vec::with_capacity(holes.len());
    let mut pcurves = profile_pcurves(&Profile::new(g, outer_loop), &plane)?;

    for hole in holes {
        let inner_loop = add_polygon(g, hole);
        pcurves.extend(profile_pcurves(&Profile::new(g, inner_loop), &plane)?);
        inner_loops.push(inner_loop);
    }

    let face_key = g.add_face(FaceAttr::with_pcurves(
        Surface::Plane(plane),
        (),
        outer_loop,
        inner_loops,
        pcurves,
    ));
    Ok(face_key)
}

fn validate_polygon(points: &[Point3]) -> Result<(), FaceCreationError> {
    if points.len() >= 3 {
        Ok(())
    } else {
        Err(FaceCreationError::InvalidPolygon {
            point_count: points.len(),
        })
    }
}

/// Adds a single polygon face to `g` with the given corner points (in order).
///
/// Sews alpha0 and alpha1 to form a closed `n`-gon, stamps the vertex positions on
/// every dart of each corner's vertex orbit, and attaches a straight
/// [`Curve::Line`] on every 1-cell so downstream consumers (edge tessellation,
/// dart geometry) have a curve to follow. Does not touch alpha2; the face is
/// returned with free boundary, ready to be stitched to neighbors.
///
/// Returns a dart on the outer <alpha0, alpha1> loop (same as the first corner dart).
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
        let curve = Curve::line(Axis3::from_points(corners[i], corners[(i + 1) % n]));
        g.add_edge(EdgeAttr::new(edge_dart, curve, P::E::default()));
    }
    darts[0]
}
