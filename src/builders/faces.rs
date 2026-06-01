use std::collections::{HashMap, HashSet};

use crate::StandardPayload;
use crate::builders::edges::add_circle as add_circle_edge;
use crate::builders::edges::{EdgeSplit, EdgeSplitError, split_face_boundary_edge};
use crate::builders::errors::FaceCreationError;
use crate::builders::profiles::{
    add_rectangle as add_rectangle_profile, add_square as add_square_profile, profile_pcurves,
};
use crate::geometry::{Curve, Curve2, LINEAR_TOLERANCE, Plane, Point2, Point3, Surface};
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

#[derive(Debug, Clone, Error, PartialEq)]
pub enum FaceImprintSplitError {
    #[error("missing face for key {face:?}")]
    MissingFace { face: FaceKey },
    #[error("face {face:?} has inner loops, which are not supported by this splitter yet")]
    InnerLoopsNotSupported { face: FaceKey },
    #[error("face {face:?} has no pcurve for boundary dart {dart:?}")]
    MissingPcurve { face: FaceKey, dart: Dart },
    #[error("face {face:?} has missing vertex geometry at dart {dart:?}")]
    MissingVertexGeometry { face: FaceKey, dart: Dart },
}

#[derive(Debug, Clone, PartialEq)]
pub struct FaceImprint {
    pub points: Vec<Point3>,
    pub pcurve: Curve2,
}

#[derive(Debug, Clone, PartialEq)]
pub struct FaceImprintSplit {
    pub first: FaceKey,
    pub second: FaceKey,
    pub section_edge: EdgeKey,
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

pub fn split_face_by_imprints<P: Payload>(
    g: &mut GMap<P>,
    face: FaceKey,
    imprints: &[FaceImprint],
) -> Result<Option<FaceImprintSplit>, FaceImprintSplitError> {
    let face_attr = g
        .face(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?;
    if !face_attr.inner_loops.is_empty() {
        return Err(FaceImprintSplitError::InnerLoopsNotSupported { face });
    }

    let boundary = face_boundary_vertices(g, face, face_attr.outer_loop)?;
    let network = FaceImprintNetwork::from_imprints(imprints, &boundary);
    let Some(cut) = network.first_cut().cloned() else {
        return Ok(None);
    };

    let old_face = g
        .remove_face(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?;
    Ok(Some(apply_outer_face_chord_split(g, face, old_face, &cut)?))
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

#[derive(Debug, Clone)]
struct FaceImprintNetwork {
    cuts: Vec<FaceImprintCut>,
}

impl FaceImprintNetwork {
    fn from_imprints(imprints: &[FaceImprint], boundary: &[BoundaryVertex]) -> Self {
        let mut seen = HashSet::<(usize, usize)>::new();
        let cuts = imprints
            .iter()
            .filter_map(|imprint| FaceImprintCut::from_imprint(imprint, boundary))
            .filter(|cut| seen.insert(cut.dedup_key()))
            .collect();
        Self { cuts }
    }

    fn first_cut(&self) -> Option<&FaceImprintCut> {
        self.cuts.first()
    }
}

#[derive(Debug, Clone)]
struct FaceImprintCut {
    start: BoundaryVertex,
    end: BoundaryVertex,
    pcurve: Curve2,
}

impl FaceImprintCut {
    fn from_imprint(imprint: &FaceImprint, boundary: &[BoundaryVertex]) -> Option<Self> {
        let start_uv = imprint.pcurve.point_at(0.0);
        let end_uv = imprint.pcurve.point_at(1.0);
        let start = snap_boundary_vertex(boundary, start_uv)?;
        let end = snap_boundary_vertex(boundary, end_uv)?;
        if !valid_chord(&start, &end) {
            return None;
        }

        Some(Self {
            start,
            end,
            pcurve: imprint.pcurve.clone(),
        })
    }

    fn dedup_key(&self) -> (usize, usize) {
        let a = self.start.dart.id();
        let b = self.end.dart.id();
        if a < b { (a, b) } else { (b, a) }
    }
}

#[derive(Debug, Clone, Copy)]
struct BoundaryVertex {
    dart: Dart,
    previous_end: Dart,
    point: Point3,
    uv: Point2,
    index: usize,
    vertex_count: usize,
}

fn face_boundary_vertices<P: Payload>(
    g: &GMap<P>,
    face: FaceKey,
    loop_dart: Dart,
) -> Result<Vec<BoundaryVertex>, FaceImprintSplitError> {
    let darts = Profile::new(g, loop_dart).darts().collect::<Vec<_>>();
    let vertex_count = darts.len() / 2;
    let mut vertices = Vec::with_capacity(vertex_count);

    for index in 0..vertex_count {
        let dart_index = index * 2;
        let dart = darts[dart_index];
        let previous_end = darts[(dart_index + darts.len() - 1) % darts.len()];
        let point = g
            .attribute::<Cell0>(dart)
            .map(|vertex| vertex.point)
            .ok_or(FaceImprintSplitError::MissingVertexGeometry { face, dart })?;
        let uv = g
            .face(face)
            .and_then(|attr| attr.pcurves.get(&dart))
            .map(|pcurve| pcurve.point_at(0.0))
            .ok_or(FaceImprintSplitError::MissingPcurve { face, dart })?;

        vertices.push(BoundaryVertex {
            dart,
            previous_end,
            point,
            uv,
            index,
            vertex_count,
        });
    }

    Ok(vertices)
}

fn snap_boundary_vertex(boundary: &[BoundaryVertex], uv: Point2) -> Option<BoundaryVertex> {
    boundary
        .iter()
        .copied()
        .filter_map(|vertex| {
            let distance = (vertex.uv - uv).norm();
            (distance <= LINEAR_TOLERANCE).then_some((distance, vertex))
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .map(|(_, vertex)| vertex)
}

fn valid_chord(start: &BoundaryVertex, end: &BoundaryVertex) -> bool {
    if start.dart == end.dart || start.vertex_count != end.vertex_count {
        return false;
    }

    let distance = start.index.abs_diff(end.index);
    distance > 1 && distance < start.vertex_count - 1
}

fn apply_outer_face_chord_split<P: Payload>(
    g: &mut GMap<P>,
    original_face: FaceKey,
    old_face: FaceAttr<P::F>,
    cut: &FaceImprintCut,
) -> Result<FaceImprintSplit, FaceImprintSplitError> {
    let pcurve_ab = oriented_cut_pcurve(cut);
    let pcurve_ba = pcurve_ab.reversed();
    let ab_start = g.add_dart();
    let ab_end = g.add_dart();
    let ba_start = g.add_dart();
    let ba_end = g.add_dart();

    g.sew_unchecked(Dim::Zero, ab_start, ab_end);
    g.sew_unchecked(Dim::Zero, ba_start, ba_end);
    g.sew_unchecked(Dim::Two, ab_start, ba_end);
    g.sew_unchecked(Dim::Two, ab_end, ba_start);

    g.unsew(cut.start.previous_end, Dim::One);
    g.unsew(cut.end.previous_end, Dim::One);
    g.sew_unchecked(Dim::One, cut.start.previous_end, ab_start);
    g.sew_unchecked(Dim::One, ab_end, cut.end.dart);
    g.sew_unchecked(Dim::One, cut.end.previous_end, ba_start);
    g.sew_unchecked(Dim::One, ba_end, cut.start.dart);

    let section_edge = g.add_edge(EdgeAttr::new(
        ab_start,
        Curve::line(cut.start.point, cut.end.point),
        P::E::default(),
    ));
    let first_pcurves = split_face_pcurves(
        g,
        original_face,
        &old_face.pcurves,
        cut.start.dart,
        ba_start,
        &pcurve_ba,
    )?;
    let second_pcurves = split_face_pcurves(
        g,
        original_face,
        &old_face.pcurves,
        cut.end.dart,
        ab_start,
        &pcurve_ab,
    )?;
    let first = g.add_face(FaceAttr::with_pcurves(
        old_face.surface.clone(),
        old_face.data.clone(),
        cut.start.dart,
        Vec::new(),
        first_pcurves,
    ));
    let second = g.add_face(FaceAttr::with_pcurves(
        old_face.surface,
        old_face.data,
        cut.end.dart,
        Vec::new(),
        second_pcurves,
    ));

    Ok(FaceImprintSplit {
        first,
        second,
        section_edge,
    })
}

fn oriented_cut_pcurve(cut: &FaceImprintCut) -> Curve2 {
    let start = cut.pcurve.point_at(0.0);
    if (start - cut.start.uv).norm() <= LINEAR_TOLERANCE {
        cut.pcurve.clone()
    } else {
        cut.pcurve.reversed()
    }
}

fn split_face_pcurves<P: Payload>(
    g: &GMap<P>,
    face: FaceKey,
    old_pcurves: &HashMap<Dart, Curve2>,
    loop_dart: Dart,
    section_dart: Dart,
    section_pcurve: &Curve2,
) -> Result<HashMap<Dart, Curve2>, FaceImprintSplitError> {
    let mut pcurves = HashMap::new();
    for edge in Profile::new(g, loop_dart).edges() {
        let pcurve = if edge.dart == section_dart {
            section_pcurve.clone()
        } else {
            old_pcurves
                .get(&edge.dart)
                .cloned()
                .ok_or(FaceImprintSplitError::MissingPcurve {
                    face,
                    dart: edge.dart,
                })?
        };
        pcurves.insert(edge.dart, pcurve);
    }
    Ok(pcurves)
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
        let curve = Curve::line(corners[i], corners[(i + 1) % n]);
        g.add_edge(EdgeAttr::new(edge_dart, curve, P::E::default()));
    }
    darts[0]
}
