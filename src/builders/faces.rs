use std::collections::{HashMap, HashSet};

use crate::StandardPayload;
use crate::builders::edges::add_circle as add_circle_edge;
use crate::builders::edges::{EdgeSplit, EdgeSplitError, split_face_boundary_edge};
use crate::builders::errors::FaceCreationError;
use crate::builders::profiles::{
    add_rectangle as add_rectangle_profile, add_square as add_square_profile, profile_pcurves,
};
use crate::geometry::{Curve, Curve2, LINEAR_TOLERANCE, Line2, Plane, Point2, Point3, Surface};
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
    #[error("face {face:?} has boundary edge without an edge attribute at dart {dart:?}")]
    MissingBoundaryEdge { face: FaceKey, dart: Dart },
    #[error("failed to split boundary edge while paving face imprints")]
    BoundaryEdgeSplitFailed(#[from] FaceEdgeSplitError),
    #[error("failed to sew closed imprint loop on face {face:?}: {reason}")]
    SectionLoopSewFailed { face: FaceKey, reason: &'static str },
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
    pub section_edges: Vec<EdgeKey>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct FaceImprintGraph {
    vertices: Vec<FaceImprintGraphVertex>,
    edges: Vec<FaceImprintGraphEdge>,
}

impl FaceImprintGraph {
    pub fn from_imprints(imprints: &[FaceImprint]) -> Self {
        let segments = imprints
            .iter()
            .flat_map(imprint_segments)
            .collect::<Vec<_>>();
        Self::from_segments(&segments)
    }

    pub fn vertices(&self) -> &[FaceImprintGraphVertex] {
        &self.vertices
    }

    pub fn edges(&self) -> &[FaceImprintGraphEdge] {
        &self.edges
    }

    pub fn vertex_degree(&self, vertex: usize) -> usize {
        self.edges
            .iter()
            .filter(|edge| edge.start == vertex || edge.end == vertex)
            .count()
    }

    pub fn branch_vertices(&self) -> Vec<usize> {
        (0..self.vertices.len())
            .filter(|vertex| self.vertex_degree(*vertex) > 2)
            .collect()
    }

    pub fn closed_components(&self) -> Vec<FaceImprintGraphLoop> {
        let mut visited = vec![false; self.vertices.len()];
        let mut loops = Vec::new();

        for start in 0..self.vertices.len() {
            if visited[start] {
                continue;
            }

            let component = self.component_vertices(start, &mut visited);
            if component.len() < 3
                || !component
                    .iter()
                    .all(|vertex| self.vertex_degree(*vertex) == 2)
            {
                continue;
            }

            if let Some(vertices) = self.ordered_closed_component(&component) {
                loops.push(FaceImprintGraphLoop { vertices });
            }
        }

        loops
    }

    pub fn closed_component_count(&self) -> usize {
        self.closed_components().len()
    }

    fn from_segments(segments: &[ImprintSegment]) -> Self {
        let split_parameters = split_parameters(segments);
        let mut vertices = Vec::<FaceImprintGraphVertex>::new();
        let mut edges = Vec::<FaceImprintGraphEdge>::new();
        let mut seen_edges = HashSet::<(usize, usize)>::new();

        for (segment, parameters) in segments.iter().zip(split_parameters.iter()) {
            for pair in parameters.windows(2) {
                if (pair[1] - pair[0]).abs() <= LINEAR_TOLERANCE {
                    continue;
                }

                let start = graph_vertex(&mut vertices, segment.point_at(pair[0]));
                let end = graph_vertex(&mut vertices, segment.point_at(pair[1]));
                if start == end {
                    continue;
                }

                let key = ordered_edge_key(start, end);
                if seen_edges.insert(key) {
                    edges.push(FaceImprintGraphEdge { start, end });
                }
            }
        }

        Self { vertices, edges }
    }

    fn component_vertices(&self, start: usize, visited: &mut [bool]) -> Vec<usize> {
        let mut stack = vec![start];
        let mut component = Vec::new();
        visited[start] = true;

        while let Some(vertex) = stack.pop() {
            component.push(vertex);
            for neighbor in self.neighbors(vertex) {
                if !visited[neighbor] {
                    visited[neighbor] = true;
                    stack.push(neighbor);
                }
            }
        }

        component
    }

    fn neighbors(&self, vertex: usize) -> impl Iterator<Item = usize> + '_ {
        self.edges.iter().filter_map(move |edge| {
            if edge.start == vertex {
                Some(edge.end)
            } else if edge.end == vertex {
                Some(edge.start)
            } else {
                None
            }
        })
    }

    fn ordered_closed_component(&self, component: &[usize]) -> Option<Vec<usize>> {
        let component_set = component.iter().copied().collect::<HashSet<_>>();
        let start = component.iter().copied().min()?;
        let mut ordered = vec![start];
        let mut previous = start;
        let mut current = self
            .neighbors(start)
            .find(|neighbor| component_set.contains(neighbor))?;

        while current != start {
            if ordered.contains(&current) {
                return None;
            }
            ordered.push(current);

            let next = self
                .neighbors(current)
                .find(|neighbor| *neighbor != previous && component_set.contains(neighbor))?;
            previous = current;
            current = next;
        }

        (ordered.len() == component.len()).then_some(ordered)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct FaceImprintGraphVertex {
    pub uv: Point2,
}

#[derive(Debug, Clone, PartialEq)]
pub struct FaceImprintGraphLoop {
    vertices: Vec<usize>,
}

impl FaceImprintGraphLoop {
    pub fn vertex_indices(&self) -> &[usize] {
        &self.vertices
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct FaceImprintGraphEdge {
    pub start: usize,
    pub end: usize,
}

#[derive(Debug, Clone, Copy)]
struct ImprintSegment {
    start: Point2,
    end: Point2,
}

impl ImprintSegment {
    fn point_at(&self, t: f64) -> Point2 {
        self.start + (self.end - self.start) * t
    }
}

fn imprint_segments(imprint: &FaceImprint) -> Vec<ImprintSegment> {
    match &imprint.pcurve {
        Curve2::Line(line) => vec![ImprintSegment {
            start: line.start,
            end: line.end,
        }],
        Curve2::Polyline(polyline) => polyline
            .points
            .windows(2)
            .map(|pair| ImprintSegment {
                start: pair[0],
                end: pair[1],
            })
            .collect(),
    }
}

fn split_parameters(segments: &[ImprintSegment]) -> Vec<Vec<f64>> {
    let mut parameters = vec![vec![0.0, 1.0]; segments.len()];

    for i in 0..segments.len() {
        for j in (i + 1)..segments.len() {
            if let Some(intersection) = segment_intersection(segments[i], segments[j]) {
                parameters[i].extend(intersection.first);
                parameters[j].extend(intersection.second);
            }
        }
    }

    for values in &mut parameters {
        values.sort_by(|a, b| a.total_cmp(b));
        values.dedup_by(|a, b| (*a - *b).abs() <= LINEAR_TOLERANCE);
    }

    parameters
}

#[derive(Debug, Clone)]
struct SegmentIntersection {
    first: Vec<f64>,
    second: Vec<f64>,
}

fn segment_intersection(a: ImprintSegment, b: ImprintSegment) -> Option<SegmentIntersection> {
    let da = a.end - a.start;
    let db = b.end - b.start;
    let denominator = cross2(da, db);

    if denominator.abs() <= LINEAR_TOLERANCE {
        return collinear_segment_intersection(a, b);
    }

    let delta = b.start - a.start;
    let ta = cross2(delta, db) / denominator;
    let tb = cross2(delta, da) / denominator;
    if in_segment_parameter(ta) && in_segment_parameter(tb) {
        Some(SegmentIntersection {
            first: vec![ta.clamp(0.0, 1.0)],
            second: vec![tb.clamp(0.0, 1.0)],
        })
    } else {
        None
    }
}

fn collinear_segment_intersection(
    a: ImprintSegment,
    b: ImprintSegment,
) -> Option<SegmentIntersection> {
    let da = a.end - a.start;
    if cross2(b.start - a.start, da).abs() > LINEAR_TOLERANCE {
        return None;
    }

    let axis = if da.x.abs() >= da.y.abs() { 0 } else { 1 };
    let a0 = coord(a.start, axis);
    let a1 = coord(a.end, axis);
    let b0 = coord(b.start, axis);
    let b1 = coord(b.end, axis);
    let a_min = a0.min(a1);
    let a_max = a0.max(a1);
    let b_min = b0.min(b1);
    let b_max = b0.max(b1);
    let overlap_min = a_min.max(b_min);
    let overlap_max = a_max.min(b_max);

    if overlap_max < overlap_min - LINEAR_TOLERANCE {
        return None;
    }

    Some(SegmentIntersection {
        first: vec![
            scalar_to_segment_parameter(a0, a1, overlap_min),
            scalar_to_segment_parameter(a0, a1, overlap_max),
        ],
        second: vec![
            scalar_to_segment_parameter(b0, b1, overlap_min),
            scalar_to_segment_parameter(b0, b1, overlap_max),
        ],
    })
}

fn graph_vertex(vertices: &mut Vec<FaceImprintGraphVertex>, uv: Point2) -> usize {
    if let Some((index, _)) = vertices
        .iter()
        .enumerate()
        .find(|(_, vertex)| (vertex.uv - uv).norm() <= LINEAR_TOLERANCE)
    {
        return index;
    }

    let index = vertices.len();
    vertices.push(FaceImprintGraphVertex { uv });
    index
}

fn ordered_edge_key(a: usize, b: usize) -> (usize, usize) {
    if a < b { (a, b) } else { (b, a) }
}

fn cross2(a: nalgebra::Vector2<f64>, b: nalgebra::Vector2<f64>) -> f64 {
    a.x * b.y - a.y * b.x
}

fn in_segment_parameter(t: f64) -> bool {
    (-LINEAR_TOLERANCE..=1.0 + LINEAR_TOLERANCE).contains(&t)
}

fn coord(point: Point2, axis: usize) -> f64 {
    if axis == 0 { point.x } else { point.y }
}

fn scalar_to_segment_parameter(start: f64, end: f64, value: f64) -> f64 {
    let length = end - start;
    if length.abs() <= LINEAR_TOLERANCE {
        0.0
    } else {
        ((value - start) / length).clamp(0.0, 1.0)
    }
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
) -> Result<Vec<FaceImprintSplit>, FaceImprintSplitError> {
    let graph = FaceImprintGraph::from_imprints(imprints);
    split_imprint_boundary_endpoints(g, face, imprints)?;
    let mut splits = add_closed_imprint_loops(g, face, &graph)?;
    let mut active_faces = vec![face];

    loop {
        let mut next_faces = Vec::new();
        let mut progressed = false;

        for face in active_faces {
            let Some(split) = split_one_face_by_imprints(g, face, imprints)? else {
                next_faces.push(face);
                continue;
            };

            next_faces.push(split.first);
            next_faces.push(split.second);
            splits.push(split);
            progressed = true;
        }

        if !progressed {
            return Ok(splits);
        }
        active_faces = next_faces;
    }
}

fn split_imprint_boundary_endpoints<P: Payload>(
    g: &mut GMap<P>,
    face: FaceKey,
    imprints: &[FaceImprint],
) -> Result<(), FaceImprintSplitError> {
    let endpoints = imprints
        .iter()
        .flat_map(|imprint| [imprint.pcurve.point_at(0.0), imprint.pcurve.point_at(1.0)])
        .collect::<Vec<_>>();

    for endpoint in endpoints {
        split_boundary_at_uv(g, face, endpoint)?;
    }

    Ok(())
}

fn add_closed_imprint_loops<P: Payload>(
    g: &mut GMap<P>,
    face: FaceKey,
    graph: &FaceImprintGraph,
) -> Result<Vec<FaceImprintSplit>, FaceImprintSplitError> {
    let outer_loop = g
        .face(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?
        .outer_loop;
    let boundary = face_boundary_vertices(g, face, outer_loop)?;
    let mut splits = Vec::new();

    for component in graph.closed_components() {
        let mut uvs = component
            .vertex_indices()
            .iter()
            .map(|vertex| graph.vertices()[*vertex].uv)
            .collect::<Vec<_>>();
        if uvs.len() < 3
            || uvs
                .iter()
                .any(|uv| snap_boundary_vertex(&boundary, *uv).is_some())
        {
            continue;
        }

        orient_inner_loop_against_boundary(&boundary, &mut uvs);
        splits.push(split_face_by_closed_imprint_loop(g, face, &uvs)?);
    }

    Ok(splits)
}

fn split_face_by_closed_imprint_loop<P: Payload>(
    g: &mut GMap<P>,
    face: FaceKey,
    uvs: &[Point2],
) -> Result<FaceImprintSplit, FaceImprintSplitError> {
    let old_face = g
        .face(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?
        .clone();
    let mut island_uvs = uvs.to_vec();
    island_uvs.reverse();

    let outside_loop = add_section_loop(g, &old_face.surface, uvs);
    let island_loop = add_section_loop(g, &old_face.surface, &island_uvs);
    let section_edges = sew_section_loops(g, face, &outside_loop, &island_loop)?;

    let face_attr = g
        .face_mut(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?;
    face_attr.inner_loops.push(outside_loop.loop_dart);
    face_attr.pcurves.extend(outside_loop.pcurves);

    let representative = g.cell_representative(outside_loop.loop_dart, Dim::Two);
    g.dart_to_face.insert(representative, face);
    let second = g.add_face(FaceAttr::with_pcurves(
        old_face.surface,
        old_face.data,
        island_loop.loop_dart,
        Vec::new(),
        island_loop.pcurves,
    ));

    Ok(FaceImprintSplit {
        first: face,
        second,
        section_edges,
    })
}

#[derive(Debug)]
struct SectionLoop {
    loop_dart: Dart,
    edges: Vec<SectionLoopEdge>,
    pcurves: HashMap<Dart, Curve2>,
}

#[derive(Debug, Clone, Copy)]
struct SectionLoopEdge {
    dart: Dart,
    start_uv: Point2,
    end_uv: Point2,
    start_point: Point3,
    end_point: Point3,
}

fn add_section_loop<P: Payload>(g: &mut GMap<P>, surface: &Surface, uvs: &[Point2]) -> SectionLoop {
    let n = uvs.len();
    let darts = (0..2 * n).map(|_| g.add_dart()).collect::<Vec<_>>();

    for edge in 0..n {
        g.sew_unchecked(Dim::Zero, darts[2 * edge], darts[2 * edge + 1]);
    }
    for edge in 0..n {
        let end = darts[2 * edge + 1];
        let next_start = darts[2 * ((edge + 1) % n)];
        g.sew_unchecked(Dim::One, end, next_start);
    }

    for vertex in 0..n {
        let dart = g.cell_representative(darts[2 * vertex], Dim::Zero);
        let uv = uvs[vertex];
        g.add_vertex(VertexAttr::new(
            dart,
            surface.point_at(uv.x, uv.y),
            P::V::default(),
        ));
    }

    let edges = (0..n)
        .map(|edge| {
            let next = (edge + 1) % n;
            SectionLoopEdge {
                dart: darts[2 * edge],
                start_uv: uvs[edge],
                end_uv: uvs[next],
                start_point: surface.point_at(uvs[edge].x, uvs[edge].y),
                end_point: surface.point_at(uvs[next].x, uvs[next].y),
            }
        })
        .collect::<Vec<_>>();
    let pcurves = edges
        .iter()
        .map(|edge| {
            (
                edge.dart,
                Curve2::Line(Line2::new(edge.start_uv, edge.end_uv)),
            )
        })
        .collect();

    SectionLoop {
        loop_dart: darts[0],
        edges,
        pcurves,
    }
}

fn sew_section_loops<P: Payload>(
    g: &mut GMap<P>,
    face: FaceKey,
    outside: &SectionLoop,
    island: &SectionLoop,
) -> Result<Vec<EdgeKey>, FaceImprintSplitError> {
    outside
        .edges
        .iter()
        .map(|outside_edge| {
            let island_edge = matching_reversed_loop_edge(outside_edge, &island.edges).ok_or(
                FaceImprintSplitError::MissingPcurve {
                    face,
                    dart: outside_edge.dart,
                },
            )?;

            let island_end = g.alpha(Dim::Zero, island_edge.dart);
            g.sew(Dim::Two, outside_edge.dart, island_end)
                .map_err(|reason| FaceImprintSplitError::SectionLoopSewFailed { face, reason })?;

            Ok(g.add_edge(EdgeAttr::new(
                outside_edge.dart,
                Curve::line(outside_edge.start_point, outside_edge.end_point),
                P::E::default(),
            )))
        })
        .collect()
}

fn matching_reversed_loop_edge(
    edge: &SectionLoopEdge,
    candidates: &[SectionLoopEdge],
) -> Option<SectionLoopEdge> {
    candidates.iter().copied().find(|candidate| {
        (candidate.start_uv - edge.end_uv).norm() <= LINEAR_TOLERANCE
            && (candidate.end_uv - edge.start_uv).norm() <= LINEAR_TOLERANCE
    })
}

fn orient_inner_loop_against_boundary(boundary: &[BoundaryVertex], uvs: &mut [Point2]) {
    let boundary_uvs = boundary.iter().map(|vertex| vertex.uv).collect::<Vec<_>>();
    let boundary_area = signed_area(&boundary_uvs);
    let loop_area = signed_area(uvs);

    if boundary_area.abs() <= LINEAR_TOLERANCE || loop_area.abs() <= LINEAR_TOLERANCE {
        return;
    }

    if boundary_area.signum() == loop_area.signum() {
        uvs.reverse();
    }
}

fn signed_area(uvs: &[Point2]) -> f64 {
    if uvs.len() < 3 {
        return 0.0;
    }

    0.5 * uvs
        .iter()
        .zip(uvs.iter().cycle().skip(1))
        .take(uvs.len())
        .map(|(a, b)| a.x * b.y - b.x * a.y)
        .sum::<f64>()
}

fn split_boundary_at_uv<P: Payload>(
    g: &mut GMap<P>,
    face: FaceKey,
    uv: Point2,
) -> Result<(), FaceImprintSplitError> {
    let face_attr = g
        .face(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?;
    let boundary = face_boundary_vertices(g, face, face_attr.outer_loop)?;
    if snap_boundary_vertex(&boundary, uv).is_some() {
        return Ok(());
    }

    let Some(target) = boundary_edge_at_uv(g, face, face_attr.outer_loop, uv)? else {
        return Ok(());
    };

    let edge_attr = g
        .edge(target.edge)
        .ok_or(FaceImprintSplitError::MissingBoundaryEdge {
            face,
            dart: target.dart,
        })?;
    let start = g
        .attribute::<Cell0>(target.dart)
        .map(|vertex| vertex.point)
        .ok_or(FaceImprintSplitError::MissingVertexGeometry {
            face,
            dart: target.dart,
        })?;
    let end_dart = g.alpha(Dim::Zero, target.dart);
    let end = g
        .attribute::<Cell0>(end_dart)
        .map(|vertex| vertex.point)
        .ok_or(FaceImprintSplitError::MissingVertexGeometry {
            face,
            dart: end_dart,
        })?;
    let interval = edge_attr.curve.parameters_between(start, end);
    let parameter = interval.start + (interval.end - interval.start) * target.fraction;
    split_face_edge(g, face, target.edge, parameter)?;
    Ok(())
}

fn split_one_face_by_imprints<P: Payload>(
    g: &mut GMap<P>,
    face: FaceKey,
    imprints: &[FaceImprint],
) -> Result<Option<FaceImprintSplit>, FaceImprintSplitError> {
    let face_attr = g
        .face(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?;

    let boundary = face_boundary_vertices(g, face, face_attr.outer_loop)?;
    let network = FaceImprintNetwork::from_imprints(imprints, &boundary);
    let Some(cut) = network.first_cut().cloned() else {
        return Ok(None);
    };

    if !face_attr.inner_loops.is_empty() {
        return Err(FaceImprintSplitError::InnerLoopsNotSupported { face });
    }

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

#[derive(Debug, Clone, Copy)]
struct BoundaryEdgeTarget {
    dart: Dart,
    edge: EdgeKey,
    fraction: f64,
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

fn boundary_edge_at_uv<P: Payload>(
    g: &GMap<P>,
    face: FaceKey,
    loop_dart: Dart,
    uv: Point2,
) -> Result<Option<BoundaryEdgeTarget>, FaceImprintSplitError> {
    let face_attr = g
        .face(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?;

    for edge in Profile::new(g, loop_dart).edges() {
        let pcurve =
            face_attr
                .pcurves
                .get(&edge.dart)
                .ok_or(FaceImprintSplitError::MissingPcurve {
                    face,
                    dart: edge.dart,
                })?;
        let Some(fraction) = pcurve_fraction_at(pcurve, uv) else {
            continue;
        };
        if fraction <= LINEAR_TOLERANCE || 1.0 - fraction <= LINEAR_TOLERANCE {
            continue;
        }

        return Ok(Some(BoundaryEdgeTarget {
            dart: edge.dart,
            edge: edge_key_for_dart(g, face, edge.dart)?,
            fraction,
        }));
    }

    Ok(None)
}

fn pcurve_fraction_at(pcurve: &Curve2, point: Point2) -> Option<f64> {
    match pcurve {
        Curve2::Line(line) => line_segment_fraction(line.start, line.end, point),
        Curve2::Polyline(polyline) => {
            polyline
                .points
                .windows(2)
                .enumerate()
                .find_map(|(index, pair)| {
                    let local = line_segment_fraction(pair[0], pair[1], point)?;
                    let segment_count = polyline.points.len().saturating_sub(1);
                    (segment_count > 0).then_some((index as f64 + local) / segment_count as f64)
                })
        }
    }
}

fn line_segment_fraction(start: Point2, end: Point2, point: Point2) -> Option<f64> {
    let direction = end - start;
    let length_sq = direction.norm_squared();
    if length_sq <= LINEAR_TOLERANCE * LINEAR_TOLERANCE {
        return None;
    }

    let t = (point - start).dot(&direction) / length_sq;
    if !(-LINEAR_TOLERANCE..=1.0 + LINEAR_TOLERANCE).contains(&t) {
        return None;
    }

    let projected = start + direction * t.clamp(0.0, 1.0);
    ((projected - point).norm() <= LINEAR_TOLERANCE).then_some(t.clamp(0.0, 1.0))
}

fn edge_key_for_dart<P: Payload>(
    g: &GMap<P>,
    face: FaceKey,
    dart: Dart,
) -> Result<EdgeKey, FaceImprintSplitError> {
    let representative = g.cell_representative(dart, Dim::One);
    g.iter_edges()
        .find_map(|(key, edge)| (edge.dart == representative).then_some(key))
        .ok_or(FaceImprintSplitError::MissingBoundaryEdge { face, dart })
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
        section_edges: vec![section_edge],
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
