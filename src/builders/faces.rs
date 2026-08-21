use std::collections::{HashMap, HashSet};

use crate::StandardPayload;
use crate::builders::edges::add_circle as add_circle_edge;
use crate::builders::edges::{EdgeSplit, EdgeSplitError, split_face_boundary_edge};
use crate::builders::errors::FaceCreationError;
use crate::builders::profiles::{
    add_rectangle as add_rectangle_profile, add_square as add_square_profile, profile_pcurves,
};
use crate::geometry::{
    Circle, Curve, Curve2, CurveCurveIntersection2, CurveIntersectionError, Interval,
    LINEAR_TOLERANCE, Line2, NurbsError, Periodicity, Plane, Point2, Point3, Surface,
    SurfacePeriodicity,
};
use crate::topology::TopologyEditError;
use crate::topology::attributes::{EdgeAttr, FaceAttr, VertexAttr};
use crate::topology::closed::Closed;
use crate::topology::edge::Edge;
use crate::topology::gmap::{Cell1, Cell2, Dart, Dim, GMap};
use crate::topology::payload::Payload;
use crate::topology::planar::Planar;
use crate::topology::profile::Profile;
use crate::topology::shape_keys::{EdgeKey, FaceKey};
use crate::topology::vertex::Vertex;
use nalgebra::Vector2;
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
    #[error("split point does not lie on face {face:?} pcurve at dart {dart:?}")]
    SplitPointNotOnPcurve { face: FaceKey, dart: Dart },
    #[error("failed to split face pcurve")]
    PcurveSplitFailed(#[from] NurbsError),
}

#[derive(Debug, Error)]
pub enum FaceImprintSplitError {
    #[error("missing face for key {face:?}")]
    MissingFace { face: FaceKey },
    #[error("face {face:?} has inner loops, which are not supported by this splitter yet")]
    InnerLoopsNotSupported { face: FaceKey },
    #[error("face {face:?} has no pcurve for boundary dart {dart:?}")]
    MissingPcurve { face: FaceKey, dart: Dart },
    #[error("missing vertex geometry at dart {dart:?}")]
    MissingVertexGeometry { dart: Dart },
    #[error("boundary edge at dart {dart:?} has no edge geometry")]
    MissingBoundaryEdge { dart: Dart },
    #[error("failed to split boundary edge while paving face imprints")]
    BoundaryEdgeSplitFailed(#[from] FaceEdgeSplitError),
    #[error("failed to sew closed imprint loop on face {face:?}: {source}")]
    SectionLoopSewFailed {
        face: FaceKey,
        #[source]
        source: TopologyEditError,
    },
    #[error("failed to convert imprint curve geometry")]
    ImprintCurveConversion(#[from] NurbsError),
    #[error("failed to intersect face imprint pcurves")]
    ImprintIntersection(#[from] CurveIntersectionError),
    #[error("failed to merge periodic face {face:?} across its parameter seam: {reason}")]
    PeriodicMergeFailed { face: FaceKey, reason: &'static str },
    #[error("failed to edit periodic face {face:?} across its parameter seam: {source}")]
    PeriodicTopologyEditFailed {
        face: FaceKey,
        #[source]
        source: TopologyEditError,
    },
    #[error("periodic face {face:?} produced {count} regions instead of two")]
    UnexpectedPeriodicRegionCount { face: FaceKey, count: usize },
}

#[derive(Debug, Clone, Copy)]
struct MissingVertexPoint(Dart);

#[derive(Debug, Clone, Copy)]
struct MissingEdgeCurve(Dart);

impl From<MissingVertexPoint> for FaceImprintSplitError {
    fn from(error: MissingVertexPoint) -> Self {
        Self::MissingVertexGeometry { dart: error.0 }
    }
}

impl From<MissingVertexPoint> for FaceEdgeSplitError {
    fn from(error: MissingVertexPoint) -> Self {
        Self::MissingEndpointGeometry { dart: error.0 }
    }
}

impl From<MissingEdgeCurve> for FaceImprintSplitError {
    fn from(error: MissingEdgeCurve) -> Self {
        Self::MissingBoundaryEdge { dart: error.0 }
    }
}

impl From<MissingEdgeCurve> for FaceEdgeSplitError {
    fn from(error: MissingEdgeCurve) -> Self {
        Self::MissingEdgeCurve { dart: error.0 }
    }
}

fn vertex_point<P: Payload>(vertex: Vertex<'_, P>) -> Result<Point3, MissingVertexPoint> {
    vertex
        .point()
        .copied()
        .ok_or(MissingVertexPoint(vertex.dart))
}

fn edge_curve<'a, P: Payload>(edge: &'a Edge<'_, P>) -> Result<&'a Curve, MissingEdgeCurve> {
    edge.curve().ok_or(MissingEdgeCurve(edge.dart()))
}

#[derive(Debug, Clone, PartialEq)]
pub struct FaceImprintSplit {
    pub first: FaceKey,
    pub second: FaceKey,
    pub section_edges: Vec<EdgeKey>,
}

/// Paired model-space and face-parameter-space geometry for a face imprint.
#[derive(Clone)]
pub struct FaceImprint {
    pub curve: Curve,
    pub pcurve: Curve2,
}

impl FaceImprint {
    /// Creates an imprint whose 3D curve and 2D pcurve share direction.
    pub fn new(curve: Curve, pcurve: Curve2) -> Self {
        Self { curve, pcurve }
    }

    /// Returns the exact synchronized fragment over a normalized interval.
    pub fn trimmed(&self, interval: Interval) -> Result<Self, NurbsError> {
        Ok(Self::new(
            self.curve.trimmed(interval)?,
            self.pcurve.trimmed(interval)?,
        ))
    }

    fn reversed(&self) -> Result<Self, NurbsError> {
        Ok(Self::new(
            Curve::Nurbs(self.curve.to_nurbs()?.reversed()),
            self.pcurve.reversed(),
        ))
    }
}

/// An exact source-curve fragment stored in a face imprint graph.
#[derive(Debug, Clone, PartialEq)]
pub struct FaceImprintGraphEdge {
    pub start: usize,
    pub end: usize,
    pub source_curve: usize,
    pub interval: Interval,
}

/// A normalized planar graph built from imprint curves in a face's UV space.
///
/// Curves are split at intersections, coincident vertices and duplicate
/// edges are merged within [`LINEAR_TOLERANCE`], and the resulting undirected
/// graph can be inspected for branches and standalone closed loops. The graph is
/// a temporary aid for face splitting and is not part of the [`GMap`] topology.
#[derive(Debug, Clone, PartialEq)]
pub struct FaceImprintGraph {
    vertices: Vec<Point2>,
    edges: Vec<FaceImprintGraphEdge>,
}

impl FaceImprintGraph {
    /// Builds an imprint graph from 2D lines and NURBS curves.
    pub fn from_curves(curves: &[Curve2]) -> Result<Self, CurveIntersectionError> {
        let split_parameters = curve_split_parameters(curves)?;
        let mut vertices = Vec::<Point2>::new();
        let mut edges = Vec::new();
        let mut seen_edges = HashSet::<(usize, usize)>::new();

        for (source_curve, (curve, parameters)) in curves.iter().zip(split_parameters).enumerate() {
            for pair in parameters.windows(2) {
                if (pair[1] - pair[0]).abs() <= LINEAR_TOLERANCE {
                    continue;
                }

                let start = graph_vertex(&mut vertices, curve.point_at(pair[0]));
                let end = graph_vertex(&mut vertices, curve.point_at(pair[1]));
                if start == end {
                    continue;
                }

                if seen_edges.insert(ordered_edge_key(start, end)) {
                    edges.push(FaceImprintGraphEdge {
                        start,
                        end,
                        source_curve,
                        interval: Interval::new(pair[0], pair[1]),
                    });
                }
            }
        }

        Ok(Self { vertices, edges })
    }

    /// Returns the graph vertices as points in the face's UV parameter space.
    pub fn vertices(&self) -> &[Point2] {
        &self.vertices
    }

    /// Returns the exact source-curve fragments forming the undirected graph.
    pub fn edges(&self) -> &[FaceImprintGraphEdge] {
        &self.edges
    }

    /// Returns the number of graph edges incident to `vertex`.
    ///
    /// An index outside [`Self::vertices`] has degree zero.
    pub fn vertex_degree(&self, vertex: usize) -> usize {
        self.edges
            .iter()
            .filter(|edge| edge.start == vertex || edge.end == vertex)
            .count()
    }

    /// Returns the indices of vertices incident to more than two edges.
    pub fn branch_vertices(&self) -> Vec<usize> {
        (0..self.vertices.len())
            .filter(|vertex| self.vertex_degree(*vertex) > 2)
            .collect()
    }

    /// Returns connected components that form standalone simple closed loops.
    ///
    /// Every vertex in a returned component has degree two, and the vertex
    /// indices are ordered around the loop without repeating the first vertex.
    /// Cycles embedded in a component containing branches are not returned.
    pub fn closed_components(&self) -> Vec<Vec<usize>> {
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

            if let Some(edges) = self.ordered_closed_component(&component) {
                loops.push(
                    edges
                        .iter()
                        .map(|edge| edge.start(self))
                        .collect::<Vec<_>>(),
                );
            }
        }

        loops
    }

    /// Returns the number of standalone simple closed-loop components.
    pub fn closed_component_count(&self) -> usize {
        self.closed_components().len()
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

    fn ordered_closed_component(&self, component: &[usize]) -> Option<Vec<OrientedGraphEdge>> {
        let component_set = component.iter().copied().collect::<HashSet<_>>();
        let start = component.iter().copied().min()?;
        let mut ordered = Vec::new();
        let mut current = start;
        let mut previous_edge = None;

        loop {
            let edge_index = self
                .edges
                .iter()
                .enumerate()
                .filter(|(_, edge)| edge.start == current || edge.end == current)
                .filter(|(index, edge)| {
                    Some(*index) != previous_edge
                        && component_set.contains(&edge.start)
                        && component_set.contains(&edge.end)
                })
                .map(|(index, _)| index)
                .min()?;
            let edge = &self.edges[edge_index];
            let reversed = edge.end == current;
            let next = if reversed { edge.start } else { edge.end };
            ordered.push(OrientedGraphEdge {
                edge: edge_index,
                reversed,
            });

            if next == start {
                break;
            }
            if ordered.len() >= component.len() {
                return None;
            }
            previous_edge = Some(edge_index);
            current = next;
        }

        (ordered.len() == component.len()).then_some(ordered)
    }

    fn closed_edge_components(&self) -> Vec<Vec<OrientedGraphEdge>> {
        let mut visited = vec![false; self.vertices.len()];
        let mut loops = Vec::new();
        for start in 0..self.vertices.len() {
            if visited[start] {
                continue;
            }
            let component = self.component_vertices(start, &mut visited);
            if component.len() >= 3
                && component
                    .iter()
                    .all(|vertex| self.vertex_degree(*vertex) == 2)
                && let Some(edges) = self.ordered_closed_component(&component)
            {
                loops.push(edges);
            }
        }
        loops
    }
}

#[derive(Debug, Clone, Copy)]
struct OrientedGraphEdge {
    edge: usize,
    reversed: bool,
}

impl OrientedGraphEdge {
    fn start(self, graph: &FaceImprintGraph) -> usize {
        let edge = &graph.edges[self.edge];
        if self.reversed { edge.end } else { edge.start }
    }
}

fn curve_split_parameters(curves: &[Curve2]) -> Result<Vec<Vec<f64>>, CurveIntersectionError> {
    let mut parameters = vec![vec![0.0, 1.0]; curves.len()];

    for i in 0..curves.len() {
        for j in (i + 1)..curves.len() {
            for intersection in curves[i].intersect_curve(&curves[j])? {
                match intersection {
                    CurveCurveIntersection2::Point { u_a, u_b, .. } => {
                        parameters[i].push(u_a);
                        parameters[j].push(u_b);
                    }
                    CurveCurveIntersection2::Overlap {
                        interval_a,
                        interval_b,
                    } => {
                        parameters[i].extend([interval_a.start, interval_a.end]);
                        parameters[j].extend([interval_b.start, interval_b.end]);
                    }
                }
            }
        }
    }

    for values in &mut parameters {
        values.sort_by(|a, b| a.total_cmp(b));
        values.dedup_by(|a, b| (*a - *b).abs() <= LINEAR_TOLERANCE);
    }

    Ok(parameters)
}

fn graph_vertex(vertices: &mut Vec<Point2>, uv: Point2) -> usize {
    if let Some((index, _)) = vertices
        .iter()
        .enumerate()
        .find(|(_, vertex)| (**vertex - uv).norm() <= LINEAR_TOLERANCE)
    {
        return index;
    }

    let index = vertices.len();
    vertices.push(uv);
    index
}

fn ordered_edge_key(a: usize, b: usize) -> (usize, usize) {
    if a < b { (a, b) } else { (b, a) }
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
        let profile =
            Profile::from_dart(g, loop_dart).expect("face loop must have a registered profile");
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
    let profile = add_rectangle_profile(g, plane, x_size, y_size)?;
    let loop_dart = g.profile_attr_unchecked(profile).dart;
    add_face(g, loop_dart)
}

pub fn add_square(
    g: &mut GMap<StandardPayload>,
    plane: Plane,
    size: f64,
) -> Result<FaceKey, FaceCreationError> {
    let profile = add_square_profile(g, plane, size)?;
    let loop_dart = g.profile_attr_unchecked(profile).dart;
    add_face(g, loop_dart)
}

pub fn split_face_edge<P: Payload>(
    g: &mut GMap<P>,
    face: FaceKey,
    edge: EdgeKey,
    parameter: f64,
) -> Result<EdgeSplit, FaceEdgeSplitError> {
    let boundary_dart = face_edge_dart(g, face, edge)?;
    let reversed = closed_boundary_curve_reversed(g, face, edge, boundary_dart)?;
    let pcurves = incident_face_pcurves(g, edge, parameter)?;

    let split = split_face_boundary_edge(g, edge, parameter, reversed)?;
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
    let (closed_imprints, open_imprints) = imprints.iter().fold(
        (Vec::new(), Vec::new()),
        |(mut closed, mut open), imprint| {
            if imprint.pcurve.is_closed() {
                closed.push(imprint);
            } else {
                open.push(imprint.clone());
            }
            (closed, open)
        },
    );
    if closed_imprints.is_empty()
        && open_imprints.len() == 2
        && let Some(period) = periodic_u_period(g, face)
        && open_imprints.iter().all(is_constant_u_imprint)
        && let Some(seam) = periodic_seam_edge(g, face)?
    {
        return split_periodic_face_by_imprints(g, face, &open_imprints, seam, period);
    }

    let pcurves = open_imprints
        .iter()
        .map(|imprint| imprint.pcurve.clone())
        .collect::<Vec<_>>();
    let graph = FaceImprintGraph::from_curves(&pcurves)?;
    split_imprint_boundary_endpoints(g, face, imprints)?;
    let mut splits = add_closed_curve_imprint_loops(g, face, &closed_imprints)?;
    splits.extend(add_closed_imprint_loops(g, face, &graph, &open_imprints)?);
    splits.extend(split_open_imprints(g, vec![face], &open_imprints)?);
    Ok(splits)
}

fn split_open_imprints<P: Payload>(
    g: &mut GMap<P>,
    mut active_faces: Vec<FaceKey>,
    imprints: &[FaceImprint],
) -> Result<Vec<FaceImprintSplit>, FaceImprintSplitError> {
    let mut splits = Vec::new();
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
            break;
        }
        active_faces = next_faces;
    }
    Ok(splits)
}

fn periodic_u_period<P: Payload>(g: &GMap<P>, face: FaceKey) -> Option<f64> {
    match g.face_attr(face)?.surface.periodicity() {
        SurfacePeriodicity::UPeriodic(period) | SurfacePeriodicity::UVPeriodic(period, _) => {
            Some(period)
        }
        SurfacePeriodicity::None | SurfacePeriodicity::VPeriodic(_) => None,
    }
}

fn is_constant_u_imprint(imprint: &FaceImprint) -> bool {
    let start = imprint.pcurve.point_at(0.0);
    let end = imprint.pcurve.point_at(1.0);
    (start.x - end.x).abs() <= LINEAR_TOLERANCE && (start.y - end.y).abs() > LINEAR_TOLERANCE
}

fn periodic_seam_edge<P: Payload>(
    g: &GMap<P>,
    face: FaceKey,
) -> Result<Option<EdgeKey>, FaceImprintSplitError> {
    let face = g
        .face(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?;
    let mut counts = HashMap::<EdgeKey, usize>::new();
    for edge in face.outer_loop().edges() {
        *counts
            .entry(boundary_edge_key(g, edge.dart())?)
            .or_default() += 1;
    }
    Ok(counts
        .into_iter()
        .find_map(|(edge, count)| (count > 1).then_some(edge)))
}

fn split_periodic_face_by_imprints<P: Payload>(
    g: &mut GMap<P>,
    face: FaceKey,
    imprints: &[FaceImprint],
    seam: EdgeKey,
    period: f64,
) -> Result<Vec<FaceImprintSplit>, FaceImprintSplitError> {
    split_imprint_boundary_endpoints(g, face, imprints)?;
    let splits = split_open_imprints(g, vec![face], imprints)?;
    let section_edges = splits
        .iter()
        .flat_map(|split| split.section_edges.iter().copied())
        .collect::<Vec<_>>();
    let seam_faces = g
        .edge(seam)
        .into_iter()
        .flat_map(|edge| edge.faces())
        .map(|face| face.key())
        .collect::<Vec<_>>();
    if splits.len() != 2 || seam_faces.len() != 2 {
        return Err(FaceImprintSplitError::UnexpectedPeriodicRegionCount {
            face,
            count: g.iter_faces().count(),
        });
    }

    let merged = merge_faces_across_edge(g, face, seam, seam_faces[0], seam_faces[1], period)?;
    let remaining = g
        .iter_faces()
        .map(|(key, _)| key)
        .find(|key| *key != merged)
        .ok_or(FaceImprintSplitError::UnexpectedPeriodicRegionCount { face, count: 1 })?;
    if g.iter_faces().count() != 2 {
        return Err(FaceImprintSplitError::UnexpectedPeriodicRegionCount {
            face,
            count: g.iter_faces().count(),
        });
    }
    unwrap_periodic_face_pcurves(g, remaining)?;
    unwrap_periodic_face_pcurves(g, merged)?;
    rebuild_periodic_boundary_curves(g, [remaining, merged])?;

    Ok(vec![FaceImprintSplit {
        first: remaining,
        second: merged,
        section_edges,
    }])
}

fn merge_faces_across_edge<P: Payload>(
    g: &mut GMap<P>,
    original_face: FaceKey,
    edge: EdgeKey,
    first: FaceKey,
    second: FaceKey,
    period: f64,
) -> Result<FaceKey, FaceImprintSplitError> {
    let first_dart = face_edge_dart_for_imprint(g, first, edge)?;
    let first_end = g.alpha(Dim::Zero, first_dart);
    let second_dart = g.alpha(Dim::Two, first_end);
    if g.attribute::<Cell2>(second_dart).copied() != Some(second) {
        return Err(FaceImprintSplitError::PeriodicMergeFailed {
            face: original_face,
            reason: "parameter seam does not separate the expected faces",
        });
    }
    let second_end = g.alpha(Dim::Zero, second_dart);
    let first_previous = g.alpha(Dim::One, first_dart);
    let first_next = g.alpha(Dim::One, first_end);
    let second_previous = g.alpha(Dim::One, second_dart);
    let second_next = g.alpha(Dim::One, second_end);

    let first_attr = g
        .remove_face(first)
        .ok_or(FaceImprintSplitError::MissingFace { face: first })?;
    let second_attr = g
        .remove_face(second)
        .ok_or(FaceImprintSplitError::MissingFace { face: second })?;
    let mut pcurves = first_attr.pcurves;
    pcurves.extend(second_attr.pcurves);
    for dart in [first_dart, first_end, second_dart, second_end] {
        pcurves.remove(&dart);
    }
    g.remove_edge(edge)
        .ok_or(FaceImprintSplitError::MissingBoundaryEdge { dart: first_dart })?;

    g.edit(|edit| {
        for dart in [first_dart, first_end, second_dart, second_end] {
            edit.unlink(Dim::One, dart)?;
        }
        edit.sew(Dim::One, first_previous, second_next)?;
        edit.sew(Dim::One, second_previous, first_next)?;

        edit.unlink(Dim::Zero, first_dart)?;
        edit.unlink(Dim::Zero, second_dart)?;
        edit.unlink(Dim::Two, first_dart)?;
        edit.unlink(Dim::Two, first_end)?;
        Ok(())
    })
    .map_err(|source| FaceImprintSplitError::PeriodicTopologyEditFailed {
        face: original_face,
        source,
    })?;

    let mut loop_dart = first_next;
    for _ in 0..2 {
        loop_dart = merge_periodic_boundary_edge(
            g,
            original_face,
            loop_dart,
            &first_attr.surface,
            &mut pcurves,
            period,
        )?
        .ok_or(FaceImprintSplitError::PeriodicMergeFailed {
            face: original_face,
            reason: "periodic boundary was not split at the parameter seam",
        })?;
    }

    Ok(g.add_face(FaceAttr::with_pcurves(
        first_attr.surface,
        first_attr.data,
        loop_dart,
        Vec::new(),
        pcurves,
    )))
}

fn merge_periodic_boundary_edge<P: Payload>(
    g: &mut GMap<P>,
    face: FaceKey,
    loop_dart: Dart,
    surface: &Surface,
    pcurves: &mut HashMap<Dart, Curve2>,
    period: f64,
) -> Result<Option<Dart>, FaceImprintSplitError> {
    g.ensure_profile(loop_dart);
    let edges = Profile::from_dart(g, loop_dart)
        .expect("face loop must have a registered profile")
        .edges();
    let Some((first, second)) = edges
        .iter()
        .zip(edges.iter().cycle().skip(1))
        .take(edges.len())
        .find_map(|(first, second)| {
            let first_pcurve = pcurves.get(&first.dart())?;
            let second_pcurve = pcurves.get(&second.dart())?;
            let first_start = first_pcurve.point_at(0.0);
            let first_end = first_pcurve.point_at(1.0);
            let second_start = second_pcurve.point_at(0.0);
            let second_end = second_pcurve.point_at(1.0);
            ((first_start.y - first_end.y).abs() <= LINEAR_TOLERANCE
                && (second_start.y - second_end.y).abs() <= LINEAR_TOLERANCE
                && (first_end.y - second_start.y).abs() <= LINEAR_TOLERANCE
                && ((first_end.x - second_start.x).abs() - period).abs() <= LINEAR_TOLERANCE)
                .then_some((first.dart(), second.dart()))
        })
    else {
        return Ok(None);
    };

    let first_pcurve = pcurves
        .remove(&first)
        .ok_or(FaceImprintSplitError::MissingPcurve { face, dart: first })?;
    let second_pcurve = pcurves
        .remove(&second)
        .ok_or(FaceImprintSplitError::MissingPcurve { face, dart: second })?;
    let start_uv = first_pcurve.point_at(0.0);
    let seam_uv = first_pcurve.point_at(1.0);
    let mut end_uv = second_pcurve.point_at(1.0);
    if seam_uv.x > start_uv.x {
        while end_uv.x <= start_uv.x {
            end_uv.x += period;
        }
    } else {
        while end_uv.x >= start_uv.x {
            end_uv.x -= period;
        }
    }

    let first_key = boundary_edge_key(g, first)?;
    let second_key = boundary_edge_key(g, second)?;
    let merged_curve = periodic_boundary_curve(face, surface, start_uv, end_uv)?;

    let first_end = g.alpha(Dim::Zero, first);
    let second_end = g.alpha(Dim::Zero, second);
    let vertex = g.cell_representative(first_end, Dim::Zero);
    let vertex_key = g.dart_to_vertex.get(&vertex).copied();
    g.edit(|edit| {
        if let Some(key) = vertex_key {
            edit.remove_vertex(key);
        }
        edit.remove_edge(first_key);
        edit.remove_edge(second_key);
        edit.unlink(Dim::Zero, first)?;
        edit.unlink(Dim::Zero, second)?;
        edit.unlink(Dim::One, first_end)?;
        edit.link(Dim::Zero, first, second_end)?;
        edit.add_edge(EdgeAttr::new(first, merged_curve, P::E::default()));
        Ok(())
    })
    .expect("prepared periodic boundary merge must commit");
    pcurves.insert(first, Curve2::Line(Line2::new(start_uv, end_uv)));

    Ok(Some(if loop_dart == second {
        first
    } else {
        loop_dart
    }))
}

fn periodic_boundary_curve(
    face: FaceKey,
    surface: &Surface,
    start_uv: Point2,
    end_uv: Point2,
) -> Result<Curve, FaceImprintSplitError> {
    let boundary = match surface {
        Surface::Ruled(ruled) => ruled.curve().translated(ruled.direction() * start_uv.y)?,
        Surface::Cylinder(cylinder) => Curve::Circle(Circle::new(
            Plane::new(
                cylinder.origin() + *cylinder.axis() * start_uv.y,
                cylinder.x_dir(),
                cylinder.axis(),
            ),
            cylinder.radius,
        )),
        _ => {
            return Err(FaceImprintSplitError::PeriodicMergeFailed {
                face,
                reason: "periodic boundary curve is not circular",
            });
        }
    };
    let Curve::Circle(circle) = boundary else {
        return Err(FaceImprintSplitError::PeriodicMergeFailed {
            face,
            reason: "periodic boundary curve is not circular",
        });
    };
    Ok(Curve::Nurbs(circle.to_nurbs_between(start_uv.x, end_uv.x)?))
}

fn rebuild_periodic_boundary_curves<P: Payload>(
    g: &mut GMap<P>,
    faces: [FaceKey; 2],
) -> Result<(), FaceImprintSplitError> {
    for face in faces {
        let (surface, pcurves) = {
            let attr = g
                .face_attr(face)
                .ok_or(FaceImprintSplitError::MissingFace { face })?;
            (
                attr.surface.clone(),
                attr.pcurves
                    .iter()
                    .map(|(dart, pcurve)| (*dart, pcurve.clone()))
                    .collect::<Vec<_>>(),
            )
        };
        for (dart, pcurve) in pcurves {
            let start = pcurve.point_at(0.0);
            let end = pcurve.point_at(1.0);
            if (start.y - end.y).abs() > LINEAR_TOLERANCE
                || (start.x - end.x).abs() <= LINEAR_TOLERANCE
            {
                continue;
            }
            let edge = boundary_edge_key(g, dart)?;
            let curve = periodic_boundary_curve(face, &surface, start, end)?;
            g.edge_attr_mut(edge)
                .ok_or(FaceImprintSplitError::MissingBoundaryEdge { dart })?
                .curve = curve;
        }
    }
    Ok(())
}

fn unwrap_periodic_face_pcurves<P: Payload>(
    g: &mut GMap<P>,
    face: FaceKey,
) -> Result<(), FaceImprintSplitError> {
    let (periodicity, pcurves) = {
        let face_view = g
            .face(face)
            .ok_or(FaceImprintSplitError::MissingFace { face })?;
        (
            face_view.surface().periodicity(),
            face_view
                .outer_loop()
                .edges()
                .into_iter()
                .map(|edge| {
                    face_view
                        .pcurve(edge.dart())
                        .map(|pcurve| (edge.dart(), pcurve))
                        .ok_or(FaceImprintSplitError::MissingPcurve {
                            face,
                            dart: edge.dart(),
                        })
                })
                .collect::<Result<Vec<_>, _>>()?,
        )
    };

    let mut previous_end = None;
    let mut unwrapped = Vec::with_capacity(pcurves.len());
    for (dart, pcurve) in pcurves {
        let start = pcurve.point_at(0.0);
        let offset = previous_end
            .map(|end| periodic_offset(periodicity, start, end))
            .unwrap_or_default();
        let pcurve = pcurve.translated(offset)?;
        previous_end = Some(pcurve.point_at(1.0));
        unwrapped.push((dart, pcurve));
    }

    let face_attr = g
        .face_attr_mut(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?;
    for (dart, pcurve) in unwrapped {
        face_attr.pcurves.insert(dart, pcurve);
    }
    Ok(())
}

fn periodic_offset(periodicity: SurfacePeriodicity, start: Point2, target: Point2) -> Vector2<f64> {
    let mut offset = Vector2::zeros();
    match periodicity {
        SurfacePeriodicity::UPeriodic(period) => {
            offset.x = ((target.x - start.x) / period).round() * period;
        }
        SurfacePeriodicity::VPeriodic(period) => {
            offset.y = ((target.y - start.y) / period).round() * period;
        }
        SurfacePeriodicity::UVPeriodic(u_period, v_period) => {
            offset.x = ((target.x - start.x) / u_period).round() * u_period;
            offset.y = ((target.y - start.y) / v_period).round() * v_period;
        }
        SurfacePeriodicity::None => {}
    }
    offset
}

fn face_edge_dart_for_imprint<P: Payload>(
    g: &GMap<P>,
    face: FaceKey,
    edge: EdgeKey,
) -> Result<Dart, FaceImprintSplitError> {
    let face_view = g
        .face(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?;
    let profile_darts: Vec<Dart> = face_view.outer_loop().darts().step_by(2).collect();
    profile_darts
        .into_iter()
        .find(|profile_dart| g.cell_key::<Cell1>(*profile_dart) == Some(edge))
        .ok_or(FaceImprintSplitError::MissingBoundaryEdge {
            dart: face_view.outer_loop().dart,
        })
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

fn add_closed_curve_imprint_loops<P: Payload>(
    g: &mut GMap<P>,
    face: FaceKey,
    imprints: &[&FaceImprint],
) -> Result<Vec<FaceImprintSplit>, FaceImprintSplitError> {
    let boundary_uvs = face_boundary_uvs(g, face)?;
    let boundary_area = signed_area(&boundary_uvs);
    let mut splits = Vec::new();

    for imprint in imprints {
        let samples = imprint
            .pcurve
            .adaptive_samples(LINEAR_TOLERANCE, 16)
            .into_iter()
            .map(|(_, point)| point)
            .collect::<Vec<_>>();
        if samples.len() < 4
            || samples
                .iter()
                .any(|point| snap_boundary_corner(&boundary_uvs, *point).is_some())
        {
            continue;
        }

        let imprint_area = signed_area(&samples[..samples.len() - 1]);
        let outside = if boundary_area.signum() == imprint_area.signum() {
            reverse_imprint(imprint)?
        } else {
            (*imprint).clone()
        };
        splits.push(split_face_by_closed_curve_imprint(g, face, &outside)?);
    }
    Ok(splits)
}

fn reverse_imprint(imprint: &FaceImprint) -> Result<FaceImprint, NurbsError> {
    imprint.reversed()
}

fn split_face_by_closed_curve_imprint<P: Payload>(
    g: &mut GMap<P>,
    face: FaceKey,
    imprint: &FaceImprint,
) -> Result<FaceImprintSplit, FaceImprintSplitError> {
    let old_face = g
        .face_attr(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?
        .clone();
    let outside_loop = add_imprint_section_loop(g, &old_face.surface, imprint);
    let island_loop = add_imprint_section_loop(g, &old_face.surface, &reverse_imprint(imprint)?);
    finish_closed_imprint_split(g, face, old_face, outside_loop, island_loop)
}

fn add_closed_imprint_loops<P: Payload>(
    g: &mut GMap<P>,
    face: FaceKey,
    graph: &FaceImprintGraph,
    imprints: &[FaceImprint],
) -> Result<Vec<FaceImprintSplit>, FaceImprintSplitError> {
    let boundary_uvs = face_boundary_uvs(g, face)?;
    let mut splits = Vec::new();

    for component in graph.closed_edge_components() {
        let mut loop_imprints = component
            .iter()
            .map(|oriented| {
                let edge = &graph.edges[oriented.edge];
                let imprint = imprints[edge.source_curve].trimmed(edge.interval)?;
                if oriented.reversed {
                    imprint.reversed()
                } else {
                    Ok(imprint)
                }
            })
            .collect::<Result<Vec<_>, NurbsError>>()?;
        let uvs = loop_imprints
            .iter()
            .map(|imprint| imprint.pcurve.point_at(0.0))
            .collect::<Vec<_>>();
        if uvs.len() < 3
            || uvs
                .iter()
                .any(|uv| snap_boundary_corner(&boundary_uvs, *uv).is_some())
        {
            continue;
        }

        orient_imprint_loop_against_boundary(&boundary_uvs, &mut loop_imprints)?;
        splits.push(split_face_by_closed_imprint_loop(g, face, &loop_imprints)?);
    }

    Ok(splits)
}

fn split_face_by_closed_imprint_loop<P: Payload>(
    g: &mut GMap<P>,
    face: FaceKey,
    imprints: &[FaceImprint],
) -> Result<FaceImprintSplit, FaceImprintSplitError> {
    let old_face = g
        .face_attr(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?
        .clone();
    let island_imprints = reversed_imprint_loop(imprints)?;

    let outside_loop = add_section_loop(g, &old_face.surface, imprints);
    let island_loop = add_section_loop(g, &old_face.surface, &island_imprints);
    finish_closed_imprint_split(g, face, old_face, outside_loop, island_loop)
}

fn finish_closed_imprint_split<P: Payload>(
    g: &mut GMap<P>,
    face: FaceKey,
    old_face: FaceAttr<P::F>,
    outside_loop: SectionLoop,
    island_loop: SectionLoop,
) -> Result<FaceImprintSplit, FaceImprintSplitError> {
    let section_edges = sew_section_loops(g, face, &outside_loop, &island_loop)?;
    g.ensure_profile(outside_loop.loop_dart);
    g.ensure_profile(island_loop.loop_dart);

    let face_attr = g
        .face_attr_mut(face)
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

struct SectionLoop {
    loop_dart: Dart,
    edges: Vec<SectionLoopEdge>,
    pcurves: HashMap<Dart, Curve2>,
}

#[derive(Clone)]
struct SectionLoopEdge {
    dart: Dart,
    start_uv: Point2,
    end_uv: Point2,
    curve: Curve,
    pcurve: Curve2,
}

fn add_section_loop<P: Payload>(
    g: &mut GMap<P>,
    surface: &Surface,
    imprints: &[FaceImprint],
) -> SectionLoop {
    let n = imprints.len();
    g.edit(|edit| {
        let darts = (0..2 * n).map(|_| edit.add_dart()).collect::<Vec<_>>();

        for edge in 0..n {
            edit.link(Dim::Zero, darts[2 * edge], darts[2 * edge + 1])?;
        }
        for edge in 0..n {
            let end = darts[2 * edge + 1];
            let next_start = darts[2 * ((edge + 1) % n)];
            edit.link(Dim::One, end, next_start)?;
        }

        for vertex in 0..n {
            let dart = edit.cell_representative(darts[2 * vertex], Dim::Zero);
            let uv = imprints[vertex].pcurve.point_at(0.0);
            edit.add_vertex(VertexAttr::new(
                dart,
                surface.point_at(uv.x, uv.y),
                P::V::default(),
            ));
        }

        let edges = (0..n)
            .map(|edge| {
                let imprint = &imprints[edge];
                SectionLoopEdge {
                    dart: darts[2 * edge],
                    start_uv: imprint.pcurve.point_at(0.0),
                    end_uv: imprint.pcurve.point_at(1.0),
                    curve: imprint.curve.clone(),
                    pcurve: imprint.pcurve.clone(),
                }
            })
            .collect::<Vec<_>>();
        let pcurves = edges
            .iter()
            .map(|edge| (edge.dart, edge.pcurve.clone()))
            .collect();

        Ok(SectionLoop {
            loop_dart: darts[0],
            edges,
            pcurves,
        })
    })
    .expect("fresh section loop topology must commit")
}

fn add_imprint_section_loop<P: Payload>(
    g: &mut GMap<P>,
    surface: &Surface,
    imprint: &FaceImprint,
) -> SectionLoop {
    add_section_loop(g, surface, std::slice::from_ref(imprint))
}

fn sew_section_loops<P: Payload>(
    g: &mut GMap<P>,
    face: FaceKey,
    outside: &SectionLoop,
    island: &SectionLoop,
) -> Result<Vec<EdgeKey>, FaceImprintSplitError> {
    let pairs = outside
        .edges
        .iter()
        .map(|outside_edge| {
            let island_edge = matching_reversed_loop_edge(outside_edge, &island.edges).ok_or(
                FaceImprintSplitError::MissingPcurve {
                    face,
                    dart: outside_edge.dart,
                },
            )?;
            Ok((outside_edge, g.alpha(Dim::Zero, island_edge.dart)))
        })
        .collect::<Result<Vec<_>, FaceImprintSplitError>>()?;
    g.edit(|edit| {
        let mut edges = Vec::with_capacity(pairs.len());
        for (outside_edge, island_end) in pairs {
            edit.sew(Dim::Two, outside_edge.dart, island_end)?;
            edges.push(edit.add_edge(EdgeAttr::new(
                outside_edge.dart,
                outside_edge.curve.clone(),
                P::E::default(),
            )));
        }
        Ok(edges)
    })
    .map_err(|source| FaceImprintSplitError::SectionLoopSewFailed { face, source })
}

fn matching_reversed_loop_edge(
    edge: &SectionLoopEdge,
    candidates: &[SectionLoopEdge],
) -> Option<SectionLoopEdge> {
    candidates
        .iter()
        .find(|candidate| {
            (candidate.start_uv - edge.end_uv).norm() <= LINEAR_TOLERANCE
                && (candidate.end_uv - edge.start_uv).norm() <= LINEAR_TOLERANCE
        })
        .cloned()
}

fn orient_imprint_loop_against_boundary(
    boundary_uvs: &[Point2],
    imprints: &mut Vec<FaceImprint>,
) -> Result<(), NurbsError> {
    let boundary_area = signed_area(boundary_uvs);
    let loop_uvs = imprints
        .iter()
        .map(|imprint| imprint.pcurve.point_at(0.0))
        .collect::<Vec<_>>();
    let loop_area = signed_area(&loop_uvs);

    if boundary_area.abs() <= LINEAR_TOLERANCE || loop_area.abs() <= LINEAR_TOLERANCE {
        return Ok(());
    }

    if boundary_area.signum() == loop_area.signum() {
        *imprints = reversed_imprint_loop(imprints)?;
    }
    Ok(())
}

fn reversed_imprint_loop(imprints: &[FaceImprint]) -> Result<Vec<FaceImprint>, NurbsError> {
    imprints.iter().rev().map(FaceImprint::reversed).collect()
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
    let boundary_uvs = face_boundary_uvs(g, face)?;
    if snap_boundary_corner(&boundary_uvs, uv).is_some() {
        return Ok(());
    }

    let Some(target) = boundary_edge_at_uv(g, face, uv)? else {
        return Ok(());
    };

    let edge = Edge::new(g, target.edge);
    let curve = edge_curve(&edge)?;
    let face_view = g
        .face(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?;
    let surface = face_view.surface();
    let mut parameter = curve.param_at(surface.point_at(uv.x, uv.y));
    if let Periodicity::Periodic(period) = curve.periodicity() {
        let domain = curve
            .parameters_between(vertex_point(edge.start())?, vertex_point(edge.end())?)
            .ordered();
        while parameter < domain.start - LINEAR_TOLERANCE {
            parameter += period;
        }
        while parameter > domain.end + LINEAR_TOLERANCE {
            parameter -= period;
        }
    }
    split_face_edge(g, face, target.edge, parameter)?;
    Ok(())
}

fn split_one_face_by_imprints<P: Payload>(
    g: &mut GMap<P>,
    face: FaceKey,
    imprints: &[FaceImprint],
) -> Result<Option<FaceImprintSplit>, FaceImprintSplitError> {
    let face_attr = g
        .face_attr(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?;

    let boundary_uvs = face_boundary_uvs(g, face)?;
    let Some(cut) = imprints
        .iter()
        .find_map(|imprint| FaceImprintCut::from_imprint(imprint, &boundary_uvs))
    else {
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
    let edge = add_circle_edge(g, plane.clone(), radius)?;
    let loop_dart = g.edge_attr_unchecked(edge).dart;
    g.ensure_profile(loop_dart);
    let profile =
        Profile::from_dart(g, loop_dart).expect("face loop must have a registered profile");
    let pcurves = profile_pcurves(&profile, &plane)?;
    let face_key = g.add_face(FaceAttr::with_pcurves(
        Surface::Plane(plane),
        (),
        loop_dart,
        Vec::new(),
        pcurves,
    ));
    Ok(face_key)
}

#[derive(Clone)]
struct FaceImprintCut {
    start_corner: usize,
    end_corner: usize,
    curve: Curve,
    pcurve: Curve2,
}

impl FaceImprintCut {
    fn from_imprint(imprint: &FaceImprint, boundary_uvs: &[Point2]) -> Option<Self> {
        let pcurve = &imprint.pcurve;
        let start_uv = pcurve.point_at(0.0);
        let end_uv = pcurve.point_at(1.0);
        let start = snap_boundary_corner(boundary_uvs, start_uv)?;
        let end = snap_boundary_corner(boundary_uvs, end_uv)?;
        if !valid_chord(start, end, boundary_uvs.len()) {
            return None;
        }

        Some(Self {
            start_corner: start,
            end_corner: end,
            curve: imprint.curve.clone(),
            pcurve: pcurve.clone(),
        })
    }
}

#[derive(Debug, Clone, Copy)]
struct BoundaryEdgeTarget {
    dart: Dart,
    edge: EdgeKey,
}

fn face_boundary_uvs<P: Payload>(
    g: &GMap<P>,
    face: FaceKey,
) -> Result<Vec<Point2>, FaceImprintSplitError> {
    let face_view = g
        .face(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?;

    face_view
        .outer_loop()
        .corners()
        .iter()
        .map(|corner| {
            let dart = corner.outgoing().dart();
            face_view
                .pcurve(dart)
                .map(|pcurve| pcurve.point_at(0.0))
                .ok_or(FaceImprintSplitError::MissingPcurve { face, dart })
        })
        .collect()
}

fn boundary_edge_at_uv<P: Payload>(
    g: &GMap<P>,
    face: FaceKey,
    uv: Point2,
) -> Result<Option<BoundaryEdgeTarget>, FaceImprintSplitError> {
    let face_view = g
        .face(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?;

    for edge in face_view.outer_loop().edges() {
        let pcurve = face_view
            .pcurve(edge.dart())
            .ok_or(FaceImprintSplitError::MissingPcurve {
                face,
                dart: edge.dart(),
            })?;
        let Some(fraction) = pcurve_fraction_at(&pcurve, uv) else {
            continue;
        };
        if fraction <= LINEAR_TOLERANCE || 1.0 - fraction <= LINEAR_TOLERANCE {
            continue;
        }

        return Ok(Some(BoundaryEdgeTarget {
            dart: edge.dart(),
            edge: boundary_edge_key(g, edge.dart())?,
        }));
    }

    Ok(None)
}

fn pcurve_fraction_at(pcurve: &Curve2, point: Point2) -> Option<f64> {
    pcurve.parameter_at(point, LINEAR_TOLERANCE)
}

fn boundary_edge_key<P: Payload>(
    g: &GMap<P>,
    dart: Dart,
) -> Result<EdgeKey, FaceImprintSplitError> {
    g.cell_key::<Cell1>(dart)
        .ok_or(FaceImprintSplitError::MissingBoundaryEdge { dart })
}

fn snap_boundary_corner(boundary_uvs: &[Point2], uv: Point2) -> Option<usize> {
    boundary_uvs
        .iter()
        .enumerate()
        .filter_map(|(index, boundary_uv)| {
            let distance = (*boundary_uv - uv).norm();
            (distance <= LINEAR_TOLERANCE).then_some((distance, index))
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .map(|(_, index)| index)
}

fn valid_chord(start: usize, end: usize, corner_count: usize) -> bool {
    if corner_count < 3 || start == end {
        return false;
    }

    let distance = start.abs_diff(end);
    distance > 1 && distance < corner_count - 1
}

fn apply_outer_face_chord_split<P: Payload>(
    g: &mut GMap<P>,
    original_face: FaceKey,
    old_face: FaceAttr<P::F>,
    cut: &FaceImprintCut,
) -> Result<FaceImprintSplit, FaceImprintSplitError> {
    let loop_ = Closed::new_unchecked(
        Profile::from_dart(g, old_face.outer_loop)
            .expect("face loop must have a registered profile"),
    );
    let corners = loop_.corners();
    let start = &corners[cut.start_corner];
    let end = &corners[cut.end_corner];
    let start_dart = start.outgoing().dart();
    let end_dart = end.outgoing().dart();
    let start_previous_end = start.incoming().end().dart;
    let end_previous_end = end.incoming().end().dart;
    let pcurve_ab = cut.pcurve.clone();
    let pcurve_ba = pcurve_ab.reversed();
    let (ab_start, ba_start) = g
        .edit(|edit| {
            let ab_start = edit.add_dart();
            let ab_end = edit.add_dart();
            let ba_start = edit.add_dart();
            let ba_end = edit.add_dart();

            edit.link(Dim::Zero, ab_start, ab_end)
                .expect("fresh section edge darts must be alpha0-free");
            edit.link(Dim::Zero, ba_start, ba_end)
                .expect("fresh section edge darts must be alpha0-free");
            edit.link(Dim::Two, ab_start, ba_end)
                .expect("fresh section sides must be alpha2-free");
            edit.link(Dim::Two, ab_end, ba_start)
                .expect("fresh section sides must be alpha2-free");

            edit.unlink(Dim::One, start_previous_end)
                .expect("split start corner must be alpha1-linked");
            edit.unlink(Dim::One, end_previous_end)
                .expect("split end corner must be alpha1-linked");
            edit.link(Dim::One, start_previous_end, ab_start)
                .expect("split start must be alpha1-free after unlink");
            edit.link(Dim::One, ab_end, end_dart)
                .expect("section endpoint must be alpha1-free");
            edit.link(Dim::One, end_previous_end, ba_start)
                .expect("split end must be alpha1-free after unlink");
            edit.link(Dim::One, ba_end, start_dart)
                .expect("section endpoint must be alpha1-free");
            Ok((ab_start, ba_start))
        })
        .expect("prepared face chord split must commit");

    let section_edge = g.add_edge(EdgeAttr::new(ab_start, cut.curve.clone(), P::E::default()));
    let first_pcurves = split_face_pcurves(
        g,
        original_face,
        &old_face.pcurves,
        start_dart,
        ba_start,
        &pcurve_ba,
    )?;
    let second_pcurves = split_face_pcurves(
        g,
        original_face,
        &old_face.pcurves,
        end_dart,
        ab_start,
        &pcurve_ab,
    )?;
    let first = g.add_face(FaceAttr::with_pcurves(
        old_face.surface.clone(),
        old_face.data.clone(),
        start_dart,
        Vec::new(),
        first_pcurves,
    ));
    let second = g.add_face(FaceAttr::with_pcurves(
        old_face.surface,
        old_face.data,
        end_dart,
        Vec::new(),
        second_pcurves,
    ));

    Ok(FaceImprintSplit {
        first,
        second,
        section_edges: vec![section_edge],
    })
}

fn split_face_pcurves<P: Payload>(
    g: &mut GMap<P>,
    face: FaceKey,
    old_pcurves: &HashMap<Dart, Curve2>,
    loop_dart: Dart,
    section_dart: Dart,
    section_pcurve: &Curve2,
) -> Result<HashMap<Dart, Curve2>, FaceImprintSplitError> {
    let mut pcurves = HashMap::new();
    g.ensure_profile(loop_dart);
    let profile =
        Profile::from_dart(g, loop_dart).expect("face loop must have a registered profile");
    for profile_dart in profile.darts().step_by(2) {
        let pcurve = if profile_dart == section_dart {
            section_pcurve.clone()
        } else {
            let candidates = [
                profile_dart,
                g.alpha(Dim::Zero, profile_dart),
                g.alpha(Dim::Two, profile_dart),
            ];
            candidates
                .iter()
                .find_map(|&d| old_pcurves.get(&d))
                .cloned()
                .ok_or(FaceImprintSplitError::MissingPcurve {
                    face,
                    dart: profile_dart,
                })?
        };
        pcurves.insert(profile_dart, pcurve);
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
    let outer_edge = add_circle_edge(g, plane.clone(), outer_radius)?;
    let inner_edge = add_circle_edge(g, inner_plane, inner_radius)?;
    let outer_loop = g.edge_attr_unchecked(outer_edge).dart;
    let inner_loop = g.edge_attr_unchecked(inner_edge).dart;
    g.ensure_profile(outer_loop);
    g.ensure_profile(inner_loop);

    let outer_profile =
        Profile::from_dart(g, outer_loop).expect("outer loop must have a registered profile");
    let inner_profile =
        Profile::from_dart(g, inner_loop).expect("inner loop must have a registered profile");
    let mut pcurves = profile_pcurves(&outer_profile, &plane)?;
    pcurves.extend(profile_pcurves(&inner_profile, &plane)?);

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
        .face_attr(face)
        .ok_or(FaceEdgeSplitError::MissingFace { face })?;
    let edge_attr = g
        .edge_attr(edge)
        .ok_or(FaceEdgeSplitError::EdgeSplitFailed(
            EdgeSplitError::MissingEdge { edge },
        ))?;
    let edge_dart = g.cell_representative(edge_attr.dart, Dim::One);
    let profile_darts: Vec<Dart> = std::iter::once(face_attr.outer_loop)
        .chain(face_attr.inner_loops.iter().copied())
        .flat_map(|loop_dart| {
            Profile::from_dart(g, loop_dart)
                .expect("face loop must have a registered profile")
                .darts()
                .step_by(2)
                .collect::<Vec<_>>()
        })
        .collect();
    profile_darts
        .into_iter()
        .find(|profile_dart| g.cell_representative(*profile_dart, Dim::One) == edge_dart)
        .ok_or(FaceEdgeSplitError::EdgeNotOnFace { face, edge })
}

fn face_pcurve<P: Payload>(
    g: &GMap<P>,
    face: FaceKey,
    dart: Dart,
) -> Result<Curve2, FaceEdgeSplitError> {
    let face_attr = g
        .face_attr(face)
        .ok_or(FaceEdgeSplitError::MissingFace { face })?;
    face_attr
        .pcurves
        .get(&dart)
        .cloned()
        .ok_or(FaceEdgeSplitError::MissingPcurve { face, dart })
}

fn closed_boundary_curve_reversed<P: Payload>(
    g: &GMap<P>,
    face: FaceKey,
    edge: EdgeKey,
    dart: Dart,
) -> Result<bool, FaceEdgeSplitError> {
    let edge_view =
        Edge::from_dart(g, dart).ok_or(FaceEdgeSplitError::EdgeNotOnFace { face, edge })?;
    let start = vertex_point(edge_view.start())?;
    let end = vertex_point(edge_view.end())?;
    if (start - end).norm() > LINEAR_TOLERANCE {
        return Ok(false);
    }

    let face_view = g
        .face(face)
        .ok_or(FaceEdgeSplitError::MissingFace { face })?;
    let pcurve = face_view
        .pcurve(dart)
        .ok_or(FaceEdgeSplitError::MissingPcurve { face, dart })?;
    let curve = g
        .edge_attr(edge)
        .ok_or(FaceEdgeSplitError::EdgeSplitFailed(
            EdgeSplitError::MissingEdge { edge },
        ))?
        .curve
        .to_nurbs()?;
    let domain = curve.domain();
    let fraction = 1.0e-4;
    let sample_uv = pcurve.point_at(fraction);
    let sample = face_view.point_at(sample_uv.x, sample_uv.y);
    let forward = curve.point_at(domain.start + domain.length() * fraction);
    let reverse = curve.point_at(domain.end - domain.length() * fraction);
    Ok((sample - reverse).norm_squared() < (sample - forward).norm_squared())
}

fn incident_face_pcurves<P: Payload>(
    g: &GMap<P>,
    edge: EdgeKey,
    parameter: f64,
) -> Result<Vec<IncidentFacePcurve>, FaceEdgeSplitError> {
    let edge_attr = g
        .edge_attr(edge)
        .ok_or(FaceEdgeSplitError::EdgeSplitFailed(
            EdgeSplitError::MissingEdge { edge },
        ))?;
    let split_point = edge_attr.curve.point_at(parameter);
    let mut seen = HashSet::new();
    g.orbit(edge_attr.dart, g.orbit_indices(Dim::One))
        .filter_map(|dart| g.attribute::<Cell2>(dart).copied())
        .filter(|face| seen.insert(*face))
        .map(|face| {
            let dart = face_edge_dart(g, face, edge)?;
            let pcurve = face_pcurve(g, face, dart)?;
            let face_view = g
                .face(face)
                .ok_or(FaceEdgeSplitError::MissingFace { face })?;
            let surface = face_view.surface();
            let uv = periodic_image_near_pcurve(
                surface,
                &pcurve,
                surface.closest_parameter(split_point)?,
            );
            let fraction = pcurve
                .parameter_at(uv, LINEAR_TOLERANCE)
                .ok_or(FaceEdgeSplitError::SplitPointNotOnPcurve { face, dart })?;
            Ok(IncidentFacePcurve {
                face,
                dart,
                pcurve,
                fraction,
            })
        })
        .collect()
}

fn periodic_image_near_pcurve(surface: &Surface, pcurve: &Curve2, mut uv: Point2) -> Point2 {
    let start = pcurve.point_at(0.0);
    let end = pcurve.point_at(1.0);
    let center = Point2::from((start.coords + end.coords) * 0.5);
    match surface.periodicity() {
        SurfacePeriodicity::UPeriodic(period) => {
            uv.x += ((center.x - uv.x) / period).round() * period;
        }
        SurfacePeriodicity::VPeriodic(period) => {
            uv.y += ((center.y - uv.y) / period).round() * period;
        }
        SurfacePeriodicity::UVPeriodic(u_period, v_period) => {
            uv.x += ((center.x - uv.x) / u_period).round() * u_period;
            uv.y += ((center.y - uv.y) / v_period).round() * v_period;
        }
        SurfacePeriodicity::None => {}
    }
    uv
}

fn assign_split_pcurves<P: Payload>(
    g: &mut GMap<P>,
    pcurve: IncidentFacePcurve,
) -> Result<(), FaceEdgeSplitError> {
    let second_dart = g.alpha(Dim::One, g.alpha(Dim::Zero, pcurve.dart));
    let (first_pcurve, second_pcurve) = pcurve.pcurve.split_at(pcurve.fraction)?;
    let face_attr = g
        .face_attr_mut(pcurve.face)
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

    let outer_profile = add_polygon(g, outer);
    let outer_loop = g.profile_attr_unchecked(outer_profile).dart;
    let mut inner_loops = Vec::with_capacity(holes.len());
    let outer_profile =
        Profile::from_dart(g, outer_loop).expect("outer loop must have a registered profile");
    let mut pcurves = profile_pcurves(&outer_profile, &plane)?;

    for hole in holes {
        let inner_profile = add_polygon(g, hole);
        let inner_loop = g.profile_attr_unchecked(inner_profile).dart;
        let inner_profile =
            Profile::from_dart(g, inner_loop).expect("inner loop must have a registered profile");
        pcurves.extend(profile_pcurves(&inner_profile, &plane)?);
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
/// Returns the profile key whose stored dart defines the polygon's orientation.
pub fn add_polygon<P: Payload>(
    g: &mut GMap<P>,
    corners: &[Point3],
) -> crate::topology::shape_keys::ProfileKey {
    assert!(
        corners.len() >= 3,
        "add_polygon requires at least 3 corners, got {}",
        corners.len()
    );
    let n = corners.len();
    g.edit(|edit| {
        let darts: Vec<Dart> = (0..2 * n).map(|_| edit.add_dart()).collect();

        for i in 0..n {
            edit.sew(Dim::Zero, darts[2 * i], darts[2 * i + 1])?;
        }
        for i in 0..n {
            let a = darts[2 * i + 1];
            let b = darts[(2 * i + 2) % (2 * n)];
            edit.sew(Dim::One, a, b)?;
        }

        for i in 0..n {
            let dart = edit.cell_representative(darts[2 * i], Dim::Zero);
            edit.add_vertex(VertexAttr::new(dart, corners[i], P::V::default()));
        }

        for i in 0..n {
            let edge_dart = darts[2 * i];
            let curve = Curve::line(corners[i], corners[(i + 1) % n]);
            edit.add_edge(EdgeAttr::new(edge_dart, curve, P::E::default()));
        }
        Ok(
            edit.add_profile(crate::topology::attributes::ProfileAttr::new(
                darts[0],
                P::Profile::default(),
            )),
        )
    })
    .expect("fresh polygon topology must commit")
}
