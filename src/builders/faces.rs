use std::collections::{HashMap, HashSet};

use crate::StandardPayload;
use crate::builders::edges::{
    EdgeSplit, EdgeSplitError, add_circle_staged as add_circle_edge_staged,
    split_face_boundary_edge,
};
use crate::builders::errors::{FaceCreationError, TopologyEditFailure};
use crate::builders::profiles::{
    add_rectangle_staged as add_rectangle_profile_staged, profile_pcurves,
};
use crate::geometry::{
    Circle, Curve, Curve2, CurveCurveIntersection2, CurveIntersectionError, Interval,
    LINEAR_TOLERANCE, Line2, NurbsError, Periodicity, Plane, Point2, Point3, Surface,
    SurfacePeriodicity,
};
use crate::topology::attributes::{EdgeAttr, FaceAttr, ProfileAttr, VertexAttr};
use crate::topology::closed::Closed;
use crate::topology::edge::Edge;
use crate::topology::gmap::{Cell0, Cell1, Cell2, Dart, Dim, GMap};
use crate::topology::payload::Payload;
use crate::topology::planar::Planar;
use crate::topology::profile::Profile;
use crate::topology::shape_keys::{EdgeKey, FaceKey, ProfileKey};
use crate::topology::vertex::Vertex;
use crate::topology::{TopologyEdit, TopologyEditError};
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
    #[error("face edge topology edit failed")]
    TopologyEditFailed(#[source] TopologyEditFailure),
}

impl From<TopologyEditError> for FaceEdgeSplitError {
    fn from(error: TopologyEditError) -> Self {
        Self::TopologyEditFailed(TopologyEditFailure::new(error))
    }
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
    #[error("face imprint topology edit failed")]
    TopologyEditFailed(#[from] TopologyEditError),
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

/// A section edge and its directed interval on the original input imprint.
#[derive(Debug, Clone, PartialEq)]
pub struct FaceImprintSection {
    pub edge: EdgeKey,
    /// Index in the input slice passed to the splitter.
    pub imprint: usize,
    /// Source parameters at the start and end of the stored edge.
    pub interval: Interval,
}

#[derive(Debug, Clone, PartialEq)]
pub struct FaceImprintSplit {
    pub first: FaceKey,
    pub second: FaceKey,
    pub sections: Vec<FaceImprintSection>,
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
        let mut edges: Vec<FaceImprintGraphEdge> = Vec::new();

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

                let mut duplicate = false;
                for existing in &edges {
                    if ordered_edge_key(start, end)
                        != ordered_edge_key(existing.start, existing.end)
                    {
                        continue;
                    }
                    duplicate |= curve.intersect_curve(&curves[existing.source_curve])?.iter().any(|hit| {
                        matches!(hit, CurveCurveIntersection2::Overlap { interval_a, interval_b }
                            if interval_a.ordered().contains(pair[0], LINEAR_TOLERANCE)
                                && interval_a.ordered().contains(pair[1], LINEAR_TOLERANCE)
                                && interval_b.ordered().contains(existing.interval.start, LINEAR_TOLERANCE)
                                && interval_b.ordered().contains(existing.interval.end, LINEAR_TOLERANCE))
                    });
                }
                if !duplicate {
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
            if component.len() < 2
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
            if component.len() >= 2
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

/// Adds a planar face bounded by an existing profile loop.
///
/// The profile must be closed and planar. Its plane becomes the supporting
/// surface, and a pcurve is generated for every oriented boundary edge. The
/// existing profile topology is reused as the face's outer loop.
///
/// # Panics
///
/// Panics if `profile` does not identify a registered profile.
pub fn add_face<P: Payload>(
    g: &mut GMap<P>,
    profile: ProfileKey,
) -> Result<FaceKey, FaceCreationError> {
    g.transaction(|g| add_face_staged(g, profile))
}

pub(crate) fn add_face_staged<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    profile: ProfileKey,
) -> Result<FaceKey, FaceCreationError> {
    let (loop_dart, plane, pcurves) = {
        let profile = g.profile_unchecked(profile);
        let loop_dart = profile.dart;
        let closed =
            Closed::new(profile).ok_or(FaceCreationError::OpenProfile { dart: loop_dart })?;
        let planar = Planar::new(closed)?;
        let (closed, plane) = planar.into_parts();
        let pcurves = profile_pcurves(closed.inner(), &plane)?;
        (loop_dart, plane, pcurves)
    };

    Ok(g.add_face(FaceAttr::with_pcurves(
        Surface::Plane(plane),
        P::F::default(),
        loop_dart,
        Vec::new(),
        pcurves,
    )))
}

/// Adds a planar rectangular face whose first corner is `plane.origin()`.
///
/// The sides follow the plane's positive x and y directions and have lengths
/// `x_size` and `y_size`. Both sizes must be positive and finite.
pub fn add_rectangle(
    g: &mut GMap<StandardPayload>,
    plane: Plane,
    x_size: f64,
    y_size: f64,
) -> Result<FaceKey, FaceCreationError> {
    g.transaction(|g| {
        let profile = add_rectangle_profile_staged(g, plane, x_size, y_size)?;
        add_face_staged(g, profile)
    })
}

/// Adds a planar square face whose first corner is `plane.origin()`.
///
/// The sides follow the plane's positive x and y directions. `size` must be
/// positive and finite.
pub fn add_square(
    g: &mut GMap<StandardPayload>,
    plane: Plane,
    size: f64,
) -> Result<FaceKey, FaceCreationError> {
    g.transaction(|g| {
        let profile = add_rectangle_profile_staged(g, plane, size, size)?;
        add_face_staged(g, profile)
    })
}

/// Splits a face-boundary edge and all of its incident face pcurves.
///
/// `parameter` is interpreted in the stored 3D curve's parameter domain. The
/// split is applied across the full topological edge, so pcurves on neighboring
/// faces sharing that edge are split at the corresponding surface points too.
/// The returned [`EdgeSplit`] identifies both resulting edges and the inserted
/// vertex; the original edge key is retained by the first segment.
pub fn split_face_edge<P: Payload>(
    g: &mut GMap<P>,
    face: FaceKey,
    edge: EdgeKey,
    parameter: f64,
) -> Result<EdgeSplit, FaceEdgeSplitError> {
    g.transaction(|g| split_face_edge_staged(g, face, edge, parameter))
}

/// Splits topology and all incident face pcurves in the same transaction.
pub(crate) fn split_face_edge_staged<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
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

/// Subdivides a face with model-space curves paired with face pcurves.
///
/// Imprint endpoints on boundary-edge interiors are inserted before paving the
/// face. Open boundary-to-boundary imprints form section edges, while closed
/// imprints form interior loops and separate faces. Intersecting pcurves are
/// normalized through [`FaceImprintGraph`] so coincident fragments are not
/// inserted twice.
///
/// Returns one [`FaceImprintSplit`] for each subdivision that was applied.
/// Imprints that do not define an applicable cut may produce no split rather
/// than an error; invalid topology or missing geometry is reported as an error.
pub fn split_face_by_imprints<P: Payload>(
    g: &mut GMap<P>,
    face: FaceKey,
    imprints: &[FaceImprint],
) -> Result<Vec<FaceImprintSplit>, FaceImprintSplitError> {
    g.transaction(|g| split_face_by_imprints_staged(g, face, imprints))
}

/// Applies every open and closed imprint before the outer transaction commits.
pub fn split_face_by_imprints_staged<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    face: FaceKey,
    imprints: &[FaceImprint],
) -> Result<Vec<FaceImprintSplit>, FaceImprintSplitError> {
    let (closed_indices, open_indices): (Vec<_>, Vec<_>) =
        (0..imprints.len()).partition(|&index| imprints[index].pcurve.is_closed());
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
    remap_section_indices(&mut splits, &closed_indices);
    let mut open_splits = add_closed_imprint_loops(g, face, &graph, &open_imprints)?;
    open_splits.extend(split_open_imprints(g, vec![face], &open_imprints)?);
    remap_section_indices(&mut open_splits, &open_indices);
    splits.extend(open_splits);
    Ok(splits)
}

/// Restores original input indices after partitioning closed and open curves.
fn remap_section_indices(splits: &mut [FaceImprintSplit], indices: &[usize]) {
    for section in splits.iter_mut().flat_map(|split| &mut split.sections) {
        section.imprint = indices[section.imprint];
    }
}

fn split_open_imprints<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
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
    g: &mut TopologyEdit<'_, P>,
    face: FaceKey,
    imprints: &[FaceImprint],
    seam: EdgeKey,
    period: f64,
) -> Result<Vec<FaceImprintSplit>, FaceImprintSplitError> {
    split_imprint_boundary_endpoints(g, face, imprints)?;
    let splits = split_open_imprints(g, vec![face], imprints)?;
    let sections = splits
        .iter()
        .flat_map(|split| split.sections.iter().cloned())
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

    let remaining = g
        .iter_faces()
        .map(|(key, _)| key)
        .find(|key| !seam_faces.contains(key))
        .ok_or(FaceImprintSplitError::UnexpectedPeriodicRegionCount {
            face,
            count: g.iter_faces().count(),
        })?;
    let merged = merge_faces_across_edge(g, face, seam, seam_faces[0], seam_faces[1], period)?;
    unwrap_periodic_face_pcurves(g, remaining)?;
    unwrap_periodic_face_pcurves(g, merged)?;
    rebuild_periodic_boundary_curves(g, [remaining, merged])?;

    let (first, second) = if remaining == face {
        (remaining, merged)
    } else {
        debug_assert_eq!(merged, face);
        (merged, remaining)
    };
    Ok(vec![FaceImprintSplit {
        first,
        second,
        sections,
    }])
}

fn merge_faces_across_edge<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
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
        .face_attr(first)
        .cloned()
        .ok_or(FaceImprintSplitError::MissingFace { face: first })?;
    let second_attr = g
        .face_attr(second)
        .cloned()
        .ok_or(FaceImprintSplitError::MissingFace { face: second })?;
    let (survivor, removed) = if first == original_face {
        (first, second)
    } else if second == original_face {
        (second, first)
    } else {
        (first, second)
    };
    let mut pcurves = first_attr.pcurves;
    pcurves.extend(second_attr.pcurves);
    for dart in [first_dart, first_end, second_dart, second_end] {
        pcurves.remove(&dart);
    }
    if g.edge_attr(edge).is_none() {
        return Err(FaceImprintSplitError::MissingBoundaryEdge { dart: first_dart });
    }

    g.remove_edge(edge)
        .expect("checked periodic seam edge must remain staged");
    for dart in [first_dart, first_end, second_dart, second_end] {
        g.unlink(Dim::One, dart).map_err(|source| {
            FaceImprintSplitError::PeriodicTopologyEditFailed {
                face: original_face,
                source,
            }
        })?;
    }
    for (first, second) in [(first_previous, second_next), (second_previous, first_next)] {
        g.sew(Dim::One, first, second).map_err(|source| {
            FaceImprintSplitError::PeriodicTopologyEditFailed {
                face: original_face,
                source,
            }
        })?;
    }
    for (dim, dart) in [
        (Dim::Zero, first_dart),
        (Dim::Zero, second_dart),
        (Dim::Two, first_dart),
        (Dim::Two, first_end),
    ] {
        g.unlink(dim, dart).map_err(|source| {
            FaceImprintSplitError::PeriodicTopologyEditFailed {
                face: original_face,
                source,
            }
        })?;
    }

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

    let survivor_attr = g
        .face_attr_mut(survivor)
        .expect("periodic face merge survivor must remain staged");
    survivor_attr.surface = first_attr.surface;
    survivor_attr.outer_loop = loop_dart;
    survivor_attr.inner_loops.clear();
    survivor_attr.pcurves = pcurves;
    g.merge_faces_into(survivor, removed);
    Ok(survivor)
}

fn merge_periodic_boundary_edge<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    face: FaceKey,
    loop_dart: Dart,
    surface: &Surface,
    pcurves: &mut HashMap<Dart, Curve2>,
    period: f64,
) -> Result<Option<Dart>, FaceImprintSplitError> {
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
    let vertex_key = g.cell_key::<Cell0>(first_end);
    if let Some(key) = vertex_key {
        g.remove_vertex(key);
    }
    g.remove_edge(first_key);
    g.remove_edge(second_key);
    g.unlink(Dim::Zero, first)
        .expect("prepared periodic boundary merge must unlink its first edge");
    g.unlink(Dim::Zero, second)
        .expect("prepared periodic boundary merge must unlink its second edge");
    g.unlink(Dim::One, first_end)
        .expect("prepared periodic boundary merge must unlink its seam vertex");
    g.link(Dim::Zero, first, second_end)
        .expect("prepared periodic boundary merge must link the merged edge");
    g.add_edge(EdgeAttr::new(first, merged_curve, P::E::default()));
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
    g: &mut TopologyEdit<'_, P>,
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
    g: &mut TopologyEdit<'_, P>,
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
    g: &mut TopologyEdit<'_, P>,
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
    g: &mut TopologyEdit<'_, P>,
    face: FaceKey,
    imprints: &[&FaceImprint],
) -> Result<Vec<FaceImprintSplit>, FaceImprintSplitError> {
    let boundary_uvs = face_boundary_uvs(g, face)?;
    let boundary_area = signed_area(&boundary_uvs);
    let mut splits = Vec::new();

    for (index, imprint) in imprints.iter().enumerate() {
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
        let mut split = split_face_by_closed_curve_imprint(g, face, &outside)?;
        let reversed = boundary_area.signum() == imprint_area.signum();
        for section in &mut split.sections {
            section.imprint = index;
            section.interval = if reversed {
                Interval::new(1.0, 0.0)
            } else {
                Interval::new(0.0, 1.0)
            };
        }
        splits.push(split);
    }
    Ok(splits)
}

fn reverse_imprint(imprint: &FaceImprint) -> Result<FaceImprint, NurbsError> {
    imprint.reversed()
}

fn split_face_by_closed_curve_imprint<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
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
    g: &mut TopologyEdit<'_, P>,
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
        if uvs.len() < 2
            || uvs
                .iter()
                .any(|uv| snap_boundary_corner(&boundary_uvs, *uv).is_some())
        {
            continue;
        }

        let mut provenance = component
            .iter()
            .map(|oriented| {
                let edge = &graph.edges[oriented.edge];
                let interval = if oriented.reversed {
                    Interval::new(edge.interval.end, edge.interval.start)
                } else {
                    edge.interval
                };
                (edge.source_curve, interval)
            })
            .collect::<Vec<_>>();
        if orient_imprint_loop_against_boundary(&boundary_uvs, &mut loop_imprints)? {
            provenance.reverse();
            for (_, interval) in &mut provenance {
                *interval = Interval::new(interval.end, interval.start);
            }
        }
        let mut split = split_face_by_closed_imprint_loop(g, face, &loop_imprints)?;
        for (section, (imprint, interval)) in split.sections.iter_mut().zip(provenance) {
            section.imprint = imprint;
            section.interval = interval;
        }
        splits.push(split);
    }

    Ok(splits)
}

fn split_face_by_closed_imprint_loop<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
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
    g: &mut TopologyEdit<'_, P>,
    face: FaceKey,
    old_face: FaceAttr<P::F>,
    outside_loop: SectionLoop,
    island_loop: SectionLoop,
) -> Result<FaceImprintSplit, FaceImprintSplitError> {
    let section_edges = sew_section_loops(g, face, &outside_loop, &island_loop)?;
    g.add_profile(ProfileAttr::new(
        outside_loop.loop_dart,
        P::Profile::default(),
    ));
    g.add_profile(ProfileAttr::new(
        island_loop.loop_dart,
        P::Profile::default(),
    ));

    let face_attr = g
        .face_attr_mut(face)
        .expect("source face must remain staged during a closed-loop split");
    face_attr.inner_loops.push(outside_loop.loop_dart);
    face_attr.pcurves.extend(outside_loop.pcurves);

    let second = g.add_face_split_from(
        face,
        FaceAttr::with_pcurves(
            old_face.surface,
            P::F::default(),
            island_loop.loop_dart,
            Vec::new(),
            island_loop.pcurves,
        ),
    );

    Ok(FaceImprintSplit {
        first: face,
        second,
        sections: section_edges
            .into_iter()
            .enumerate()
            .map(|(imprint, edge)| FaceImprintSection {
                edge,
                imprint,
                interval: Interval::new(0.0, 1.0),
            })
            .collect(),
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
    g: &mut TopologyEdit<'_, P>,
    surface: &Surface,
    imprints: &[FaceImprint],
) -> SectionLoop {
    let n = imprints.len();
    let darts = (0..2 * n).map(|_| g.add_dart()).collect::<Vec<_>>();

    for edge in 0..n {
        g.link(Dim::Zero, darts[2 * edge], darts[2 * edge + 1])
            .expect("fresh section edge darts must be alpha0-free");
    }
    for edge in 0..n {
        let end = darts[2 * edge + 1];
        let next_start = darts[2 * ((edge + 1) % n)];
        g.link(Dim::One, end, next_start)
            .expect("fresh section loop darts must be alpha1-free");
    }

    for vertex in 0..n {
        let dart = g.cell_representative(darts[2 * vertex], Dim::Zero);
        let uv = imprints[vertex].pcurve.point_at(0.0);
        g.add_vertex(VertexAttr::new(
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

    SectionLoop {
        loop_dart: darts[0],
        edges,
        pcurves,
    }
}

fn add_imprint_section_loop<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    surface: &Surface,
    imprint: &FaceImprint,
) -> SectionLoop {
    add_section_loop(g, surface, std::slice::from_ref(imprint))
}

fn sew_section_loops<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
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
    let mut edges = Vec::with_capacity(pairs.len());
    for (outside_edge, island_end) in pairs {
        g.sew(Dim::Two, outside_edge.dart, island_end)
            .map_err(|source| FaceImprintSplitError::SectionLoopSewFailed { face, source })?;
        edges.push(g.add_edge(EdgeAttr::new(
            outside_edge.dart,
            outside_edge.curve.clone(),
            P::E::default(),
        )));
    }
    Ok(edges)
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
) -> Result<bool, NurbsError> {
    let boundary_area = signed_area(boundary_uvs);
    let loop_uvs = imprints
        .iter()
        .flat_map(|imprint| imprint.pcurve.sample(16).into_iter().take(16))
        .collect::<Vec<_>>();
    let loop_area = signed_area(&loop_uvs);

    if boundary_area.abs() <= LINEAR_TOLERANCE || loop_area.abs() <= LINEAR_TOLERANCE {
        return Ok(false);
    }

    if boundary_area.signum() == loop_area.signum() {
        *imprints = reversed_imprint_loop(imprints)?;
        return Ok(true);
    }
    Ok(false)
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
    g: &mut TopologyEdit<'_, P>,
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
    split_face_edge_staged(g, face, target.edge, parameter)?;
    Ok(())
}

fn split_one_face_by_imprints<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    face: FaceKey,
    imprints: &[FaceImprint],
) -> Result<Option<FaceImprintSplit>, FaceImprintSplitError> {
    let face_attr = g
        .face_attr(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?;

    let boundary = face_boundary_edges(g, face)?;
    let Some(cut) = FaceImprintCut::from_chain(imprints, &boundary)? else {
        return Ok(None);
    };

    if !face_attr.inner_loops.is_empty() {
        return Err(FaceImprintSplitError::InnerLoopsNotSupported { face });
    }

    let old_face = face_attr.clone();
    let split = apply_outer_face_chord_split(g, face, old_face, &cut)?;
    Ok(Some(split))
}

/// Adds a planar disk face bounded by one circular edge.
///
/// The circle is centered at `plane.origin()` and uses the plane orientation.
/// `radius` must be positive and finite.
pub fn add_circle(
    g: &mut GMap<StandardPayload>,
    plane: Plane,
    radius: f64,
) -> Result<FaceKey, FaceCreationError> {
    g.transaction(|g| add_circle_staged(g, plane, radius))
}

/// Builds a circular boundary and its face within one staged operation.
fn add_circle_staged(
    g: &mut TopologyEdit<'_, StandardPayload>,
    plane: Plane,
    radius: f64,
) -> Result<FaceKey, FaceCreationError> {
    let edge = add_circle_edge_staged(g, plane.clone(), radius)?;
    let loop_dart = g.edge_attr_unchecked(edge).dart;
    g.add_profile(ProfileAttr::new(loop_dart, ()));
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
    sections: Vec<(usize, bool, FaceImprint)>,
}

impl FaceImprintCut {
    /// Follows a nonbranching path from one boundary corner to another.
    fn from_chain(
        imprints: &[FaceImprint],
        boundary: &[(Point2, Curve2)],
    ) -> Result<Option<Self>, NurbsError> {
        for (index, imprint) in imprints.iter().enumerate() {
            for reversed in [false, true] {
                let uv = imprint.pcurve.point_at(if reversed { 1.0 } else { 0.0 });
                let Some(start) = snap_boundary_corner_in(boundary, uv) else {
                    continue;
                };
                if let Some(cut) = Self::follow(imprints, boundary, start, index, reversed)? {
                    return Ok(Some(cut));
                }
            }
        }
        Ok(None)
    }

    /// Stops at boundary vertices or ambiguous junctions rather than inventing a path.
    fn follow(
        imprints: &[FaceImprint],
        boundary: &[(Point2, Curve2)],
        start: usize,
        index: usize,
        reversed: bool,
    ) -> Result<Option<Self>, NurbsError> {
        let mut next = (index, reversed);
        let mut sections = Vec::new();
        let mut visited = HashSet::new();
        loop {
            let (index, reversed) = next;
            if !visited.insert(index) {
                return Ok(None);
            }
            let imprint = if reversed {
                imprints[index].reversed()?
            } else {
                imprints[index].clone()
            };
            let end_uv = imprint.pcurve.point_at(1.0);
            sections.push((index, reversed, imprint));
            if let Some(end) = snap_boundary_corner_in(boundary, end_uv) {
                return Ok(
                    valid_chord(start, end, boundary, &sections).then_some(Self {
                        start_corner: start,
                        end_corner: end,
                        sections,
                    }),
                );
            }
            let candidates = imprints
                .iter()
                .enumerate()
                .filter(|(index, _)| !visited.contains(index))
                .flat_map(|(index, imprint)| {
                    [false, true].into_iter().filter_map(move |reversed| {
                        ((imprint.pcurve.point_at(if reversed { 1.0 } else { 0.0 }) - end_uv)
                            .norm()
                            <= LINEAR_TOLERANCE)
                            .then_some((index, reversed))
                    })
                })
                .collect::<Vec<_>>();
            if candidates.len() != 1 {
                return Ok(None);
            }
            next = candidates[0];
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct BoundaryEdgeTarget {
    edge: EdgeKey,
}

fn face_boundary_uvs<P: Payload>(
    g: &GMap<P>,
    face: FaceKey,
) -> Result<Vec<Point2>, FaceImprintSplitError> {
    Ok(face_boundary_edges(g, face)?
        .into_iter()
        .map(|(uv, _)| uv)
        .collect())
}

/// Each outer-loop corner with the pcurve leaving it, in loop order.
fn face_boundary_edges<P: Payload>(
    g: &GMap<P>,
    face: FaceKey,
) -> Result<Vec<(Point2, Curve2)>, FaceImprintSplitError> {
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
                .map(|pcurve| (pcurve.point_at(0.0), pcurve))
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

/// [`snap_boundary_corner`] over corners paired with their outgoing pcurves.
fn snap_boundary_corner_in(boundary: &[(Point2, Curve2)], uv: Point2) -> Option<usize> {
    boundary
        .iter()
        .enumerate()
        .filter_map(|(index, (corner, _))| {
            let distance = (*corner - uv).norm();
            (distance <= LINEAR_TOLERANCE).then_some((distance, index))
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .map(|(_, index)| index)
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

/// Whether two boundary corners can bound a chord.
///
/// The ends have to be two distinct corners. Neighbouring corners are allowed —
/// the chord that cuts a lens off a disc joins two of them — as long as the
/// chain does not simply retrace the one boundary edge between them. That case
/// would leave a fragment with no interior, and, because the chord it produces
/// is itself that boundary edge, would let the splitter cut the same face
/// forever.
fn valid_chord(
    start: usize,
    end: usize,
    boundary: &[(Point2, Curve2)],
    sections: &[(usize, bool, FaceImprint)],
) -> bool {
    boundary.len() >= 2 && start != end && !retraces_boundary(boundary, sections)
}

/// Whether a chain already runs along the face's own boundary.
///
/// This is what bounds the splitter's work: cutting a face turns the chain into
/// boundary edges of both fragments, so the same chain is refused on everything
/// it has already produced.
fn retraces_boundary(
    boundary: &[(Point2, Curve2)],
    sections: &[(usize, bool, FaceImprint)],
) -> bool {
    sections.iter().all(|(_, _, imprint)| {
        [0.25, 0.5, 0.75].iter().all(|fraction| {
            let uv = imprint.pcurve.point_at(*fraction);
            boundary
                .iter()
                .any(|(_, pcurve)| pcurve.parameter_at(uv, LINEAR_TOLERANCE).is_some())
        })
    })
}

fn apply_outer_face_chord_split<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    original_face: FaceKey,
    mut old_face: FaceAttr<P::F>,
    cut: &FaceImprintCut,
) -> Result<FaceImprintSplit, FaceImprintSplitError> {
    let source_profile = g
        .profile_key(old_face.outer_loop)
        .expect("face loop must have a registered profile");
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
    let darts = cut
        .sections
        .iter()
        .map(|_| [g.add_dart(), g.add_dart(), g.add_dart(), g.add_dart()])
        .collect::<Vec<_>>();
    for (index, (_, _, imprint)) in cut.sections.iter().enumerate() {
        let [a, b, c, d] = darts[index];
        g.link(Dim::Zero, a, b).expect("fresh section edge");
        g.link(Dim::Zero, c, d).expect("fresh section edge");
        g.link(Dim::Two, a, d).expect("fresh section sides");
        g.link(Dim::Two, b, c).expect("fresh section sides");
        old_face.pcurves.insert(a, imprint.pcurve.clone());
        old_face.pcurves.insert(c, imprint.pcurve.reversed());
        if index > 0 {
            g.link(Dim::One, darts[index - 1][1], a)
                .expect("chain vertex");
            g.link(Dim::One, d, darts[index - 1][2])
                .expect("reverse chain vertex");
            let uv = imprint.pcurve.point_at(0.0);
            g.add_vertex(VertexAttr::new(
                a,
                old_face.surface.point_at(uv.x, uv.y),
                P::V::default(),
            ));
        }
    }
    let ab_start = darts[0][0];
    let ab_end = darts.last().unwrap()[1];
    let ba_start = darts.last().unwrap()[2];
    let ba_end = darts[0][3];
    let pcurve_ab = old_face.pcurves[&ab_start].clone();
    let pcurve_ba = old_face.pcurves[&ba_start].clone();

    g.unlink(Dim::One, start_previous_end)
        .expect("split start corner must be alpha1-linked");
    g.unlink(Dim::One, end_previous_end)
        .expect("split end corner must be alpha1-linked");
    g.link(Dim::One, start_previous_end, ab_start)
        .expect("split start must be alpha1-free after unlink");
    g.link(Dim::One, ab_end, end_dart)
        .expect("section endpoint must be alpha1-free");
    g.link(Dim::One, end_previous_end, ba_start)
        .expect("split end must be alpha1-free after unlink");
    g.link(Dim::One, ba_end, start_dart)
        .expect("section endpoint must be alpha1-free");

    let start_profile = g.profile_key(start_dart);
    let end_profile = g.profile_key(end_dart);
    match (start_profile, end_profile) {
        (Some(key), None) if key == source_profile => {
            g.add_profile_split_from(
                source_profile,
                ProfileAttr::new(end_dart, P::Profile::default()),
            );
        }
        (None, Some(key)) if key == source_profile => {
            g.add_profile_split_from(
                source_profile,
                ProfileAttr::new(start_dart, P::Profile::default()),
            );
        }
        _ => panic!("a chord split must retain one source profile and create one split profile"),
    }

    let start_pcurves = split_face_pcurves(
        g,
        original_face,
        &old_face.pcurves,
        start_dart,
        ba_start,
        &pcurve_ba,
    )?;
    let end_pcurves = split_face_pcurves(
        g,
        original_face,
        &old_face.pcurves,
        end_dart,
        ab_start,
        &pcurve_ab,
    )?;
    let source_uses_start_loop = g.cell_key::<Cell2>(start_dart) == Some(original_face);
    let source_uses_end_loop = g.cell_key::<Cell2>(end_dart) == Some(original_face);
    assert_ne!(
        source_uses_start_loop, source_uses_end_loop,
        "exactly one split region must contain the source face root"
    );
    let (source_loop, source_pcurves, created_loop, created_pcurves) = if source_uses_start_loop {
        (start_dart, start_pcurves, end_dart, end_pcurves)
    } else {
        (end_dart, end_pcurves, start_dart, start_pcurves)
    };

    let sections = cut
        .sections
        .iter()
        .zip(&darts)
        .map(|((index, reversed, imprint), darts)| {
            let edge = g.add_edge(EdgeAttr::new(
                darts[0],
                imprint.curve.clone(),
                P::E::default(),
            ));
            FaceImprintSection {
                edge,
                imprint: *index,
                interval: if *reversed {
                    Interval::new(1.0, 0.0)
                } else {
                    Interval::new(0.0, 1.0)
                },
            }
        })
        .collect();
    let source_attr = g
        .face_attr_mut(original_face)
        .expect("source face must remain staged during a chord split");
    source_attr.surface = old_face.surface.clone();
    source_attr.outer_loop = source_loop;
    source_attr.inner_loops.clear();
    source_attr.pcurves = source_pcurves;

    let second = g.add_face_split_from(
        original_face,
        FaceAttr::with_pcurves(
            old_face.surface,
            P::F::default(),
            created_loop,
            Vec::new(),
            created_pcurves,
        ),
    );

    Ok(FaceImprintSplit {
        first: original_face,
        second,
        sections,
    })
}

fn split_face_pcurves<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    face: FaceKey,
    old_pcurves: &HashMap<Dart, Curve2>,
    loop_dart: Dart,
    section_dart: Dart,
    section_pcurve: &Curve2,
) -> Result<HashMap<Dart, Curve2>, FaceImprintSplitError> {
    let mut pcurves = HashMap::new();
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

/// Adds a planar annular face with concentric circular boundary loops.
///
/// The outer loop follows `plane` orientation and the inner loop is reversed to
/// represent a hole. Both radii must be positive and finite, and `outer_radius`
/// must be greater than `inner_radius`.
pub fn add_annulus(
    g: &mut GMap<StandardPayload>,
    plane: Plane,
    outer_radius: f64,
    inner_radius: f64,
) -> Result<FaceKey, FaceCreationError> {
    g.transaction(|g| add_annulus_staged(g, plane, outer_radius, inner_radius))
}

/// Builds both annulus boundaries and registers their shared face atomically.
fn add_annulus_staged(
    g: &mut TopologyEdit<'_, StandardPayload>,
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
    let outer_edge = add_circle_edge_staged(g, plane.clone(), outer_radius)?;
    let inner_edge = add_circle_edge_staged(g, inner_plane, inner_radius)?;
    let outer_loop = g.edge_attr_unchecked(outer_edge).dart;
    let inner_loop = g.edge_attr_unchecked(inner_edge).dart;
    g.add_profile(ProfileAttr::new(outer_loop, ()));
    g.add_profile(ProfileAttr::new(inner_loop, ()));

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
    let edge_view = g.edge(edge).ok_or(FaceEdgeSplitError::EdgeSplitFailed(
        EdgeSplitError::MissingEdge { edge },
    ))?;
    let split_point = edge_view
        .curve()
        .ok_or(FaceEdgeSplitError::MissingEdgeCurve {
            dart: edge_view.dart(),
        })?
        .point_at(parameter);
    // A seam has two boundary occurrences on one face, each with its own UV curve.
    let mut occurrences = HashSet::new();
    for face in edge_view.faces() {
        for boundary in g.face_unchecked(face.key()).edges() {
            if boundary.key() == edge {
                occurrences.insert((face.key(), boundary.dart()));
            }
        }
    }
    occurrences
        .into_iter()
        .map(|(face, dart)| {
            let face_view = g
                .face(face)
                .ok_or(FaceEdgeSplitError::MissingFace { face })?;
            let pcurve = face_view
                .pcurve(dart)
                .ok_or(FaceEdgeSplitError::MissingPcurve { face, dart })?;
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
    g: &mut TopologyEdit<'_, P>,
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

/// Adds a planar polygon face with zero or more polygonal holes.
///
/// `outer` and every entry in `holes` are interpreted in the supplied order and
/// projected into `plane` to build their pcurves. Each loop must contain at
/// least three points. The caller is responsible for supplying coplanar,
/// non-self-intersecting loops with suitable winding and containment; those
/// geometric relationships are not validated here.
pub fn add_polygon_with_holes(
    g: &mut GMap<StandardPayload>,
    plane: Plane,
    outer: &[Point3],
    holes: &[&[Point3]],
) -> Result<FaceKey, FaceCreationError> {
    g.transaction(|g| add_polygon_with_holes_staged(g, plane, outer, holes))
}

/// Builds the outer polygon and all hole loops before registering the face.
fn add_polygon_with_holes_staged(
    g: &mut TopologyEdit<'_, StandardPayload>,
    plane: Plane,
    outer: &[Point3],
    holes: &[&[Point3]],
) -> Result<FaceKey, FaceCreationError> {
    validate_polygon(outer)?;
    for hole in holes {
        validate_polygon(hole)?;
    }

    let outer_profile = add_polygon_staged(g, outer);
    let outer_loop = g.profile_attr_unchecked(outer_profile).dart;
    let mut inner_loops = Vec::with_capacity(holes.len());
    let outer_profile =
        Profile::from_dart(g, outer_loop).expect("outer loop must have a registered profile");
    let mut pcurves = profile_pcurves(&outer_profile, &plane)?;

    for hole in holes {
        let inner_profile = add_polygon_staged(g, hole);
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
    g.transaction(|g| Ok::<_, TopologyEditError>(add_polygon_staged(g, corners)))
        .expect("fresh polygon operation must commit")
}

/// Creates and links polygon segments without opening another transaction scope.
pub(crate) fn add_polygon_staged<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    corners: &[Point3],
) -> crate::topology::shape_keys::ProfileKey {
    assert!(
        corners.len() >= 3,
        "add_polygon requires at least 3 corners, got {}",
        corners.len()
    );
    let n = corners.len();
    let darts: Vec<Dart> = (0..2 * n).map(|_| g.add_dart()).collect();

    for i in 0..n {
        g.sew(Dim::Zero, darts[2 * i], darts[2 * i + 1])
            .expect("fresh polygon edge darts must be alpha0-free");
    }
    for i in 0..n {
        let a = darts[2 * i + 1];
        let b = darts[(2 * i + 2) % (2 * n)];
        g.sew(Dim::One, a, b)
            .expect("fresh polygon boundary darts must be alpha1-free");
    }

    for i in 0..n {
        let dart = g.cell_representative(darts[2 * i], Dim::Zero);
        g.add_vertex(VertexAttr::new(dart, corners[i], P::V::default()));
    }

    for i in 0..n {
        let edge_dart = darts[2 * i];
        let curve = Curve::line(corners[i], corners[(i + 1) % n]);
        g.add_edge(EdgeAttr::new(edge_dart, curve, P::E::default()));
    }
    g.add_profile(crate::topology::attributes::ProfileAttr::new(
        darts[0],
        P::Profile::default(),
    ))
}

/// Flips a face's orientation in place.
///
/// Every boundary loop is re-rooted on its `alpha0` partner and every pcurve is
/// re-keyed to that partner and reversed, so the loops are traversed the other
/// way round and [`Face::normal_at`](crate::topology::face::Face::normal_at)
/// returns the opposite normal. The map's topology is untouched — only the
/// face attribute changes — so darts captured for sewing stay valid.
///
/// Does nothing when `face` is not a registered face.
pub fn reverse_face_winding<P: Payload>(g: &mut TopologyEdit<'_, P>, face: FaceKey) {
    let Some(face_attr) = g.face_attr(face).cloned() else {
        return;
    };

    let outer_loop = g.alpha(Dim::Zero, face_attr.outer_loop);
    let inner_loops = face_attr
        .inner_loops
        .iter()
        .map(|dart| g.alpha(Dim::Zero, *dart))
        .collect::<Vec<_>>();
    let pcurves = face_attr
        .face(g)
        .edges()
        .into_iter()
        .filter_map(|edge| {
            face_attr
                .pcurves
                .get(&edge.dart())
                .map(|pcurve| (g.alpha(Dim::Zero, edge.dart()), pcurve.reversed()))
        })
        .collect();

    if let Some(face) = g.face_attr_mut(face) {
        face.outer_loop = outer_loop;
        face.inner_loops = inner_loops;
        face.pcurves = pcurves;
    }
}
