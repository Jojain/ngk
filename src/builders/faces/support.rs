use std::collections::{HashMap, HashSet};

use super::{chord_loop_kinds, periodic_image_near_pcurve};
use crate::builders::edges::EdgeSplitError;
use crate::builders::errors::ModelEditFailure;
use crate::builders::scaffold::CutAttachment;
use crate::geometry::parameter::{Fraction, Normalized};
use crate::geometry::{
    Curve, CurveCurveIntersection2, CurveIntersectionError, Interval, LINEAR_TOLERANCE, NurbsError,
    Point2, Point3, TrimmedCurve, TrimmedCurve2,
};
use crate::model::{Cell1, Model};
use crate::topology::attributes::{
    EdgeAttr, FaceAttr, LoopDefinition, LoopKind, ProfileAttr, VertexAttr,
};
use crate::topology::edge::Edge;
use crate::topology::embedding::EntityOwner;
use crate::topology::gmap::{Dart, Dim};
use crate::topology::payload::Payload;
use crate::topology::profile::Profile;
use crate::topology::shape_keys::{EdgeKey, FaceKey, VertexKey};
use crate::topology::vertex::Vertex;
use crate::topology::{ModelEdit, ModelEditError};
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
    #[error("face edge model edit failed")]
    ModelEditFailed(#[source] ModelEditFailure),
}

impl From<ModelEditError> for FaceEdgeSplitError {
    fn from(error: ModelEditError) -> Self {
        Self::ModelEditFailed(ModelEditFailure::new(error))
    }
}

#[derive(Debug, Error)]
pub enum FaceImprintSplitError {
    #[error("missing face for key {face:?}")]
    MissingFace { face: FaceKey },
    #[error("face {face:?} has an inner loop crossed or ambiguously divided by the split")]
    InnerLoopsNotSupported { face: FaceKey },
    #[error("face {face:?} has no pcurve for boundary dart {dart:?}")]
    MissingPcurve { face: FaceKey, dart: Dart },
    #[error("no boundary of face {face:?} runs along its stored loop seed {dart:?}")]
    SeedNotOnBoundary { face: FaceKey, dart: Dart },
    #[error("ring face {face:?} requires one cut attachment on loop {dart:?}")]
    RingCutAttachment { face: FaceKey, dart: Dart },
    #[error("the cut of face {face:?} at {dart:?} reaches no loop either half kept")]
    CutReachesNoLoop { face: FaceKey, dart: Dart },
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
        source: ModelEditError,
    },
    #[error("imprint {imprint} of face {face:?} was not realized by any section")]
    ImprintNotRealized { face: FaceKey, imprint: usize },
    #[error("failed to convert imprint curve geometry")]
    ImprintCurveConversion(#[from] NurbsError),
    #[error("failed to intersect face imprint pcurves")]
    ImprintIntersection(#[from] CurveIntersectionError),
    #[error("face imprint model edit failed")]
    ModelEditFailed(#[from] ModelEditError),
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct MissingVertexPoint(Dart);

#[derive(Debug, Clone, Copy)]
pub(crate) struct MissingEdgeCurve(Dart);

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

pub(crate) fn edge_curve<'a, P: Payload>(edge: &'a Edge<'_, P>) -> &'a Curve {
    edge.curve()
}

/// A section edge and its directed interval on the original input imprint.
#[derive(Debug, Clone, PartialEq)]
pub struct FaceImprintSection {
    pub edge: EdgeKey,
    /// Index in the input slice passed to the splitter.
    pub imprint: usize,
    /// Fractions of the source imprint at the start and end of the stored edge.
    pub interval: Interval<Normalized>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct FaceImprintSplit {
    pub first: FaceKey,
    pub second: FaceKey,
    pub sections: Vec<FaceImprintSection>,
}

/// Paired model-space and face-parameter-space geometry for a face imprint.
///
/// The two halves are **synchronized**: the same normalized fraction of
/// `curve` and of `pcurve` is the same point. That is a stronger requirement
/// than each half merely being correct, and it constrains which support may be
/// kept — a `Circle` spans its arc in angle while the rational quadratic its
/// pcurve is fitted from does not, so an imprint of an arc carries the section
/// as NURBS rather than as the circle it came from.
#[derive(Clone)]
pub struct FaceImprint {
    pub curve: TrimmedCurve,
    pub pcurve: TrimmedCurve2,
}

impl FaceImprint {
    /// Creates an imprint whose 3D curve and 2D pcurve share direction.
    pub fn new(curve: Curve, pcurve: TrimmedCurve2) -> Self {
        let interval = match &curve {
            Curve::Circle(_) | Curve::Ellipse(_) | Curve::Nurbs(_) => curve.domain(),
            Curve::Line(_) => Interval::new(0.0, 1.0),
        };
        Self {
            curve: TrimmedCurve::new(curve, interval),
            pcurve,
        }
    }

    /// Creates an imprint over an explicit span of its 3D support.
    pub fn with_section(curve: TrimmedCurve, pcurve: TrimmedCurve2) -> Self {
        Self { curve, pcurve }
    }

    pub fn point_at(&self, parameter: Fraction) -> Point3 {
        self.curve.point_at(parameter)
    }

    pub fn parameter_at(&self, point: Point3) -> Fraction {
        self.curve.parameter_at(point)
    }

    /// Returns the exact synchronized fragment over a span of the traversal.
    ///
    /// One interval cuts both halves, which is only meaningful because they are
    /// synchronized: the same fraction is the same point on each.
    pub fn trimmed(&self, interval: Interval<Normalized>) -> Result<Self, NurbsError> {
        Ok(Self::with_section(
            self.curve.sub(interval),
            self.pcurve.sub(interval),
        ))
    }

    pub(crate) fn reversed(&self) -> Result<Self, NurbsError> {
        Ok(Self::with_section(
            self.curve.reversed(),
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
    pub interval: Interval<Normalized>,
}

/// A normalized planar graph built from imprint curves in a face's UV space.
///
/// Curves are split at intersections, coincident vertices and duplicate
/// edges are merged within [`LINEAR_TOLERANCE`], and the resulting undirected
/// graph can be inspected for branches and standalone closed loops. The graph is
/// a temporary aid for face splitting and is not part of the [`Model`] topology.
#[derive(Debug, Clone, PartialEq)]
pub struct FaceImprintGraph {
    pub(crate) vertices: Vec<Point2>,
    pub(crate) edges: Vec<FaceImprintGraphEdge>,
}

impl FaceImprintGraph {
    /// Builds an imprint graph from 2D lines and NURBS curves.
    pub fn from_curves(curves: &[TrimmedCurve2]) -> Result<Self, CurveIntersectionError> {
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

    pub(crate) fn closed_edge_components(&self) -> Vec<Vec<OrientedGraphEdge>> {
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
pub(crate) struct OrientedGraphEdge {
    pub(crate) edge: usize,
    pub(crate) reversed: bool,
}

impl OrientedGraphEdge {
    pub(crate) fn start(self, graph: &FaceImprintGraph) -> usize {
        let edge = &graph.edges[self.edge];
        if self.reversed { edge.end } else { edge.start }
    }
}

pub(crate) fn curve_split_parameters(
    curves: &[TrimmedCurve2],
) -> Result<Vec<Vec<Fraction>>, CurveIntersectionError> {
    let mut parameters = vec![vec![Fraction::START, Fraction::END]; curves.len()];

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
        values.sort_by(Fraction::total_cmp);
        values.dedup_by(|a, b| (*a - *b).abs() <= LINEAR_TOLERANCE);
    }

    Ok(parameters)
}

pub(crate) fn graph_vertex(vertices: &mut Vec<Point2>, uv: Point2) -> usize {
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

pub(crate) fn ordered_edge_key(a: usize, b: usize) -> (usize, usize) {
    if a < b { (a, b) } else { (b, a) }
}

pub(crate) struct IncidentFacePcurve {
    pub(crate) face: FaceKey,
    pub(crate) dart: Dart,
    pub(crate) pcurve: TrimmedCurve2,
    pub(crate) fraction: Fraction,
}

/// A face pcurve turned to begin where a closed edge was just marked.
pub(crate) struct RebasedFacePcurve {
    pub(crate) face: FaceKey,
    pub(crate) dart: Dart,
    pub(crate) pcurve: TrimmedCurve2,
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
pub(crate) struct FaceImprintCut {
    pub(crate) start_corner: usize,
    pub(crate) end_corner: usize,
    pub(crate) sections: Vec<(usize, bool, FaceImprint)>,
}

impl FaceImprintCut {
    /// Follows a nonbranching path from one boundary corner to another.
    pub(crate) fn from_chain(
        imprints: &[FaceImprint],
        boundary: &[BoundaryCorner],
        placed: &[[Option<VertexKey>; 2]],
    ) -> Result<Option<Self>, NurbsError> {
        for (index, imprint) in imprints.iter().enumerate() {
            for reversed in [false, true] {
                let slot = usize::from(reversed);
                let uv = imprint
                    .pcurve
                    .point_at(Fraction::new(if reversed { 1.0 } else { 0.0 }));
                let Some(start) = imprint_endpoint_corner(boundary, placed[index][slot], uv) else {
                    continue;
                };
                if let Some(cut) = Self::follow(imprints, boundary, placed, start, index, reversed)?
                {
                    return Ok(Some(cut));
                }
            }
        }
        Ok(None)
    }

    /// Stops at boundary vertices or ambiguous junctions rather than inventing a path.
    pub(crate) fn follow(
        imprints: &[FaceImprint],
        boundary: &[BoundaryCorner],
        placed: &[[Option<VertexKey>; 2]],
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
            let end_uv = imprint.pcurve.point_at(Fraction::new(1.0));
            sections.push((index, reversed, imprint));
            let end_slot = usize::from(!reversed);
            if let Some(end) = imprint_endpoint_corner(boundary, placed[index][end_slot], end_uv) {
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
                        ((imprint.pcurve.point_at(if reversed {
                            Fraction::new(1.0)
                        } else {
                            Fraction::new(0.0)
                        }) - end_uv)
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
pub(crate) struct BoundaryEdgeTarget {
    pub(crate) edge: EdgeKey,
}

pub(crate) fn face_boundary_uvs<P: Payload>(
    g: &Model<P>,
    face: FaceKey,
) -> Result<Vec<Point2>, FaceImprintSplitError> {
    Ok(face_boundary_edges(g, face)?
        .into_iter()
        .map(|corner| corner.uv)
        .collect())
}

/// Every bounding-loop corner with the pcurve leaving it, in loop order.
///
/// A wrapping loop bounds its face exactly as an outer loop does, so a query
/// asking whether a parameter point sits on the boundary must see both. Only
/// holes are left out, which is what the callers mean by "the boundary".
pub(crate) fn face_boundary_edges<P: Payload>(
    g: &Model<P>,
    face: FaceKey,
) -> Result<Vec<BoundaryCorner>, FaceImprintSplitError> {
    let face_view = g
        .face(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?;
    let mut boundary = Vec::new();
    // Started at each stored seed rather than wherever the boundary walk
    // happened to begin. An imprint is located against this list by index, and
    // the pcurves it is compared with are keyed against the seed's own
    // direction, so rotating the list moves every corner it can land on.
    let seeds: Vec<Dart> = face_view
        .attr_loops()
        .iter()
        .filter(|definition| definition.kind() != LoopKind::Inner)
        .map(|definition| definition.seed())
        .collect();
    for seed in seeds {
        boundary.extend(loop_boundary_edges(g, face, seed)?);
    }
    Ok(boundary)
}

/// Each corner of the loop seeded at `loop_dart`, with the pcurve leaving it.
/// One corner of a face's bounding loop, with the pcurve leaving it.
///
/// The vertex is carried rather than re-found from `uv`, because the two are
/// derived by different routes: `uv` projects the vertex onto the surface,
/// while an imprint endpoint comes from the solver that traced the contact, and
/// they agree only to that solver's residual. Identity is what an imprint is
/// matched by wherever the splitter placed the corner itself; the position is
/// what is left for corners it did not place.
#[derive(Clone)]
pub(crate) struct BoundaryCorner {
    pub(crate) uv: Point2,
    pub(crate) pcurve: TrimmedCurve2,
    pub(crate) vertex: Option<VertexKey>,
}

pub(crate) fn loop_boundary_edges<P: Payload>(
    g: &Model<P>,
    face: FaceKey,
    loop_dart: Dart,
) -> Result<Vec<BoundaryCorner>, FaceImprintSplitError> {
    let face_view = g
        .face(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?;
    // Anchored at the seed, not at wherever the walk began: an imprint is
    // located against this list by index, so a rotation moves every corner it
    // can land on.
    face_view
        .loop_from_seed(loop_dart)
        .starting_at(loop_dart)
        .ok_or(FaceImprintSplitError::SeedNotOnBoundary {
            face,
            dart: loop_dart,
        })?
        .corners()
        .iter()
        .map(|corner| {
            let dart = corner.outgoing().dart();
            let pcurve = face_view
                .pcurve(dart)
                .ok_or(FaceImprintSplitError::MissingPcurve { face, dart })?;
            // Where the corner is, asked of the corner. A marked edge's corner
            // need not sit where its pcurve starts -- marking says where a
            // closed edge now begins, while the pcurve keeps its own anchoring
            // -- so reading the pcurve would put the corner in the wrong place.
            // An unmarked loop has no corner at all, and the pcurve's start is
            // then the only place to begin the walk from.
            let corner = Vertex::from_dart(g, dart);
            let uv = corner
                .as_ref()
                .map(|vertex| *vertex.point())
                .and_then(|point| face_view.surface().param_at(point).ok())
                .map(|uv| periodic_image_near_pcurve(face_view.surface(), &pcurve, uv))
                .unwrap_or_else(|| pcurve.point_at(Fraction::new(0.0)));
            Ok(BoundaryCorner {
                uv,
                pcurve,
                vertex: corner.map(|vertex| vertex.key()),
            })
        })
        .collect()
}

/// The loops bounding a face from outside: every loop that is not a hole.
pub(crate) fn bounding_loops(boundary: &[LoopDefinition]) -> Vec<LoopDefinition> {
    boundary
        .iter()
        .filter(|loop_| loop_.kind() != LoopKind::Inner)
        .copied()
        .collect()
}

pub(crate) fn boundary_edge_at_uv<P: Payload>(
    g: &Model<P>,
    face: FaceKey,
    uv: Point2,
) -> Result<Option<BoundaryEdgeTarget>, FaceImprintSplitError> {
    let face_view = g
        .face(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?;

    for edge in face_view.edges() {
        let pcurve = face_view
            .pcurve(edge.dart())
            .ok_or(FaceImprintSplitError::MissingPcurve {
                face,
                dart: edge.dart(),
            })?;
        let Some(fraction) = pcurve_fraction_at(&pcurve, uv) else {
            continue;
        };
        // Landing on an end of the pcurve means landing on an end of the edge
        // only where the edge has ends. A closed edge's pcurve runs a whole
        // loop, and where that loop happens to start says nothing about where
        // the edge begins -- which is its corner, if it has one.
        let ends_where_it_ends = matches!(edge, Edge::Bounded(_));
        if ends_where_it_ends
            && (fraction <= Fraction::new(LINEAR_TOLERANCE)
                || fraction >= Fraction::new(1.0 - LINEAR_TOLERANCE))
        {
            continue;
        }

        return Ok(Some(BoundaryEdgeTarget {
            edge: boundary_edge_key(g, edge.dart())?,
        }));
    }

    Ok(None)
}

pub(crate) fn pcurve_fraction_at(pcurve: &TrimmedCurve2, point: Point2) -> Option<Fraction> {
    pcurve.try_parameter_at(point, LINEAR_TOLERANCE)
}

pub(crate) fn boundary_edge_key<P: Payload>(
    g: &Model<P>,
    dart: Dart,
) -> Result<EdgeKey, FaceImprintSplitError> {
    g.cell_key::<Cell1>(dart)
        .ok_or(FaceImprintSplitError::MissingBoundaryEdge { dart })
}

/// [`snap_boundary_corner`] over corners paired with their outgoing pcurves.
pub(crate) fn snap_boundary_corner_in(boundary: &[BoundaryCorner], uv: Point2) -> Option<usize> {
    boundary
        .iter()
        .enumerate()
        .filter_map(|(index, corner)| {
            let distance = (corner.uv - uv).norm();
            (distance <= LINEAR_TOLERANCE).then_some((distance, index))
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .map(|(_, index)| index)
}

/// The corner `vertex` is, by identity rather than by position.
pub(crate) fn corner_of_vertex(boundary: &[BoundaryCorner], vertex: VertexKey) -> Option<usize> {
    boundary
        .iter()
        .position(|corner| corner.vertex == Some(vertex))
}

/// Where an imprint endpoint meets the boundary.
///
/// The vertex the splitter cut for that endpoint settles it outright; a corner
/// it did not place is left to the position it reports. Snapping by position
/// alone is what dropped a chord whose endpoint sat a solver residual away from
/// the corner cut for it, and left the Boolean a span one side never imprinted.
pub(crate) fn imprint_endpoint_corner(
    boundary: &[BoundaryCorner],
    placed: Option<VertexKey>,
    uv: Point2,
) -> Option<usize> {
    placed
        .and_then(|vertex| corner_of_vertex(boundary, vertex))
        .or_else(|| snap_boundary_corner_in(boundary, uv))
}

pub(crate) fn snap_boundary_corner(boundary_uvs: &[Point2], uv: Point2) -> Option<usize> {
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
pub(crate) fn valid_chord(
    start: usize,
    end: usize,
    boundary: &[BoundaryCorner],
    sections: &[(usize, bool, FaceImprint)],
) -> bool {
    boundary.len() >= 2 && start != end && !retraces_boundary(boundary, sections)
}

/// Whether a chain already runs along the face's own boundary.
///
/// This is what bounds the splitter's work: cutting a face turns the chain into
/// boundary edges of both fragments, so the same chain is refused on everything
/// it has already produced.
pub(crate) fn retraces_boundary(
    boundary: &[BoundaryCorner],
    sections: &[(usize, bool, FaceImprint)],
) -> bool {
    sections.iter().all(|(_, _, imprint)| {
        [0.25, 0.5, 0.75].iter().all(|fraction| {
            let uv = imprint.pcurve.point_at(Fraction::new(*fraction));
            boundary.iter().any(|corner| {
                corner
                    .pcurve
                    .try_parameter_at(uv, LINEAR_TOLERANCE)
                    .is_some()
            })
        })
    })
}

/// The cut a chord corner hands the boundary walk over on, when there is one.
///
/// A corner where the walk turns across a cut has no `alpha1` link between the
/// occurrence it arrives on and the one it leaves on: the link off the arriving
/// dart reaches the cut instead. That is the whole test, and it needs no
/// classification, because on a corner with no cut the link reaches `outgoing`.
pub(crate) fn corner_cut<P: Payload>(
    edit: &ModelEdit<'_, P>,
    previous_end: Dart,
    outgoing: Dart,
) -> Option<CutAttachment> {
    let next = edit.alpha(Dim::One, previous_end);
    (next != outgoing).then(|| CutAttachment::at(edit, next))
}

/// Cuts a face in two along a chord between two corners of one bounding loop.
///
/// The chorded loop may be outer or wrapping. Chording an outer loop yields two
/// outer loops, as it always has. Chording a wrapping loop leaves one half still
/// spanning the period and bounds the other in that axis, so exactly one half
/// stays a ring and the other becomes a disk — which is what an imprint chording
/// a cylinder wall produces, with no seam anywhere in the answer.
pub(crate) fn apply_face_chord_split<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    original_face: FaceKey,
    mut old_face: FaceAttr<P::F>,
    chorded: LoopDefinition,
    cut: &FaceImprintCut,
) -> Result<FaceImprintSplit, FaceImprintSplitError> {
    let source_profile = edit
        .profile_key(chorded.seed())
        .expect("face loop must have a registered profile");
    // The same list `loop_boundary_edges` counted the cut's corner indices
    // against: the face's boundary walk anchored at this seed. The raw chain
    // cannot stand in for it, because a face that owns a cut carries all of its
    // loops on one chain and the walk runs off this loop part way round.
    let face_view = edit
        .face(original_face)
        .ok_or(FaceImprintSplitError::MissingFace {
            face: original_face,
        })?;
    let corners = face_view
        .loop_from_seed(chorded.seed())
        .starting_at(chorded.seed())
        .ok_or(FaceImprintSplitError::SeedNotOnBoundary {
            face: original_face,
            dart: chorded.seed(),
        })?
        .corners();
    let start = &corners[cut.start_corner];
    let end = &corners[cut.end_corner];
    let start_dart = start.outgoing().dart();
    let end_dart = end.outgoing().dart();
    // The dart the loop arrives on at each corner: a dart-level step, so it
    // holds however the incoming edge is bounded.
    let start_previous_end = edit.alpha(Dim::Zero, start.incoming().dart());
    let end_previous_end = edit.alpha(Dim::Zero, end.incoming().dart());
    // A chord corner is also where the face may reach another of its loops. The
    // walk crosses that cut rather than emitting it, so the two occurrences it
    // joins are not `alpha1`-linked and the splice cannot unlink them. It
    // splices past the cut instead, which leaves the cut — and the loop beyond
    // it — on the half the boundary arrives from. Whether that is the half the
    // loop belongs to is a question about where the loop lies, answered once
    // the halves are placed.
    let start_cut = corner_cut(edit, start_previous_end, start_dart);
    let end_cut = corner_cut(edit, end_previous_end, end_dart);
    let start_entry = start_cut.as_ref().map_or(start_previous_end, |attachment| {
        edit.alpha(Dim::Two, attachment.incoming())
    });
    let end_entry = end_cut.as_ref().map_or(end_previous_end, |attachment| {
        edit.alpha(Dim::Two, attachment.incoming())
    });
    let darts = cut
        .sections
        .iter()
        .map(|_| {
            [
                edit.add_dart(),
                edit.add_dart(),
                edit.add_dart(),
                edit.add_dart(),
            ]
        })
        .collect::<Vec<_>>();
    for (index, (_, _, imprint)) in cut.sections.iter().enumerate() {
        let [a, b, c, d] = darts[index];
        edit.link(Dim::Zero, a, b).expect("fresh section edge");
        edit.link(Dim::Zero, c, d).expect("fresh section edge");
        edit.link(Dim::Two, a, d).expect("fresh section sides");
        edit.link(Dim::Two, b, c).expect("fresh section sides");
        old_face.pcurves.insert(a, imprint.pcurve.clone());
        old_face.pcurves.insert(c, imprint.pcurve.reversed());
        if index > 0 {
            edit.link(Dim::One, darts[index - 1][1], a)
                .expect("chain vertex");
            edit.link(Dim::One, d, darts[index - 1][2])
                .expect("reverse chain vertex");
            let uv = imprint.pcurve.point_at(Fraction::new(0.0));
            edit.add_vertex(VertexAttr::new(a, old_face.surface.point_at(uv.x, uv.y)));
        }
    }
    let ab_start = darts[0][0];
    let ab_end = darts.last().unwrap()[1];
    let ba_start = darts.last().unwrap()[2];
    let ba_end = darts[0][3];
    let pcurve_ab = old_face.pcurves[&ab_start].clone();
    let pcurve_ba = old_face.pcurves[&ba_start].clone();

    edit.unlink(Dim::One, start_entry)
        .expect("split start corner must be alpha1-linked");
    edit.unlink(Dim::One, end_entry)
        .expect("split end corner must be alpha1-linked");
    edit.link(Dim::One, start_entry, ab_start)
        .expect("split start must be alpha1-free after unlink");
    edit.link(Dim::One, ab_end, end_dart)
        .expect("section endpoint must be alpha1-free");
    edit.link(Dim::One, end_entry, ba_start)
        .expect("split end must be alpha1-free after unlink");
    edit.link(Dim::One, ba_end, start_dart)
        .expect("section endpoint must be alpha1-free");

    let start_profile = edit.profile_key(start_dart);
    let end_profile = edit.profile_key(end_dart);
    match (start_profile, end_profile) {
        (Some(key), None) if key == source_profile => {
            edit.add_profile_split_from(source_profile, ProfileAttr::new(end_dart));
        }
        (None, Some(key)) if key == source_profile => {
            edit.add_profile_split_from(source_profile, ProfileAttr::new(start_dart));
        }
        _ => panic!("a chord split must retain one source profile and create one split profile"),
    }

    let start_pcurves = split_face_pcurves(
        edit,
        original_face,
        &old_face.pcurves,
        start_dart,
        ba_start,
        &pcurve_ba,
    )?;
    let end_pcurves = split_face_pcurves(
        edit,
        original_face,
        &old_face.pcurves,
        end_dart,
        ab_start,
        &pcurve_ab,
    )?;
    // The source keeps the half its chorded loop was seeded on. Asking the
    // 2-cell which face is registered in it would not say: a cut carries the
    // face's other loops into one of the halves, and those loops are seeded on
    // the source too, so both halves would answer yes.
    let chorded_cell = edit.cell_representative(chorded.seed(), Dim::Two);
    let source_uses_start_loop = edit.cell_representative(start_dart, Dim::Two) == chorded_cell;
    let source_uses_end_loop = edit.cell_representative(end_dart, Dim::Two) == chorded_cell;
    assert_ne!(
        source_uses_start_loop, source_uses_end_loop,
        "exactly one split region must contain the source face root"
    );
    let (source_loop, mut source_pcurves, created_loop, mut created_pcurves) =
        if source_uses_start_loop {
            (start_dart, start_pcurves, end_dart, end_pcurves)
        } else {
            (end_dart, end_pcurves, start_dart, start_pcurves)
        };
    // Before the holes are placed, because which half is a ring and which a
    // disk is what says which of the two can be asked to hold one.
    let (source_kind, created_kind) = chord_loop_kinds(
        edit,
        chorded.kind(),
        (source_loop, &source_pcurves),
        (created_loop, &created_pcurves),
    )?;
    let (source_inner_loops, created_inner_loops) = partition_inner_loops(
        edit,
        original_face,
        &old_face.inner_vec(),
        &old_face.pcurves,
        (source_loop, &source_pcurves, source_kind),
        (created_loop, &created_pcurves, created_kind),
    )?;
    for &loop_dart in &source_inner_loops {
        extend_loop_pcurves(
            edit,
            original_face,
            loop_dart,
            &old_face.pcurves,
            &mut source_pcurves,
        )?;
    }
    for &loop_dart in &created_inner_loops {
        extend_loop_pcurves(
            edit,
            original_face,
            loop_dart,
            &old_face.pcurves,
            &mut created_pcurves,
        )?;
    }

    let sections = cut
        .sections
        .iter()
        .zip(&darts)
        .map(|((index, reversed, imprint), darts)| {
            let edge = edit.add_edge(EdgeAttr::new(darts[0], imprint.curve.curve().clone()));
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
    let mut source_loops = vec![LoopDefinition::from_kind(source_loop, source_kind)];
    let mut created_loops = vec![LoopDefinition::from_kind(created_loop, created_kind)];

    // A loop spanning a whole period cannot sit inside the half the chord
    // bounded in that axis, so every other wrapping loop belongs to the half
    // that still wraps. No sampling can answer this, and none needs to.
    let source_wraps = source_kind.wrapped_axis().is_some();
    for other in bounding_loops(old_face.loops())
        .into_iter()
        .filter(|other| other.seed() != chorded.seed())
    {
        let (loops, pcurves) = if source_wraps {
            (&mut source_loops, &mut source_pcurves)
        } else {
            (&mut created_loops, &mut created_pcurves)
        };
        extend_loop_pcurves(
            edit,
            original_face,
            other.seed(),
            &old_face.pcurves,
            pcurves,
        )?;
        loops.push(other);
    }
    source_loops.extend(
        source_inner_loops
            .into_iter()
            .map(|dart| LoopDefinition::from_kind(dart, LoopKind::Inner)),
    );
    created_loops.extend(
        created_inner_loops
            .into_iter()
            .map(|dart| LoopDefinition::from_kind(dart, LoopKind::Inner)),
    );

    let source_seeds = source_loops
        .iter()
        .map(|definition| definition.seed())
        .collect::<Vec<_>>();
    let created_seeds = created_loops
        .iter()
        .map(|definition| definition.seed())
        .collect::<Vec<_>>();

    let source_attr = edit
        .face_attr_mut(original_face)
        .expect("source face must remain staged during a chord split");
    source_attr.surface = old_face.surface.clone();
    let anchor = source_attr.seed();
    source_attr.set_boundary(source_loops, anchor);
    source_attr.pcurves = source_pcurves;

    let second = edit.add_face_split_from(
        original_face,
        FaceAttr::with_loops(old_face.surface, created_loops, created_pcurves),
    );

    // The splice left each corner's cut on the half the walk arrived from,
    // which is where it belongs only if the loop it reaches was placed there
    // too. A cut has no identity, so following the loop is one relink; the
    // label follows as well, because a cut is scaffold of whichever face's
    // 2-cell its word runs through.
    for (attachment, splice_half) in [(start_cut, end_dart), (end_cut, start_dart)] {
        let Some(attachment) = attachment else {
            continue;
        };
        let reached = edit.profile_key(attachment.across(edit));
        let carries = |seeds: &[Dart]| {
            reached.is_some() && seeds.iter().any(|&seed| edit.profile_key(seed) == reached)
        };
        let (half, owner) = if carries(&source_seeds[1..]) {
            (source_loop, original_face)
        } else if carries(&created_seeds[1..]) {
            (created_loop, second)
        } else {
            return Err(FaceImprintSplitError::CutReachesNoLoop {
                face: original_face,
                dart: attachment.incoming(),
            });
        };
        let cut = attachment.incoming();
        if half != splice_half {
            attachment.move_after(edit, half)?;
        }
        edit.own_cell(Dim::One, cut, EntityOwner::Face(owner));
    }

    // Every other cell the face owned is scaffold of whichever half its word
    // now runs through, and the chord left the two halves in two 2-cells. The
    // relabelling above reaches only the cuts at the chord's own corners; a
    // bridge the chord ran nowhere near still names the source, and a walk of
    // the created half would then emit its darts as boundary rather than turn
    // across them -- a loop carrying a dart that names no edge.
    let source_cell = edit.cell_representative(source_loop, Dim::Two);
    let owned = edit
        .embedding()
        .records_of(EntityOwner::Face(original_face))
        .collect::<Vec<_>>();
    for record in owned {
        let half = if edit.cell_representative(record.representative, Dim::Two) == source_cell {
            original_face
        } else {
            second
        };
        edit.own_cell(
            record.dimension,
            record.representative,
            EntityOwner::Face(half),
        );
    }

    Ok(FaceImprintSplit {
        first: original_face,
        second,
        sections,
    })
}

/// Assigns each existing hole to the one child region that contains it.
///
/// A half that still spans the period cannot be asked: its sampled boundary is
/// an open polyline, and a crossing count over one says nothing. Chording a
/// wrapping loop leaves exactly one such ring and bounds the other half into a
/// disk, so the disk is the half that answers and the ring takes every hole the
/// disk does not hold — there being nowhere else for one to be.
///
/// With both halves bounded, both are asked. A chord may touch a hole, but it
/// must not cross one: crossing would require splitting that inner loop as part
/// of the same edit. Sampling the complete loop rather than one seed point
/// distinguishes a tangent touch from a crossing.
#[allow(clippy::too_many_arguments)]
pub(crate) fn partition_inner_loops<P: Payload>(
    edit: &ModelEdit<'_, P>,
    face: FaceKey,
    inner_loops: &[Dart],
    old_pcurves: &HashMap<Dart, TrimmedCurve2>,
    source_half: (Dart, &HashMap<Dart, TrimmedCurve2>, LoopKind),
    created_half: (Dart, &HashMap<Dart, TrimmedCurve2>, LoopKind),
) -> Result<(Vec<Dart>, Vec<Dart>), FaceImprintSplitError> {
    // With no hole to place there is nothing to ask in the first place.
    if inner_loops.is_empty() {
        return Ok((Vec::new(), Vec::new()));
    }
    let (source_loop, source_pcurves, source_kind) = source_half;
    let (created_loop, created_pcurves, created_kind) = created_half;
    // Which half, if either, still spans the period after the chord.
    let ring = match (source_kind.wrapped_axis(), created_kind.wrapped_axis()) {
        (Some(_), None) => Some(ChordHalf::Source),
        (None, Some(_)) => Some(ChordHalf::Created),
        _ => None,
    };
    let source_boundary = sampled_loop_uvs(edit, face, source_loop, source_pcurves)?;
    let created_boundary = sampled_loop_uvs(edit, face, created_loop, created_pcurves)?;
    let mut source = Vec::new();
    let mut created = Vec::new();
    for &inner_loop in inner_loops {
        let samples = sampled_loop_uvs(edit, face, inner_loop, old_pcurves)?;
        let held_by = |boundary: &[Point2]| {
            samples
                .iter()
                .all(|point| sampled_loop_contains(boundary, *point))
        };
        let half = match ring {
            Some(ChordHalf::Source) => match held_by(&created_boundary) {
                true => ChordHalf::Created,
                false => ChordHalf::Source,
            },
            Some(ChordHalf::Created) => match held_by(&source_boundary) {
                true => ChordHalf::Source,
                false => ChordHalf::Created,
            },
            None => match (held_by(&source_boundary), held_by(&created_boundary)) {
                (true, false) => ChordHalf::Source,
                (false, true) => ChordHalf::Created,
                _ => return Err(FaceImprintSplitError::InnerLoopsNotSupported { face }),
            },
        };
        match half {
            ChordHalf::Source => source.push(inner_loop),
            ChordHalf::Created => created.push(inner_loop),
        }
    }
    Ok((source, created))
}

/// One of the two regions a chord split leaves behind.
#[derive(Clone, Copy, PartialEq)]
pub(crate) enum ChordHalf {
    /// The half the source face keeps.
    Source,
    /// The half that becomes a new face.
    Created,
}

pub(crate) fn sampled_loop_uvs<P: Payload>(
    edit: &ModelEdit<'_, P>,
    face: FaceKey,
    loop_dart: Dart,
    pcurves: &HashMap<Dart, TrimmedCurve2>,
) -> Result<Vec<Point2>, FaceImprintSplitError> {
    let profile =
        Profile::from_dart(edit, loop_dart).expect("face loop must have a registered profile");
    let mut samples = Vec::new();
    for dart in profile.darts().step_by(2) {
        let pcurve = pcurves
            .get(&dart)
            .ok_or(FaceImprintSplitError::MissingPcurve { face, dart })?;
        samples.extend(pcurve.sample(64).into_iter().take(64));
    }
    Ok(samples)
}

pub(crate) fn sampled_loop_contains(boundary: &[Point2], point: Point2) -> bool {
    let mut inside = false;
    for (&start, &end) in boundary
        .iter()
        .zip(boundary.iter().cycle().skip(1))
        .take(boundary.len())
    {
        let segment = end - start;
        let parameter = if segment.norm_squared() <= f64::EPSILON {
            0.0
        } else {
            (point - start).dot(&segment) / segment.norm_squared()
        }
        .clamp(0.0, 1.0);
        if (start + segment * parameter - point).norm() <= LINEAR_TOLERANCE {
            return true;
        }
        if (start.y > point.y) != (end.y > point.y)
            && point.x < start.x + (point.y - start.y) * (end.x - start.x) / (end.y - start.y)
        {
            inside = !inside;
        }
    }
    inside
}

pub(crate) fn extend_loop_pcurves<P: Payload>(
    edit: &ModelEdit<'_, P>,
    face: FaceKey,
    loop_dart: Dart,
    old_pcurves: &HashMap<Dart, TrimmedCurve2>,
    destination: &mut HashMap<Dart, TrimmedCurve2>,
) -> Result<(), FaceImprintSplitError> {
    let profile =
        Profile::from_dart(edit, loop_dart).expect("face loop must have a registered profile");
    for dart in profile.darts().step_by(2) {
        let pcurve = old_pcurves
            .get(&dart)
            .cloned()
            .ok_or(FaceImprintSplitError::MissingPcurve { face, dart })?;
        destination.insert(dart, pcurve);
    }
    Ok(())
}

pub(crate) fn split_face_pcurves<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    old_pcurves: &HashMap<Dart, TrimmedCurve2>,
    loop_dart: Dart,
    section_dart: Dart,
    section_pcurve: &TrimmedCurve2,
) -> Result<HashMap<Dart, TrimmedCurve2>, FaceImprintSplitError> {
    let mut pcurves = HashMap::new();
    let profile =
        Profile::from_dart(edit, loop_dart).expect("face loop must have a registered profile");
    for profile_dart in profile.darts().step_by(2) {
        let pcurve = if profile_dart == section_dart {
            section_pcurve.clone()
        } else {
            let candidates = [
                profile_dart,
                edit.alpha(Dim::Zero, profile_dart),
                edit.alpha(Dim::Two, profile_dart),
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
