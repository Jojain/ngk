use std::collections::{HashMap, HashSet};

use crate::StandardPayload;
use crate::builders::edges::{
    EdgeSplit, EdgeSplitError, add_circle_staged as add_circle_edge_staged,
    split_face_boundary_edge,
};
use crate::builders::errors::{FaceCreationError, ModelEditFailure};
use crate::builders::profiles::{
    add_rectangle_staged as add_rectangle_profile_staged, profile_pcurves,
};
use crate::builders::scaffold::{CutAttachment, cut_between_loops};
use crate::geometry::parameter::{Fraction, NativeParam, Normalized};
use crate::geometry::{
    Axis2, Curve, CurveCurveIntersection2, CurveIntersectionError, DomainSide, Interval,
    LINEAR_TOLERANCE, NurbsError, Periodicity, Plane, Point2, Point3, Surface, SurfacePeriodicity,
    TrimmedCurve, TrimmedCurve2, Vector2,
};
use crate::model::{Cell1, Model};
use crate::topology::attributes::{
    EdgeAttr, FaceAttr, LoopDefinition, LoopKind, ProfileAttr, VertexAttr,
};
use crate::topology::closed::Closed;
use crate::topology::edge::Edge;
use crate::topology::embedding::EntityOwner;
use crate::topology::gmap::{Dart, Dim};
use crate::topology::orientation::Orientation;
use crate::topology::payload::{DefaultPayload, Payload};
use crate::topology::planar::Planar;
use crate::topology::profile::Profile;
use crate::topology::shape_keys::{EdgeKey, FaceKey, ProfileKey, VertexKey};
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

fn edge_curve<'a, P: Payload>(edge: &'a Edge<'_, P>) -> &'a Curve {
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

    fn reversed(&self) -> Result<Self, NurbsError> {
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
    vertices: Vec<Point2>,
    edges: Vec<FaceImprintGraphEdge>,
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

fn curve_split_parameters(
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
    pcurve: TrimmedCurve2,
    fraction: Fraction,
}

/// A face pcurve turned to begin where a closed edge was just marked.
struct RebasedFacePcurve {
    face: FaceKey,
    dart: Dart,
    pcurve: TrimmedCurve2,
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
pub fn add_face<P: DefaultPayload>(
    g: &mut Model<P>,
    profile: ProfileKey,
) -> Result<FaceKey, FaceCreationError> {
    g.transaction(|edit| add_face_staged(edit, profile))
}

pub(crate) fn add_face_staged<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    profile: ProfileKey,
) -> Result<FaceKey, FaceCreationError> {
    let (loop_dart, plane, pcurves) = {
        let profile = edit.profile_unchecked(profile);
        let loop_dart = profile.dart;
        let closed =
            Closed::new(profile).ok_or(FaceCreationError::OpenProfile { dart: loop_dart })?;
        let planar = Planar::new(closed)?;
        let (closed, plane) = planar.into_parts();
        let pcurves = profile_pcurves(closed.inner(), &plane)?;
        (loop_dart, plane, pcurves)
    };

    Ok(edit.add_face(FaceAttr::with_pcurves(
        Surface::Plane(plane),
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
    g: &mut Model<StandardPayload>,
    plane: Plane,
    x_size: f64,
    y_size: f64,
) -> Result<FaceKey, FaceCreationError> {
    g.transaction(|edit| {
        let profile = add_rectangle_profile_staged(edit, plane, x_size, y_size)?;
        add_face_staged(edit, profile)
    })
}

/// Adds a planar square face whose first corner is `plane.origin()`.
///
/// The sides follow the plane's positive x and y directions. `size` must be
/// positive and finite.
pub fn add_square(
    g: &mut Model<StandardPayload>,
    plane: Plane,
    size: f64,
) -> Result<FaceKey, FaceCreationError> {
    g.transaction(|edit| {
        let profile = add_rectangle_profile_staged(edit, plane, size, size)?;
        add_face_staged(edit, profile)
    })
}

/// Splits a face-boundary edge and all of its incident face pcurves.
///
/// `parameter` is interpreted as a fraction of the edge's parameter domain. The
/// split is applied across the full topological edge, so pcurves on neighboring
/// faces sharing that edge are split at the corresponding surface points too.
/// The returned [`EdgeSplit`] identifies both resulting edges and the inserted
/// vertex; the original edge key is retained by the first segment.
pub fn split_face_edge<P: DefaultPayload>(
    g: &mut Model<P>,
    face: FaceKey,
    edge: EdgeKey,
    parameter: Fraction,
) -> Result<EdgeSplit, FaceEdgeSplitError> {
    g.transaction(|edit| split_face_edge_staged(edit, face, edge, parameter))
}

/// Splits topology and all incident face pcurves in the same transaction.
pub(crate) fn split_face_edge_staged<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    edge: EdgeKey,
    parameter: Fraction,
) -> Result<EdgeSplit, FaceEdgeSplitError> {
    let boundary_dart = face_edge_dart(edit, face, edge)?;
    let reversed = closed_boundary_curve_reversed(edit, face, edge, boundary_dart)?;
    // Cutting an unmarked edge marks it and leaves one edge, so no pcurve is
    // cut in two. Each incident pcurve is turned to begin at the mark instead,
    // because that is where the marked edge's own span now begins.
    let separates = !matches!(edit.edge_unchecked(edge), Edge::Unmarked(_));
    let pcurves = separates
        .then(|| incident_face_pcurves(edit, edge, parameter))
        .transpose()?
        .unwrap_or_default();
    let rebased = (!separates)
        .then(|| rebased_face_pcurves(edit.model(), edge, parameter))
        .transpose()?
        .unwrap_or_default();

    let split = split_face_boundary_edge(edit, edge, parameter, reversed)?;
    for pcurve in pcurves {
        assign_split_pcurves(edit, pcurve)?;
    }
    for pcurve in rebased {
        assign_rebased_pcurve(edit, pcurve)?;
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
/// Returns one [`FaceImprintSplit`] for each embedding that was applied.
/// Imprints that do not define an applicable cut may produce no split rather
/// than an error; invalid topology or missing geometry is reported as an error.
pub fn split_face_by_imprints<P: DefaultPayload>(
    g: &mut Model<P>,
    face: FaceKey,
    imprints: &[FaceImprint],
) -> Result<Vec<FaceImprintSplit>, FaceImprintSplitError> {
    g.transaction(|edit| split_face_by_imprints_staged(edit, face, imprints))
}

/// Applies every open and closed imprint before the outer transaction commits.
pub fn split_face_by_imprints_staged<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
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
    let open_imprints = imprints_on_one_periodic_image(
        edit.face_attr(face)
            .ok_or(FaceImprintSplitError::MissingFace { face })?
            .surface
            .periodicity(),
        &open_imprints,
    )?;
    if closed_imprints.is_empty() {
        let mut splits = split_ring_face_by_wrapping_chains(edit, face, &open_imprints)?;
        if !splits.is_empty() {
            remap_section_indices(&mut splits, &open_indices);
            return Ok(splits);
        }
    }

    let pcurves = open_imprints
        .iter()
        .map(|imprint| imprint.pcurve.clone())
        .collect::<Vec<_>>();
    let graph = FaceImprintGraph::from_curves(&pcurves)?;
    // Cut for the imprints as handed in, so a corner is placed once, and read
    // back against the open ones the walk uses: a periodic image is the same
    // imprint written elsewhere in the domain, and lands on the same vertex.
    let placed = split_imprint_boundary_endpoints(edit, face, imprints)?;
    let open_placed = open_indices
        .iter()
        .map(|index| placed[*index])
        .collect::<Vec<_>>();
    let mut splits = add_closed_curve_imprint_loops(edit, face, &closed_imprints)?;
    remap_section_indices(&mut splits, &closed_indices);
    let mut open_splits = add_closed_imprint_loops(edit, face, &graph, &open_imprints)?;
    open_splits.extend(split_open_imprints(
        edit,
        vec![face],
        &open_imprints,
        &open_placed,
    )?);
    remap_section_indices(&mut open_splits, &open_indices);
    splits.extend(open_splits);
    check_imprints_realized(face, &splits, &graph, &closed_indices, &open_indices)?;
    Ok(splits)
}

/// Refuses a split that left an imprint with nothing to show for it.
///
/// An imprint the splitter cannot place is a cut the caller asked for and did
/// not get, and returning the other cuts alone hands back a face that is wrong
/// rather than incomplete — the Boolean that asked for it then fails much later,
/// where the span it cannot sew says nothing about the imprint that went
/// missing here.
///
/// An imprint that contributed no graph edge is *not* missing: the graph drops a
/// fragment that duplicates one already recorded, so a chord handed in twice,
/// or in both directions, is realized once and the copy has nothing left to
/// realize.
fn check_imprints_realized(
    face: FaceKey,
    splits: &[FaceImprintSplit],
    graph: &FaceImprintGraph,
    closed_indices: &[usize],
    open_indices: &[usize],
) -> Result<(), FaceImprintSplitError> {
    let realized = splits
        .iter()
        .flat_map(|split| &split.sections)
        .map(|section| section.imprint)
        .collect::<HashSet<_>>();
    let contributing = graph
        .edges()
        .iter()
        .filter_map(|edge| open_indices.get(edge.source_curve).copied())
        .collect::<HashSet<_>>();
    let mut expected = closed_indices
        .iter()
        .copied()
        .chain(contributing)
        .collect::<Vec<_>>();
    expected.sort_unstable();
    match expected.into_iter().find(|index| !realized.contains(index)) {
        Some(imprint) => Err(FaceImprintSplitError::ImprintNotRealized { face, imprint }),
        None => Ok(()),
    }
}

/// Rewrites imprints onto one continuous image of a periodic face's domain.
///
/// Two parameter points a whole period apart name the same point of a periodic
/// surface, so imprints written on different images still meet end to end on
/// the face itself — an arc crossing the seam has to run past the domain's edge
/// to stay continuous, and lands a period away from where the arc it meets was
/// written. Joining them by position alone then finds no walk at all, and a
/// face the walk should cut is left whole: this is the whole of why a torus
/// survives a Boolean that any other support would be split by.
///
/// Only whole periods are ever added, so every imprint still names the points it
/// named, and its 3D curve is left exactly as it was.
fn imprints_on_one_periodic_image(
    periodicity: SurfacePeriodicity,
    imprints: &[FaceImprint],
) -> Result<Vec<FaceImprint>, NurbsError> {
    let periods = match periodicity {
        SurfacePeriodicity::None => return Ok(imprints.to_vec()),
        SurfacePeriodicity::UPeriodic(u) => [Some(u), None],
        SurfacePeriodicity::VPeriodic(v) => [None, Some(v)],
        SurfacePeriodicity::UVPeriodic(u, v) => [Some(u), Some(v)],
    };
    let mut placed = vec![None; imprints.len()];
    for seed in 0..imprints.len() {
        if placed[seed].is_some() {
            continue;
        }
        // The seed anchors its own walk: which image that walk is written on is
        // arbitrary, only that the walk agrees with itself matters.
        let mut frontier = endpoints(&imprints[seed]).to_vec();
        placed[seed] = Some(imprints[seed].clone());
        while let Some(anchor) = frontier.pop() {
            for index in 0..imprints.len() {
                if placed[index].is_some() {
                    continue;
                }
                let Some(offset) = endpoints(&imprints[index])
                    .into_iter()
                    .find_map(|end| period_offset(periods, anchor, end))
                else {
                    continue;
                };
                let moved = FaceImprint::with_section(
                    imprints[index].curve.clone(),
                    imprints[index].pcurve.translated(offset)?,
                );
                frontier.extend(endpoints(&moved));
                placed[index] = Some(moved);
            }
        }
    }
    Ok(placed
        .into_iter()
        .map(|imprint| {
            imprint.expect("every imprint is placed, by its own walk if by nothing else")
        })
        .collect())
}

fn endpoints(imprint: &FaceImprint) -> [Point2; 2] {
    [
        imprint.pcurve.point_at(Fraction::new(0.0)),
        imprint.pcurve.point_at(Fraction::new(1.0)),
    ]
}

/// The whole-period translation carrying `point` onto `anchor`, if one does.
fn period_offset(periods: [Option<f64>; 2], anchor: Point2, point: Point2) -> Option<Vector2> {
    let mut offset = Vector2::zeros();
    for (axis, period) in periods.into_iter().enumerate() {
        let Some(period) = period.filter(|period| period.is_finite() && *period > 0.0) else {
            continue;
        };
        offset[axis] = ((anchor[axis] - point[axis]) / period).round() * period;
    }
    ((point + offset) - anchor)
        .norm()
        .le(&LINEAR_TOLERANCE)
        .then_some(offset)
}

/// Restores original input indices after partitioning closed and open curves.
fn remap_section_indices(splits: &mut [FaceImprintSplit], indices: &[usize]) {
    for section in splits.iter_mut().flat_map(|split| &mut split.sections) {
        section.imprint = indices[section.imprint];
    }
}

fn split_open_imprints<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    mut active_faces: Vec<FaceKey>,
    imprints: &[FaceImprint],
    placed: &[[Option<VertexKey>; 2]],
) -> Result<Vec<FaceImprintSplit>, FaceImprintSplitError> {
    let mut splits = Vec::new();
    loop {
        let mut next_faces = Vec::new();
        let mut progressed = false;

        for face in active_faces {
            let Some(split) = split_one_face_by_imprints(edit, face, imprints, placed)? else {
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

/// Cuts the boundary at every imprint end, returning the corner each became.
///
/// The result is indexed like `imprints`, each entry holding the corner for the
/// imprint's start and for its end. An endpoint that meets no boundary has none.
fn split_imprint_boundary_endpoints<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    imprints: &[FaceImprint],
) -> Result<Vec<[Option<VertexKey>; 2]>, FaceImprintSplitError> {
    let mut placed = Vec::with_capacity(imprints.len());
    for imprint in imprints {
        let mut corners = [None, None];
        for (slot, fraction) in [0.0, 1.0].into_iter().enumerate() {
            let uv = imprint.pcurve.point_at(Fraction::new(fraction));
            corners[slot] = split_boundary_at_uv(edit, face, uv)?;
        }
        placed.push(corners);
    }
    Ok(placed)
}

fn add_closed_curve_imprint_loops<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    imprints: &[&FaceImprint],
) -> Result<Vec<FaceImprintSplit>, FaceImprintSplitError> {
    let boundary_uvs = face_boundary_uvs(edit, face)?;
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
        let mut split = split_face_by_closed_curve_imprint(edit, face, &outside)?;
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

/// One link of a wrapping chain: the imprint as travelled, and where it came from.
#[derive(Clone)]
struct ChainLink {
    imprint: FaceImprint,
    /// Index of the input imprint this link was cut from.
    source: usize,
    /// The part of that input used, backwards when the chain travels it so.
    interval: Interval<Normalized>,
}

/// Imprints that together close on a face's periodic quotient.
///
/// Such a chain bounds no island: in parameter space it is a line one period
/// long, not a closed polygon, so it cuts a ring face in two rather than
/// carving a hole out of one.
struct WrappingChain {
    /// The axis the chain spans exactly one period of.
    axis: Axis2,
    links: Vec<ChainLink>,
}

impl WrappingChain {
    /// The imprints in travel order.
    fn imprints(&self) -> Vec<FaceImprint> {
        self.links.iter().map(|link| link.imprint.clone()).collect()
    }

    /// How far the chain travels along its axis, signed.
    fn travel(&self) -> f64 {
        travel_along(self.axis, &self.imprints())
    }
}

/// Signed travel of a chain of imprints along one parameter axis.
fn travel_along(axis: Axis2, imprints: &[FaceImprint]) -> f64 {
    imprints
        .iter()
        .map(|imprint| {
            axis.of(imprint.pcurve.point_at(Fraction::new(1.0)))
                - axis.of(imprint.pcurve.point_at(Fraction::new(0.0)))
        })
        .sum()
}

/// Cuts a ring face by every imprint chain that wraps its periodic direction.
///
/// Returns no splits — leaving the imprints to the planar paths — unless the
/// face is a ring bounded by exactly two wrapping loops and *every* imprint
/// belongs to a chain spanning one whole period of the wrapped axis. A chain
/// that does not wrap is a chord or an island, which those paths already know
/// how to apply.
fn split_ring_face_by_wrapping_chains<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    imprints: &[FaceImprint],
) -> Result<Vec<FaceImprintSplit>, FaceImprintSplitError> {
    let Some(chains) = wrapping_chains(edit, face, imprints)? else {
        return Ok(Vec::new());
    };
    // A face with no loops at all is cut into two caps rather than two rings:
    // each half is bounded by one copy of the chain and closed on its far side
    // by the degeneracy there. It has no existing loop to pair a copy with, so
    // only the first chain can be taken this way — after it there are caps, not
    // a boundaryless face.
    if edit
        .face_attr(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?
        .is_boundaryless()
    {
        let [chain] = &chains[..] else {
            return Ok(Vec::new());
        };
        return Ok(
            split_boundaryless_face_by_wrapping_chain(edit, face, chain)?
                .into_iter()
                .collect(),
        );
    }
    let mut rings = vec![face];
    let mut splits = Vec::new();
    for chain in chains {
        let Some(target) = ring_face_for_chain(edit, &rings, &chain)? else {
            return Ok(Vec::new());
        };
        let split = split_ring_face_by_wrapping_chain(edit, target, &chain)?;
        rings.push(split.second);
        splits.push(split);
    }
    Ok(splits)
}

/// Cuts a boundaryless face in two along a chain that wraps a periodic
/// direction.
///
/// A sphere cut by a plane: the chain is a latitude circle, and each half is a
/// cap — bounded by one copy of it and closed on its far side by a pole. Which
/// copy bounds which half follows from direction alone, and needs no sampling: a
/// face's interior lies to the left of its boundary, so the copy travelling
/// forward along the wrapped axis bounds the half above it and the reversed copy
/// bounds the half below.
///
/// Returns `None` when the support names no degeneracy on one of the two sides,
/// which would leave a half nothing closes.
fn split_boundaryless_face_by_wrapping_chain<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    chain: &WrappingChain,
) -> Result<Option<FaceImprintSplit>, FaceImprintSplitError> {
    let old_face = edit
        .face_attr(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?
        .clone();
    let axis = chain.axis;
    let transverse = axis.transverse();
    let rows = old_face.surface.degenerate_rows(transverse);
    let at = chain
        .links
        .first()
        .map(|link| transverse.of(link.imprint.pcurve.point_at(Fraction::new(0.0))))
        .ok_or(FaceImprintSplitError::MissingFace { face })?;
    if [DomainSide::Low, DomainSide::High]
        .into_iter()
        .any(|side| side.nearest(at, rows.iter().copied()).is_none())
    {
        return Ok(None);
    }

    let forward = chain.imprints();
    let backward = reversed_imprint_loop(&forward)?;
    let forward_loop = add_section_loop(edit, &old_face.surface, &forward);
    let backward_loop = add_section_loop(edit, &old_face.surface, &backward);
    let section_edges = sew_section_loops(edit, face, &forward_loop, &backward_loop)?;
    for loop_ in [&forward_loop, &backward_loop] {
        edit.add_profile(ProfileAttr::new(loop_.loop_dart));
    }

    // The forward copy travels the chain's own direction; the reversed one
    // travels against it.
    let (high, low) = if chain.travel() > 0.0 {
        (&forward_loop, &backward_loop)
    } else {
        (&backward_loop, &forward_loop)
    };
    let cap = |loop_: &SectionLoop, side: DomainSide| {
        vec![LoopDefinition::from_kind(
            loop_.loop_dart,
            LoopKind::Capping { axis, side },
        )]
    };

    let face_attr = edit
        .face_attr_mut(face)
        .expect("source face must remain staged during a cap split");
    let anchor = face_attr.seed();
    face_attr.set_boundary(cap(high, DomainSide::High), anchor);
    face_attr.pcurves = high.pcurves.clone();

    let second = edit.add_face_split_from(
        face,
        FaceAttr::with_loops(
            old_face.surface,
            cap(low, DomainSide::Low),
            low.pcurves.clone(),
        ),
    );

    Ok(Some(FaceImprintSplit {
        first: face,
        second,
        sections: section_edges
            .into_iter()
            .zip(&chain.links)
            .map(|(edge, link)| FaceImprintSection {
                edge,
                imprint: link.source,
                interval: link.interval,
            })
            .collect(),
    }))
}

/// Reads the imprints as chains wrapping `face`'s periodic direction, if they all are.
fn wrapping_chains<P: Payload>(
    edit: &ModelEdit<'_, P>,
    face: FaceKey,
    imprints: &[FaceImprint],
) -> Result<Option<Vec<WrappingChain>>, FaceImprintSplitError> {
    if imprints.is_empty() {
        return Ok(None);
    }
    let attr = edit
        .face_attr(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?;
    let periods = match attr.surface.periodicity() {
        SurfacePeriodicity::None => return Ok(None),
        SurfacePeriodicity::UPeriodic(u) => [Some(u), None],
        SurfacePeriodicity::VPeriodic(v) => [None, Some(v)],
        SurfacePeriodicity::UVPeriodic(u, v) => [Some(u), Some(v)],
    };
    // A ring names the axis through the loops that wrap it. A boundaryless face
    // has no loops to name one, so every periodic axis is a candidate and the
    // chain's own travel picks the one it spans.
    let candidates = match attr.wrapping().collect::<Vec<_>>()[..] {
        [(_, axis), (_, second_axis)] if axis == second_axis && attr.loops().len() == 2 => {
            vec![axis]
        }
        [] if attr.is_boundaryless() => Axis2::ALL.into_iter().collect(),
        _ => return Ok(None),
    };

    let Some(links) = chain_imprints(imprints) else {
        return Ok(None);
    };
    for axis in candidates {
        let Some(period) = periods[axis.index()] else {
            continue;
        };
        let chains = links
            .iter()
            .cloned()
            .map(|links| WrappingChain { axis, links })
            .collect::<Vec<_>>();
        if chains
            .iter()
            .all(|chain| (chain.travel().abs() - period).abs() <= LINEAR_TOLERANCE)
        {
            return Ok(Some(chains));
        }
    }
    Ok(None)
}

/// Joins imprints end to end into walks, reversing any written backwards.
///
/// Returns `None` when an imprint joins nothing, which is the normal answer
/// for a chord ending on the face's own boundary.
fn chain_imprints(imprints: &[FaceImprint]) -> Option<Vec<Vec<ChainLink>>> {
    let mut remaining = (0..imprints.len()).collect::<Vec<_>>();
    let mut chains = Vec::new();
    while !remaining.is_empty() {
        let source = remaining.remove(0);
        let mut links = vec![ChainLink {
            imprint: imprints[source].clone(),
            source,
            interval: Interval::UNIT,
        }];
        loop {
            let end = links.last()?.imprint.pcurve.point_at(Fraction::new(1.0));
            let meets = |index: &usize| {
                let pcurve = &imprints[*index].pcurve;
                [
                    pcurve.point_at(Fraction::new(0.0)),
                    pcurve.point_at(Fraction::new(1.0)),
                ]
                .iter()
                .any(|point| (point - end).norm() <= LINEAR_TOLERANCE)
            };
            let Some(position) = remaining.iter().position(meets) else {
                break;
            };
            let index = remaining.remove(position);
            let backwards = (imprints[index].pcurve.point_at(Fraction::new(0.0)) - end).norm()
                > LINEAR_TOLERANCE;
            links.push(ChainLink {
                imprint: if backwards {
                    imprints[index].reversed().ok()?
                } else {
                    imprints[index].clone()
                },
                source: index,
                interval: if backwards {
                    Interval::new(1.0, 0.0)
                } else {
                    Interval::new(0.0, 1.0)
                },
            });
        }
        chains.push(links);
    }
    Some(chains)
}

/// The ring among `rings` that `chain` runs across.
///
/// A wrapping chain spans its axis entirely, so what places it is the axis it
/// is transverse to: the chain lies on the ring whose two wrapping loops it
/// runs between.
fn ring_face_for_chain<P: Payload>(
    edit: &ModelEdit<'_, P>,
    rings: &[FaceKey],
    chain: &WrappingChain,
) -> Result<Option<FaceKey>, FaceImprintSplitError> {
    let transverse = chain.axis.transverse();
    let at = |pcurve: &TrimmedCurve2| transverse.of(pcurve.point_at(Fraction::new(0.5)));
    let position = chain
        .links
        .iter()
        .map(|link| at(&link.imprint.pcurve))
        .sum::<f64>()
        / chain.links.len() as f64;
    for &face in rings {
        let attr = edit
            .face_attr(face)
            .ok_or(FaceImprintSplitError::MissingFace { face })?;
        let bounds = attr
            .wrapping()
            .filter_map(|(seed, _)| attr.pcurves.get(&seed).map(at))
            .collect::<Vec<_>>();
        let [low, high] = bounds[..] else {
            continue;
        };
        let (low, high) = (low.min(high), low.max(high));
        if position > low + LINEAR_TOLERANCE && position < high - LINEAR_TOLERANCE {
            return Ok(Some(face));
        }
    }
    Ok(None)
}

/// Cuts a ring face in two along a chain that wraps its periodic direction.
///
/// Each half keeps one of the original wrapping loops and gains one copy of the
/// chain. Which copy goes where follows from direction alone: a face's boundary
/// runs one way round, so the new loop bounding a half must travel against the
/// original loop it is paired with. No seam anchors anything, and no vertex is
/// created beyond the chain's own junctions.
fn split_ring_face_by_wrapping_chain<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    chain: &WrappingChain,
) -> Result<FaceImprintSplit, FaceImprintSplitError> {
    let old_face = edit
        .face_attr(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?
        .clone();
    let axis = chain.axis;
    let forward = chain.imprints();
    let backward = reversed_imprint_loop(&forward)?;

    let forward_loop = add_section_loop(edit, &old_face.surface, &forward);
    let backward_loop = add_section_loop(edit, &old_face.surface, &backward);
    let section_edges = sew_section_loops(edit, face, &forward_loop, &backward_loop)?;
    edit.add_profile(ProfileAttr::new(forward_loop.loop_dart));
    edit.add_profile(ProfileAttr::new(backward_loop.loop_dart));

    let seeds = old_face.wrapping().collect::<Vec<_>>();
    let [(first_seed, _), (second_seed, _)] = seeds[..] else {
        return Err(FaceImprintSplitError::MissingFace { face });
    };
    // The half keeping `first_seed` is bounded by whichever copy runs against it.
    let first_travel = loop_travel(edit, &old_face.pcurves, first_seed, axis)?;
    let (first_new, second_new) = if first_travel * chain.travel() < 0.0 {
        (&forward_loop, &backward_loop)
    } else {
        (&backward_loop, &forward_loop)
    };

    let mut first_pcurves = wrapping_loop_pcurves(edit, face, &old_face.pcurves, first_seed)?;
    first_pcurves.extend(first_new.pcurves.clone());
    let mut second_pcurves = wrapping_loop_pcurves(edit, face, &old_face.pcurves, second_seed)?;
    second_pcurves.extend(second_new.pcurves.clone());
    let ring = |seed: Dart, added: Dart| {
        vec![
            LoopDefinition::from_kind(seed, LoopKind::Wrapping { axis }),
            LoopDefinition::from_kind(added, LoopKind::Wrapping { axis }),
        ]
    };
    let second_boundary = ring(second_seed, second_new.loop_dart);
    let first_boundary = ring(first_seed, first_new.loop_dart);

    let boundary = edit
        .face_unchecked(face)
        .loop_from_seed(second_seed)
        .starting_at(second_seed)
        .ok_or(FaceImprintSplitError::SeedNotOnBoundary {
            face,
            dart: second_seed,
        })?;
    let attachment = CutAttachment::on_loop(edit, &boundary).ok_or(
        FaceImprintSplitError::RingCutAttachment {
            face,
            dart: second_seed,
        },
    )?;
    attachment.move_after(edit, first_new.loop_dart)?;

    let face_attr = edit
        .face_attr_mut(face)
        .expect("source face must remain staged during a ring split");
    let anchor = face_attr.seed();
    face_attr.set_boundary(first_boundary, anchor);
    face_attr.pcurves = first_pcurves;

    let second = edit.add_face_split_from(
        face,
        FaceAttr::with_loops(old_face.surface, second_boundary, second_pcurves),
    );
    cut_between_loops(edit, second, second_seed, second_new.loop_dart)?;

    Ok(FaceImprintSplit {
        first: face,
        second,
        sections: section_edges
            .into_iter()
            .zip(&chain.links)
            .map(|(edge, link)| FaceImprintSection {
                edge,
                imprint: link.source,
                interval: link.interval,
            })
            .collect(),
    })
}

/// Signed travel of one stored boundary loop along `axis`.
fn loop_travel<P: Payload>(
    edit: &ModelEdit<'_, P>,
    pcurves: &HashMap<Dart, TrimmedCurve2>,
    seed: Dart,
    axis: Axis2,
) -> Result<f64, FaceImprintSplitError> {
    let profile = Profile::from_dart(edit, seed).expect("face loop must have a registered profile");
    Ok(profile
        .darts()
        .step_by(2)
        .filter_map(|dart| pcurves.get(&dart))
        .map(|pcurve| {
            axis.of(pcurve.point_at(Fraction::new(1.0)))
                - axis.of(pcurve.point_at(Fraction::new(0.0)))
        })
        .sum())
}

/// What the two halves of a chord split bound, given what the chorded loop did.
///
/// Chording an outer loop splits a disk into two disks. Chording a wrapping loop
/// splits a ring into a ring and a disk: one half still walks a whole period of
/// the axis and the other closes back on itself, so the halves are told apart by
/// how far each travels — no tolerance to tune, since one runs a period and the
/// other runs nothing.
fn chord_loop_kinds<P: Payload>(
    edit: &ModelEdit<'_, P>,
    chorded: LoopKind,
    source: (Dart, &HashMap<Dart, TrimmedCurve2>),
    created: (Dart, &HashMap<Dart, TrimmedCurve2>),
) -> Result<(LoopKind, LoopKind), FaceImprintSplitError> {
    let Some(axis) = chorded.wrapped_axis() else {
        return Ok((chorded, chorded));
    };
    let source_travel = loop_travel(edit, source.1, source.0, axis)?.abs();
    let created_travel = loop_travel(edit, created.1, created.0, axis)?.abs();
    Ok(if source_travel > created_travel {
        (LoopKind::Wrapping { axis }, LoopKind::Outer)
    } else {
        (LoopKind::Outer, LoopKind::Wrapping { axis })
    })
}

/// The stored pcurves of one boundary loop, keyed by its own darts.
fn wrapping_loop_pcurves<P: Payload>(
    edit: &ModelEdit<'_, P>,
    face: FaceKey,
    old_pcurves: &HashMap<Dart, TrimmedCurve2>,
    seed: Dart,
) -> Result<HashMap<Dart, TrimmedCurve2>, FaceImprintSplitError> {
    let profile = Profile::from_dart(edit, seed).expect("face loop must have a registered profile");
    profile
        .darts()
        .step_by(2)
        .map(|dart| {
            let candidates = [
                dart,
                edit.alpha(Dim::Zero, dart),
                edit.alpha(Dim::Two, dart),
            ];
            candidates
                .iter()
                .find_map(|d| old_pcurves.get(d))
                .cloned()
                .map(|pcurve| (dart, pcurve))
                .ok_or(FaceImprintSplitError::MissingPcurve { face, dart })
        })
        .collect()
}

fn split_face_by_closed_curve_imprint<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    imprint: &FaceImprint,
) -> Result<FaceImprintSplit, FaceImprintSplitError> {
    let old_face = edit
        .face_attr(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?
        .clone();
    let outside_loop = add_imprint_section_loop(edit, &old_face.surface, imprint);
    let island_loop = add_imprint_section_loop(edit, &old_face.surface, &reverse_imprint(imprint)?);
    finish_closed_imprint_split(edit, face, old_face, outside_loop, island_loop)
}

fn add_closed_imprint_loops<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    graph: &FaceImprintGraph,
    imprints: &[FaceImprint],
) -> Result<Vec<FaceImprintSplit>, FaceImprintSplitError> {
    let boundary_uvs = face_boundary_uvs(edit, face)?;
    // The winding every hole added here must turn against, read once before
    // any of them is added and while the face is still whole.
    let boundary_area = boundary_winding(edit, face);
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
            .map(|imprint| imprint.pcurve.point_at(Fraction::new(0.0)))
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
                    Interval::new(edge.interval.end.value(), edge.interval.start.value())
                } else {
                    edge.interval
                };
                (edge.source_curve, interval)
            })
            .collect::<Vec<_>>();
        if orient_imprint_loop_against_boundary(boundary_area, &mut loop_imprints)? {
            provenance.reverse();
            for (_, interval) in &mut provenance {
                *interval = interval.reversed();
            }
        }
        let mut split = split_face_by_closed_imprint_loop(edit, face, &loop_imprints)?;
        for (section, (imprint, interval)) in split.sections.iter_mut().zip(provenance) {
            section.imprint = imprint;
            section.interval = interval;
        }
        splits.push(split);
    }

    Ok(splits)
}

fn split_face_by_closed_imprint_loop<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    imprints: &[FaceImprint],
) -> Result<FaceImprintSplit, FaceImprintSplitError> {
    let old_face = edit
        .face_attr(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?
        .clone();
    let island_imprints = reversed_imprint_loop(imprints)?;

    let outside_loop = add_section_loop(edit, &old_face.surface, imprints);
    let island_loop = add_section_loop(edit, &old_face.surface, &island_imprints);
    finish_closed_imprint_split(edit, face, old_face, outside_loop, island_loop)
}

fn finish_closed_imprint_split<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    old_face: FaceAttr<P::F>,
    outside_loop: SectionLoop,
    island_loop: SectionLoop,
) -> Result<FaceImprintSplit, FaceImprintSplitError> {
    let section_edges = sew_section_loops(edit, face, &outside_loop, &island_loop)?;
    edit.add_profile(ProfileAttr::new(outside_loop.loop_dart));
    edit.add_profile(ProfileAttr::new(island_loop.loop_dart));

    // The face the island sits in reaches it along a cut it owns. Without one
    // the new hole would sit in a 2-cell of its own, leaving the face two
    // disconnected pieces that only its loop list joined up.
    let reached_from = old_face.seed();
    cut_between_loops(edit, face, reached_from, outside_loop.loop_dart)?;

    let face_attr = edit
        .face_attr_mut(face)
        .expect("source face must remain staged during a closed-loop split");
    face_attr.push_inner(outside_loop.loop_dart);
    face_attr.pcurves.extend(outside_loop.pcurves);

    let second = edit.add_face_split_from(
        face,
        FaceAttr::with_pcurves(
            old_face.surface,
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
                interval: Interval::UNIT,
            })
            .collect(),
    })
}

struct SectionLoop {
    loop_dart: Dart,
    edges: Vec<SectionLoopEdge>,
    pcurves: HashMap<Dart, TrimmedCurve2>,
}

#[derive(Clone)]
struct SectionLoopEdge {
    dart: Dart,
    start_uv: Point2,
    end_uv: Point2,
    curve: TrimmedCurve,
    pcurve: TrimmedCurve2,
}

fn add_section_loop<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    surface: &Surface,
    imprints: &[FaceImprint],
) -> SectionLoop {
    let n = imprints.len();
    let darts = (0..2 * n).map(|_| edit.add_dart()).collect::<Vec<_>>();

    for edge in 0..n {
        edit.link(Dim::Zero, darts[2 * edge], darts[2 * edge + 1])
            .expect("fresh section edge darts must be alpha0-free");
    }
    for edge in 0..n {
        let end = darts[2 * edge + 1];
        let next_start = darts[2 * ((edge + 1) % n)];
        edit.link(Dim::One, end, next_start)
            .expect("fresh section loop darts must be alpha1-free");
    }

    // A loop of one section closes on itself, so its single 0-cell is where
    // that section comes back round rather than a place two of them meet.
    // Nothing is bounded there, so no vertex is registered; `sew_section_loops`
    // classifies the 0-cell inside the edge instead, which is what leaves a
    // closed section an unmarked edge rather than a circle with an invented
    // corner.
    if n > 1 {
        for vertex in 0..n {
            let dart = edit.cell_representative(darts[2 * vertex], Dim::Zero);
            let uv = imprints[vertex].pcurve.point_at(Fraction::new(0.0));
            edit.add_vertex(VertexAttr::new(dart, surface.point_at(uv.x, uv.y)));
        }
    }

    let edges = (0..n)
        .map(|edge| {
            let imprint = &imprints[edge];
            SectionLoopEdge {
                dart: darts[2 * edge],
                start_uv: imprint.pcurve.point_at(Fraction::new(0.0)),
                end_uv: imprint.pcurve.point_at(Fraction::new(1.0)),
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
    edit: &mut ModelEdit<'_, P>,
    surface: &Surface,
    imprint: &FaceImprint,
) -> SectionLoop {
    add_section_loop(edit, surface, std::slice::from_ref(imprint))
}

fn sew_section_loops<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
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
            Ok((outside_edge, edit.alpha(Dim::Zero, island_edge.dart)))
        })
        .collect::<Result<Vec<_>, FaceImprintSplitError>>()?;
    // One pair is one section closing on itself, which `add_section_loop` left
    // without a vertex.
    let closes_on_itself = pairs.len() == 1;
    let mut edges = Vec::with_capacity(pairs.len());
    for (outside_edge, island_end) in pairs {
        edit.sew(Dim::Two, outside_edge.dart, island_end)
            .map_err(|source| FaceImprintSplitError::SectionLoopSewFailed { face, source })?;
        // The edge's default traversal follows the section, including its
        // directed source interval. Orient the support to that traversal before
        // discarding the span: a marked circle's corner cannot encode direction.
        let interval = outside_edge.curve.interval();
        let curve = if interval.end.value() < interval.start.value() {
            outside_edge.curve.curve().reversed()
        } else {
            outside_edge.curve.curve().clone()
        };
        let key = edit.add_edge(EdgeAttr::new(outside_edge.dart, curve));
        if closes_on_itself {
            edit.own_cell(Dim::Zero, outside_edge.dart, EntityOwner::Edge(key));
        }
        edges.push(key);
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

/// Which way a face's boundary turns in its own parameter domain.
///
/// Read from the realized domain, never from the face's corners. A ring face's
/// loops each run exactly one period and carry no winding of their own, and
/// the corners they happen to have — the two ends of a chord imprinted in the
/// same pass, say — are not a polygon: an area taken over them has a sign
/// decided by how many corners some unrelated cut left on the rim, so one more
/// arriving flips it.
///
/// A face nothing encloses has no boundary to read and needs none: it covers
/// the support's whole domain as the support is parameterized, so
/// counter-clockwise is its material side, and the positive sense stands in.
fn boundary_winding<P: Payload>(edit: &ModelEdit<'_, P>, face: FaceKey) -> f64 {
    edit.face(face)
        .and_then(|view| view.boundary_signed_area())
        .filter(|area| area.abs() > LINEAR_TOLERANCE)
        .unwrap_or(1.0)
}

/// Winds an imprint loop so it cuts a hole rather than bounding one.
///
/// A face's material lies to the left of its boundary, so a hole runs the
/// opposite way round from whatever encloses the face, which is what
/// `boundary_area` states. Leaving that to whichever way the intersection
/// chain happened to be walked instead makes the face's winding depend on which
/// Boolean operand it belonged to — the same cut then sews up correctly one way
/// round and inside-out the other.
fn orient_imprint_loop_against_boundary(
    boundary_area: f64,
    imprints: &mut Vec<FaceImprint>,
) -> Result<bool, NurbsError> {
    let loop_uvs = imprints
        .iter()
        .flat_map(|imprint| imprint.pcurve.sample(16).into_iter().take(16))
        .collect::<Vec<_>>();
    let loop_area = signed_area(&loop_uvs);

    if loop_area.abs() <= LINEAR_TOLERANCE {
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

/// Cuts the boundary where an imprint meets it, and says which corner that is.
///
/// The corner is returned rather than left to be found again: the caller has to
/// know which vertex this endpoint became, and asking the boundary afterwards
/// puts it back to comparing a projected corner with a traced endpoint.
fn split_boundary_at_uv<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    uv: Point2,
) -> Result<Option<VertexKey>, FaceImprintSplitError> {
    let boundary = face_boundary_edges(edit, face)?;
    if let Some(index) = snap_boundary_corner_in(&boundary, uv) {
        return Ok(boundary[index].vertex);
    }

    let Some(target) = boundary_edge_at_uv(edit, face, uv)? else {
        return Ok(None);
    };

    let edge = Edge::new(edit, target.edge);
    let curve = edge_curve(&edge);
    let face_view = edit
        .face(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?;
    let surface = face_view.surface();
    let interval = edge.parameter_interval();
    let mut parameter = curve.parameter_at(surface.point_at(uv.x, uv.y));
    if let Periodicity::Periodic(period) = curve.periodicity() {
        let domain = interval.ordered();
        while parameter < NativeParam::new(domain.start.value() - LINEAR_TOLERANCE) {
            parameter += period;
        }
        while parameter > NativeParam::new(domain.end.value() + LINEAR_TOLERANCE) {
            parameter -= period;
        }
    }
    // The splitter cuts at a fraction of the edge's span, and the span that
    // fraction is of is the one the parameter was just brought onto.
    let parameter = interval.fraction_of(parameter);
    match split_face_edge_staged(edit, face, target.edge, parameter) {
        Ok(split) => Ok(Some(split.vertex())),
        // A cut that lands on an end of the edge adds no vertex because one is
        // already there, so the corner is the one the boundary now reports.
        Err(FaceEdgeSplitError::EdgeSplitFailed(EdgeSplitError::DegenerateSplit { .. })) => {
            let boundary = face_boundary_edges(edit, face)?;
            Ok(snap_boundary_corner_in(&boundary, uv).and_then(|index| boundary[index].vertex))
        }
        Err(error) => Err(error.into()),
    }
}

fn split_one_face_by_imprints<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    imprints: &[FaceImprint],
    placed: &[[Option<VertexKey>; 2]],
) -> Result<Option<FaceImprintSplit>, FaceImprintSplitError> {
    let face_attr = edit
        .face_attr(face)
        .ok_or(FaceImprintSplitError::MissingFace { face })?;

    let old_face = face_attr.clone();
    // A chord runs between two corners of one loop, so each bounding loop is
    // tried on its own: a ring has two, and the imprint chord lands on one.
    let bounding = bounding_loops(old_face.loops());
    for chorded in bounding {
        let boundary = loop_boundary_edges(edit, face, chorded.seed())?;
        let Some(cut) = FaceImprintCut::from_chain(imprints, &boundary, placed)? else {
            continue;
        };
        let split = apply_face_chord_split(edit, face, old_face, chorded, &cut)?;
        return Ok(Some(split));
    }
    Ok(None)
}

/// Adds a planar disk face bounded by one circular edge.
///
/// The circle is centered at `plane.origin()` and uses the plane orientation.
/// `radius` must be positive and finite.
pub fn add_circle(
    g: &mut Model<StandardPayload>,
    plane: Plane,
    radius: f64,
) -> Result<FaceKey, FaceCreationError> {
    g.transaction(|edit| add_circle_staged(edit, plane, radius))
}

/// Builds a circular boundary and its face within one staged operation.
fn add_circle_staged(
    edit: &mut ModelEdit<'_, StandardPayload>,
    plane: Plane,
    radius: f64,
) -> Result<FaceKey, FaceCreationError> {
    let edge = add_circle_edge_staged(edit, plane.clone(), radius)?;
    let loop_dart = edit.edge_attr_unchecked(edge).dart;
    edit.add_profile(ProfileAttr::new(loop_dart));
    let profile =
        Profile::from_dart(edit, loop_dart).expect("face loop must have a registered profile");
    let pcurves = profile_pcurves(&profile, &plane)?;
    let face_key = edit.add_face(FaceAttr::with_pcurves(
        Surface::Plane(plane),
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
    fn follow(
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
struct BoundaryEdgeTarget {
    edge: EdgeKey,
}

fn face_boundary_uvs<P: Payload>(
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
fn face_boundary_edges<P: Payload>(
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
struct BoundaryCorner {
    uv: Point2,
    pcurve: TrimmedCurve2,
    vertex: Option<VertexKey>,
}

fn loop_boundary_edges<P: Payload>(
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
fn bounding_loops(boundary: &[LoopDefinition]) -> Vec<LoopDefinition> {
    boundary
        .iter()
        .filter(|loop_| loop_.kind() != LoopKind::Inner)
        .copied()
        .collect()
}

fn boundary_edge_at_uv<P: Payload>(
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

fn pcurve_fraction_at(pcurve: &TrimmedCurve2, point: Point2) -> Option<Fraction> {
    pcurve.try_parameter_at(point, LINEAR_TOLERANCE)
}

fn boundary_edge_key<P: Payload>(
    g: &Model<P>,
    dart: Dart,
) -> Result<EdgeKey, FaceImprintSplitError> {
    g.cell_key::<Cell1>(dart)
        .ok_or(FaceImprintSplitError::MissingBoundaryEdge { dart })
}

/// [`snap_boundary_corner`] over corners paired with their outgoing pcurves.
fn snap_boundary_corner_in(boundary: &[BoundaryCorner], uv: Point2) -> Option<usize> {
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
fn corner_of_vertex(boundary: &[BoundaryCorner], vertex: VertexKey) -> Option<usize> {
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
fn imprint_endpoint_corner(
    boundary: &[BoundaryCorner],
    placed: Option<VertexKey>,
    uv: Point2,
) -> Option<usize> {
    placed
        .and_then(|vertex| corner_of_vertex(boundary, vertex))
        .or_else(|| snap_boundary_corner_in(boundary, uv))
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
fn retraces_boundary(boundary: &[BoundaryCorner], sections: &[(usize, bool, FaceImprint)]) -> bool {
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
fn corner_cut<P: Payload>(
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
fn apply_face_chord_split<P: Payload>(
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
fn partition_inner_loops<P: Payload>(
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
enum ChordHalf {
    /// The half the source face keeps.
    Source,
    /// The half that becomes a new face.
    Created,
}

fn sampled_loop_uvs<P: Payload>(
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

fn sampled_loop_contains(boundary: &[Point2], point: Point2) -> bool {
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

fn extend_loop_pcurves<P: Payload>(
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

fn split_face_pcurves<P: Payload>(
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

/// Adds a planar annular face with concentric circular boundary loops.
///
/// The outer loop follows `plane` orientation and the inner loop is reversed to
/// represent a hole. Both radii must be positive and finite, and `outer_radius`
/// must be greater than `inner_radius`.
pub fn add_annulus(
    g: &mut Model<StandardPayload>,
    plane: Plane,
    outer_radius: f64,
    inner_radius: f64,
) -> Result<FaceKey, FaceCreationError> {
    g.transaction(|edit| add_annulus_staged(edit, plane, outer_radius, inner_radius))
}

/// Builds both annulus boundaries, bridges them, and registers the face.
///
/// The two rims are joined by a **bridge**: one 1-cell the face's boundary
/// walk uses twice, with the two uses `alpha2`-linked to each other. Without it
/// the rims would sit in two disconnected 2-cells and only the face attribute
/// would say they belong to the same face; with it the face is one raw 2-cell
/// and the involutions alone carry that fact.
///
/// The bridge is scaffold, not shape: it is owned by the face, so it is never
/// emitted as a boundary and carries no logical edge of its own. Each of its
/// feet is the 0-cell where a rim closes, owned by that rim's edge — which is
/// what keeps both circles unmarked.
fn add_annulus_staged(
    edit: &mut ModelEdit<'_, StandardPayload>,
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

    // The boundary word, in cyclic order: out along the bridge, once round the
    // hole, back along the bridge, once round the outer rim. Reading the bridge
    // before and after the hole is what makes the walk turn out of the hole and
    // back onto the outer rim rather than round the hole for ever.
    let slots: Vec<[Dart; 2]> = (0..4).map(|_| [edit.add_dart(), edit.add_dart()]).collect();
    for slot in &slots {
        edit.link(Dim::Zero, slot[0], slot[1])?;
    }
    for i in 0..slots.len() {
        edit.link(Dim::One, slots[i][1], slots[(i + 1) % slots.len()][0])?;
    }
    let [bridge_out, inner_loop_slot, bridge_back, outer_loop_slot] =
        [0, 1, 2, 3].map(|i| slots[i]);
    edit.link(Dim::Two, bridge_out[0], bridge_back[1])?;
    edit.link(Dim::Two, bridge_out[1], bridge_back[0])?;

    let inner_plane = Plane::new(plane.origin(), plane.x_dir(), -plane.normal());
    let outer_edge = edit.add_edge(EdgeAttr::new(
        outer_loop_slot[0],
        Curve::circle(plane.clone(), outer_radius),
    ));
    let inner_edge = edit.add_edge(EdgeAttr::new(
        inner_loop_slot[0],
        Curve::circle(inner_plane, inner_radius),
    ));
    // Each rim closes where the bridge meets it. That 0-cell is interior to the
    // rim, not a corner: nothing else meets there, and the rim stays unmarked.
    edit.own_cell(Dim::Zero, outer_loop_slot[0], EntityOwner::Edge(outer_edge));
    edit.own_cell(Dim::Zero, inner_loop_slot[0], EntityOwner::Edge(inner_edge));

    let outer_loop = outer_loop_slot[0];
    let inner_loop = inner_loop_slot[0];
    edit.add_profile(ProfileAttr::new(outer_loop));
    edit.add_profile(ProfileAttr::new(inner_loop));

    let outer_profile =
        Profile::from_dart(edit, outer_loop).expect("outer loop must have a registered profile");
    let inner_profile =
        Profile::from_dart(edit, inner_loop).expect("inner loop must have a registered profile");
    let mut pcurves = profile_pcurves(&outer_profile, &plane)?;
    pcurves.extend(profile_pcurves(&inner_profile, &plane)?);

    let face_key = edit.add_face(FaceAttr::with_pcurves(
        Surface::Plane(plane),
        outer_loop,
        vec![inner_loop],
        pcurves,
    ));
    // The bridge belongs to the face's interior, which is what stops the
    // boundary walk emitting it and lets the walk cross it into the hole.
    edit.own_cell(Dim::One, bridge_out[0], EntityOwner::Face(face_key));
    Ok(face_key)
}

fn face_edge_dart<P: Payload>(
    g: &Model<P>,
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
    let profile_darts: Vec<Dart> = face_attr
        .darts()
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
    g: &Model<P>,
    face: FaceKey,
    edge: EdgeKey,
    dart: Dart,
) -> Result<bool, FaceEdgeSplitError> {
    let edge_view =
        Edge::from_dart(g, dart).ok_or(FaceEdgeSplitError::EdgeNotOnFace { face, edge })?;
    // Only a closed edge can have its pcurve reversed relative to its curve
    // without that showing up in its endpoints — asked of the map rather than by
    // measuring whether two points happen to land within a tolerance.
    let (Edge::Marked(_) | Edge::Unmarked(_)) = edge_view else {
        return Ok(false);
    };

    let face_view = g
        .face(face)
        .ok_or(FaceEdgeSplitError::MissingFace { face })?;
    let pcurve = face_view
        .pcurve(dart)
        .ok_or(FaceEdgeSplitError::MissingPcurve { face, dart })?;
    // Against the edge's own span, not the support's extent: a marked closed
    // edge begins at its corner, which is not in general where the support's
    // domain starts. Reading the support would compare the sample with two
    // points it is nowhere near, and a closed span's ends are the one place
    // the two directions are told apart.
    let reference = g
        .edge(edge)
        .ok_or(FaceEdgeSplitError::EdgeSplitFailed(
            EdgeSplitError::MissingEdge { edge },
        ))?
        .parameter_interval();
    let curve = edge_view.curve();
    let fraction = 1.0e-4;
    let sample_uv = pcurve.point_at(Fraction::new(fraction));
    let sample = face_view.point_at(sample_uv.x, sample_uv.y);
    let forward = curve.point_at(reference.at(Fraction::new(fraction)));
    let reverse = curve.point_at(reference.at(Fraction::new(1.0 - fraction)));
    let against = (sample - reverse).norm_squared() < (sample - forward).norm_squared();

    // A closed boundary starts and ends at one point, so only its direction
    // says which half is which. The sample above follows the face from `dart`,
    // while the split hands its first half to the edge's own reference dart. On
    // an open edge the two endpoints settle that; here, when those two darts
    // run opposite ways, the halves have to be read the other way round.
    Ok(match g.edge_orientation_at_dart(edge, dart) {
        Orientation::Same => against,
        Orientation::Reversed => !against,
    })
}

/// Where a fraction of an edge's own span falls in model space.
fn edge_point_at<P: Payload>(
    g: &Model<P>,
    edge: EdgeKey,
    parameter: Fraction,
) -> Result<Point3, FaceEdgeSplitError> {
    let edge_view = g.edge(edge).ok_or(FaceEdgeSplitError::EdgeSplitFailed(
        EdgeSplitError::MissingEdge { edge },
    ))?;
    Ok(edge_view
        .curve()
        .point_at(edge_view.parameter_interval().at(parameter)))
}

/// Every boundary dart a face uses `edge` at.
///
/// A seam has two boundary occurrences on one face, each with its own UV curve.
fn edge_face_occurrences<P: Payload>(
    g: &Model<P>,
    edge: EdgeKey,
) -> Result<HashSet<(FaceKey, Dart)>, FaceEdgeSplitError> {
    let edge_view = g.edge(edge).ok_or(FaceEdgeSplitError::EdgeSplitFailed(
        EdgeSplitError::MissingEdge { edge },
    ))?;
    let mut occurrences = HashSet::new();
    for face in edge_view.faces() {
        for boundary in g.face_unchecked(face.key()).edges() {
            if boundary.key() == edge {
                occurrences.insert((face.key(), boundary.dart()));
            }
        }
    }
    Ok(occurrences)
}

fn incident_face_pcurves<P: Payload>(
    g: &Model<P>,
    edge: EdgeKey,
    parameter: Fraction,
) -> Result<Vec<IncidentFacePcurve>, FaceEdgeSplitError> {
    let split_point = edge_point_at(g, edge, parameter)?;
    edge_face_occurrences(g, edge)?
        .into_iter()
        .map(|(face, dart)| {
            let face_view = g
                .face(face)
                .ok_or(FaceEdgeSplitError::MissingFace { face })?;
            let pcurve = face_view
                .pcurve(dart)
                .ok_or(FaceEdgeSplitError::MissingPcurve { face, dart })?;
            let surface = face_view.surface();
            let uv = periodic_image_near_pcurve(surface, &pcurve, surface.param_at(split_point)?);
            let fraction = pcurve
                .try_parameter_at(uv, LINEAR_TOLERANCE)
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

/// Turns each face pcurve of a closed edge to begin where a mark lands.
///
/// A marked closed edge derives its span from its corner, so the span begins at
/// the mark rather than wherever the support's own domain does. A pcurve has no
/// corner to derive from and carries its own span, so it has to be turned with
/// it: left where it was it would run the same closed curve from a different
/// starting point, and every later cut would read the wrong arc of it.
fn rebased_face_pcurves<P: Payload>(
    g: &Model<P>,
    edge: EdgeKey,
    parameter: Fraction,
) -> Result<Vec<RebasedFacePcurve>, FaceEdgeSplitError> {
    let mark = edge_point_at(g, edge, parameter)?;
    edge_face_occurrences(g, edge)?
        .into_iter()
        .filter_map(|(face, dart)| rebased_face_pcurve(g, face, dart, mark).transpose())
        .collect()
}

/// One face's pcurve for a closed edge, turned to begin at `mark`.
///
/// `None` where there is nothing to turn: a pcurve that does not close in
/// parameter space runs between two distinct ends — a seam crossing, where the
/// face's own domain is what pins them — and only a closed one is free to say
/// where it starts.
fn rebased_face_pcurve<P: Payload>(
    g: &Model<P>,
    face: FaceKey,
    dart: Dart,
    mark: Point3,
) -> Result<Option<RebasedFacePcurve>, FaceEdgeSplitError> {
    let face_view = g
        .face(face)
        .ok_or(FaceEdgeSplitError::MissingFace { face })?;
    let pcurve = face_view
        .pcurve(dart)
        .ok_or(FaceEdgeSplitError::MissingPcurve { face, dart })?;
    if !pcurve.is_closed() {
        return Ok(None);
    }
    let surface = face_view.surface();
    let uv = periodic_image_near_pcurve(surface, &pcurve, surface.param_at(mark)?);
    if !pcurve.contains(uv, LINEAR_TOLERANCE) {
        return Err(FaceEdgeSplitError::SplitPointNotOnPcurve { face, dart });
    }
    // The span keeps its length and direction and only moves its ends, so the
    // turned pcurve still runs the whole closed curve once.
    let start = pcurve.native_parameter_at(uv);
    let span = Interval::new(start, start + pcurve.interval().delta());
    Ok(Some(RebasedFacePcurve {
        face,
        dart: stored_pcurve_dart(g, face, dart).unwrap_or(dart),
        pcurve: TrimmedCurve2::new(pcurve.into_curve(), span),
    }))
}

/// The dart a face keeps a boundary occurrence's pcurve under.
///
/// A pcurve is stored once per boundary occurrence but looked up from any of
/// the darts that occurrence covers, so replacing one means writing back to the
/// key it already lives at rather than adding a second entry beside it.
fn stored_pcurve_dart<P: Payload>(g: &Model<P>, face: FaceKey, dart: Dart) -> Option<Dart> {
    let attr = g.face_attr(face)?;
    [dart, g.alpha(Dim::Zero, dart), g.alpha(Dim::Two, dart)]
        .into_iter()
        .find(|candidate| attr.pcurves.contains_key(candidate))
}

/// Stores a turned pcurve back on the face it belongs to.
fn assign_rebased_pcurve<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    rebased: RebasedFacePcurve,
) -> Result<(), FaceEdgeSplitError> {
    let face_attr = edit
        .face_attr_mut(rebased.face)
        .ok_or(FaceEdgeSplitError::MissingFace { face: rebased.face })?;
    face_attr.pcurves.insert(rebased.dart, rebased.pcurve);
    Ok(())
}

fn periodic_image_near_pcurve(surface: &Surface, pcurve: &TrimmedCurve2, mut uv: Point2) -> Point2 {
    let start = pcurve.point_at(Fraction::new(0.0));
    let end = pcurve.point_at(Fraction::new(1.0));
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
    edit: &mut ModelEdit<'_, P>,
    pcurve: IncidentFacePcurve,
) -> Result<(), FaceEdgeSplitError> {
    let second_dart = edit.alpha(Dim::One, edit.alpha(Dim::Zero, pcurve.dart));
    let (first_pcurve, second_pcurve) = pcurve.pcurve.split_at(pcurve.fraction);
    let face_attr = edit
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
    g: &mut Model<StandardPayload>,
    plane: Plane,
    outer: &[Point3],
    holes: &[&[Point3]],
) -> Result<FaceKey, FaceCreationError> {
    g.transaction(|edit| add_polygon_with_holes_staged(edit, plane, outer, holes))
}

/// Builds the outer polygon and all hole loops before registering the face.
fn add_polygon_with_holes_staged(
    edit: &mut ModelEdit<'_, StandardPayload>,
    plane: Plane,
    outer: &[Point3],
    holes: &[&[Point3]],
) -> Result<FaceKey, FaceCreationError> {
    validate_polygon(outer)?;
    for hole in holes {
        validate_polygon(hole)?;
    }

    let outer_profile = add_polygon_staged(edit, outer);
    let outer_loop = edit.profile_attr_unchecked(outer_profile).dart;
    let mut inner_loops = Vec::with_capacity(holes.len());
    let outer_profile =
        Profile::from_dart(edit, outer_loop).expect("outer loop must have a registered profile");
    let mut pcurves = profile_pcurves(&outer_profile, &plane)?;

    for hole in holes {
        let inner_profile = add_polygon_staged(edit, hole);
        let inner_loop = edit.profile_attr_unchecked(inner_profile).dart;
        let inner_profile = Profile::from_dart(edit, inner_loop)
            .expect("inner loop must have a registered profile");
        pcurves.extend(profile_pcurves(&inner_profile, &plane)?);
        inner_loops.push(inner_loop);
    }

    let face_key = edit.add_face(FaceAttr::with_pcurves(
        Surface::Plane(plane),
        outer_loop,
        inner_loops.clone(),
        pcurves,
    ));
    // Each hole is reached from the outer boundary along a cut the face owns,
    // so the face is one 2-cell rather than one boundary per hole with nothing
    // joining them.
    for inner_loop in inner_loops {
        cut_between_loops(edit, face_key, outer_loop, inner_loop)?;
    }
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
pub fn add_polygon<P: DefaultPayload>(
    g: &mut Model<P>,
    corners: &[Point3],
) -> crate::topology::shape_keys::ProfileKey {
    g.transaction(|edit| Ok::<_, ModelEditError>(add_polygon_staged(edit, corners)))
        .expect("fresh polygon operation must commit")
}

/// Creates and links polygon segments without opening another transaction scope.
pub(crate) fn add_polygon_staged<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    corners: &[Point3],
) -> crate::topology::shape_keys::ProfileKey {
    assert!(
        corners.len() >= 3,
        "add_polygon requires at least 3 corners, got {}",
        corners.len()
    );
    let n = corners.len();
    let darts: Vec<Dart> = (0..2 * n).map(|_| edit.add_dart()).collect();

    for i in 0..n {
        edit.sew(Dim::Zero, darts[2 * i], darts[2 * i + 1])
            .expect("fresh polygon edge darts must be alpha0-free");
    }
    for i in 0..n {
        let a = darts[2 * i + 1];
        let b = darts[(2 * i + 2) % (2 * n)];
        edit.sew(Dim::One, a, b)
            .expect("fresh polygon boundary darts must be alpha1-free");
    }

    for i in 0..n {
        let dart = edit.cell_representative(darts[2 * i], Dim::Zero);
        edit.add_vertex(VertexAttr::new(dart, corners[i]));
    }

    for i in 0..n {
        let edge_dart = darts[2 * i];
        let curve = Curve::line(corners[i], corners[(i + 1) % n]);
        edit.add_edge(EdgeAttr::new(edge_dart, curve));
    }
    edit.add_profile(crate::topology::attributes::ProfileAttr::new(darts[0]))
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
pub fn reverse_face_winding<P: Payload>(edit: &mut ModelEdit<'_, P>, face: FaceKey) {
    let Some(face_attr) = edit.face_attr(face).cloned() else {
        return;
    };

    // Reversing is atomic over the whole boundary: every loop seed becomes its
    // `alpha0`, whatever that loop bounds, and every pcurve is reversed onto
    // the dart that now carries it.
    let mut loops = face_attr.loops().to_vec();
    for loop_ in &mut loops {
        loop_.set_seed(edit.alpha(Dim::Zero, loop_.seed()));
    }
    let pcurves = face_attr
        .face(edit)
        .edges()
        .into_iter()
        .filter_map(|edge| {
            face_attr
                .pcurves
                .get(&edge.dart())
                .map(|pcurve| (edit.alpha(Dim::Zero, edge.dart()), pcurve.reversed()))
        })
        .collect();

    if let Some(face) = edit.face_attr_mut(face) {
        let anchor = face.seed();
        face.set_boundary(loops, anchor);
        face.pcurves = pcurves;
    }
}
