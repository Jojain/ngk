use std::collections::HashSet;

use nalgebra::Vector3;
use thiserror::Error;

use crate::builders::faces::{FaceImprint, FaceImprintSplitError, split_face_by_imprints};
use crate::builders::split::{SplitError, apply_edge_split_parameters};
use crate::geometry::dim3::intersections::{
    intersect_curve_surface_with_options, intersect_curves_with_options,
    intersect_surfaces_with_options,
};
use crate::geometry::{
    BBox, Curve, Curve2, CurveCurveIntersection, CurveSurfaceIntersection, IntersectionError,
    IntersectionOptions, Interval, LINEAR_TOLERANCE, NurbsCurve, NurbsCurve2, NurbsError,
    NurbsSurface, Plane, Point2, Point3, PointCoincidence, Surface, SurfaceSurfaceIntersection,
};
use crate::topology::attributes::{FaceAttr, SolidAttr};
use crate::topology::edge::Edge;
use crate::topology::face::Face;
use crate::topology::gmap::{Cell0, Cell2, Dart, Dim, GMap, MergeTopology, TopologyMerge};
use crate::topology::payload::Payload;
use crate::topology::profile::Loop;
use crate::topology::shape::{Shape, SolidTag};
use crate::topology::shape_keys::{EdgeKey, FaceKey, SolidKey, VertexKey};
use crate::topology::solid::Solid;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BooleanSource {
    Object,
    Tool,
}

#[derive(Debug, Error)]
pub enum BooleanError {
    #[error("failed to convert face surface to NURBS")]
    NurbsConversion(#[from] NurbsError),
    #[error("failed to intersect face surfaces")]
    Intersection(#[from] IntersectionError),
    #[error("edge {dart:?} has no attached curve")]
    MissingEdgeCurve { dart: Dart },
    #[error("edge handle {edge:?} cannot be resolved in its operand map")]
    MissingEdgeHandle { edge: EdgeHandle },
    #[error("edge {edge:?} has no incident face in its operand map")]
    MissingIncidentFace { edge: EdgeHandle },
    #[error("face handle {face:?} cannot be resolved in its operand map")]
    MissingFaceHandle { face: FaceHandle },
    #[error("edge {edge:?} has missing endpoint geometry")]
    MissingEndpointGeometry { edge: EdgeKey },
    #[error("split parameter {parameter} does not belong to a current segment of edge {edge:?}")]
    MissingSplitSegment { edge: EdgeHandle, parameter: f64 },
    #[error("failed to apply split parameter {parameter} to edge {edge:?}")]
    EdgeSplitApplicationFailed {
        edge: EdgeHandle,
        parameter: f64,
        source: SplitError,
    },
    #[error("failed to split face {face:?} by imprints")]
    FaceSplitApplicationFailed {
        face: FaceHandle,
        source: FaceImprintSplitError,
    },
    #[error("solid handle {solid:?} cannot be resolved in its operand map")]
    MissingSolidHandle { solid: SolidKey },
    #[error("face {face:?} has no sample point for classification")]
    MissingFaceSample { face: FaceKey },
    #[error("face has no usable UV loop sample for classification")]
    MissingFaceUvSample,
    #[error("boolean {operation:?} selected no faces for the result")]
    EmptyResultSelection { operation: BooleanOperation },
    #[error(
        "boolean {operation:?} selected faces do not form a closed shell ({free_edge_count} free result edges)"
    )]
    OpenResultShell {
        operation: BooleanOperation,
        free_edge_count: usize,
    },
    #[error("failed to sew selected result edges {first:?} and {second:?}: {reason}")]
    ResultEdgeSewFailed {
        first: Dart,
        second: Dart,
        reason: &'static str,
    },
    #[error("face {face:?} has no orientation sample")]
    MissingOrientationSample { face: FaceKey },
    #[error("face {face:?} uses a surface that cannot be orientation-flipped yet")]
    UnsupportedFaceOrientationFlip { face: FaceKey },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct FaceHandle {
    pub source: BooleanSource,
    pub dart: Dart,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct EdgeHandle {
    pub source: BooleanSource,
    pub dart: Dart,
}

#[derive(Clone)]
pub struct BooleanFace {
    pub handle: FaceHandle,
    bbox: BBox,
    surface: NurbsSurface,
}

impl BooleanFace {
    fn from_face<P: Payload>(
        source: BooleanSource,
        face: &Face<'_, P>,
    ) -> Result<Self, NurbsError> {
        Ok(Self {
            handle: FaceHandle {
                source,
                dart: face.outer_loop().dart,
            },
            bbox: face_bbox(face),
            surface: face.surface().to_nurbs()?,
        })
    }

    pub fn handle(&self) -> FaceHandle {
        self.handle
    }

    pub fn source(&self) -> BooleanSource {
        self.handle.source
    }

    pub fn dart(&self) -> Dart {
        self.handle.dart
    }
}

#[derive(Clone)]
pub struct BooleanEdge {
    pub handle: EdgeHandle,
    bbox: BBox,
    curve: NurbsCurve,
}

impl BooleanEdge {
    fn from_edge<P: Payload>(
        source: BooleanSource,
        edge: &Edge<'_, P>,
    ) -> Result<Self, BooleanError> {
        let curve = edge
            .curve()
            .ok_or(BooleanError::MissingEdgeCurve { dart: edge.dart })?
            .to_nurbs()?;

        Ok(Self {
            handle: EdgeHandle {
                source,
                dart: edge_handle_dart(edge),
            },
            bbox: edge_bbox(edge, &curve),
            curve,
        })
    }

    pub fn handle(&self) -> EdgeHandle {
        self.handle
    }

    pub fn source(&self) -> BooleanSource {
        self.handle.source
    }

    pub fn dart(&self) -> Dart {
        self.handle.dart
    }
}

#[derive(Clone)]
pub struct BooleanOperand {
    source: BooleanSource,
    faces: Vec<BooleanFace>,
    edges: Vec<BooleanEdge>,
}

impl BooleanOperand {
    fn from_solid<P: Payload>(
        source: BooleanSource,
        solid: &Solid<'_, P>,
    ) -> Result<Self, BooleanError> {
        let faces = solid
            .shells()
            .into_iter()
            .flat_map(|shell| shell.faces())
            .map(|face| BooleanFace::from_face(source, &face))
            .collect::<Result<Vec<_>, _>>()?;
        let edges = collect_operand_edges(source, &faces, solid)?;

        Ok(Self {
            source,
            faces,
            edges,
        })
    }

    pub fn source(&self) -> BooleanSource {
        self.source
    }

    pub fn faces(&self) -> &[BooleanFace] {
        &self.faces
    }

    pub fn edges(&self) -> &[BooleanEdge] {
        &self.edges
    }

    pub fn contains(&self, handle: FaceHandle) -> bool {
        self.contains_face(handle)
    }

    pub fn contains_face(&self, handle: FaceHandle) -> bool {
        handle.source == self.source && self.faces.iter().any(|face| face.handle == handle)
    }

    pub fn contains_edge(&self, handle: EdgeHandle) -> bool {
        handle.source == self.source && self.edges.iter().any(|edge| edge.handle == handle)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct FaceFaceInterference {
    pub object: FaceHandle,
    pub tool: FaceHandle,
    pub intersection: SurfaceSurfaceIntersection,
}

#[derive(Debug, Clone, PartialEq)]
pub struct EdgeFaceInterference {
    pub edge: EdgeHandle,
    pub face: FaceHandle,
    pub intersection: CurveSurfaceIntersection,
}

#[derive(Debug, Clone, PartialEq)]
pub struct EdgeEdgeInterference {
    pub object: EdgeHandle,
    pub tool: EdgeHandle,
    pub intersection: CurveCurveIntersection,
}

#[derive(Debug, Clone, PartialEq)]
pub struct EdgeSplit {
    pub edge: EdgeHandle,
    pub parameter: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EdgeOverlap {
    pub edge: EdgeHandle,
    pub interval: Interval,
}

#[derive(Debug, Clone, PartialEq)]
pub enum FaceSectionKind {
    Point(Point3),
    Curve {
        points: Vec<Point3>,
    },
    SameDomainRegion,
    EdgeOverlap {
        edge: EdgeHandle,
        interval: Interval,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub struct FaceSection {
    pub face: FaceHandle,
    pub kind: FaceSectionKind,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct BooleanSplitPlan {
    edge_splits: Vec<EdgeSplit>,
    edge_overlaps: Vec<EdgeOverlap>,
    face_sections: Vec<FaceSection>,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct BooleanSplitApplication {
    edge_splits: Vec<AppliedEdgeSplit>,
    face_sections: Vec<AppliedFaceSection>,
    face_splits: Vec<AppliedFaceSplit>,
}

impl BooleanSplitApplication {
    pub fn edge_splits(&self) -> &[AppliedEdgeSplit] {
        &self.edge_splits
    }

    pub fn face_sections(&self) -> &[AppliedFaceSection] {
        &self.face_sections
    }

    pub fn face_splits(&self) -> &[AppliedFaceSplit] {
        &self.face_splits
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct AppliedEdgeSplit {
    pub edge: EdgeHandle,
    pub parameter: f64,
    pub first: EdgeKey,
    pub second: EdgeKey,
    pub vertex: VertexKey,
}

#[derive(Debug, Clone, PartialEq)]
pub struct AppliedFaceSection {
    pub face: FaceHandle,
    pub kind: AppliedFaceSectionKind,
}

#[derive(Debug, Clone, PartialEq)]
pub enum AppliedFaceSectionKind {
    Point {
        point: Point3,
        uv: Point2,
    },
    Curve {
        points: Vec<Point3>,
        curve: NurbsCurve,
        pcurve: Curve2,
    },
    SameDomainRegion,
    EdgeOverlap {
        edge: EdgeHandle,
        interval: Interval,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub struct AppliedFaceSplit {
    pub face: FaceHandle,
    pub first: FaceKey,
    pub second: FaceKey,
    pub section_edges: Vec<EdgeKey>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BooleanClassification {
    Inside,
    Outside,
    Boundary,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BooleanOperation {
    Union,
    Intersection,
    Difference,
}

#[derive(Debug, Clone, PartialEq)]
pub struct BooleanFaceClassification {
    pub source: BooleanSource,
    pub face: FaceKey,
    pub classification: BooleanClassification,
}

#[derive(Debug, Clone, PartialEq)]
pub struct BooleanFaceSelection {
    pub source: BooleanSource,
    pub face: FaceKey,
    pub classification: BooleanClassification,
    pub keep: bool,
}

pub fn boolean<P: Payload>(
    object: &Shape<SolidTag, P>,
    tool: &Shape<SolidTag, P>,
    operation: BooleanOperation,
) -> Result<Shape<SolidTag, P>, BooleanError> {
    let workspace = BooleanWorkspace::from_solids(&object.solid(), &tool.solid())?;
    let split = workspace.split_solid_shapes(object, tool)?;
    split.build_result(operation)
}

pub fn boolean_union<P: Payload>(
    object: &Shape<SolidTag, P>,
    tool: &Shape<SolidTag, P>,
) -> Result<Shape<SolidTag, P>, BooleanError> {
    boolean(object, tool, BooleanOperation::Union)
}

pub fn boolean_intersection<P: Payload>(
    object: &Shape<SolidTag, P>,
    tool: &Shape<SolidTag, P>,
) -> Result<Shape<SolidTag, P>, BooleanError> {
    boolean(object, tool, BooleanOperation::Intersection)
}

pub fn boolean_difference<P: Payload>(
    object: &Shape<SolidTag, P>,
    tool: &Shape<SolidTag, P>,
) -> Result<Shape<SolidTag, P>, BooleanError> {
    boolean(object, tool, BooleanOperation::Difference)
}

#[derive(Clone)]
pub struct BooleanSplitOperands<P: Payload> {
    object: GMap<P>,
    object_solid: SolidKey,
    tool: GMap<P>,
    tool_solid: SolidKey,
    application: BooleanSplitApplication,
}

impl<P: Payload> BooleanSplitOperands<P> {
    pub fn object_map(&self) -> &GMap<P> {
        &self.object
    }

    pub fn object_map_mut(&mut self) -> &mut GMap<P> {
        &mut self.object
    }

    pub fn object_solid(&self) -> SolidKey {
        self.object_solid
    }

    pub fn tool_map(&self) -> &GMap<P> {
        &self.tool
    }

    pub fn tool_map_mut(&mut self) -> &mut GMap<P> {
        &mut self.tool
    }

    pub fn tool_solid(&self) -> SolidKey {
        self.tool_solid
    }

    pub fn application(&self) -> &BooleanSplitApplication {
        &self.application
    }

    pub fn classify_faces(&self) -> Result<Vec<BooleanFaceClassification>, BooleanError> {
        let object_solid = solid_for_key(&self.object, self.object_solid)?;
        let tool_solid = solid_for_key(&self.tool, self.tool_solid)?;
        let mut classifications = classify_operand_faces(
            BooleanSource::Object,
            &self.object,
            &object_solid,
            &tool_solid,
        )?;
        classifications.extend(classify_operand_faces(
            BooleanSource::Tool,
            &self.tool,
            &tool_solid,
            &object_solid,
        )?);
        Ok(classifications)
    }

    pub fn select_faces(
        &self,
        operation: BooleanOperation,
    ) -> Result<Vec<BooleanFaceSelection>, BooleanError> {
        Ok(self
            .classify_faces()?
            .into_iter()
            .map(|face| BooleanFaceSelection {
                keep: should_keep_face(operation, face.source, face.classification),
                source: face.source,
                face: face.face,
                classification: face.classification,
            })
            .collect())
    }

    pub fn build_result(
        &self,
        operation: BooleanOperation,
    ) -> Result<Shape<SolidTag, P>, BooleanError> {
        let selections = self.select_faces(operation)?;
        let mut result = merge_selected_faces(&self.object, &self.tool, &selections)?;
        sew_matching_result_edges(&mut result)?;

        let Some(shell_dart) = result_shell_dart(&result) else {
            return Err(BooleanError::OpenResultShell {
                operation,
                free_edge_count: free_result_edge_count(&result),
            });
        };
        orient_result_shell(&mut result, shell_dart)?;

        let solid = result.add_solid(SolidAttr::new(
            P::S::default(),
            result.cell_representative(shell_dart, Dim::Three),
            None,
        ));
        Ok(Shape::new(result, solid))
    }
}

pub fn classify_point_against_solid<P: Payload>(
    solid: &Solid<'_, P>,
    point: Point3,
) -> Result<BooleanClassification, BooleanError> {
    let faces = solid
        .shells()
        .into_iter()
        .flat_map(|shell| shell.faces())
        .collect::<Vec<_>>();

    if faces.is_empty() {
        return Ok(BooleanClassification::Outside);
    }

    for face in &faces {
        if point_lies_on_face(face, point)? {
            return Ok(BooleanClassification::Boundary);
        }
    }

    let ray = classification_ray(&faces);
    let mut hits = Vec::new();
    for face in &faces {
        if let Some(distance) = ray_face_intersection(point, ray, face)? {
            hits.push(distance);
        }
    }

    hits.sort_by(|a, b| a.total_cmp(b));
    hits.dedup_by(|a, b| (*a - *b).abs() <= LINEAR_TOLERANCE * 10.0);

    if hits.len() % 2 == 1 {
        Ok(BooleanClassification::Inside)
    } else {
        Ok(BooleanClassification::Outside)
    }
}

#[derive(Debug, Clone, Copy)]
struct ClassificationRay {
    direction: Vector3<f64>,
    length: f64,
}

fn solid_for_key<P: Payload>(g: &GMap<P>, solid: SolidKey) -> Result<Solid<'_, P>, BooleanError> {
    g.solid_attr(solid)
        .map(|attr| Solid::new(g, attr))
        .ok_or(BooleanError::MissingSolidHandle { solid })
}

fn classify_operand_faces<P: Payload>(
    source: BooleanSource,
    map: &GMap<P>,
    solid: &Solid<'_, P>,
    against: &Solid<'_, P>,
) -> Result<Vec<BooleanFaceClassification>, BooleanError> {
    let mut classifications = Vec::new();
    for face in solid.shells().into_iter().flat_map(|shell| shell.faces()) {
        let face_key = map
            .attribute::<Cell2>(face.outer_loop().dart)
            .copied()
            .ok_or(BooleanError::MissingFaceHandle {
                face: FaceHandle {
                    source,
                    dart: face.outer_loop().dart,
                },
            })?;
        let point = sample_face_region_point(&face)
            .ok_or(BooleanError::MissingFaceSample { face: face_key })?;
        let classification = classify_point_against_solid(against, point)?;
        classifications.push(BooleanFaceClassification {
            source,
            face: face_key,
            classification,
        });
    }
    Ok(classifications)
}

fn should_keep_face(
    operation: BooleanOperation,
    source: BooleanSource,
    classification: BooleanClassification,
) -> bool {
    match operation {
        BooleanOperation::Union => match classification {
            BooleanClassification::Outside => true,
            BooleanClassification::Inside => false,
            BooleanClassification::Boundary => source == BooleanSource::Object,
        },
        BooleanOperation::Intersection => match classification {
            BooleanClassification::Inside => true,
            BooleanClassification::Outside => false,
            BooleanClassification::Boundary => source == BooleanSource::Object,
        },
        BooleanOperation::Difference => matches!(
            (source, classification),
            (
                BooleanSource::Object,
                BooleanClassification::Outside | BooleanClassification::Boundary
            ) | (BooleanSource::Tool, BooleanClassification::Inside)
        ),
    }
}

#[derive(Debug, Clone, Copy)]
struct ResultEdge {
    dart: Dart,
    reversed_dart: Dart,
    start: Point3,
    end: Point3,
}

fn merge_selected_faces<P: Payload>(
    object: &GMap<P>,
    tool: &GMap<P>,
    selections: &[BooleanFaceSelection],
) -> Result<GMap<P>, BooleanError> {
    let mut result = GMap::new();
    for source in [BooleanSource::Object, BooleanSource::Tool] {
        let map = operand_map(source, object, tool);
        let faces = selections
            .iter()
            .filter(|selection| selection.keep && selection.source == source)
            .map(|selection| selection.face)
            .collect::<Vec<_>>();
        if faces.is_empty() {
            continue;
        }
        result.merge(SelectedFaceSet::new(source, map, &faces)?);
    }
    Ok(result)
}

struct SelectedFaceSet<'a, P: Payload> {
    map: &'a GMap<P>,
    handle: Dart,
    darts: Vec<Dart>,
}

impl<'a, P: Payload> SelectedFaceSet<'a, P> {
    fn new(
        source: BooleanSource,
        map: &'a GMap<P>,
        faces: &[FaceKey],
    ) -> Result<Self, BooleanError> {
        let mut darts = Vec::new();
        for face in faces {
            let attr = map
                .face_attr(*face)
                .ok_or(BooleanError::MissingFaceHandle {
                    face: FaceHandle {
                        source,
                        dart: Dart::new(0),
                    },
                })?;
            let view = attr.face(map);
            darts.extend(view.outer_loop().darts());
            for loop_ in view.inner_loops() {
                darts.extend(loop_.darts());
            }
        }

        let handle = *darts.first().ok_or(BooleanError::EmptyResultSelection {
            operation: BooleanOperation::Union,
        })?;
        Ok(Self { map, handle, darts })
    }
}

impl<P: Payload> MergeTopology<P> for SelectedFaceSet<'_, P> {
    fn merge_topology(&self) -> TopologyMerge<'_, P> {
        TopologyMerge::new(self.map, self.darts.clone(), self.handle)
    }
}

fn sew_matching_result_edges<P: Payload>(g: &mut GMap<P>) -> Result<(), BooleanError> {
    loop {
        let free_edges = result_free_edges(g)?;
        let Some((first, second)) = matching_free_edge_pair(&free_edges) else {
            break;
        };

        g.sew(Dim::Two, first.dart, second).map_err(|reason| {
            BooleanError::ResultEdgeSewFailed {
                first: first.dart,
                second,
                reason,
            }
        })?;
    }

    sew_degenerate_result_edges(g)?;
    Ok(())
}

fn matching_free_edge_pair(free_edges: &[ResultEdge]) -> Option<(ResultEdge, Dart)> {
    free_edges.iter().enumerate().find_map(|(index, first)| {
        free_edges[index + 1..]
            .iter()
            .find_map(|second| matching_result_edge_dart(*first, *second))
            .map(|second| (*first, second))
    })
}

fn sew_degenerate_result_edges<P: Payload>(g: &mut GMap<P>) -> Result<(), BooleanError> {
    for edge in result_free_edges(g)? {
        if !g.is_free(edge.dart, Dim::Two) || !edge_is_degenerate(edge) {
            continue;
        }

        g.sew(Dim::Two, edge.dart, edge.reversed_dart)
            .map_err(|reason| BooleanError::ResultEdgeSewFailed {
                first: edge.dart,
                second: edge.reversed_dart,
                reason,
            })?;
    }
    Ok(())
}

fn result_free_edges<P: Payload>(g: &GMap<P>) -> Result<Vec<ResultEdge>, BooleanError> {
    let mut edges = Vec::new();
    for (key, edge) in g.iter_edges() {
        for dart in g
            .orbit(edge.dart, g.orbit_indices(Dim::One))
            .filter(|dart| g.is_free(*dart, Dim::Two))
        {
            let reversed_dart = g.alpha(Dim::Zero, dart);
            let start = vertex_orbit_point(g, dart)
                .ok_or(BooleanError::MissingEndpointGeometry { edge: key })?;
            let end = vertex_orbit_point(g, reversed_dart)
                .ok_or(BooleanError::MissingEndpointGeometry { edge: key })?;
            edges.push(ResultEdge {
                dart,
                reversed_dart,
                start,
                end,
            });
        }
    }
    Ok(edges)
}

fn vertex_orbit_point<P: Payload>(g: &GMap<P>, dart: Dart) -> Option<Point3> {
    g.orbit(dart, g.orbit_indices(Dim::Zero))
        .find_map(|candidate| g.attribute::<Cell0>(candidate).map(|vertex| vertex.point))
}

fn matching_result_edge_dart(first: ResultEdge, second: ResultEdge) -> Option<Dart> {
    if first.start.coincides(second.end, LINEAR_TOLERANCE)
        && first.end.coincides(second.start, LINEAR_TOLERANCE)
    {
        return Some(second.dart);
    }

    if first.start.coincides(second.start, LINEAR_TOLERANCE)
        && first.end.coincides(second.end, LINEAR_TOLERANCE)
    {
        return Some(second.reversed_dart);
    }

    None
}

fn edge_is_degenerate(edge: ResultEdge) -> bool {
    edge.start.coincides(edge.end, LINEAR_TOLERANCE)
}

fn result_shell_dart<P: Payload>(g: &GMap<P>) -> Option<Dart> {
    let mut visited = HashSet::new();
    let mut best = None;

    for (_, face) in g.iter_faces() {
        if !visited.insert(face.outer_loop) {
            continue;
        }

        let darts = g.orbit(face.outer_loop, vec![0, 1, 2]).collect::<Vec<_>>();
        visited.extend(darts.iter().copied());
        if !darts.iter().all(|dart| {
            !g.is_free(*dart, Dim::Zero)
                && !g.is_free(*dart, Dim::One)
                && !g.is_free(*dart, Dim::Two)
        }) {
            continue;
        }

        let face_count = g
            .iter_faces()
            .filter(|(_, candidate)| darts.contains(&candidate.outer_loop))
            .count();
        if best.is_none_or(|(_, best_count)| face_count > best_count) {
            best = Some((face.outer_loop, face_count));
        }
    }

    best.map(|(dart, _)| dart)
}

fn free_result_edge_count<P: Payload>(g: &GMap<P>) -> usize {
    g.iter_edges()
        .filter(|(_, edge)| g.is_free(edge.dart, Dim::Two))
        .count()
}

fn orient_result_shell<P: Payload>(g: &mut GMap<P>, shell_dart: Dart) -> Result<(), BooleanError> {
    let shell_darts = g.orbit(shell_dart, vec![0, 1, 2]).collect::<Vec<_>>();
    let Some(shell_center) = shell_centroid(g, &shell_darts) else {
        return Ok(());
    };
    let shell_darts = shell_darts.into_iter().collect::<HashSet<_>>();
    let flips = g
        .iter_faces()
        .filter(|(_, face)| shell_darts.contains(&face.outer_loop))
        .map(|(key, _)| {
            let (face_center, normal) = face_orientation_sample(g, key)
                .ok_or(BooleanError::MissingOrientationSample { face: key })?;
            Ok((
                key,
                normal.dot(&(face_center - shell_center)) <= LINEAR_TOLERANCE,
            ))
        })
        .collect::<Result<Vec<_>, BooleanError>>()?;

    for (face, should_flip) in flips {
        if should_flip {
            flip_face_surface_orientation(g, face)?;
        }
    }
    Ok(())
}

fn shell_centroid<P: Payload>(g: &GMap<P>, shell_darts: &[Dart]) -> Option<Vector3<f64>> {
    let shell_darts = shell_darts.iter().copied().collect::<HashSet<_>>();
    let mut sum = Vector3::zeros();
    let mut count = 0;

    for (face, attr) in g.iter_faces() {
        if !shell_darts.contains(&attr.outer_loop) {
            continue;
        }
        let (center, _) = face_orientation_sample(g, face)?;
        sum += center;
        count += 1;
    }

    (count > 0).then_some(sum / count as f64)
}

fn face_orientation_sample<P: Payload>(
    g: &GMap<P>,
    face: FaceKey,
) -> Option<(Vector3<f64>, Vector3<f64>)> {
    let face = g.face_attr(face)?.face(g);
    let uv = sample_face_uv(&face)?;
    Some((
        face.point_at(uv.x, uv.y).coords,
        *face.normal_at(uv.x, uv.y),
    ))
}

fn sample_face_uv<P: Payload>(face: &Face<'_, P>) -> Option<Point2> {
    let mut outer_uv = Vec::new();
    for edge in face.outer_loop().edges() {
        let samples = face.pcurve(edge.dart)?.sample(1);
        let count = samples.len();
        outer_uv.extend(samples.into_iter().take(count.saturating_sub(1)));
    }

    if outer_uv.is_empty() {
        return None;
    }
    Some(uv_centroid(&outer_uv))
}

fn flip_face_surface_orientation<P: Payload>(
    g: &mut GMap<P>,
    face: FaceKey,
) -> Result<(), BooleanError> {
    let attr = g
        .face_attr(face)
        .cloned()
        .ok_or(BooleanError::MissingOrientationSample { face })?;
    let outer_loop = g.alpha(Dim::Zero, attr.outer_loop);
    let inner_loops = attr
        .inner_loops
        .iter()
        .map(|dart| g.alpha(Dim::Zero, *dart))
        .collect::<Vec<_>>();
    let pcurves = attr
        .face(g)
        .edges()
        .into_iter()
        .filter_map(|edge| {
            attr.pcurves
                .get(&edge.dart)
                .map(|pcurve| (g.alpha(Dim::Zero, edge.dart), pcurve.reversed()))
        })
        .collect();

    if let Some(face) = g.face_attr_mut(face) {
        face.outer_loop = outer_loop;
        face.inner_loops = inner_loops;
        face.pcurves = pcurves;
    }
    Ok(())
}

fn classification_ray<P: Payload>(faces: &[Face<'_, P>]) -> ClassificationRay {
    let bbox = BBox::from_points(faces.iter().flat_map(face_points));
    let length = bbox.diagonal_length().max(1.0) * 4.0 + 1.0;
    let direction = Vector3::new(1.0, 0.371, 0.217).normalize();
    ClassificationRay { direction, length }
}

fn ray_face_intersection<P: Payload>(
    origin: Point3,
    ray: ClassificationRay,
    face: &Face<'_, P>,
) -> Result<Option<f64>, BooleanError> {
    if let Surface::Plane(plane) = face.surface() {
        return ray_plane_face_intersection(origin, ray, plane, face);
    }

    let curve = Curve::line(origin, origin + ray.direction * ray.length);
    let intersections = intersect_curve_surface_with_options(
        &curve,
        face.surface(),
        boolean_intersection_options(),
    )?;
    let mut distances = intersections.into_iter().filter_map(|intersection| {
        let CurveSurfaceIntersection::Point {
            curve_u,
            surface_u,
            surface_v,
            ..
        } = intersection
        else {
            return None;
        };
        if curve_u <= LINEAR_TOLERANCE || curve_u > 1.0 + LINEAR_TOLERANCE {
            return None;
        }
        let uv = Point2::new(surface_u, surface_v);
        matches!(
            face_uv_containment(face, uv),
            Ok(BooleanClassification::Inside | BooleanClassification::Boundary)
        )
        .then_some(curve_u * ray.length)
    });

    Ok(distances.next())
}

fn ray_plane_face_intersection<P: Payload>(
    origin: Point3,
    ray: ClassificationRay,
    plane: &Plane,
    face: &Face<'_, P>,
) -> Result<Option<f64>, BooleanError> {
    let normal = *plane.normal();
    let denominator = normal.dot(&ray.direction);
    if denominator.abs() <= LINEAR_TOLERANCE {
        return Ok(None);
    }

    let distance = (plane.origin() - origin).dot(&normal) / denominator;
    if distance <= LINEAR_TOLERANCE || distance > ray.length + LINEAR_TOLERANCE {
        return Ok(None);
    }

    let point = origin + ray.direction * distance;
    let classification = face_projected_point_containment(face, point)?;
    Ok(matches!(
        classification,
        BooleanClassification::Inside | BooleanClassification::Boundary
    )
    .then_some(distance))
}

fn point_lies_on_face<P: Payload>(face: &Face<'_, P>, point: Point3) -> Result<bool, BooleanError> {
    Ok(face_projected_point_containment(face, point)? != BooleanClassification::Outside)
}

fn face_projected_point_containment<P: Payload>(
    face: &Face<'_, P>,
    point: Point3,
) -> Result<BooleanClassification, BooleanError> {
    let uv = face.surface().closest_parameter(point)?;
    let projected = face.point_at(uv.x, uv.y);
    if (projected - point).norm() > LINEAR_TOLERANCE * 10.0 {
        return Ok(BooleanClassification::Outside);
    }
    face_uv_containment(face, uv)
}

fn sample_face_region_point<P: Payload>(face: &Face<'_, P>) -> Option<Point3> {
    let outer_uv = sample_loop_uv(&face.outer_loop(), face)?;
    let centroid = uv_centroid(&outer_uv);
    let mut candidates = vec![centroid];

    candidates.extend(outer_uv.iter().enumerate().map(|(index, uv)| {
        let next = outer_uv[(index + 1) % outer_uv.len()];
        let midpoint = Point2::from((uv.coords + next.coords) * 0.5);
        Point2::from(midpoint.coords * 0.8 + centroid.coords * 0.2)
    }));

    candidates
        .into_iter()
        .find(|uv| {
            matches!(
                face_uv_containment(face, *uv),
                Ok(BooleanClassification::Inside)
            )
        })
        .map(|uv| face.point_at(uv.x, uv.y))
}

fn face_uv_containment<P: Payload>(
    face: &Face<'_, P>,
    uv: Point2,
) -> Result<BooleanClassification, BooleanError> {
    let outer =
        sample_loop_uv(&face.outer_loop(), face).ok_or(BooleanError::MissingFaceUvSample)?;
    match loop_uv_containment(&outer, uv) {
        BooleanClassification::Outside => return Ok(BooleanClassification::Outside),
        BooleanClassification::Boundary => return Ok(BooleanClassification::Boundary),
        BooleanClassification::Inside => {}
    }

    for inner in face.inner_loops() {
        let Some(inner_uv) = sample_loop_uv(&inner, face) else {
            continue;
        };
        match loop_uv_containment(&inner_uv, uv) {
            BooleanClassification::Boundary => return Ok(BooleanClassification::Boundary),
            BooleanClassification::Inside => return Ok(BooleanClassification::Outside),
            BooleanClassification::Outside => {}
        }
    }

    Ok(BooleanClassification::Inside)
}

fn sample_loop_uv<P: Payload>(loop_: &Loop<'_, P>, face: &Face<'_, P>) -> Option<Vec<Point2>> {
    let mut points = Vec::new();
    for edge in loop_.edges() {
        let samples = face.pcurve(edge.dart)?.sample(8);
        let count = samples.len();
        points.extend(samples.into_iter().take(count.saturating_sub(1)));
    }
    (points.len() >= 3).then_some(points)
}

fn loop_uv_containment(points: &[Point2], uv: Point2) -> BooleanClassification {
    let mut inside = false;
    for (a, b) in points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .take(points.len())
    {
        if uv_on_segment(uv, *a, *b) {
            return BooleanClassification::Boundary;
        }

        let crosses = (a.y > uv.y) != (b.y > uv.y);
        if !crosses {
            continue;
        }

        let x = a.x + (uv.y - a.y) * (b.x - a.x) / (b.y - a.y);
        if x > uv.x + LINEAR_TOLERANCE {
            inside = !inside;
        }
    }

    if inside {
        BooleanClassification::Inside
    } else {
        BooleanClassification::Outside
    }
}

fn uv_on_segment(point: Point2, start: Point2, end: Point2) -> bool {
    let direction = end - start;
    let length_sq = direction.norm_squared();
    if length_sq <= LINEAR_TOLERANCE * LINEAR_TOLERANCE {
        return (point - start).norm() <= LINEAR_TOLERANCE;
    }

    let t = (point - start).dot(&direction) / length_sq;
    if !(-LINEAR_TOLERANCE..=1.0 + LINEAR_TOLERANCE).contains(&t) {
        return false;
    }
    let projected = start + direction * t.clamp(0.0, 1.0);
    (projected - point).norm() <= LINEAR_TOLERANCE
}

fn uv_centroid(points: &[Point2]) -> Point2 {
    let sum = points
        .iter()
        .fold(nalgebra::Vector2::zeros(), |sum, point| sum + point.coords);
    Point2::from(sum / points.len() as f64)
}

impl BooleanSplitPlan {
    pub fn from_edge_splits(edge_splits: impl IntoIterator<Item = EdgeSplit>) -> Self {
        let mut plan = Self::default();
        for split in edge_splits {
            plan.add_edge_split(split.edge, split.parameter);
        }
        plan.sort();
        plan
    }

    pub fn from_edge_overlaps(edge_overlaps: impl IntoIterator<Item = EdgeOverlap>) -> Self {
        let mut plan = Self::default();
        for overlap in edge_overlaps {
            plan.add_edge_overlap(overlap.edge, overlap.interval);
        }
        plan.sort();
        plan
    }

    pub fn from_face_sections(face_sections: impl IntoIterator<Item = FaceSection>) -> Self {
        Self {
            face_sections: face_sections.into_iter().collect(),
            ..Self::default()
        }
    }

    fn from_interferences(
        face_face_interferences: &[FaceFaceInterference],
        edge_face_interferences: &[EdgeFaceInterference],
        edge_edge_interferences: &[EdgeEdgeInterference],
    ) -> Self {
        let mut plan = Self::default();

        for interference in face_face_interferences {
            plan.add_face_face_interference(interference);
        }
        for interference in edge_face_interferences {
            plan.add_edge_face_interference(interference);
        }
        for interference in edge_edge_interferences {
            plan.add_edge_edge_interference(interference);
        }

        plan.sort();
        plan
    }

    pub fn edge_splits(&self) -> &[EdgeSplit] {
        &self.edge_splits
    }

    pub fn edge_overlaps(&self) -> &[EdgeOverlap] {
        &self.edge_overlaps
    }

    pub fn face_sections(&self) -> &[FaceSection] {
        &self.face_sections
    }

    pub fn apply_to_maps<P: Payload>(
        &self,
        object: &mut GMap<P>,
        tool: &mut GMap<P>,
    ) -> Result<BooleanSplitApplication, BooleanError> {
        let mut application = BooleanSplitApplication::default();

        for (edge, parameters) in self.edge_split_groups() {
            let map = operand_map_mut(edge.source, object, tool);
            apply_edge_splits_to_map(map, edge, &parameters, &mut application)?;
        }
        apply_face_sections_to_maps(self.face_sections(), object, tool, &mut application)?;

        Ok(application)
    }

    fn edge_split_groups(&self) -> Vec<(EdgeHandle, Vec<f64>)> {
        let mut groups = Vec::<(EdgeHandle, Vec<f64>)>::new();
        for split in &self.edge_splits {
            match groups.last_mut() {
                Some((edge, parameters)) if *edge == split.edge => {
                    parameters.push(split.parameter);
                }
                _ => groups.push((split.edge, vec![split.parameter])),
            }
        }
        groups
    }

    fn add_face_face_interference(&mut self, interference: &FaceFaceInterference) {
        match &interference.intersection {
            SurfaceSurfaceIntersection::Point { point, .. } => {
                self.add_face_section(interference.object, FaceSectionKind::Point(*point));
                self.add_face_section(interference.tool, FaceSectionKind::Point(*point));
            }
            SurfaceSurfaceIntersection::Curve { points } => {
                self.add_face_section(
                    interference.object,
                    FaceSectionKind::Curve {
                        points: points.clone(),
                    },
                );
                self.add_face_section(
                    interference.tool,
                    FaceSectionKind::Curve {
                        points: points.clone(),
                    },
                );
            }
            SurfaceSurfaceIntersection::Region => {
                self.add_face_section(interference.object, FaceSectionKind::SameDomainRegion);
                self.add_face_section(interference.tool, FaceSectionKind::SameDomainRegion);
            }
        }
    }

    fn add_edge_face_interference(&mut self, interference: &EdgeFaceInterference) {
        match &interference.intersection {
            CurveSurfaceIntersection::Point { point, curve_u, .. } => {
                self.add_edge_split(interference.edge, *curve_u);
                self.add_face_section(interference.face, FaceSectionKind::Point(*point));
            }
            CurveSurfaceIntersection::Overlap { curve_interval } => {
                self.add_edge_overlap(interference.edge, *curve_interval);
                self.add_face_section(
                    interference.face,
                    FaceSectionKind::EdgeOverlap {
                        edge: interference.edge,
                        interval: *curve_interval,
                    },
                );
            }
        }
    }

    fn add_edge_edge_interference(&mut self, interference: &EdgeEdgeInterference) {
        match interference.intersection {
            CurveCurveIntersection::Point { u_a, u_b, .. } => {
                self.add_edge_split(interference.object, u_a);
                self.add_edge_split(interference.tool, u_b);
            }
            CurveCurveIntersection::Overlap {
                interval_a,
                interval_b,
            } => {
                self.add_edge_overlap(interference.object, interval_a);
                self.add_edge_overlap(interference.tool, interval_b);
            }
        }
    }

    fn add_edge_split(&mut self, edge: EdgeHandle, parameter: f64) {
        if !parameter.is_finite() {
            return;
        }

        if self
            .edge_splits
            .iter()
            .any(|split| split.edge == edge && parameters_close(split.parameter, parameter))
        {
            return;
        }

        self.edge_splits.push(EdgeSplit { edge, parameter });
    }

    fn add_edge_overlap(&mut self, edge: EdgeHandle, interval: Interval) {
        if !interval.start.is_finite() || !interval.end.is_finite() {
            return;
        }

        let interval = interval.ordered();
        if self
            .edge_overlaps
            .iter()
            .any(|overlap| overlap.edge == edge && intervals_close(overlap.interval, interval))
        {
            return;
        }

        self.add_edge_split(edge, interval.start);
        self.add_edge_split(edge, interval.end);
        self.edge_overlaps.push(EdgeOverlap { edge, interval });
    }

    fn add_face_section(&mut self, face: FaceHandle, kind: FaceSectionKind) {
        self.face_sections.push(FaceSection { face, kind });
    }

    fn sort(&mut self) {
        self.edge_splits.sort_by(|a, b| {
            edge_handle_sort_key(a.edge)
                .cmp(&edge_handle_sort_key(b.edge))
                .then_with(|| a.parameter.total_cmp(&b.parameter))
        });
        self.edge_overlaps.sort_by(|a, b| {
            edge_handle_sort_key(a.edge)
                .cmp(&edge_handle_sort_key(b.edge))
                .then_with(|| a.interval.start.total_cmp(&b.interval.start))
                .then_with(|| a.interval.end.total_cmp(&b.interval.end))
        });
    }
}

fn apply_edge_splits_to_map<P: Payload>(
    g: &mut GMap<P>,
    edge: EdgeHandle,
    parameters: &[f64],
    application: &mut BooleanSplitApplication,
) -> Result<(), BooleanError> {
    let original_edge = edge_key_for_handle(g, edge)?;
    let (_, splits) =
        apply_edge_split_parameters(g, original_edge, parameters).map_err(|source| {
            BooleanError::EdgeSplitApplicationFailed {
                edge,
                parameter: parameters.first().copied().unwrap_or_default(),
                source,
            }
        })?;
    for split in splits {
        application.edge_splits.push(AppliedEdgeSplit {
            edge,
            parameter: split.parameter,
            first: split.first,
            second: split.second,
            vertex: split.vertex,
        });
    }

    Ok(())
}

fn apply_face_sections_to_maps<P: Payload>(
    sections: &[FaceSection],
    object: &mut GMap<P>,
    tool: &mut GMap<P>,
    application: &mut BooleanSplitApplication,
) -> Result<(), BooleanError> {
    let applied = {
        let object = &*object;
        let tool = &*tool;
        sections
            .iter()
            .map(|section| {
                let map = operand_map(section.face.source, object, tool);
                resolve_face_section(map, section)
            })
            .collect::<Result<Vec<_>, _>>()?
    };
    let imprint_groups = face_imprint_groups(&applied);
    application.face_sections.extend(applied);

    for (face, imprints) in imprint_groups {
        let map = operand_map_mut(face.source, object, tool);
        let face_key = face_key_for_handle(map, face)?;
        for split in split_face_by_imprints(map, face_key, &imprints)
            .map_err(|source| BooleanError::FaceSplitApplicationFailed { face, source })?
        {
            application.face_splits.push(AppliedFaceSplit {
                face,
                first: split.first,
                second: split.second,
                section_edges: split.section_edges,
            });
        }
    }

    Ok(())
}

fn resolve_face_section<P: Payload>(
    g: &GMap<P>,
    section: &FaceSection,
) -> Result<AppliedFaceSection, BooleanError> {
    let face_attr = face_attr_for_handle(g, section.face)?;
    let kind = match &section.kind {
        FaceSectionKind::Point(point) => AppliedFaceSectionKind::Point {
            point: *point,
            uv: face_attr.surface.closest_parameter(*point)?,
        },
        FaceSectionKind::Curve { points } => {
            let (curve, pcurve) = face_section_curves(&face_attr.surface, points)?;
            AppliedFaceSectionKind::Curve {
                points: points.clone(),
                curve,
                pcurve,
            }
        }
        FaceSectionKind::SameDomainRegion => AppliedFaceSectionKind::SameDomainRegion,
        FaceSectionKind::EdgeOverlap { edge, interval } => AppliedFaceSectionKind::EdgeOverlap {
            edge: *edge,
            interval: *interval,
        },
    };

    Ok(AppliedFaceSection {
        face: section.face,
        kind,
    })
}

fn face_section_curves(
    surface: &Surface,
    points: &[Point3],
) -> Result<(NurbsCurve, Curve2), BooleanError> {
    let parameters = NurbsCurve::chord_length_parameters(points)?;
    let uv_points = points
        .iter()
        .map(|point| surface.closest_parameter(*point))
        .collect::<Result<Vec<_>, _>>()?;
    let curve = NurbsCurve::interpolate_with_parameters(points, &parameters)?;
    let pcurve = Curve2::Nurbs(NurbsCurve2::interpolate_with_parameters(
        &uv_points,
        &parameters,
    )?);
    Ok((curve, pcurve))
}

fn face_imprint_groups(sections: &[AppliedFaceSection]) -> Vec<(FaceHandle, Vec<FaceImprint>)> {
    let mut groups = Vec::<(FaceHandle, Vec<FaceImprint>)>::new();
    for section in sections {
        let AppliedFaceSectionKind::Curve { curve, pcurve, .. } = &section.kind else {
            continue;
        };
        let imprint = FaceImprint::new(Curve::Nurbs(curve.clone()), pcurve.clone());

        match groups.iter_mut().find(|(face, _)| *face == section.face) {
            Some((_, imprints)) => imprints.push(imprint),
            None => groups.push((section.face, vec![imprint])),
        }
    }
    groups
}

fn operand_map_mut<'a, P: Payload>(
    source: BooleanSource,
    object: &'a mut GMap<P>,
    tool: &'a mut GMap<P>,
) -> &'a mut GMap<P> {
    match source {
        BooleanSource::Object => object,
        BooleanSource::Tool => tool,
    }
}

fn operand_map<'a, P: Payload>(
    source: BooleanSource,
    object: &'a GMap<P>,
    tool: &'a GMap<P>,
) -> &'a GMap<P> {
    match source {
        BooleanSource::Object => object,
        BooleanSource::Tool => tool,
    }
}

fn face_attr_for_handle<P: Payload>(
    g: &GMap<P>,
    face: FaceHandle,
) -> Result<&FaceAttr<P::F>, BooleanError> {
    let face_key = g
        .attribute::<Cell2>(face.dart)
        .copied()
        .ok_or(BooleanError::MissingFaceHandle { face })?;
    g.face_attr(face_key)
        .ok_or(BooleanError::MissingFaceHandle { face })
}

fn face_key_for_handle<P: Payload>(g: &GMap<P>, face: FaceHandle) -> Result<FaceKey, BooleanError> {
    g.attribute::<Cell2>(face.dart)
        .copied()
        .ok_or(BooleanError::MissingFaceHandle { face })
}

fn edge_key_for_handle<P: Payload>(g: &GMap<P>, edge: EdgeHandle) -> Result<EdgeKey, BooleanError> {
    let representative = g.cell_representative(edge.dart, Dim::One);
    g.iter_edges()
        .find_map(|(key, attr)| (attr.dart == representative).then_some(key))
        .ok_or(BooleanError::MissingEdgeHandle { edge })
}

#[derive(Clone)]
pub struct BooleanWorkspace {
    object: BooleanOperand,
    tool: BooleanOperand,
    face_pair_count: usize,
    edge_face_pair_count: usize,
    edge_pair_count: usize,
    face_face_interferences: Vec<FaceFaceInterference>,
    edge_face_interferences: Vec<EdgeFaceInterference>,
    edge_edge_interferences: Vec<EdgeEdgeInterference>,
    split_plan: BooleanSplitPlan,
}

impl BooleanWorkspace {
    pub fn from_solids<P: Payload>(
        object: &Solid<'_, P>,
        tool: &Solid<'_, P>,
    ) -> Result<Self, BooleanError> {
        Self::from_solids_with_options(object, tool, boolean_intersection_options())
    }

    pub fn from_solids_with_options<P: Payload>(
        object: &Solid<'_, P>,
        tool: &Solid<'_, P>,
        options: IntersectionOptions,
    ) -> Result<Self, BooleanError> {
        let object = BooleanOperand::from_solid(BooleanSource::Object, object)?;
        let tool = BooleanOperand::from_solid(BooleanSource::Tool, tool)?;
        let face_pair_count = object.faces().len() * tool.faces().len();
        let edge_face_pair_count =
            object.edges().len() * tool.faces().len() + tool.edges().len() * object.faces().len();
        let edge_pair_count = object.edges().len() * tool.edges().len();
        let face_face_interferences = collect_face_interferences(&object, &tool, options)?;
        let edge_face_interferences = collect_edge_face_interferences(&object, &tool, options)?;
        let edge_edge_interferences = collect_edge_edge_interferences(&object, &tool, options)?;
        let split_plan = BooleanSplitPlan::from_interferences(
            &face_face_interferences,
            &edge_face_interferences,
            &edge_edge_interferences,
        );

        Ok(Self {
            object,
            tool,
            face_pair_count,
            edge_face_pair_count,
            edge_pair_count,
            face_face_interferences,
            edge_face_interferences,
            edge_edge_interferences,
            split_plan,
        })
    }

    pub fn object(&self) -> &BooleanOperand {
        &self.object
    }

    pub fn tool(&self) -> &BooleanOperand {
        &self.tool
    }

    pub fn face_pair_count(&self) -> usize {
        self.face_pair_count
    }

    pub fn edge_face_pair_count(&self) -> usize {
        self.edge_face_pair_count
    }

    pub fn edge_pair_count(&self) -> usize {
        self.edge_pair_count
    }

    pub fn face_face_interferences(&self) -> &[FaceFaceInterference] {
        &self.face_face_interferences
    }

    pub fn edge_face_interferences(&self) -> &[EdgeFaceInterference] {
        &self.edge_face_interferences
    }

    pub fn edge_edge_interferences(&self) -> &[EdgeEdgeInterference] {
        &self.edge_edge_interferences
    }

    pub fn split_plan(&self) -> &BooleanSplitPlan {
        &self.split_plan
    }

    pub fn split_solid_shapes<P: Payload>(
        &self,
        object: &Shape<SolidTag, P>,
        tool: &Shape<SolidTag, P>,
    ) -> Result<BooleanSplitOperands<P>, BooleanError> {
        let mut object_map = object.map().clone();
        let mut tool_map = tool.map().clone();
        let application = self
            .split_plan
            .apply_to_maps(&mut object_map, &mut tool_map)?;

        Ok(BooleanSplitOperands {
            object: object_map,
            object_solid: object.handle(),
            tool: tool_map,
            tool_solid: tool.handle(),
            application,
        })
    }
}

fn collect_operand_edges<P: Payload>(
    source: BooleanSource,
    faces: &[BooleanFace],
    solid: &Solid<'_, P>,
) -> Result<Vec<BooleanEdge>, BooleanError> {
    let face_handles = faces.iter().map(|face| face.handle).collect::<HashSet<_>>();
    let mut seen = HashSet::new();
    let mut edges = Vec::new();

    for shell in solid.shells() {
        for face in shell.faces() {
            if !face_handles.contains(&FaceHandle {
                source,
                dart: face.outer_loop().dart,
            }) {
                continue;
            }

            for edge in face_edges(&face) {
                let handle = EdgeHandle {
                    source,
                    dart: edge_handle_dart(&edge),
                };
                if seen.insert(handle) {
                    edges.push(BooleanEdge::from_edge(source, &edge)?);
                }
            }
        }
    }

    Ok(edges)
}

fn collect_face_interferences(
    object: &BooleanOperand,
    tool: &BooleanOperand,
    options: IntersectionOptions,
) -> Result<Vec<FaceFaceInterference>, IntersectionError> {
    let mut interferences = Vec::new();

    for object_face in object.faces() {
        for tool_face in tool.faces() {
            interferences.extend(intersect_face_pair(object_face, tool_face, options)?);
        }
    }

    Ok(interferences)
}

fn intersect_face_pair(
    object_face: &BooleanFace,
    tool_face: &BooleanFace,
    options: IntersectionOptions,
) -> Result<Vec<FaceFaceInterference>, IntersectionError> {
    if !object_face
        .bbox
        .expanded(options.bbox_tolerance)
        .intersects(&tool_face.bbox, options.bbox_tolerance)
    {
        return Ok(Vec::new());
    }

    let object_surface = Surface::Nurbs(object_face.surface.clone());
    let tool_surface = Surface::Nurbs(tool_face.surface.clone());
    let intersections = intersect_surfaces_with_options(&object_surface, &tool_surface, options)?;

    Ok(intersections
        .into_iter()
        .map(|intersection| FaceFaceInterference {
            object: object_face.handle(),
            tool: tool_face.handle(),
            intersection,
        })
        .collect())
}

fn collect_edge_face_interferences(
    object: &BooleanOperand,
    tool: &BooleanOperand,
    options: IntersectionOptions,
) -> Result<Vec<EdgeFaceInterference>, IntersectionError> {
    let mut interferences = Vec::new();

    for edge in object.edges() {
        for face in tool.faces() {
            interferences.extend(intersect_edge_face_pair(edge, face, options)?);
        }
    }

    for edge in tool.edges() {
        for face in object.faces() {
            interferences.extend(intersect_edge_face_pair(edge, face, options)?);
        }
    }

    Ok(interferences)
}

fn intersect_edge_face_pair(
    edge: &BooleanEdge,
    face: &BooleanFace,
    options: IntersectionOptions,
) -> Result<Vec<EdgeFaceInterference>, IntersectionError> {
    if !edge
        .bbox
        .expanded(options.bbox_tolerance)
        .intersects(&face.bbox, options.bbox_tolerance)
    {
        return Ok(Vec::new());
    }

    let curve = Curve::Nurbs(edge.curve.clone());
    let surface = Surface::Nurbs(face.surface.clone());
    let intersections = intersect_curve_surface_with_options(&curve, &surface, options)?;

    Ok(intersections
        .into_iter()
        .map(|intersection| EdgeFaceInterference {
            edge: edge.handle(),
            face: face.handle(),
            intersection,
        })
        .collect())
}

fn collect_edge_edge_interferences(
    object: &BooleanOperand,
    tool: &BooleanOperand,
    options: IntersectionOptions,
) -> Result<Vec<EdgeEdgeInterference>, IntersectionError> {
    let mut interferences = Vec::new();

    for object_edge in object.edges() {
        for tool_edge in tool.edges() {
            interferences.extend(intersect_edge_pair(object_edge, tool_edge, options)?);
        }
    }

    Ok(interferences)
}

fn intersect_edge_pair(
    object_edge: &BooleanEdge,
    tool_edge: &BooleanEdge,
    options: IntersectionOptions,
) -> Result<Vec<EdgeEdgeInterference>, IntersectionError> {
    if !object_edge
        .bbox
        .expanded(options.bbox_tolerance)
        .intersects(&tool_edge.bbox, options.bbox_tolerance)
    {
        return Ok(Vec::new());
    }

    let object_curve = Curve::Nurbs(object_edge.curve.clone());
    let tool_curve = Curve::Nurbs(tool_edge.curve.clone());
    let intersections = intersect_curves_with_options(&object_curve, &tool_curve, options)?;

    Ok(intersections
        .into_iter()
        .map(|intersection| EdgeEdgeInterference {
            object: object_edge.handle(),
            tool: tool_edge.handle(),
            intersection,
        })
        .collect())
}

pub fn boolean_intersection_options() -> IntersectionOptions {
    IntersectionOptions {
        curve_sample_count: 16,
        surface_u_sample_count: 6,
        surface_v_sample_count: 6,
        ..IntersectionOptions::default()
    }
}

fn face_bbox<P: Payload>(face: &Face<'_, P>) -> BBox {
    BBox::from_points(face_points(face))
}

fn edge_bbox<P: Payload>(edge: &Edge<'_, P>, curve: &NurbsCurve) -> BBox {
    let mut points = curve
        .control_points()
        .iter()
        .map(|point| point.to_cartesian())
        .collect::<Vec<_>>();

    points.extend(
        edge.vertices()
            .into_iter()
            .filter_map(|vertex| vertex.point().copied()),
    );
    BBox::from_points(points)
}

fn edge_handle_dart<P: Payload>(edge: &Edge<'_, P>) -> Dart {
    edge.darts()
        .min()
        .expect("edge orbit must include at least the edge dart")
}

fn face_edges<'a, P: Payload>(face: &Face<'a, P>) -> Vec<Edge<'a, P>> {
    let mut edges = face.outer_loop().edges();
    for loop_ in face.inner_loops() {
        edges.extend(loop_.edges());
    }
    edges
}

fn face_points<P: Payload>(face: &Face<'_, P>) -> Vec<Point3> {
    let mut points = face
        .outer_loop()
        .vertices()
        .into_iter()
        .filter_map(|vertex| vertex.point().copied())
        .collect::<Vec<_>>();

    for loop_ in face.inner_loops() {
        points.extend(
            loop_
                .vertices()
                .into_iter()
                .filter_map(|vertex| vertex.point().copied()),
        );
    }

    points
}

fn parameters_close(a: f64, b: f64) -> bool {
    (a - b).abs() <= LINEAR_TOLERANCE
}

fn intervals_close(a: Interval, b: Interval) -> bool {
    parameters_close(a.start, b.start) && parameters_close(a.end, b.end)
}

fn edge_handle_sort_key(handle: EdgeHandle) -> (usize, usize) {
    (source_sort_key(handle.source), handle.dart.id())
}

fn source_sort_key(source: BooleanSource) -> usize {
    match source {
        BooleanSource::Object => 0,
        BooleanSource::Tool => 1,
    }
}
