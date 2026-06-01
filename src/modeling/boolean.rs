use std::collections::HashSet;

use thiserror::Error;

use crate::geometry::dim3::intersections::{
    intersect_curve_surface_with_options, intersect_curves_with_options,
    intersect_surfaces_with_options,
};
use crate::geometry::{
    BBox, Curve, CurveCurveIntersection, CurveSurfaceIntersection, IntersectionError,
    IntersectionOptions, Interval, LINEAR_TOLERANCE, NurbsCurve, NurbsError, NurbsSurface, Point3,
    Surface, SurfaceSurfaceIntersection,
};
use crate::topology::edge::Edge;
use crate::topology::face::Face;
use crate::topology::gmap::Dart;
use crate::topology::payload::Payload;
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

impl BooleanSplitPlan {
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
