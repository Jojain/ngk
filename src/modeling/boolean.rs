use std::collections::HashSet;

use thiserror::Error;

use crate::builders::faces::{
    FaceEdgeSplitError, FaceImprint, FaceImprintSplitError, split_face_by_imprints, split_face_edge,
};
use crate::geometry::dim3::intersections::{
    intersect_curve_surface_with_options, intersect_curves_with_options,
    intersect_surfaces_with_options,
};
use crate::geometry::{
    BBox, Curve, Curve2, CurveCurveIntersection, CurveSurfaceIntersection, IntersectionError,
    IntersectionOptions, Interval, LINEAR_TOLERANCE, NurbsCurve, NurbsError, NurbsSurface, Point2,
    Point3, Polyline2, Surface, SurfaceSurfaceIntersection,
};
use crate::topology::attributes::FaceAttr;
use crate::topology::edge::Edge;
use crate::topology::face::Face;
use crate::topology::gmap::{Cell0, Cell2, Dart, Dim, GMap};
use crate::topology::payload::Payload;
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
        source: FaceEdgeSplitError,
    },
    #[error("failed to split face {face:?} by imprints")]
    FaceSplitApplicationFailed {
        face: FaceHandle,
        source: FaceImprintSplitError,
    },
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
    pub section_edge: EdgeKey,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct EdgeSegment {
    edge: EdgeKey,
    domain: Interval,
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
    let mut segments = vec![EdgeSegment {
        edge: original_edge,
        domain: edge_domain(g, edge, original_edge)?,
    }];

    for &parameter in parameters {
        let Some(segment_index) = split_segment_index(&segments, parameter) else {
            if touches_existing_segment_boundary(&segments, parameter) {
                continue;
            }
            return Err(BooleanError::MissingSplitSegment { edge, parameter });
        };
        let segment = segments[segment_index];
        let face = incident_face_for_edge(g, edge, segment.edge)?;
        let split = split_face_edge(g, face, segment.edge, parameter).map_err(|source| {
            BooleanError::EdgeSplitApplicationFailed {
                edge,
                parameter,
                source,
            }
        })?;

        segments.remove(segment_index);
        segments.push(EdgeSegment {
            edge: split.first,
            domain: edge_domain(g, edge, split.first)?,
        });
        segments.push(EdgeSegment {
            edge: split.second,
            domain: edge_domain(g, edge, split.second)?,
        });
        segments.sort_by(|a, b| a.domain.start.total_cmp(&b.domain.start));
        application.edge_splits.push(AppliedEdgeSplit {
            edge,
            parameter,
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
                section_edge: split.section_edge,
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
        FaceSectionKind::Curve { points } => AppliedFaceSectionKind::Curve {
            points: points.clone(),
            pcurve: face_section_pcurve(&face_attr.surface, points)?,
        },
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

fn face_section_pcurve(surface: &Surface, points: &[Point3]) -> Result<Curve2, BooleanError> {
    let uv_points = points
        .iter()
        .map(|point| surface.closest_parameter(*point))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(Curve2::Polyline(Polyline2::new(uv_points)))
}

fn face_imprint_groups(sections: &[AppliedFaceSection]) -> Vec<(FaceHandle, Vec<FaceImprint>)> {
    let mut groups = Vec::<(FaceHandle, Vec<FaceImprint>)>::new();
    for section in sections {
        let AppliedFaceSectionKind::Curve { points, pcurve } = &section.kind else {
            continue;
        };
        let imprint = FaceImprint {
            points: points.clone(),
            pcurve: pcurve.clone(),
        };

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
    g.face(face_key)
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

fn incident_face_for_edge<P: Payload>(
    g: &GMap<P>,
    handle: EdgeHandle,
    edge: EdgeKey,
) -> Result<FaceKey, BooleanError> {
    let edge_attr = g
        .edge(edge)
        .ok_or(BooleanError::MissingEdgeHandle { edge: handle })?;
    g.orbit(edge_attr.dart, g.orbit_indices(Dim::One))
        .find_map(|dart| g.attribute::<Cell2>(dart).copied())
        .ok_or(BooleanError::MissingIncidentFace { edge: handle })
}

fn edge_domain<P: Payload>(
    g: &GMap<P>,
    handle: EdgeHandle,
    edge: EdgeKey,
) -> Result<Interval, BooleanError> {
    let attr = g
        .edge(edge)
        .ok_or(BooleanError::MissingEdgeHandle { edge: handle })?;
    let start = g
        .attribute::<Cell0>(attr.dart)
        .map(|vertex| vertex.point)
        .ok_or(BooleanError::MissingEndpointGeometry { edge })?;
    let end_dart = g.alpha(Dim::Zero, attr.dart);
    let end = g
        .attribute::<Cell0>(end_dart)
        .map(|vertex| vertex.point)
        .ok_or(BooleanError::MissingEndpointGeometry { edge })?;
    Ok(attr.curve.parameters_between(start, end).ordered())
}

fn split_segment_index(segments: &[EdgeSegment], parameter: f64) -> Option<usize> {
    segments
        .iter()
        .position(|segment| contains_interior(segment.domain, parameter))
}

fn touches_existing_segment_boundary(segments: &[EdgeSegment], parameter: f64) -> bool {
    segments.iter().any(|segment| {
        (parameter - segment.domain.start).abs() <= LINEAR_TOLERANCE
            || (parameter - segment.domain.end).abs() <= LINEAR_TOLERANCE
    })
}

fn contains_interior(interval: Interval, parameter: f64) -> bool {
    interval.contains(parameter, LINEAR_TOLERANCE)
        && (parameter - interval.start).abs() > LINEAR_TOLERANCE
        && (parameter - interval.end).abs() > LINEAR_TOLERANCE
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
