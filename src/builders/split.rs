use std::collections::{HashMap, HashSet};

use thiserror::Error;

use crate::builders::edges::{EdgeSplitError, split_edge};
use crate::builders::faces::{
    FaceEdgeSplitError, FaceImprint, FaceImprintSplitError, add_polygon, split_face_by_imprints,
    split_face_edge,
};
use crate::geometry::dim3::intersections::{intersect_curve_surface, intersect_surfaces};
use crate::geometry::{
    Curve, Curve2, CurveSurfaceIntersection, IntersectionError, Interval, LINEAR_TOLERANCE, Line2,
    NurbsCurve, NurbsCurve2, NurbsError, Point2, Point3, PointCoincidence, Surface,
    SurfaceSurfaceIntersection,
};
use crate::topology::attributes::FaceAttr;
use crate::topology::gmap::{Dart, DetachError, Dim, GMap, SolidRegistrationError};
use crate::topology::payload::Payload;
use crate::topology::profile::Profile;
use crate::topology::shape_keys::{EdgeKey, FaceKey, SolidKey, VertexKey};

/// Keys partitioned by the oriented negative and positive sides of a cutter.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Partition<K> {
    /// Components on the side opposite the cutter's oriented normal.
    pub negative: Vec<K>,
    /// Components on the side pointed to by the cutter's oriented normal.
    pub positive: Vec<K>,
}

impl<K> Default for Partition<K> {
    fn default() -> Self {
        Self {
            negative: Vec::new(),
            positive: Vec::new(),
        }
    }
}

/// Result of splitting one edge by a surface or trimmed face.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EdgeSurfaceSplit {
    /// Resulting edge segments classified by cutter side.
    pub edges: Partition<EdgeKey>,
    /// Vertices created at cutter intersections.
    pub section_vertices: Vec<VertexKey>,
}

/// Result of splitting one face by a surface or trimmed face.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FaceSurfaceSplit {
    /// Resulting face regions classified by cutter side.
    pub faces: Partition<FaceKey>,
    /// Edges created along the section.
    pub section_edges: Vec<EdgeKey>,
}

/// Result of splitting one solid by a surface or trimmed face.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SolidSurfaceSplit {
    /// Resulting solids classified by cutter side.
    pub solids: Partition<SolidKey>,
    /// Independent cap faces classified by the solid they close.
    pub section_faces: Partition<FaceKey>,
}

/// One target-local edge split applied at an original curve parameter.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct AppliedEdgeParameterSplit {
    pub parameter: f64,
    pub first: EdgeKey,
    pub second: EdgeKey,
    pub vertex: VertexKey,
}

/// Errors reported by in-place builder split operations.
#[derive(Debug, Error)]
pub enum SplitError {
    /// The target edge key does not exist.
    #[error("edge {edge:?} does not exist")]
    MissingEdge { edge: EdgeKey },
    /// The target face key does not exist.
    #[error("face {face:?} does not exist")]
    MissingFace { face: FaceKey },
    /// The target solid key does not exist.
    #[error("solid {solid:?} does not exist")]
    MissingSolid { solid: SolidKey },
    /// The cutter face key does not exist in its map.
    #[error("cutter face {face:?} does not exist")]
    MissingCutterFace { face: FaceKey },
    /// The target and cutter do not intersect.
    #[error("target and cutter do not intersect")]
    NoIntersection,
    /// The cutter only touches the target and does not produce both sides.
    #[error("cutter is tangent to the target")]
    Tangent,
    /// The target contains geometry coincident with the cutter.
    #[error("target and cutter contain a coincident region")]
    Coincident,
    /// A finite cutter intersects the target but does not partition it.
    #[error("finite cutter does not partition the target")]
    NonSeparatingCutter,
    /// Intersected cavity shells are not supported by the first implementation.
    #[error("solid {solid:?} has intersected cavity shells, which are not supported")]
    IntersectedCavityShellsUnsupported { solid: SolidKey },
    /// The current face splitter cannot represent this section topology.
    #[error("face section contains {count} boundary intersections; exactly two are supported")]
    UnsupportedFaceSection { count: usize },
    /// A resulting section boundary could not be grouped into a closed loop.
    #[error("section edges do not form closed loops")]
    OpenSectionLoop,
    /// A low-level edge split failed.
    #[error("failed to split edge")]
    EdgeSplit(#[from] EdgeSplitError),
    /// A face boundary edge split failed.
    #[error("failed to split a face boundary edge")]
    FaceEdgeSplit(#[from] FaceEdgeSplitError),
    /// A face imprint split failed.
    #[error("failed to apply a face section imprint")]
    FaceImprintSplit(#[from] FaceImprintSplitError),
    /// Curve/surface intersection failed.
    #[error("failed to intersect target geometry with cutter")]
    Intersection(#[from] IntersectionError),
    /// Surface parameter or curve construction failed.
    #[error("failed to evaluate split geometry")]
    Geometry(#[from] NurbsError),
    /// Attribute-aware topology detachment failed.
    #[error("failed to detach section topology")]
    Detach(#[from] DetachError),
    /// Solid shell registration failed.
    #[error("failed to register split solid components")]
    SolidRegistration(#[from] SolidRegistrationError),
    /// A cap edge could not be sewn to its section boundary.
    #[error("failed to sew cap edge {cap:?} to section edge {section:?}: {reason}")]
    CapSewFailed {
        cap: Dart,
        section: Dart,
        reason: &'static str,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Side {
    Negative,
    Positive,
    Boundary,
}

struct TrimDomain {
    outer: Vec<Point2>,
    inner: Vec<Vec<Point2>>,
}

struct Cutter {
    surface: Surface,
    trim: Option<TrimDomain>,
    orientation: f64,
}

impl Cutter {
    fn surface(surface: &Surface) -> Self {
        Self {
            surface: surface.clone(),
            trim: None,
            orientation: 1.0,
        }
    }

    fn face<P: Payload>(g: &GMap<P>, face: FaceKey) -> Result<Self, SplitError> {
        let face_view = g.face(face).ok_or(SplitError::MissingCutterFace { face })?;
        let sample = sample_face_uv(&face_view).unwrap_or(Point2::origin());
        let support_normal = face_view.surface().normal_at(sample.x, sample.y);
        let orientation = if face_view.normal_at(sample.x, sample.y).dot(&support_normal) < 0.0 {
            -1.0
        } else {
            1.0
        };
        let outer = sample_loop_uv(&face_view, &face_view.outer_loop())
            .ok_or(SplitError::MissingCutterFace { face })?;
        let inner = face_view
            .inner_loops()
            .iter()
            .filter_map(|loop_| sample_loop_uv(&face_view, loop_))
            .collect();
        Ok(Self {
            surface: face_view.surface().clone(),
            trim: Some(TrimDomain { outer, inner }),
            orientation,
        })
    }

    fn is_finite(&self) -> bool {
        self.trim.is_some()
    }

    fn contains(&self, point: Point3) -> Result<bool, SplitError> {
        let Some(trim) = &self.trim else {
            return Ok(true);
        };
        let uv = self.surface.closest_parameter(point)?;
        Ok(point_in_loop(&trim.outer, uv)
            && !trim.inner.iter().any(|inner| point_in_loop(inner, uv)))
    }

    fn side(&self, point: Point3) -> Result<Side, SplitError> {
        let uv = self.surface.closest_parameter(point)?;
        let projected = self.surface.point_at(uv.x, uv.y);
        let signed =
            (point - projected).dot(&self.surface.normal_at(uv.x, uv.y)) * self.orientation;
        if signed > LINEAR_TOLERANCE {
            Ok(Side::Positive)
        } else if signed < -LINEAR_TOLERANCE {
            Ok(Side::Negative)
        } else {
            Ok(Side::Boundary)
        }
    }

    fn desired_support_winding(&self, side: Side) -> f64 {
        match side {
            Side::Negative => self.orientation,
            Side::Positive => -self.orientation,
            Side::Boundary => self.orientation,
        }
    }
}

/// Splits `edge` by an oriented support surface, replacing `g` only on success.
pub fn split_edge_by_surface<P: Payload>(
    g: &mut GMap<P>,
    edge: EdgeKey,
    surface: &Surface,
) -> Result<EdgeSurfaceSplit, SplitError> {
    let cutter = Cutter::surface(surface);
    transactional(g, |work| split_edge_with_cutter(work, edge, &cutter))
}

/// Splits `face` by an oriented support surface, replacing `g` only on success.
pub fn split_face_by_surface<P: Payload>(
    g: &mut GMap<P>,
    face: FaceKey,
    surface: &Surface,
) -> Result<FaceSurfaceSplit, SplitError> {
    let cutter = Cutter::surface(surface);
    transactional(g, |work| split_face_with_cutter(work, face, &cutter))
}

/// Splits `solid` by an oriented support surface, replacing `g` only on success.
pub fn split_solid_by_surface<P: Payload>(
    g: &mut GMap<P>,
    solid: SolidKey,
    surface: &Surface,
) -> Result<SolidSurfaceSplit, SplitError> {
    let cutter = Cutter::surface(surface);
    transactional(g, |work| split_solid_with_cutter(work, solid, &cutter))
}

/// Splits `edge` by a trimmed cutter face stored in a separate immutable map.
pub fn split_edge_by_face<P: Payload, C: Payload>(
    g: &mut GMap<P>,
    edge: EdgeKey,
    cutter_g: &GMap<C>,
    cutter_face: FaceKey,
) -> Result<EdgeSurfaceSplit, SplitError> {
    let cutter = Cutter::face(cutter_g, cutter_face)?;
    transactional(g, |work| split_edge_with_cutter(work, edge, &cutter))
}

/// Splits `face` by a trimmed cutter face stored in a separate immutable map.
pub fn split_face_by_face<P: Payload, C: Payload>(
    g: &mut GMap<P>,
    face: FaceKey,
    cutter_g: &GMap<C>,
    cutter_face: FaceKey,
) -> Result<FaceSurfaceSplit, SplitError> {
    let cutter = Cutter::face(cutter_g, cutter_face)?;
    transactional(g, |work| split_face_with_cutter(work, face, &cutter))
}

/// Splits `solid` by a trimmed cutter face stored in a separate immutable map.
pub fn split_solid_by_face<P: Payload, C: Payload>(
    g: &mut GMap<P>,
    solid: SolidKey,
    cutter_g: &GMap<C>,
    cutter_face: FaceKey,
) -> Result<SolidSurfaceSplit, SplitError> {
    let cutter = Cutter::face(cutter_g, cutter_face)?;
    transactional(g, |work| split_solid_with_cutter(work, solid, &cutter))
}

fn transactional<P: Payload, T>(
    g: &mut GMap<P>,
    operation: impl FnOnce(&mut GMap<P>) -> Result<T, SplitError>,
) -> Result<T, SplitError> {
    let mut work = g.clone();
    let result = operation(&mut work)?;
    *g = work;
    Ok(result)
}

fn split_edge_with_cutter<P: Payload>(
    g: &mut GMap<P>,
    edge: EdgeKey,
    cutter: &Cutter,
) -> Result<EdgeSurfaceSplit, SplitError> {
    let intersections = edge_cutter_intersections(g, edge, cutter)?;
    if intersections.is_empty() {
        return Err(SplitError::NoIntersection);
    }

    let parameters = intersections
        .iter()
        .map(|intersection| intersection.parameter)
        .collect::<Vec<_>>();
    let (segments, applied) = apply_edge_split_parameters(g, edge, &parameters)?;
    let mut partition = Partition::default();
    for segment in segments {
        match edge_side(g, segment, cutter)? {
            Side::Negative => partition.negative.push(segment),
            Side::Positive => partition.positive.push(segment),
            Side::Boundary => return Err(SplitError::Coincident),
        }
    }
    require_both_sides(&partition, cutter.is_finite())?;
    Ok(EdgeSurfaceSplit {
        edges: partition,
        section_vertices: applied.into_iter().map(|split| split.vertex).collect(),
    })
}

#[derive(Clone, Copy)]
struct EdgeIntersection {
    parameter: f64,
    point: Point3,
}

fn edge_cutter_intersections<P: Payload>(
    g: &GMap<P>,
    edge: EdgeKey,
    cutter: &Cutter,
) -> Result<Vec<EdgeIntersection>, SplitError> {
    let edge_view = g.edge(edge).ok_or(SplitError::MissingEdge { edge })?;
    let curve = edge_view.curve().ok_or(SplitError::MissingEdge { edge })?;
    let domain = edge_domain(g, edge)?;
    let mut intersections = Vec::new();
    let mut point_count = 0;
    for intersection in curve_support_intersections(curve, domain, &cutter.surface)? {
        point_count += 1;
        if !cutter.contains(intersection.point)?
            || (intersection.parameter - domain.start).abs() <= LINEAR_TOLERANCE
            || (intersection.parameter - domain.end).abs() <= LINEAR_TOLERANCE
        {
            continue;
        }
        intersections.push(intersection);
    }
    dedup_edge_intersections(&mut intersections);
    if intersections.is_empty() && point_count > 0 {
        return Err(if cutter.is_finite() {
            SplitError::NonSeparatingCutter
        } else {
            SplitError::Tangent
        });
    }
    if intersections.len() == 1 {
        let start = *edge_view
            .start()
            .point()
            .ok_or(SplitError::MissingEdge { edge })?;
        let end = *edge_view
            .end()
            .point()
            .ok_or(SplitError::MissingEdge { edge })?;
        if cutter.side(start)? == cutter.side(end)? {
            return Err(SplitError::Tangent);
        }
    }
    Ok(intersections)
}

fn dedup_edge_intersections(intersections: &mut Vec<EdgeIntersection>) {
    intersections.sort_by(|a, b| a.parameter.total_cmp(&b.parameter));
    intersections.dedup_by(|a, b| {
        (a.parameter - b.parameter).abs() <= LINEAR_TOLERANCE
            || a.point.coincides(b.point, LINEAR_TOLERANCE)
    });
}

/// Applies sorted target-local split parameters and returns current segments.
///
/// Attached edges use the face-boundary primitive so all incident pcurves are
/// updated; profile-only edges use the ordinary edge split primitive.
pub(crate) fn apply_edge_split_parameters<P: Payload>(
    g: &mut GMap<P>,
    edge: EdgeKey,
    parameters: &[f64],
) -> Result<(Vec<EdgeKey>, Vec<AppliedEdgeParameterSplit>), SplitError> {
    let mut segments = vec![(edge, edge_domain(g, edge)?)];
    let mut applied = Vec::new();
    let mut parameters = parameters.to_vec();
    parameters.sort_by(f64::total_cmp);
    parameters.dedup_by(|a, b| (*a - *b).abs() <= LINEAR_TOLERANCE);

    for parameter in parameters {
        let Some(index) = segments.iter().position(|(_, domain)| {
            domain.contains(parameter, LINEAR_TOLERANCE)
                && (parameter - domain.start).abs() > LINEAR_TOLERANCE
                && (parameter - domain.end).abs() > LINEAR_TOLERANCE
        }) else {
            continue;
        };
        let segment = segments[index].0;
        let incident_face = g
            .edge(segment)
            .and_then(|edge| edge.faces().first().map(|face| face.key()));
        let split = if let Some(face) = incident_face {
            split_face_edge(g, face, segment, parameter)?
        } else {
            split_edge(g, segment, parameter)?
        };
        applied.push(AppliedEdgeParameterSplit {
            parameter,
            first: split.first,
            second: split.second,
            vertex: split.vertex,
        });
        segments.remove(index);
        segments.push((split.first, edge_domain(g, split.first)?));
        segments.push((split.second, edge_domain(g, split.second)?));
        segments.sort_by(|a, b| a.1.start.total_cmp(&b.1.start));
    }
    Ok((
        segments.into_iter().map(|(edge, _)| edge).collect(),
        applied,
    ))
}

fn edge_domain<P: Payload>(g: &GMap<P>, edge: EdgeKey) -> Result<Interval, SplitError> {
    let edge = g.edge(edge).ok_or(SplitError::MissingEdge { edge })?;
    let curve = edge
        .curve()
        .ok_or(SplitError::MissingEdge { edge: edge.key() })?;
    let start = *edge
        .start()
        .point()
        .ok_or(SplitError::MissingEdge { edge: edge.key() })?;
    let end = *edge
        .end()
        .point()
        .ok_or(SplitError::MissingEdge { edge: edge.key() })?;
    Ok(curve.parameters_between(start, end).ordered())
}

fn edge_side<P: Payload>(g: &GMap<P>, edge: EdgeKey, cutter: &Cutter) -> Result<Side, SplitError> {
    let edge = g.edge(edge).ok_or(SplitError::MissingEdge { edge })?;
    let curve = edge
        .curve()
        .ok_or(SplitError::MissingEdge { edge: edge.key() })?;
    let domain = edge_domain(g, edge.key())?;
    cutter.side(curve.point_at((domain.start + domain.end) * 0.5))
}

#[derive(Clone, Copy)]
struct FaceBoundaryHit {
    edge: EdgeKey,
    parameter: f64,
    point: Point3,
}

fn split_face_with_cutter<P: Payload>(
    g: &mut GMap<P>,
    face: FaceKey,
    cutter: &Cutter,
) -> Result<FaceSurfaceSplit, SplitError> {
    let face_view = g.face(face).ok_or(SplitError::MissingFace { face })?;
    let boundary_edges = face_view
        .edges()
        .into_iter()
        .map(|edge| edge.key())
        .collect::<Vec<_>>();
    let mut raw_hit_count = 0;
    let mut hits = Vec::new();
    for edge in boundary_edges {
        let edge_view = g.edge(edge).ok_or(SplitError::MissingEdge { edge })?;
        let curve = edge_view.curve().ok_or(SplitError::MissingEdge { edge })?;
        let domain = edge_domain(g, edge)?;
        for intersection in curve_support_intersections(curve, domain, &cutter.surface)? {
            raw_hit_count += 1;
            if cutter.contains(intersection.point)? {
                hits.push(FaceBoundaryHit {
                    edge,
                    parameter: intersection.parameter,
                    point: intersection.point,
                });
            }
        }
    }
    dedup_face_hits(&mut hits);
    if hits.is_empty() {
        return Err(if cutter.is_finite() && raw_hit_count > 0 {
            SplitError::NonSeparatingCutter
        } else {
            SplitError::NoIntersection
        });
    }
    if hits.len() == 1 {
        return Err(if cutter.is_finite() {
            SplitError::NonSeparatingCutter
        } else {
            SplitError::Tangent
        });
    }
    if hits.len() != 2 {
        return Err(SplitError::UnsupportedFaceSection { count: hits.len() });
    }

    for hit in &hits {
        let domain = edge_domain(g, hit.edge)?;
        if (hit.parameter - domain.start).abs() > LINEAR_TOLERANCE
            && (hit.parameter - domain.end).abs() > LINEAR_TOLERANCE
        {
            split_face_edge(g, face, hit.edge, hit.parameter)?;
        }
    }

    let target_surface = g
        .face(face)
        .ok_or(SplitError::MissingFace { face })?
        .surface()
        .clone();
    let imprint = section_imprint(g, face, &target_surface, cutter, &hits)?;
    let splits = split_face_by_imprints(g, face, &[imprint])?;
    if splits.is_empty() {
        return Err(if cutter.is_finite() {
            SplitError::NonSeparatingCutter
        } else {
            SplitError::Tangent
        });
    }

    let mut result = FaceSurfaceSplit {
        faces: Partition::default(),
        section_edges: Vec::new(),
    };
    let mut faces = HashSet::new();
    for split in splits {
        faces.insert(split.first);
        faces.insert(split.second);
        result.section_edges.extend(split.section_edges);
    }
    for face in faces {
        match face_side(g, face, cutter)? {
            Side::Negative => result.faces.negative.push(face),
            Side::Positive => result.faces.positive.push(face),
            Side::Boundary => return Err(SplitError::Coincident),
        }
    }
    require_both_sides(&result.faces, cutter.is_finite())?;
    Ok(result)
}

fn dedup_face_hits(hits: &mut Vec<FaceBoundaryHit>) {
    let mut deduped = Vec::new();
    for hit in hits.drain(..) {
        if !deduped
            .iter()
            .any(|existing: &FaceBoundaryHit| existing.point.coincides(hit.point, LINEAR_TOLERANCE))
        {
            deduped.push(hit);
        }
    }
    *hits = deduped;
}

fn section_imprint<P: Payload>(
    g: &GMap<P>,
    face: FaceKey,
    target_surface: &Surface,
    cutter: &Cutter,
    hits: &[FaceBoundaryHit],
) -> Result<FaceImprint, SplitError> {
    let start_uv = target_surface.closest_parameter(hits[0].point)?;
    let end_uv = target_surface.closest_parameter(hits[1].point)?;
    if matches!(target_surface, Surface::Plane(_)) && matches!(cutter.surface, Surface::Plane(_)) {
        return Ok(FaceImprint::new(
            Curve::line(hits[0].point, hits[1].point),
            Curve2::Line(Line2::new(start_uv, end_uv)),
        ));
    }

    let face_view = g.face(face).ok_or(SplitError::MissingFace { face })?;
    let mut points = vec![hits[0].point, hits[1].point];
    for intersection in intersect_surfaces(target_surface, &cutter.surface)? {
        match intersection {
            SurfaceSurfaceIntersection::Curve {
                points: intersection_points,
            } => {
                for point in intersection_points {
                    let uv = target_surface.closest_parameter(point)?;
                    if face_uv_contains(&face_view, uv) && cutter.contains(point)? {
                        points.push(point);
                    }
                }
            }
            SurfaceSurfaceIntersection::Point { point, .. } => points.push(point),
            SurfaceSurfaceIntersection::Region => return Err(SplitError::Coincident),
        }
    }
    dedup_points(&mut points);
    order_section_points(&mut points, hits[0].point, hits[1].point);
    if points.len() <= 2 {
        return Ok(FaceImprint::new(
            Curve::line(hits[0].point, hits[1].point),
            Curve2::Line(Line2::new(start_uv, end_uv)),
        ));
    }

    let parameters = NurbsCurve::chord_length_parameters(&points)?;
    let uv_points = points
        .iter()
        .map(|point| target_surface.closest_parameter(*point))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(FaceImprint::new(
        Curve::Nurbs(NurbsCurve::interpolate_with_parameters(
            &points,
            &parameters,
        )?),
        Curve2::Nurbs(NurbsCurve2::interpolate_with_parameters(
            &uv_points,
            &parameters,
        )?),
    ))
}

fn dedup_points(points: &mut Vec<Point3>) {
    let mut deduped = Vec::new();
    for point in points.drain(..) {
        if !deduped
            .iter()
            .any(|existing| point.coincides(*existing, LINEAR_TOLERANCE))
        {
            deduped.push(point);
        }
    }
    *points = deduped;
}

fn order_section_points(points: &mut [Point3], start: Point3, end: Point3) {
    let direction = end - start;
    let length_squared = direction.norm_squared();
    if length_squared <= LINEAR_TOLERANCE * LINEAR_TOLERANCE {
        return;
    }
    points.sort_by(|a, b| {
        ((a - start).dot(&direction) / length_squared)
            .total_cmp(&((b - start).dot(&direction) / length_squared))
    });
}

fn curve_support_intersections(
    curve: &Curve,
    domain: Interval,
    surface: &Surface,
) -> Result<Vec<EdgeIntersection>, SplitError> {
    if let Surface::Plane(plane) = surface {
        return curve_plane_intersections(curve, domain, plane);
    }

    let mut intersections = Vec::new();
    for intersection in intersect_curve_surface(curve, surface)? {
        match intersection {
            CurveSurfaceIntersection::Point { point, curve_u, .. }
                if domain.contains(curve_u, LINEAR_TOLERANCE) =>
            {
                intersections.push(EdgeIntersection {
                    parameter: curve_u,
                    point,
                });
            }
            CurveSurfaceIntersection::Point { .. } => {}
            CurveSurfaceIntersection::Overlap { .. } => return Err(SplitError::Coincident),
        }
    }
    dedup_edge_intersections(&mut intersections);
    Ok(intersections)
}

fn curve_plane_intersections(
    curve: &Curve,
    domain: Interval,
    plane: &crate::geometry::Plane,
) -> Result<Vec<EdgeIntersection>, SplitError> {
    let signed_distance =
        |parameter: f64| (curve.point_at(parameter) - plane.origin()).dot(&plane.normal());
    let sample_count = 64usize;
    let samples = (0..=sample_count)
        .map(|index| {
            let fraction = index as f64 / sample_count as f64;
            let parameter = domain.start + domain.length() * fraction;
            (parameter, signed_distance(parameter))
        })
        .collect::<Vec<_>>();
    if samples
        .iter()
        .all(|(_, distance)| distance.abs() <= LINEAR_TOLERANCE)
    {
        return Err(SplitError::Coincident);
    }

    let mut intersections = Vec::new();
    for pair in samples.windows(2) {
        let (start_parameter, start_distance) = pair[0];
        let (end_parameter, end_distance) = pair[1];
        if start_distance.abs() <= LINEAR_TOLERANCE {
            intersections.push(EdgeIntersection {
                parameter: start_parameter,
                point: curve.point_at(start_parameter),
            });
        }
        if start_distance.signum() == end_distance.signum() && end_distance.abs() > LINEAR_TOLERANCE
        {
            continue;
        }
        if start_distance.abs() <= LINEAR_TOLERANCE || end_distance.abs() <= LINEAR_TOLERANCE {
            continue;
        }

        let mut low = start_parameter;
        let mut high = end_parameter;
        let mut low_distance = start_distance;
        for _ in 0..64 {
            let middle = (low + high) * 0.5;
            let middle_distance = signed_distance(middle);
            if middle_distance.abs() <= LINEAR_TOLERANCE {
                low = middle;
                high = middle;
                break;
            }
            if middle_distance.signum() == low_distance.signum() {
                low = middle;
                low_distance = middle_distance;
            } else {
                high = middle;
            }
        }
        let parameter = (low + high) * 0.5;
        intersections.push(EdgeIntersection {
            parameter,
            point: curve.point_at(parameter),
        });
    }
    if let Some((parameter, distance)) = samples.last().copied()
        && distance.abs() <= LINEAR_TOLERANCE
    {
        intersections.push(EdgeIntersection {
            parameter,
            point: curve.point_at(parameter),
        });
    }
    dedup_edge_intersections(&mut intersections);
    Ok(intersections)
}

fn face_side<P: Payload>(g: &GMap<P>, face: FaceKey, cutter: &Cutter) -> Result<Side, SplitError> {
    let face = g.face(face).ok_or(SplitError::MissingFace { face })?;
    let point = sample_face_point(&face).ok_or(SplitError::MissingFace { face: face.key() })?;
    cutter.side(point)
}

fn require_both_sides<K>(partition: &Partition<K>, finite: bool) -> Result<(), SplitError> {
    if partition.negative.is_empty() || partition.positive.is_empty() {
        Err(if finite {
            SplitError::NonSeparatingCutter
        } else {
            SplitError::Tangent
        })
    } else {
        Ok(())
    }
}

struct ShellComponent {
    seed: Dart,
    side: Side,
    darts: HashSet<Dart>,
}

fn split_solid_with_cutter<P: Payload>(
    g: &mut GMap<P>,
    solid: SolidKey,
    cutter: &Cutter,
) -> Result<SolidSurfaceSplit, SplitError> {
    let solid_view = g.solid(solid).ok_or(SplitError::MissingSolid { solid })?;
    if solid_view
        .inner_shells()
        .is_some_and(|shells| !shells.is_empty())
    {
        return Err(SplitError::IntersectedCavityShellsUnsupported { solid });
    }
    let original_representative = solid_view.outer_shell().inner().dart;
    let original_faces = solid_view
        .faces()
        .into_iter()
        .map(|face| face.key())
        .collect::<Vec<_>>();
    let mut active_faces = original_faces.iter().copied().collect::<HashSet<_>>();
    let mut section_edges = Vec::new();

    for face in original_faces {
        match split_face_with_cutter(g, face, cutter) {
            Ok(split) => {
                active_faces.remove(&face);
                active_faces.extend(split.faces.negative);
                active_faces.extend(split.faces.positive);
                section_edges.extend(split.section_edges);
            }
            Err(SplitError::NoIntersection | SplitError::Tangent) => {}
            Err(error) => return Err(error),
        }
    }
    if section_edges.is_empty() {
        return Err(SplitError::NoIntersection);
    }

    let mut detached_section_edges = HashSet::new();
    for edge in section_edges {
        let dart = g.edge(edge).ok_or(SplitError::MissingEdge { edge })?.dart;
        let detached = g.detach(Dim::Two, dart)?;
        detached_section_edges.insert(edge);
        detached_section_edges.extend(detached.new_edges);
    }

    let mut components = discover_shell_components(g, &active_faces, cutter)?;
    require_component_sides(&components, cutter.is_finite())?;
    let mut section_faces = Partition::default();
    for component in &mut components {
        let loops = component_section_loops(g, component, &detached_section_edges)?;
        if loops.is_empty() {
            return Err(SplitError::OpenSectionLoop);
        }
        for loop_ in loops {
            let cap = add_component_cap(g, component.side, cutter, &loop_)?;
            match component.side {
                Side::Negative => section_faces.negative.push(cap),
                Side::Positive => section_faces.positive.push(cap),
                Side::Boundary => return Err(SplitError::Coincident),
            }
        }
    }

    let original_component = components
        .iter()
        .position(|component| component.darts.contains(&original_representative))
        .unwrap_or(0);
    let original_seed = components[original_component].seed;
    g.replace_solid_shell(solid, original_seed, None)?;

    let mut solids = Partition::default();
    for (index, component) in components.iter().enumerate() {
        let key = if index == original_component {
            solid
        } else {
            g.register_solid_component(solid, component.seed, None)?
        };
        match component.side {
            Side::Negative => solids.negative.push(key),
            Side::Positive => solids.positive.push(key),
            Side::Boundary => return Err(SplitError::Coincident),
        }
    }
    Ok(SolidSurfaceSplit {
        solids,
        section_faces,
    })
}

fn discover_shell_components<P: Payload>(
    g: &GMap<P>,
    faces: &HashSet<FaceKey>,
    cutter: &Cutter,
) -> Result<Vec<ShellComponent>, SplitError> {
    let mut visited = HashSet::new();
    let mut components = Vec::new();
    for face in faces {
        let attr = g
            .face_attr(*face)
            .ok_or(SplitError::MissingFace { face: *face })?;
        if visited.contains(&attr.outer_loop) {
            continue;
        }
        let darts = g
            .orbit(attr.outer_loop, vec![0, 1, 2])
            .collect::<HashSet<_>>();
        visited.extend(darts.iter().copied());
        let side = faces
            .iter()
            .filter(|candidate| {
                g.face_attr(**candidate)
                    .is_some_and(|attr| darts.contains(&attr.outer_loop))
            })
            .find_map(|candidate| match face_side(g, *candidate, cutter) {
                Ok(Side::Boundary) => None,
                result => Some(result),
            })
            .transpose()?
            .ok_or(SplitError::Coincident)?;
        components.push(ShellComponent {
            seed: attr.outer_loop,
            side,
            darts,
        });
    }
    Ok(components)
}

fn require_component_sides(components: &[ShellComponent], finite: bool) -> Result<(), SplitError> {
    let negative = components
        .iter()
        .any(|component| component.side == Side::Negative);
    let positive = components
        .iter()
        .any(|component| component.side == Side::Positive);
    if negative && positive {
        Ok(())
    } else if finite {
        Err(SplitError::NonSeparatingCutter)
    } else {
        Err(SplitError::Tangent)
    }
}

#[derive(Clone, Copy)]
struct SectionBoundaryEdge {
    edge: EdgeKey,
    dart: Dart,
    start: Point3,
    end: Point3,
}

fn component_section_loops<P: Payload>(
    g: &GMap<P>,
    component: &ShellComponent,
    section_edges: &HashSet<EdgeKey>,
) -> Result<Vec<Vec<SectionBoundaryEdge>>, SplitError> {
    let mut edges = Vec::new();
    for edge in section_edges {
        let edge_view = g
            .edge(*edge)
            .ok_or(SplitError::MissingEdge { edge: *edge })?;
        let Some(dart) = edge_view
            .darts()
            .find(|dart| component.darts.contains(dart) && g.is_free(*dart, Dim::Two))
        else {
            continue;
        };
        edges.push(section_boundary_edge(g, *edge, dart)?);
    }

    let mut loops = Vec::new();
    while let Some(first) = edges.pop() {
        let mut loop_ = vec![first];
        loop {
            let end = loop_.last().expect("loop has a first edge").end;
            if end.coincides(loop_[0].start, LINEAR_TOLERANCE) {
                break;
            }
            let Some((index, reversed)) = edges.iter().enumerate().find_map(|(index, edge)| {
                if edge.start.coincides(end, LINEAR_TOLERANCE) {
                    Some((index, false))
                } else if edge.end.coincides(end, LINEAR_TOLERANCE) {
                    Some((index, true))
                } else {
                    None
                }
            }) else {
                return Err(SplitError::OpenSectionLoop);
            };
            let edge = edges.swap_remove(index);
            loop_.push(if reversed {
                section_boundary_edge(g, edge.edge, g.alpha(Dim::Zero, edge.dart))?
            } else {
                edge
            });
        }
        loops.push(loop_);
    }
    Ok(loops)
}

fn section_boundary_edge<P: Payload>(
    g: &GMap<P>,
    edge: EdgeKey,
    dart: Dart,
) -> Result<SectionBoundaryEdge, SplitError> {
    let view = crate::topology::edge::Edge::new(g, dart);
    let start = *view
        .start()
        .point()
        .ok_or(SplitError::MissingEdge { edge })?;
    let end = *view.end().point().ok_or(SplitError::MissingEdge { edge })?;
    Ok(SectionBoundaryEdge {
        edge,
        dart,
        start,
        end,
    })
}

fn add_component_cap<P: Payload>(
    g: &mut GMap<P>,
    side: Side,
    cutter: &Cutter,
    boundary: &[SectionBoundaryEdge],
) -> Result<FaceKey, SplitError> {
    let mut points = boundary.iter().map(|edge| edge.start).collect::<Vec<_>>();
    let mut uvs = points
        .iter()
        .map(|point| cutter.surface.closest_parameter(*point))
        .collect::<Result<Vec<_>, _>>()?;
    let area = signed_area(&uvs);
    if area.signum() != cutter.desired_support_winding(side).signum() {
        points.reverse();
        uvs.reverse();
    }

    let loop_dart = add_polygon(g, &points);
    let cap_edges = Profile::new(g, loop_dart)
        .edges()
        .into_iter()
        .map(|edge| {
            (
                edge.dart,
                *edge.start().point().expect("cap vertices have geometry"),
                *edge.end().point().expect("cap vertices have geometry"),
            )
        })
        .collect::<Vec<_>>();
    let pcurves = cap_edges
        .iter()
        .enumerate()
        .map(|(index, (dart, _, _))| {
            (
                *dart,
                Curve2::Line(Line2::new(uvs[index], uvs[(index + 1) % uvs.len()])),
            )
        })
        .collect::<HashMap<_, _>>();
    let cap = g.add_face(FaceAttr::with_pcurves(
        cutter.surface.clone(),
        P::F::default(),
        loop_dart,
        Vec::new(),
        pcurves,
    ));

    for (cap_dart, start, end) in cap_edges {
        let section = boundary
            .iter()
            .find(|edge| {
                edge.start.coincides(end, LINEAR_TOLERANCE)
                    && edge.end.coincides(start, LINEAR_TOLERANCE)
            })
            .ok_or(SplitError::OpenSectionLoop)?;
        g.sew(Dim::Two, cap_dart, section.dart)
            .map_err(|reason| SplitError::CapSewFailed {
                cap: cap_dart,
                section: section.dart,
                reason,
            })?;
    }
    Ok(cap)
}

fn sample_face_uv<P: Payload>(face: &crate::topology::face::Face<'_, P>) -> Option<Point2> {
    let points = sample_loop_uv(face, &face.outer_loop())?;
    let centroid = uv_centroid(&points);
    if face_uv_contains(face, centroid) {
        return Some(centroid);
    }
    points.iter().enumerate().find_map(|(index, point)| {
        let next = points[(index + 1) % points.len()];
        let midpoint = Point2::from((point.coords + next.coords) * 0.5);
        let candidate = Point2::from(midpoint.coords * 0.8 + centroid.coords * 0.2);
        face_uv_contains(face, candidate).then_some(candidate)
    })
}

fn sample_face_point<P: Payload>(face: &crate::topology::face::Face<'_, P>) -> Option<Point3> {
    let uv = sample_face_uv(face)?;
    Some(face.point_at(uv.x, uv.y))
}

fn face_uv_contains<P: Payload>(face: &crate::topology::face::Face<'_, P>, uv: Point2) -> bool {
    let Some(outer) = sample_loop_uv(face, &face.outer_loop()) else {
        return false;
    };
    if !point_in_loop(&outer, uv) {
        return false;
    }
    !face
        .inner_loops()
        .iter()
        .filter_map(|loop_| sample_loop_uv(face, loop_))
        .any(|inner| point_in_loop(&inner, uv))
}

fn sample_loop_uv<P: Payload>(
    face: &crate::topology::face::Face<'_, P>,
    loop_: &crate::topology::profile::Loop<'_, P>,
) -> Option<Vec<Point2>> {
    let mut points = Vec::new();
    for edge in loop_.edges() {
        let samples = face.pcurve(edge.dart)?.sample(8);
        let count = samples.len();
        points.extend(samples.into_iter().take(count.saturating_sub(1)));
    }
    (points.len() >= 3).then_some(points)
}

fn point_in_loop(points: &[Point2], point: Point2) -> bool {
    let mut inside = false;
    for (start, end) in points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .take(points.len())
    {
        let direction = *end - *start;
        let length_squared = direction.norm_squared();
        if length_squared > LINEAR_TOLERANCE * LINEAR_TOLERANCE {
            let t = ((point - *start).dot(&direction) / length_squared).clamp(0.0, 1.0);
            if (point - (*start + direction * t)).norm() <= LINEAR_TOLERANCE {
                return true;
            }
        }
        if (start.y > point.y) != (end.y > point.y) {
            let x = start.x + (point.y - start.y) * (end.x - start.x) / (end.y - start.y);
            if x > point.x + LINEAR_TOLERANCE {
                inside = !inside;
            }
        }
    }
    inside
}

fn uv_centroid(points: &[Point2]) -> Point2 {
    let sum = points
        .iter()
        .fold(nalgebra::Vector2::zeros(), |sum, point| sum + point.coords);
    Point2::from(sum / points.len() as f64)
}

fn signed_area(points: &[Point2]) -> f64 {
    0.5 * points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .take(points.len())
        .map(|(a, b)| a.x * b.y - b.x * a.y)
        .sum::<f64>()
}
