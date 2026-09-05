//! Deterministic ray classification and surface-evaluated interior fragment probes.

use super::broad_phase::face_uv_bounds;
use super::{
    BooleanError, BooleanOperand, BooleanOptions, BooleanSide, BooleanTolerances,
    neighborhood::FragmentGraph, operand::operand_cells, trim::FaceTrimDomain,
};
use crate::geometry::{
    Curve, CurveSurfaceIntersection, IntersectionCoverage, IntersectionOptions, Point2, Point3,
    PreparedCurve, PreparedSurface, Surface, SurfacePeriodicity, intersect_prepared_curve_surface,
};
use crate::tessellate::{TessellateOpts, tessellate_face_key};
use crate::topology::{
    gmap::GMap,
    payload::Payload,
    shape_keys::{FaceKey, SolidKey},
};
use nalgebra::Vector3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RelativeLocation {
    Inside,
    Outside,
    OnBoundarySame,
    OnBoundaryOpposite,
}

struct RayFace {
    key: FaceKey,
    origin: Point3,
    normal: Vector3<f64>,
    trim: FaceTrimDomain,
    curved: Option<PreparedSurface>,
    uv_center: Point2,
}

pub(crate) struct SolidRayCaster<'a, P: Payload> {
    map: &'a GMap<P>,
    faces: Vec<RayFace>,
    tolerances: BooleanTolerances,
    max_rays: usize,
}

impl<'a, P: Payload> SolidRayCaster<'a, P> {
    /// Builds a classifier only for surfaces with a complete ray/trim predicate.
    pub(crate) fn new(
        map: &'a GMap<P>,
        keys: impl IntoIterator<Item = FaceKey>,
        options: BooleanOptions,
        tolerances: BooleanTolerances,
    ) -> Result<Self, BooleanError> {
        let mut faces = Vec::new();
        for key in keys {
            let face = map.face_unchecked(key);
            let (origin, normal, curved) = if let Surface::Plane(plane) = face.surface() {
                (plane.origin(), *plane.normal(), None)
            } else {
                let (u, v) = face_uv_bounds(&face)
                    .ok_or(BooleanError::UncertifiedClassificationSurface { face: key })?;
                let prepared = PreparedSurface::over(face.surface(), u, v)?;
                (Point3::origin(), Vector3::zeros(), Some(prepared))
            };
            let trim = FaceTrimDomain::new(&face, tolerances.parameter)?;
            faces.push(RayFace {
                key,
                origin,
                normal,
                trim,
                curved,
                uv_center: face_uv_bounds(&face)
                    .map(|(u, v)| Point2::new((u.start + u.end) * 0.5, (v.start + v.end) * 0.5))
                    .unwrap_or(Point2::origin()),
            });
        }
        Ok(Self {
            map,
            faces,
            tolerances,
            max_rays: options.max_classification_rays,
        })
    }

    /// Classifies a point with two independent accepted rays; disagreement is an error.
    pub(crate) fn classify(
        &self,
        point: Point3,
        source: FaceKey,
        rays: &mut usize,
    ) -> Result<RelativeLocation, BooleanError> {
        let mut answer = None;
        let mut accepted = 0;
        for i in 0..self.max_rays {
            let z = 1.0 - 2.0 * (i as f64 + 0.5) / self.max_rays as f64;
            let angle = i as f64 * 2.399963229728653;
            let radius = (1.0 - z * z).sqrt();
            let direction = Vector3::new(radius * angle.cos(), radius * angle.sin(), z);
            *rays += 1;
            let Some(inside) = self.ray(point, direction) else {
                continue;
            };
            if answer.is_some_and(|previous| previous != inside) {
                break;
            }
            answer = Some(inside);
            accepted += 1;
            if accepted == 2 {
                return Ok(if inside {
                    RelativeLocation::Inside
                } else {
                    RelativeLocation::Outside
                });
            }
        }
        Err(BooleanError::AmbiguousClassification {
            face: source,
            point,
            directions: self.max_rays,
        })
    }

    /// Rejects edge, vertex, tangent, and origin hits instead of assigning uncertain parity.
    fn ray(&self, point: Point3, direction: Vector3<f64>) -> Option<bool> {
        let mut count = 0;
        for face in &self.faces {
            if let Some(surface) = &face.curved {
                count += self.curved_ray(face, surface, point, direction)?;
                continue;
            }
            let distance = face.normal.dot(&(face.origin - point));
            let incidence = face.normal.dot(&direction);
            if incidence.abs() <= self.tolerances.angular {
                if distance.abs() <= self.tolerances.linear {
                    return None;
                }
                continue;
            }
            let t = distance / incidence;
            if t < -self.tolerances.linear {
                continue;
            }
            let hit = point + direction * t;
            let uv = self
                .map
                .face_unchecked(face.key)
                .surface()
                .closest_parameter(hit)
                .ok()?;
            if face.trim.boundary_distance(uv)
                <= face
                    .trim
                    .boundary_epsilon()
                    .max(2.0 * self.tolerances.parameter)
            {
                return None;
            }
            if !face.trim.contains(uv) {
                continue;
            }
            if t <= self.tolerances.linear {
                return None;
            }
            count += 1;
        }
        Some(count % 2 == 1)
    }

    /// Bounds a finite ray by the positive-weight control hull and rejects incomplete searches.
    fn curved_ray(
        &self,
        face: &RayFace,
        surface: &PreparedSurface,
        point: Point3,
        direction: Vector3<f64>,
    ) -> Option<usize> {
        let mut length: f64 = 1.0;
        for control in surface.nurbs().control_points().as_slice() {
            if control.weight() <= 0.0 || !control.weight().is_finite() {
                return None;
            }
            length = length.max((control.to_cartesian() - point).norm() + 1.0);
        }
        let curve = PreparedCurve::new(&Curve::line(point, point + direction * length)).ok()?;
        let options = IntersectionOptions {
            linear_tolerance: self.tolerances.linear,
            parameter_tolerance: self.tolerances.parameter,
            ..IntersectionOptions::default()
        };
        let hits = intersect_prepared_curve_surface(&curve, surface, options).ok()?;
        if !matches!(hits.coverage(), IntersectionCoverage::Complete) {
            return None;
        }
        let mut count = 0;
        for hit in hits {
            let CurveSurfaceIntersection::Point { point: hit, .. } = hit else {
                return None;
            };
            let uv = periodic_uv(
                surface.source().closest_parameter(hit).ok()?,
                face.uv_center,
                surface.source().periodicity(),
            );
            if face.trim.boundary_distance(uv)
                <= face
                    .trim
                    .boundary_epsilon()
                    .max(2.0 * self.tolerances.parameter)
            {
                return None;
            }
            if !face.trim.contains(uv) {
                continue;
            }
            if (hit - point).dot(&direction) <= self.tolerances.linear
                || surface.source().normal_at(uv.x, uv.y).dot(&direction).abs()
                    <= self.tolerances.angular
            {
                return None;
            }
            count += 1;
        }
        Some(count)
    }

    /// Detects coincidence on the other solid before attempting origin-sensitive rays.
    fn boundary(&self, point: Point3, normal: Vector3<f64>) -> Option<RelativeLocation> {
        for face in &self.faces {
            if face.curved.is_none()
                && face.normal.dot(&(point - face.origin)).abs() > self.tolerances.linear
            {
                continue;
            }
            let view = self.map.face_unchecked(face.key);
            let Ok(uv) = view.surface().closest_parameter(point) else {
                continue;
            };
            let uv = periodic_uv(uv, face.uv_center, view.surface().periodicity());
            if (view.point_at(uv.x, uv.y) - point).norm() <= self.tolerances.linear
                && face.trim.contains(uv)
            {
                return Some(if normal.dot(&view.normal_at(uv.x, uv.y)) > 0.0 {
                    RelativeLocation::OnBoundarySame
                } else {
                    RelativeLocation::OnBoundaryOpposite
                });
            }
        }
        None
    }
}

/// Chooses the equivalent periodic image in the face's trimming chart.
fn periodic_uv(mut uv: Point2, center: Point2, periodicity: SurfacePeriodicity) -> Point2 {
    let (u, v) = match periodicity {
        SurfacePeriodicity::None => (None, None),
        SurfacePeriodicity::UPeriodic(u) => (Some(u), None),
        SurfacePeriodicity::VPeriodic(v) => (None, Some(v)),
        SurfacePeriodicity::UVPeriodic(u, v) => (Some(u), Some(v)),
    };
    if let Some(period) = u {
        uv.x += ((center.x - uv.x) / period).round() * period;
    }
    if let Some(period) = v {
        uv.y += ((center.y - uv.y) / period).round() * period;
    }
    uv
}

/// Chooses a mesh-derived witness only after checking exact polygonal trim clearance.
fn probe<P: Payload>(
    map: &GMap<P>,
    face: FaceKey,
    tolerances: BooleanTolerances,
) -> Result<(Point3, Point2), BooleanError> {
    let view = map.face_unchecked(face);
    let trim = FaceTrimDomain::new(&view, tolerances.parameter)?;
    let mesh = tessellate_face_key(map, face, TessellateOpts::default())
        .ok_or(BooleanError::MissingFragmentProbe { face })?;
    let mut triangles = mesh
        .indices
        .chunks_exact(3)
        .map(|ids| {
            let (a, b, c) = (
                mesh.positions[ids[0] as usize],
                mesh.positions[ids[1] as usize],
                mesh.positions[ids[2] as usize],
            );
            (
                (b - a).cross(&(c - a)).norm_squared(),
                Point3::from((a.coords + b.coords + c.coords) / 3.0),
            )
        })
        .collect::<Vec<_>>();
    triangles.sort_by(|a, b| b.0.total_cmp(&a.0));
    for (_, point) in triangles {
        let uv = view.surface().closest_parameter(point)?;
        if trim.contains(uv)
            && trim.boundary_distance(uv) > tolerances.probe_margin.max(trim.boundary_epsilon())
        {
            return Ok((view.point_at(uv.x, uv.y), uv));
        }
    }
    Err(BooleanError::MissingFragmentProbe { face })
}

/// Classifies each fragment independently, avoiding propagation across an incomplete barrier graph.
pub(crate) fn run<P: Payload>(
    map: &GMap<P>,
    graph: &FragmentGraph,
    options: BooleanOptions,
    tolerances: BooleanTolerances,
) -> Result<(Vec<RelativeLocation>, usize), BooleanError> {
    let first = SolidRayCaster::new(
        map,
        graph
            .fragments
            .iter()
            .filter(|f| f.side == BooleanSide::First)
            .map(|f| f.face),
        options,
        tolerances,
    )?;
    let second = SolidRayCaster::new(
        map,
        graph
            .fragments
            .iter()
            .filter(|f| f.side == BooleanSide::Second)
            .map(|f| f.face),
        options,
        tolerances,
    )?;
    let mut rays = 0;
    let mut result = Vec::new();
    for fragment in &graph.fragments {
        let (point, uv) = probe(map, fragment.face, tolerances)?;
        let normal = *map.face_unchecked(fragment.face).normal_at(uv.x, uv.y);
        let caster = match fragment.side {
            BooleanSide::First => &second,
            BooleanSide::Second => &first,
        };
        let location = match caster.boundary(point, normal) {
            Some(location) => location,
            None => caster.classify(point, fragment.face, &mut rays)?,
        };
        result.push(location);
    }
    Ok((result, rays))
}

/// Reports whether `point` lies inside a registered solid, using the same
/// certified ray classifier the Boolean selection stage relies on.
///
/// A point on the boundary has no answer here: every ray from it is rejected,
/// so the classification is reported as ambiguous rather than guessed.
pub fn solid_contains_point<P: Payload>(
    map: &GMap<P>,
    solid: SolidKey,
    point: Point3,
    options: BooleanOptions,
) -> Result<bool, BooleanError> {
    let cells = operand_cells(map, BooleanOperand::Solid(solid))?;
    let tolerances = BooleanTolerances::from_cells(map, &cells, &cells, options.tolerances)?;
    let source = *cells
        .faces
        .iter()
        .next()
        .ok_or(BooleanError::MissingOperand {
            operand: BooleanOperand::Solid(solid),
        })?;
    let caster = SolidRayCaster::new(map, cells.faces.iter().copied(), options, tolerances)?;
    let mut rays = 0;
    Ok(caster.classify(point, source, &mut rays)? == RelativeLocation::Inside)
}
