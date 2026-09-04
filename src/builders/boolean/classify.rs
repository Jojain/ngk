//! Deterministic planar ray classification and interior fragment probes.

use super::{
    BooleanError, BooleanOperand, BooleanOptions, BooleanSide, BooleanTolerances,
    neighborhood::FragmentGraph, operand::operand_cells, trim::FaceTrimDomain,
};
use crate::geometry::{Point2, Point3, Surface};
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
            let Surface::Plane(plane) = face.surface() else {
                return Err(BooleanError::UncertifiedClassificationSurface { face: key });
            };
            let trim = FaceTrimDomain::new(&face, tolerances.parameter)?;
            if !trim.is_polygonal() {
                return Err(BooleanError::UncertifiedClassificationSurface { face: key });
            }
            faces.push(RayFace {
                key,
                origin: plane.origin(),
                normal: *plane.normal(),
                trim,
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
            if face.trim.boundary_distance(uv) <= self.tolerances.parameter {
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

    /// Detects coincidence on the other solid before attempting origin-sensitive rays.
    fn boundary(&self, point: Point3, normal: Vector3<f64>) -> Option<RelativeLocation> {
        for face in &self.faces {
            if face.normal.dot(&(point - face.origin)).abs() > self.tolerances.linear {
                continue;
            }
            let view = self.map.face_unchecked(face.key);
            let Ok(uv) = view.surface().closest_parameter(point) else {
                continue;
            };
            if face.trim.contains(uv) {
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
        if trim.contains(uv) && trim.boundary_distance(uv) > tolerances.probe_margin {
            return Ok((point, uv));
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
