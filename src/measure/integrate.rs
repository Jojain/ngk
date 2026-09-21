use nalgebra::{Matrix3, Vector3};
use thiserror::Error;

use crate::geometry::dim2::curves::Curve2;
use crate::geometry::parameter::Fraction;
use crate::geometry::{Point3, Surface};
use crate::model::Model;
use crate::tessellate::{
    CurveOpts, IndexedMesh, TessellateError, TessellateOpts, curve::tessellate_curve,
    face::tessellate_face,
};
use crate::topology::attributes::LoopKind;
use crate::topology::edge::{Edge, EdgeCore};
use crate::topology::face::Face;
use crate::topology::gmap::Dart;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::FaceKey;
use crate::topology::sheet::Sheet;
use crate::topology::solid::Solid;

use super::types::{Inertia, LinearProperties, SurfaceProperties, VolumeProperties};

const COMPUTATION_TESSELLATION: TessellateOpts = TessellateOpts {
    curve: CurveOpts { segments: 64 },
    surface: crate::tessellate::SurfaceOpts { nu: 64, nv: 32 },
};

/// Why a mass-property integration could not produce a finite result.
#[derive(Debug, Error)]
pub enum MeasureError {
    #[error("face {face:?} could not be tessellated: {source}")]
    FaceTessellation {
        face: FaceKey,
        #[source]
        source: TessellateError,
    },
    #[error("edge {edge:?} has an unbounded parameter interval")]
    UnboundedEdge {
        edge: crate::topology::shape_keys::EdgeKey,
    },
    #[error("the measured geometry has no non-degenerate measure")]
    Degenerate,
    #[error("the measured geometry produced a non-finite result")]
    NonFinite,
    #[error("shell {shell} has the wrong orientation for its solid side")]
    InvalidOrientation { shell: usize, signed_volume: f64 },
}

#[derive(Default)]
struct Moments {
    measure: f64,
    first: Vector3<f64>,
    second: Matrix3<f64>,
}

impl Moments {
    fn add(&mut self, measure: f64, centroid: Vector3<f64>, second: Matrix3<f64>) {
        self.measure += measure;
        self.first += centroid * measure;
        self.second += second;
    }

    fn merge(&mut self, other: Self) {
        self.measure += other.measure;
        self.first += other.first;
        self.second += other.second;
    }
}

fn outer(a: Vector3<f64>, b: Vector3<f64>) -> Matrix3<f64> {
    a * b.transpose()
}

fn inertia_from_moments<D>(
    moments: Moments,
    reference: Point3,
) -> Result<(f64, Point3, Inertia<D>), MeasureError> {
    if !moments.measure.is_finite() || moments.measure <= 0.0 {
        return Err(MeasureError::Degenerate);
    }
    let local_centroid = moments.first / moments.measure;
    let centroid = reference + local_centroid;
    let central_second = moments.second - moments.measure * outer(local_centroid, local_centroid);
    let inertia = Matrix3::identity() * central_second.trace() - central_second;
    if !centroid.coords.iter().all(|value| value.is_finite())
        || !inertia.iter().all(|value| value.is_finite())
    {
        return Err(MeasureError::NonFinite);
    }
    Ok((
        moments.measure,
        centroid,
        Inertia::new(inertia, moments.measure),
    ))
}

fn triangle_surface_moments(mesh: &IndexedMesh, reference: Point3) -> Moments {
    let mut moments = Moments::default();
    for triangle in mesh.indices.chunks_exact(3) {
        let points = [
            mesh.positions[triangle[0] as usize] - reference,
            mesh.positions[triangle[1] as usize] - reference,
            mesh.positions[triangle[2] as usize] - reference,
        ];
        let area = 0.5
            * (points[1] - points[0])
                .cross(&(points[2] - points[0]))
                .norm();
        if area <= f64::EPSILON {
            continue;
        }
        let sum = points[0] + points[1] + points[2];
        let second = (outer(points[0], points[0])
            + outer(points[1], points[1])
            + outer(points[2], points[2])
            + outer(sum, sum))
            * (area / 12.0);
        moments.add(area, sum / 3.0, second);
    }
    moments
}

fn triangle_volume_moments(mesh: &IndexedMesh, reference: Point3) -> Moments {
    let mut moments = Moments::default();
    for triangle in mesh.indices.chunks_exact(3) {
        let points = [
            mesh.positions[triangle[0] as usize] - reference,
            mesh.positions[triangle[1] as usize] - reference,
            mesh.positions[triangle[2] as usize] - reference,
        ];
        let signed_volume = points[0].dot(&points[1].cross(&points[2])) / 6.0;
        let sum = points[0] + points[1] + points[2];
        let second = (outer(points[0], points[0])
            + outer(points[1], points[1])
            + outer(points[2], points[2])
            + outer(sum, sum))
            * (signed_volume / 20.0);
        moments.add(signed_volume, sum / 4.0, second);
    }
    moments
}

fn segment_linear_moments(points: &[Point3], reference: Point3) -> Moments {
    let mut moments = Moments::default();
    for pair in points.windows(2) {
        let a = pair[0] - reference;
        let b = pair[1] - reference;
        let length = (b - a).norm();
        if length <= f64::EPSILON {
            continue;
        }
        let second =
            (outer(a, a) * 2.0 + outer(b, b) * 2.0 + outer(a, b) + outer(b, a)) * (length / 6.0);
        moments.add(length, (a + b) / 2.0, second);
    }
    moments
}

fn first_point(mesh: &IndexedMesh) -> Option<Point3> {
    mesh.indices
        .first()
        .map(|index| mesh.positions[*index as usize])
}

fn mesh_for_face<P: Payload>(face: &Face<'_, P>) -> Result<IndexedMesh, MeasureError> {
    tessellate_face(face, COMPUTATION_TESSELLATION).map_err(|source| {
        MeasureError::FaceTessellation {
            face: face.key(),
            source,
        }
    })
}

/// Computes positive area properties for one face.
pub(crate) fn face_surface_properties<P: Payload>(
    face: &Face<'_, P>,
) -> Result<SurfaceProperties, MeasureError> {
    let mesh = mesh_for_face(face)?;
    let reference = first_point(&mesh).ok_or(MeasureError::Degenerate)?;
    let moments = triangle_surface_moments(&mesh, reference);
    let (area, centroid, inertia) = inertia_from_moments(moments, reference)?;
    Ok(SurfaceProperties {
        area,
        centroid,
        inertia,
    })
}

/// Computes positive area properties for all faces of one sheet.
pub(crate) fn sheet_surface_properties<P: Payload>(
    sheet: &Sheet<'_, P>,
) -> Result<SurfaceProperties, MeasureError> {
    let meshes = sheet
        .faces()
        .into_iter()
        .map(|face| mesh_for_face(&face))
        .collect::<Result<Vec<_>, _>>()?;
    surface_properties_from_meshes(&meshes)
}

fn surface_properties_from_meshes(
    meshes: &[IndexedMesh],
) -> Result<SurfaceProperties, MeasureError> {
    let reference = meshes
        .iter()
        .find_map(first_point)
        .ok_or(MeasureError::Degenerate)?;
    let moments = meshes.iter().fold(Moments::default(), |mut total, mesh| {
        let part = triangle_surface_moments(mesh, reference);
        if part.measure > 0.0 {
            total.add(part.measure, part.first / part.measure, part.second);
        }
        total
    });
    let (area, centroid, inertia) = inertia_from_moments(moments, reference)?;
    Ok(SurfaceProperties {
        area,
        centroid,
        inertia,
    })
}

/// Computes linear properties for one oriented edge.
pub(crate) fn linear_properties_for_edge<P: Payload>(
    edge: &EdgeCore<'_, P>,
) -> Result<LinearProperties, MeasureError> {
    let interval = edge.parameter_interval();
    if !interval.is_finite() {
        return Err(MeasureError::UnboundedEdge { edge: edge.key() });
    }
    let polyline = tessellate_curve(
        edge.curve(),
        interval.start.value(),
        interval.end.value(),
        COMPUTATION_TESSELLATION.curve,
    );
    let reference = *polyline.points.first().ok_or(MeasureError::Degenerate)?;
    let moments = segment_linear_moments(&polyline.points, reference);
    let exact_length = edge.length();
    let scale = exact_length / moments.measure;
    let moments = Moments {
        measure: exact_length,
        first: moments.first * scale,
        second: moments.second * scale,
    };
    let (length, centroid, inertia) = inertia_from_moments(moments, reference)?;
    Ok(LinearProperties {
        length,
        centroid,
        inertia,
    })
}

/// Computes linear properties for every edge in a profile.
pub(crate) fn linear_properties_for_edges<'a, P: Payload>(
    edges: impl IntoIterator<Item = Edge<'a, P>>,
) -> Result<LinearProperties, MeasureError> {
    let edges = edges.into_iter().collect::<Vec<_>>();
    let mut all = Moments::default();
    let mut reference = None;
    for edge in edges {
        let properties = linear_properties_for_edge(&edge)?;
        let origin = reference.get_or_insert(properties.centroid);
        let local_centroid = properties.centroid - *origin;
        let central = inverse_inertia(properties.inertia);
        all.add(
            properties.length,
            local_centroid,
            central + properties.length * outer(local_centroid, local_centroid),
        );
    }
    let reference = reference.ok_or(MeasureError::Degenerate)?;
    let (length, centroid, inertia) = inertia_from_moments(all, reference)?;
    Ok(LinearProperties {
        length,
        centroid,
        inertia,
    })
}

/// Adds two positive surface-property bundles using a common reference point.
pub(crate) fn combine_surface_properties(
    first: SurfaceProperties,
    second: SurfaceProperties,
) -> SurfaceProperties {
    let reference = first.centroid;
    let mut moments = Moments::default();
    for properties in [first, second] {
        let measure = properties.area;
        let local_centroid = properties.centroid - reference;
        let central = inverse_inertia(properties.inertia);
        moments.add(
            measure,
            local_centroid,
            central + measure * outer(local_centroid, local_centroid),
        );
    }
    let (area, centroid, inertia) = inertia_from_moments(moments, reference)
        .expect("combining two valid surface-property bundles must remain valid");
    SurfaceProperties {
        area,
        centroid,
        inertia,
    }
}

fn inverse_inertia<D>(inertia: Inertia<D>) -> Matrix3<f64> {
    let tensor = inertia.tensor();
    Matrix3::identity() * (tensor.trace() / 2.0) - tensor
}

fn shell_faces<P: Payload>(model: &Model<P>, shell: Dart) -> Vec<Face<'_, P>> {
    Sheet::from_dart(model, shell)
        .expect("a measured shell must have a registered sheet")
        .faces()
        .into_iter()
        .map(|face| {
            if face.loops().is_empty() {
                face
            } else {
                model.face_unchecked(face.key())
            }
        })
        .collect()
}

fn shell_meshes<P: Payload>(
    model: &Model<P>,
    shell: Dart,
) -> Result<Vec<IndexedMesh>, MeasureError> {
    shell_faces(model, shell)
        .iter()
        .map(mesh_for_face)
        .collect()
}

fn signed_volume_from_meshes(meshes: &[IndexedMesh]) -> Result<f64, MeasureError> {
    let reference = meshes
        .iter()
        .find_map(first_point)
        .ok_or(MeasureError::Degenerate)?;
    let volume = meshes
        .iter()
        .map(|mesh| triangle_volume_moments(mesh, reference).measure)
        .sum::<f64>();
    volume
        .is_finite()
        .then_some(volume)
        .ok_or(MeasureError::NonFinite)
}

fn face_reference<P: Payload>(face: &Face<'_, P>) -> Option<Point3> {
    face.vertices()
        .first()
        .map(|vertex| *vertex.point())
        .or_else(|| {
            face.loops()
                .into_iter()
                .next()?
                .edges()
                .into_iter()
                .next()
                .map(|edge| edge.trimmed_curve().point_at(Fraction::new(0.0)))
        })
        .or_else(|| {
            let (u, v) = face.surface().domain();
            (u.is_finite() && v.is_finite()).then(|| {
                face.point_at(
                    u.at(Fraction::new(0.5)).value(),
                    v.at(Fraction::new(0.5)).value(),
                )
            })
        })
}

/// Retains the sign-only pcurve fan for faces whose imported UV boundary is
/// not readable by the general tessellator. It is used only as a validation
/// fallback; public mass properties report the tessellation error instead.
fn fallback_face_signed_volume<P: Payload>(face: &Face<'_, P>, reference: Point3) -> Option<f64> {
    let enclosed = face
        .loops()
        .iter()
        .any(|boundary| !matches!(boundary.kind(), LoopKind::Inner));
    let mut volume = if enclosed {
        0.0
    } else {
        let (u_span, v_span) = face.surface().domain();
        if !u_span.is_finite() || !v_span.is_finite() {
            return None;
        }
        const STEPS: usize = 24;
        let point = |i: usize, j: usize| {
            let u = u_span.at(Fraction::new(i as f64 / STEPS as f64));
            let v = v_span.at(Fraction::new(j as f64 / STEPS as f64));
            face.point_at(u.value(), v.value()) - reference
        };
        let mut volume = 0.0;
        for i in 0..STEPS {
            for j in 0..STEPS {
                let (a, b, c, d) = (
                    point(i, j),
                    point(i + 1, j),
                    point(i + 1, j + 1),
                    point(i, j + 1),
                );
                volume += a.dot(&b.cross(&c)) / 6.0;
                volume += a.dot(&c.cross(&d)) / 6.0;
            }
        }
        volume
    };

    let planar = matches!(face.surface(), Surface::Plane(_));
    for boundary in face.loops() {
        let mut uvs = Vec::new();
        for edge in boundary.edges() {
            let curve = face.pcurve(edge.dart())?;
            let count = if planar && matches!(curve.curve(), Curve2::Line(_)) {
                1
            } else {
                32
            };
            uvs.extend(curve.sample(count).into_iter().take(count));
        }
        let origin = *uvs.first()?;
        for pair in uvs[1..].windows(2) {
            let count = if planar { 1 } else { 16 };
            let point = |i: usize, j: usize| {
                let uv = origin
                    + (pair[0] - origin) * (i as f64 / count as f64)
                    + (pair[1] - origin) * (j as f64 / count as f64);
                face.point_at(uv.x, uv.y) - reference
            };
            for i in 0..count {
                for j in 0..count - i {
                    let (a, b, c) = (point(i, j), point(i + 1, j), point(i, j + 1));
                    volume += a.dot(&b.cross(&c)) / 6.0;
                    if i + j + 1 < count {
                        volume += b.dot(&point(i + 1, j + 1).cross(&c)) / 6.0;
                    }
                }
            }
        }
    }
    let volume = match face.sense() {
        crate::topology::orientation::Orientation::Same => volume,
        crate::topology::orientation::Orientation::Reversed => -volume,
    };
    volume.is_finite().then_some(volume)
}

fn fallback_signed_volume<P: Payload>(faces: &[Face<'_, P>]) -> Result<f64, MeasureError> {
    let reference = faces
        .iter()
        .find_map(face_reference)
        .ok_or(MeasureError::Degenerate)?;
    faces
        .iter()
        .map(|face| fallback_face_signed_volume(face, reference).ok_or(MeasureError::Degenerate))
        .try_fold(0.0, |total, contribution| {
            contribution.map(|value| total + value)
        })
}

/// Computes the signed volume of one oriented shell for validation.
pub(crate) fn signed_shell_volume<P: Payload>(
    model: &Model<P>,
    shell: Dart,
) -> Result<f64, MeasureError> {
    let faces = shell_faces(model, shell);
    let meshes = faces
        .iter()
        .map(mesh_for_face)
        .collect::<Result<Vec<_>, _>>();
    match meshes {
        Ok(meshes) => signed_volume_from_meshes(&meshes),
        Err(_) => fallback_signed_volume(&faces),
    }
}

/// Computes the signed volume of a set of oriented faces during assembly.
pub(crate) fn signed_volume_for_faces<P: Payload>(
    model: &Model<P>,
    faces: &[FaceKey],
) -> Result<f64, MeasureError> {
    let views = faces
        .iter()
        .map(|&key| model.face_unchecked(key))
        .collect::<Vec<_>>();
    let meshes = views
        .iter()
        .map(mesh_for_face)
        .collect::<Result<Vec<_>, _>>();
    match meshes {
        Ok(meshes) => signed_volume_from_meshes(&meshes),
        Err(_) => fallback_signed_volume(&views),
    }
}

/// Computes non-negative volume properties for a solid and its oriented shells.
pub(crate) fn solid_volume_properties<P: Payload>(
    solid: &Solid<'_, P>,
) -> Result<VolumeProperties, MeasureError> {
    let shells = solid.shells();
    let meshes = shells
        .iter()
        .map(|shell| shell_meshes(solid.model(), shell.dart()))
        .collect::<Result<Vec<_>, _>>()?;
    let reference = meshes
        .iter()
        .flat_map(|shell| shell.iter().filter_map(first_point))
        .next()
        .ok_or(MeasureError::Degenerate)?;
    let mut moments = Moments::default();
    for (index, shell_meshes) in meshes.iter().enumerate() {
        let signed_volume = signed_volume_from_meshes(shell_meshes)?;
        let expected_positive = index == 0;
        if (expected_positive && signed_volume <= 0.0)
            || (!expected_positive && signed_volume >= 0.0)
        {
            return Err(MeasureError::InvalidOrientation {
                shell: index,
                signed_volume,
            });
        }
        for mesh in shell_meshes {
            let part = triangle_volume_moments(mesh, reference);
            moments.merge(part);
        }
    }
    let (volume, centroid, inertia) = inertia_from_moments(moments, reference)?;
    Ok(VolumeProperties {
        volume,
        centroid,
        inertia,
    })
}
