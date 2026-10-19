use nalgebra::{Matrix3, SVector, Vector3};
use thiserror::Error;

use crate::geometry::parameter::Fraction;
use crate::geometry::{Point3, TrimmedCurve};
use crate::model::Model;
use crate::topology::edge::{Edge, EdgeCore};
use crate::topology::face::Face;
use crate::topology::gmap::Dart;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey};
use crate::topology::sheet::Sheet;
use crate::topology::solid::Solid;
use crate::topology::unwrapped_face_domain::UnwrappedFaceDomainError;

use super::green::{FaceDomain, curve_breaks};
use super::quadrature::{Quadrature, integrate};
use super::types::{Inertia, LinearProperties, SurfaceProperties, VolumeProperties};

/// What the public properties are evaluated to: each moment's error within
/// this fraction of the integral of its magnitude.
const PROPERTIES: Quadrature = Quadrature::Adaptive(1e-10);

/// Why a mass-property integration could not produce a finite result.
#[derive(Debug, Error)]
pub enum MeasureError {
    #[error("face {face:?} has no readable parameter-space boundary: {source}")]
    UnreadableBoundary {
        face: FaceKey,
        #[source]
        source: UnwrappedFaceDomainError,
    },
    #[error("face {face:?} is bounded by no loop and its support's domain is unbounded")]
    UnboundedFace { face: FaceKey },
    #[error("face {face:?} did not reach the measuring tolerance")]
    FaceUnconverged { face: FaceKey },
    #[error("edge {edge:?} did not reach the measuring tolerance")]
    EdgeUnconverged { edge: EdgeKey },
    #[error("edge {edge:?} has an unbounded parameter interval")]
    UnboundedEdge { edge: EdgeKey },
    #[error("the measured geometry has no non-degenerate measure")]
    Degenerate,
    #[error("the measured geometry produced a non-finite result")]
    NonFinite,
    #[error("shell {shell} has the wrong orientation for its solid side")]
    InvalidOrientation { shell: usize, signed_volume: f64 },
    #[error("the bounding box of edge {edge:?} on a {curve} is not implemented yet")]
    EdgeBoundingBoxNotImplemented { edge: EdgeKey, curve: &'static str },
    #[error("the bounding box of face {face:?} on a {surface} is not implemented yet")]
    FaceBoundingBoxNotImplemented {
        face: FaceKey,
        surface: &'static str,
    },
}

/// Zeroth, first and second moments about a reference point, in the order
/// `1, x, y, z, x², y², z², xy, xz, yz`.
type MomentVector = SVector<f64, 10>;

/// The moments of `weight` placed at `offset` from the reference point.
fn weighted_moments(offset: Vector3<f64>, weight: f64) -> MomentVector {
    let [x, y, z] = [offset.x, offset.y, offset.z];
    MomentVector::from([1.0, x, y, z, x * x, y * y, z * z, x * y, x * z, y * z]) * weight
}

/// The flux through a surface element of one vector field per volume moment.
///
/// By the divergence theorem a solid's integral of `φ` is the flux out of it of
/// any field whose divergence is `φ`. Each field here points along one axis,
/// so its flux needs one normal component.
fn volume_moments(offset: Vector3<f64>, normal: Vector3<f64>) -> MomentVector {
    let [x, y, z] = [offset.x, offset.y, offset.z];
    let [nx, ny, nz] = [normal.x, normal.y, normal.z];
    MomentVector::from([
        x * nx,
        x * x / 2.0 * nx,
        y * y / 2.0 * ny,
        z * z / 2.0 * nz,
        x * x * x / 3.0 * nx,
        y * y * y / 3.0 * ny,
        z * z * z / 3.0 * nz,
        x * x * y / 2.0 * nx,
        x * x * z / 2.0 * nx,
        y * y * z / 2.0 * ny,
    ])
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
}

impl From<MomentVector> for Moments {
    fn from(moments: MomentVector) -> Self {
        let m = moments;
        Self {
            measure: m[0],
            first: Vector3::new(m[1], m[2], m[3]),
            second: Matrix3::new(m[4], m[7], m[8], m[7], m[5], m[9], m[8], m[9], m[6]),
        }
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

/// The point and length the moments are taken about and measured in.
///
/// Any point gives the same properties in exact arithmetic. The centre of the
/// box around the shape keeps the moments the size of the shape rather than of
/// its distance from the origin, so nothing large cancels when they are moved
/// to the centroid. Offsets are taken in units of half the box's diagonal, so
/// the measure and its first and second moments are all about the same size
/// and one relative tolerance holds every one of them to the shape's scale.
#[derive(Debug, Clone, Copy)]
struct Reference {
    centre: Point3,
    scale: f64,
}

impl Reference {
    fn around(points: impl IntoIterator<Item = Point3>) -> Result<Self, MeasureError> {
        let mut points = points.into_iter();
        let first = points.next().ok_or(MeasureError::Degenerate)?;
        let (low, high) = points.fold((first, first), |(low, high), point| {
            (low.inf(&point), high.sup(&point))
        });
        let scale = 0.5 * (high - low).norm();
        if !scale.is_finite() || scale <= 0.0 {
            return Err(MeasureError::Degenerate);
        }
        Ok(Self {
            centre: nalgebra::center(&low, &high),
            scale,
        })
    }

    /// `point`'s offset from the centre, in units of the scale.
    fn offset(&self, point: Point3) -> Vector3<f64> {
        (point - self.centre) / self.scale
    }

    /// Moments taken in scaled offsets, back in world units, where the
    /// measure itself carries `power` factors of the scale.
    fn unscale(&self, moments: MomentVector, power: i32) -> MomentVector {
        let [zeroth, first, second] = [power, power + 1, power + 2].map(|p| self.scale.powi(p));
        let mut moments = moments;
        moments[0] *= zeroth;
        for index in 1..4 {
            moments[index] *= first;
        }
        for index in 4..10 {
            moments[index] *= second;
        }
        moments
    }
}

/// Reads every face's domain, keyed for the errors that name it.
fn face_domains<'f, P: Payload>(
    faces: &'f [Face<'_, P>],
) -> Result<Vec<(FaceKey, FaceDomain<'f>)>, MeasureError> {
    faces
        .iter()
        .map(|face| Ok((face.key(), FaceDomain::of_face(face)?)))
        .collect()
}

fn domains_reference(domains: &[(FaceKey, FaceDomain<'_>)]) -> Result<Reference, MeasureError> {
    Reference::around(
        domains
            .iter()
            .flat_map(|(_, domain)| domain.sample_points()),
    )
}

/// Positive area moments of one face about `reference`, in world units.
fn face_area_moments(
    face: FaceKey,
    domain: &FaceDomain<'_>,
    reference: Reference,
) -> Result<MomentVector, MeasureError> {
    let integral = domain
        .integrate(PROPERTIES, |point, du, dv| {
            weighted_moments(reference.offset(point), du.cross(&dv).norm())
        })
        .map_err(|_| MeasureError::FaceUnconverged { face })?;
    // The loops wind with the face's sense, and an area has none.
    let moments = match integral.value[0] < 0.0 {
        true => -integral.value,
        false => integral.value,
    };
    Ok(reference.unscale(moments, 0))
}

/// Signed volume moments one face contributes about `reference`, in world
/// units.
fn face_volume_moments(
    face: FaceKey,
    domain: &FaceDomain<'_>,
    reference: Reference,
) -> Result<MomentVector, MeasureError> {
    domain
        .integrate(PROPERTIES, |point, du, dv| {
            volume_moments(reference.offset(point), du.cross(&dv))
        })
        .map(|integral| reference.unscale(integral.value, 1))
        .map_err(|_| MeasureError::FaceUnconverged { face })
}

fn surface_properties_from(
    moments: MomentVector,
    reference: Reference,
) -> Result<SurfaceProperties, MeasureError> {
    let (area, centroid, inertia) = inertia_from_moments(moments.into(), reference.centre)?;
    Ok(SurfaceProperties {
        area,
        centroid,
        inertia,
    })
}

/// Computes positive area properties for one face.
pub(crate) fn face_surface_properties<P: Payload>(
    face: &Face<'_, P>,
) -> Result<SurfaceProperties, MeasureError> {
    sheet_faces_surface_properties(std::slice::from_ref(face))
}

/// Computes positive area properties for all faces of one sheet.
pub(crate) fn sheet_surface_properties<P: Payload>(
    sheet: &Sheet<'_, P>,
) -> Result<SurfaceProperties, MeasureError> {
    sheet_faces_surface_properties(&sheet.faces())
}

fn sheet_faces_surface_properties<P: Payload>(
    faces: &[Face<'_, P>],
) -> Result<SurfaceProperties, MeasureError> {
    let domains = face_domains(faces)?;
    let reference = domains_reference(&domains)?;
    let mut total = MomentVector::zeros();
    for (face, domain) in &domains {
        total += face_area_moments(*face, domain, reference)?;
    }
    surface_properties_from(total, reference)
}

/// Computes linear properties for one oriented edge.
pub(crate) fn linear_properties_for_edge<P: Payload>(
    edge: &EdgeCore<'_, P>,
) -> Result<LinearProperties, MeasureError> {
    linear_properties_for_spans(&[(edge.key(), edge_span(edge)?)])
}

/// Computes linear properties for every edge in a profile.
pub(crate) fn linear_properties_for_edges<'a, P: Payload>(
    edges: impl IntoIterator<Item = Edge<'a, P>>,
) -> Result<LinearProperties, MeasureError> {
    let spans = edges
        .into_iter()
        .map(|edge| Ok((edge.key(), edge_span(&edge)?)))
        .collect::<Result<Vec<_>, MeasureError>>()?;
    linear_properties_for_spans(&spans)
}

fn edge_span<P: Payload>(edge: &EdgeCore<'_, P>) -> Result<TrimmedCurve, MeasureError> {
    if !edge.parameter_interval().is_finite() {
        return Err(MeasureError::UnboundedEdge { edge: edge.key() });
    }
    Ok(edge.trimmed_curve())
}

/// Length moments of the curves, integrated along each one's 3D span.
fn linear_properties_for_spans(
    spans: &[(EdgeKey, TrimmedCurve)],
) -> Result<LinearProperties, MeasureError> {
    const SAMPLES: [f64; 5] = [0.0, 0.25, 0.5, 0.75, 1.0];
    let reference = Reference::around(spans.iter().flat_map(|(_, span)| {
        SAMPLES
            .iter()
            .map(|fraction| span.point_at(Fraction::new(*fraction)))
    }))?;
    let mut total = MomentVector::zeros();
    for (edge, span) in spans {
        let interval = span.interval();
        let mut breaks = vec![0.0];
        breaks.extend(
            curve_breaks(span.curve(), interval)
                .into_iter()
                .map(|parameter| interval.fraction_of(parameter).value()),
        );
        breaks.push(1.0);
        breaks.sort_by(f64::total_cmp);
        let integral = integrate(PROPERTIES, &breaks, |fraction| {
            let fraction = Fraction::new(fraction);
            let speed = span.derivative_at(fraction, 1).norm();
            Ok(weighted_moments(
                reference.offset(span.point_at(fraction)),
                speed,
            ))
        })
        .map_err(|_| MeasureError::EdgeUnconverged { edge: *edge })?;
        total += reference.unscale(integral.value, 0);
    }
    let (length, centroid, inertia) = inertia_from_moments(total.into(), reference.centre)?;
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

/// Signed volume enclosed by oriented faces, at the fixed low order.
///
/// Only the sign and the rough size are asked of it, so no panel is refined.
fn signed_volume_of<P: Payload>(faces: &[Face<'_, P>]) -> Result<f64, MeasureError> {
    let domains = face_domains(faces)?;
    let centre = domains_reference(&domains)?.centre;
    let mut volume = 0.0;
    for (face, domain) in &domains {
        volume += domain
            .integrate(Quadrature::Fixed, |point, du, dv| {
                SVector::<f64, 1>::new((point.x - centre.x) * du.cross(&dv).x)
            })
            .map_err(|_| MeasureError::FaceUnconverged { face: *face })?
            .value[0];
    }
    volume
        .is_finite()
        .then_some(volume)
        .ok_or(MeasureError::NonFinite)
}

/// Computes the signed volume of one oriented shell for validation.
pub(crate) fn signed_shell_volume<P: Payload>(
    model: &Model<P>,
    shell: Dart,
) -> Result<f64, MeasureError> {
    signed_volume_of(&shell_faces(model, shell))
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
    signed_volume_of(&views)
}

/// Computes non-negative volume properties for a solid and its oriented shells.
pub(crate) fn solid_volume_properties<P: Payload>(
    solid: &Solid<'_, P>,
) -> Result<VolumeProperties, MeasureError> {
    let shells = solid
        .shells()
        .iter()
        .map(|shell| shell_faces(solid.model(), shell.dart()))
        .collect::<Vec<_>>();
    let domains = shells
        .iter()
        .map(|faces| face_domains(faces))
        .collect::<Result<Vec<_>, _>>()?;
    let reference = Reference::around(
        domains
            .iter()
            .flatten()
            .flat_map(|(_, domain)| domain.sample_points()),
    )?;
    let mut total = MomentVector::zeros();
    for (index, shell) in domains.iter().enumerate() {
        let mut moments = MomentVector::zeros();
        for (face, domain) in shell {
            moments += face_volume_moments(*face, domain, reference)?;
        }
        let signed_volume = moments[0];
        if !signed_volume.is_finite() {
            return Err(MeasureError::NonFinite);
        }
        let expected_positive = index == 0;
        if (expected_positive && signed_volume <= 0.0)
            || (!expected_positive && signed_volume >= 0.0)
        {
            return Err(MeasureError::InvalidOrientation {
                shell: index,
                signed_volume,
            });
        }
        total += moments;
    }
    let (volume, centroid, inertia) = inertia_from_moments(total.into(), reference.centre)?;
    Ok(VolumeProperties {
        volume,
        centroid,
        inertia,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builders::faces::reverse_face_winding_edit;
    use crate::modeling::solids::block;
    use crate::topology::ModelEditError;
    use crate::topology::validation::validate_solid_orientation;

    #[test]
    fn an_inside_out_solid_is_refused_rather_than_measured_negative() {
        let mut shape = block(2.0, 3.0, 4.0).expect("block should build");
        let solid = shape.key();
        let faces = shape
            .solid()
            .faces()
            .iter()
            .map(Face::key)
            .collect::<Vec<_>>();
        shape
            .model_mut()
            .transaction(|edit| {
                for face in &faces {
                    reverse_face_winding_edit(edit, *face);
                }
                Ok::<_, ModelEditError>(())
            })
            .expect("turning every face over commits");

        let model = shape.model();
        let shell = model.solid_unchecked(solid).shells()[0].dart();
        let signed = signed_shell_volume(model, shell).expect("the shell still measures");
        assert!(
            (signed + 24.0).abs() <= 1e-9,
            "an inside-out block encloses minus its volume, got {signed}"
        );
        assert!(validate_solid_orientation(model, solid).is_err());
        assert!(matches!(
            model.solid_unchecked(solid).volume(),
            Err(MeasureError::InvalidOrientation { shell: 0, signed_volume }) if signed_volume < 0.0
        ));
    }
}
