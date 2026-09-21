use nalgebra::Vector3;

use crate::builders::solids::add_extruded_face;
use crate::builders::sweep::{FaceSweep, SweepError, SweepOptions, SweepSpine, add_swept_face};
use crate::builders::{errors::ExtrudeError, sheets::add_extruded_profile};
use crate::model::MergeTopology;
use crate::topology::payload::Payload;
use crate::topology::profile::Profile;
use crate::topology::shape::{FaceTag, Shape, SheetTag, SolidTag};

pub fn extrude_profile<P: Payload>(
    profile: Profile<'_, P>,
    direction: Vector3<f64>,
) -> Result<Shape<SheetTag, P>, ExtrudeError> {
    let (mut g, profile_dart) = profile.isolate();
    let profile_key = g.profile_key_unchecked(profile_dart);
    let sheet = add_extruded_profile(&mut g, profile_key, direction)?;
    Ok(Shape::new(g, sheet.sheet))
}

pub fn extrude_face<P: Payload>(
    face: Shape<FaceTag, P>,
    direction: Vector3<f64>,
) -> Result<Shape<SolidTag, P>, ExtrudeError> {
    let (mut g, face_key) = face.into_model();
    let solid_key = add_extruded_face(&mut g, face_key, direction)?;
    Ok(Shape::new(g, solid_key.solid))
}

/// Sweeps an owned face along a borrowed edge or profile view.
///
/// The face is already placed at the first spine point and perpendicular to
/// its first tangent. Sharp junctions follow `options.transition`; smooth
/// junctions add no corner tape.
pub fn sweep_face<P: Payload, S: SweepSpine + ?Sized>(
    face: Shape<FaceTag, P>,
    spine: &S,
    options: SweepOptions,
) -> Result<Shape<SolidTag, P>, SweepError> {
    let (mut model, face) = face.into_model();
    let FaceSweep { solid, .. } = add_swept_face(&mut model, face, spine, options)?;
    Ok(Shape::new(model, solid))
}
