use nalgebra::Vector3;

use crate::builders::solids::add_extruded_face;
use crate::builders::{errors::ExtrudeError, sheets::add_extruded_profile};
use crate::topology::gmap::MergeTopology;
use crate::topology::payload::Payload;
use crate::topology::profile::Profile;
use crate::topology::shape::{FaceTag, Shape, SheetTag, SolidTag};

pub fn extrude_profile<P: Payload>(
    profile: Profile<'_, P>,
    direction: Vector3<f64>,
) -> Result<Shape<SheetTag, P>, ExtrudeError> {
    let (mut g, profile_dart) = profile.isolate();
    let sheet_dart = add_extruded_profile(&mut g, profile_dart, direction)?;
    Ok(Shape::new(g, sheet_dart))
}

pub fn extrude_face<P: Payload>(
    face: Shape<FaceTag, P>,
    direction: Vector3<f64>,
) -> Result<Shape<SolidTag, P>, ExtrudeError> {
    let (mut g, face_key) = face.into_map();
    let solid_key = add_extruded_face(&mut g, face_key, direction)?;
    Ok(Shape::new(g, solid_key))
}
