use radians::Rad64;

use crate::builders::revolve::{RevolveError, add_revolved_face, add_revolved_profile};
use crate::geometry::axis::Axis3;
use crate::model::MergeTopology;
use crate::topology::payload::Payload;
use crate::topology::profile::Profile;
use crate::topology::shape::{FaceTag, Shape, SheetTag, SolidTag};

pub fn revolve_profile<P: Payload>(
    profile: Profile<'_, P>,
    axis: Axis3,
    angle: Rad64,
) -> Result<Shape<SheetTag, P>, RevolveError> {
    let (mut g, profile_dart) = profile.isolate();
    let profile_key = g.profile_key_unchecked(profile_dart);
    let sheet = add_revolved_profile(&mut g, profile_key, axis, angle)?;
    Ok(Shape::new(g, sheet.sheet))
}

pub fn revolve_face<P: Payload>(
    face: Shape<FaceTag, P>,
    axis: Axis3,
    angle: Rad64,
) -> Result<Shape<SolidTag, P>, RevolveError> {
    let (mut g, face_key) = face.into_model();
    let solid = add_revolved_face(&mut g, face_key, axis, angle)?;
    Ok(Shape::new(g, solid.solid))
}
