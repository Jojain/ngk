use radians::Rad64;

use crate::builders::revolve::{RevolveError, add_revolved_face, add_revolved_profile_from_dart};
use crate::geometry::axis::Axis3;
use crate::topology::gmap::MergeTopology;
use crate::topology::payload::Payload;
use crate::topology::profile::Profile;
use crate::topology::shape::{FaceTag, Shape, SheetTag, SolidTag};

pub fn revolve_profile<P: Payload>(
    profile: Profile<'_, P>,
    axis: Axis3,
    angle: Rad64,
) -> Result<Shape<SheetTag, P>, RevolveError> {
    let (mut g, profile_dart) = profile.isolate();
    let sheet_dart = add_revolved_profile_from_dart(&mut g, profile_dart, axis, angle)?;
    Ok(Shape::new(g, sheet_dart))
}

pub fn revolve_face<P: Payload>(
    face: Shape<FaceTag, P>,
    axis: Axis3,
    angle: Rad64,
) -> Result<Shape<SolidTag, P>, RevolveError> {
    let (mut g, face_key) = face.into_map();
    let solid_key = add_revolved_face(&mut g, face_key, axis, angle)?;
    Ok(Shape::new(g, solid_key))
}
