use crate::builders::edges::_add_circle as add_circle_edge;
use crate::builders::errors::FaceCreationError;
use crate::builders::profiles::profile_pcurves;
use crate::geometry::{Plane, Surface};
use crate::model::Model;
use crate::topology::ModelEdit;
use crate::topology::attributes::{FaceAttr, ProfileAttr};
use crate::topology::payload::Payload;
use crate::topology::profile::Profile;
use crate::topology::shape_keys::FaceKey;

pub fn add_circle<P: Payload>(
    g: &mut Model<P>,
    plane: Plane,
    radius: f64,
) -> Result<FaceKey, FaceCreationError> {
    g.transaction(|edit| _add_circle(edit, plane, radius))
}

/// Builds a circular boundary and its face within one staged operation.
pub(crate) fn _add_circle<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    plane: Plane,
    radius: f64,
) -> Result<FaceKey, FaceCreationError> {
    let edge = add_circle_edge(edit, plane.clone(), radius)?;
    let loop_dart = edit.edge_attr_unchecked(edge).dart;
    edit.add_profile(ProfileAttr::new(loop_dart));
    let profile =
        Profile::from_dart(edit, loop_dart).expect("face loop must have a registered profile");
    let pcurves = profile_pcurves(&profile, &plane)?;
    let face_key = edit.add_face(FaceAttr::with_pcurves(
        Surface::Plane(plane),
        loop_dart,
        Vec::new(),
        pcurves,
    ));
    Ok(face_key)
}
