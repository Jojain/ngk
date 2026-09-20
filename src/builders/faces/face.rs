use crate::builders::errors::FaceCreationError;
use crate::builders::profiles::{_add_rectangle as add_rectangle_profile, profile_pcurves};
use crate::geometry::{Plane, Surface};
use crate::model::Model;
use crate::topology::ModelEdit;
use crate::topology::attributes::FaceAttr;
use crate::topology::closed::Closed;
use crate::topology::payload::Payload;
use crate::topology::planar::Planar;
use crate::topology::shape_keys::{FaceKey, ProfileKey};

pub fn add_face<P: Payload>(
    g: &mut Model<P>,
    profile: ProfileKey,
) -> Result<FaceKey, FaceCreationError> {
    g.transaction(|edit| _add_face(edit, profile))
}

pub(crate) fn _add_face<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    profile: ProfileKey,
) -> Result<FaceKey, FaceCreationError> {
    let (loop_dart, plane, pcurves) = {
        let profile = edit.profile_unchecked(profile);
        let loop_dart = profile.dart;
        let closed =
            Closed::new(profile).ok_or(FaceCreationError::OpenProfile { dart: loop_dart })?;
        let planar = Planar::new(closed)?;
        let (closed, plane) = planar.into_parts();
        let pcurves = profile_pcurves(closed.inner(), &plane)?;
        (loop_dart, plane, pcurves)
    };

    Ok(edit.add_face(FaceAttr::with_pcurves(
        Surface::Plane(plane),
        loop_dart,
        Vec::new(),
        pcurves,
    )))
}

/// Adds a planar rectangular face whose first corner is `plane.origin()`.
///
/// The sides follow the plane's positive x and y directions and have lengths
/// `x_size` and `y_size`. Both sizes must be positive and finite.
pub fn add_rectangle<P: Payload>(
    g: &mut Model<P>,
    plane: Plane,
    x_size: f64,
    y_size: f64,
) -> Result<FaceKey, FaceCreationError> {
    g.transaction(|edit| _add_rectangle(edit, plane, x_size, y_size))
}

/// Builds a rectangular profile and fills it inside an existing edit.
pub(crate) fn _add_rectangle<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    plane: Plane,
    x_size: f64,
    y_size: f64,
) -> Result<FaceKey, FaceCreationError> {
    let profile = add_rectangle_profile(edit, plane, x_size, y_size)?;
    _add_face(edit, profile)
}

/// Adds a planar square face whose first corner is `plane.origin()`.
///
/// The sides follow the plane's positive x and y directions. `size` must be
/// positive and finite.
pub fn add_square<P: Payload>(
    g: &mut Model<P>,
    plane: Plane,
    size: f64,
) -> Result<FaceKey, FaceCreationError> {
    g.transaction(|edit| _add_square(edit, plane, size))
}

/// Builds a square profile and fills it inside an existing edit.
pub(crate) fn _add_square<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    plane: Plane,
    size: f64,
) -> Result<FaceKey, FaceCreationError> {
    let profile = add_rectangle_profile(edit, plane, size, size)?;
    _add_face(edit, profile)
}
