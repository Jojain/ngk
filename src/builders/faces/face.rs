use crate::builders::errors::FaceCreationError;
use crate::builders::profiles::{add_rectangle_edit as add_rectangle_profile, profile_pcurves};
use crate::geometry::{Plane, Surface};
use crate::model::Model;
use crate::topology::ModelEdit;
use crate::topology::attributes::FaceAttr;
use crate::topology::closed::Closed;
use crate::topology::edit::EditKey;
use crate::topology::payload::Payload;
use crate::topology::planar::Planar;
use crate::topology::shape_keys::{FaceKey, ProfileKey};

pub fn add_face<P: Payload>(
    g: &mut Model<P>,
    profile: ProfileKey,
) -> Result<FaceKey, FaceCreationError> {
    g.transaction(|edit| add_face_edit(edit, profile))
}

pub(crate) fn add_face_edit<P: Payload>(
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

    Ok(edit.add_face_derived_from(
        vec![EditKey::Profile(profile)],
        FaceAttr::with_pcurves(Surface::Plane(plane), loop_dart, Vec::new(), pcurves),
    ))
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
    g.transaction(|edit| add_rectangle_edit(edit, plane, x_size, y_size))
}

/// Builds a rectangular profile and fills it inside an existing edit.
pub(crate) fn add_rectangle_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    plane: Plane,
    x_size: f64,
    y_size: f64,
) -> Result<FaceKey, FaceCreationError> {
    let profile = add_rectangle_profile(edit, plane, x_size, y_size)?;
    add_face_edit(edit, profile)
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
    g.transaction(|edit| add_square_edit(edit, plane, size))
}

/// Builds a square profile and fills it inside an existing edit.
pub(crate) fn add_square_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    plane: Plane,
    size: f64,
) -> Result<FaceKey, FaceCreationError> {
    let profile = add_rectangle_profile(edit, plane, size, size)?;
    add_face_edit(edit, profile)
}

#[cfg(test)]
mod tests {
    use super::add_face_edit;
    use crate::builders::profiles::add_rectangle;
    use crate::builders::test_support::LineageRecorder;
    use crate::geometry::Plane;
    use crate::model::Model;
    use crate::topology::edit::{EditKey, Origin};
    use crate::topology::payload::StandardPayload;

    #[test]
    fn face_creation_declares_its_profile_lineage() {
        let mut model = Model::<StandardPayload>::new();
        let profile =
            add_rectangle(&mut model, Plane::xy(), 2.0, 2.0).expect("source profile should build");
        let mut recorder = LineageRecorder::default();

        let face = model
            .transaction_with_policy(&mut recorder, |edit| add_face_edit(edit, profile))
            .expect("face should build");

        recorder.assert_exact(&[(
            EditKey::Face(face),
            Origin::derived(EditKey::Profile(profile)),
        )]);
    }
}
