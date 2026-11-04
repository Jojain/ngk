//! Offset builders: an edge, a profile, a face, a sheet or a solid, copied and
//! moved a signed distance.
//!
//! **Every offset is a new entity beside its source.** The source is copied
//! into the same model and the copy is re-embedded, so the source keeps its
//! key and its geometry, and an operation that needs both (a thickened sheet,
//! a shelled solid) has both. The copy shares nothing with what surrounded the
//! source: a face offset from a solid is a free face.
//!
//! - An edge or a profile moves along `T × direction` (see
//!   [`Curve::offset`](crate::geometry::Curve::offset)), and a profile's
//!   corners go where its moved edges meet.
//! - A face, sheet or solid moves each face along its own normal: a positive
//!   distance grows a solid. Edges between faces meeting at an angle become
//!   the meeting of the moved faces.

use nalgebra::UnitVector3;

use super::errors::OffsetBuildError;
use super::reembed::{face_sign, plan_faces, plan_wire};
use crate::model::{Cell1, Cell2, Model, OpResult, StaleResult};
use crate::topology::ModelEdit;
use crate::topology::edge::Edge;
use crate::topology::face::Face;
use crate::topology::payload::Payload;
use crate::topology::profile::Profile;
use crate::topology::shape_keys::{EdgeKey, FaceKey, ProfileKey, SheetKey, SolidKey};
use crate::topology::sheet::Sheet;
use crate::topology::solid::Solid;

macro_rules! offset_result {
    ($(#[$doc:meta])* $name:ident, $field:ident: $key:ty => $view:ident, $lookup:ident) => {
        $(#[$doc])*
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub struct $name {
            /// The moved copy.
            pub $field: $key,
            revision: Option<u64>,
        }

        impl OpResult for $name {
            fn stamp(&mut self, revision: u64) {
                self.revision = Some(revision);
            }
        }

        impl $name {
            fn new($field: $key) -> Self {
                Self {
                    $field,
                    revision: None,
                }
            }

            /// Resolves the moved copy against the model at the result's
            /// committed revision.
            pub fn view<'m, P: Payload>(
                &self,
                model: &'m Model<P>,
            ) -> Result<$view<'m, P>, StaleResult> {
                StaleResult::check(self.revision, model)?;
                Ok(model.$lookup(self.$field))
            }
        }
    };
}

offset_result!(
    /// An edge was offset into a new free edge.
    EdgeOffset, edge: EdgeKey => Edge, edge_unchecked
);
offset_result!(
    /// A profile was offset into a new free profile.
    ProfileOffset, profile: ProfileKey => Profile, profile_unchecked
);
offset_result!(
    /// A face was offset into a new free face.
    FaceOffset, face: FaceKey => Face, face_unchecked
);
offset_result!(
    /// A sheet was offset into a new sheet.
    SheetOffset, sheet: SheetKey => Sheet, sheet_unchecked
);
offset_result!(
    /// A solid was offset into a new solid.
    SolidOffset, solid: SolidKey => Solid, solid_unchecked
);

fn check_distance(distance: f64) -> Result<(), OffsetBuildError> {
    if distance.is_finite() {
        Ok(())
    } else {
        Err(OffsetBuildError::NonFiniteDistance { distance })
    }
}

/// Adds a copy of `edge` moved `distance` along `T × direction`.
pub fn offset_edge<P: Payload>(
    model: &mut Model<P>,
    edge: EdgeKey,
    direction: UnitVector3<f64>,
    distance: f64,
) -> Result<EdgeOffset, OffsetBuildError> {
    model.transaction_result(|edit| offset_edge_edit(edit, edge, direction, distance))
}

pub(crate) fn offset_edge_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    edge: EdgeKey,
    direction: UnitVector3<f64>,
    distance: f64,
) -> Result<EdgeOffset, OffsetBuildError> {
    check_distance(distance)?;
    let source = edit.model().clone();
    let view = source
        .edge(edge)
        .ok_or(OffsetBuildError::MissingEdge { edge })?;
    let dart = edit.merge(view);
    let copy = edit.model().cell_key_unchecked::<Cell1>(dart);
    plan_wire(edit.model(), &[copy], direction, distance)?.apply(edit);
    Ok(EdgeOffset::new(copy))
}

/// Adds a copy of `profile` with every edge moved `distance` along
/// `T × direction` and every corner where the moved edges meet.
///
/// For a profile in a plane normal to `direction`, a positive distance moves a
/// loop running counter-clockwise about `direction` outward.
pub fn offset_profile<P: Payload>(
    model: &mut Model<P>,
    profile: ProfileKey,
    direction: UnitVector3<f64>,
    distance: f64,
) -> Result<ProfileOffset, OffsetBuildError> {
    model.transaction_result(|edit| offset_profile_edit(edit, profile, direction, distance))
}

pub(crate) fn offset_profile_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    profile: ProfileKey,
    direction: UnitVector3<f64>,
    distance: f64,
) -> Result<ProfileOffset, OffsetBuildError> {
    check_distance(distance)?;
    let source = edit.model().clone();
    let view = source
        .profile(profile)
        .ok_or(OffsetBuildError::MissingProfile { profile })?;
    let dart = edit.merge(view);
    let copy = edit.model().profile_key_unchecked(dart);
    let edges = unique(
        edit.model()
            .profile_unchecked(copy)
            .edges()
            .iter()
            .map(|edge| edge.key()),
    );
    plan_wire(edit.model(), &edges, direction, distance)?.apply(edit);
    Ok(ProfileOffset::new(copy))
}

/// Adds a copy of `face` moved `distance` along its own normal.
pub fn offset_face<P: Payload>(
    model: &mut Model<P>,
    face: FaceKey,
    distance: f64,
) -> Result<FaceOffset, OffsetBuildError> {
    model.transaction_result(|edit| offset_face_edit(edit, face, distance))
}

pub(crate) fn offset_face_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    face: FaceKey,
    distance: f64,
) -> Result<FaceOffset, OffsetBuildError> {
    check_distance(distance)?;
    let source = edit.model().clone();
    let view = source
        .face(face)
        .ok_or(OffsetBuildError::MissingFace { face })?;
    let dart = edit.merge(view);
    let copy = edit.model().cell_key_unchecked::<Cell2>(dart);
    offset_faces(edit, &[copy], distance)?;
    Ok(FaceOffset::new(copy))
}

/// Adds a copy of `sheet` with every face moved `distance` along its own
/// normal.
pub fn offset_sheet<P: Payload>(
    model: &mut Model<P>,
    sheet: SheetKey,
    distance: f64,
) -> Result<SheetOffset, OffsetBuildError> {
    model.transaction_result(|edit| offset_sheet_edit(edit, sheet, distance))
}

pub(crate) fn offset_sheet_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    sheet: SheetKey,
    distance: f64,
) -> Result<SheetOffset, OffsetBuildError> {
    check_distance(distance)?;
    let source = edit.model().clone();
    let view = source
        .sheet(sheet)
        .ok_or(OffsetBuildError::MissingSheet { sheet })?;
    let dart = edit.merge(view);
    let copy = edit.model().sheet_key_unchecked(dart);
    let faces = unique(
        edit.model()
            .sheet_unchecked(copy)
            .faces()
            .iter()
            .map(|face| face.key()),
    );
    offset_faces(edit, &faces, distance)?;
    Ok(SheetOffset::new(copy))
}

/// Adds a copy of `solid` with every face moved `distance` along its outward
/// normal: a positive distance grows it, a negative one shrinks it.
pub fn offset_solid<P: Payload>(
    model: &mut Model<P>,
    solid: SolidKey,
    distance: f64,
) -> Result<SolidOffset, OffsetBuildError> {
    model.transaction_result(|edit| offset_solid_edit(edit, solid, distance))
}

pub(crate) fn offset_solid_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    solid: SolidKey,
    distance: f64,
) -> Result<SolidOffset, OffsetBuildError> {
    check_distance(distance)?;
    let copy = copy_solid(edit, solid)?;
    let faces = unique(
        edit.model()
            .solid_unchecked(copy)
            .faces()
            .iter()
            .map(|face| face.key()),
    );
    offset_faces(edit, &faces, distance)?;
    Ok(SolidOffset::new(copy))
}

/// Merges a copy of `solid` into the model and returns the copy's key.
pub(super) fn copy_solid<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    solid: SolidKey,
) -> Result<SolidKey, OffsetBuildError> {
    let source = edit.model().clone();
    let view = source
        .solid(solid)
        .ok_or(OffsetBuildError::MissingSolid { solid })?;
    let dart = edit.merge(view);
    Ok(edit
        .model()
        .solid_key(dart)
        .expect("a merged solid keeps its registration"))
}

/// Moves a closed set of faces, each along its own normal.
fn offset_faces<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    faces: &[FaceKey],
    distance: f64,
) -> Result<(), OffsetBuildError> {
    let signed = faces
        .iter()
        .map(|&face| (face, face_sign(edit.model(), face), distance))
        .collect::<Vec<_>>();
    plan_faces(edit.model(), &signed)?.apply(edit);
    Ok(())
}

fn unique<T: Copy + Ord>(values: impl IntoIterator<Item = T>) -> Vec<T> {
    let mut values = values.into_iter().collect::<Vec<_>>();
    values.sort();
    values.dedup();
    values
}
