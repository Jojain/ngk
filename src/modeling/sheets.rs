//! Sheet constructors.

use crate::builders::errors::SheetFromFacesError;
use crate::builders::sheets::add_sheet_from_faces_edit;
use crate::model::{Cell2, Model};
use crate::topology::payload::Payload;
use crate::topology::shape::{FaceTag, Shape, SheetTag};

/// Builds an owned sheet from face shapes in any input order.
///
/// The returned shape owns copies of the input faces. The source shapes remain
/// unchanged. The faces must join into one connected, consistently wound
/// sheet along edges they share whole; see
/// [`crate::builders::sheets::add_sheet_from_faces`].
pub fn from_faces<P: Payload>(
    faces: &[&Shape<FaceTag, P>],
) -> Result<Shape<SheetTag, P>, SheetFromFacesError> {
    let mut g = Model::new();
    let sheet = g.transaction(|edit| {
        let keys = faces
            .iter()
            .map(|face| {
                let dart = edit.merge(face.face());
                edit.cell_key_unchecked::<Cell2>(dart)
            })
            .collect::<Vec<_>>();
        add_sheet_from_faces_edit(edit, &keys)
    })?;
    Ok(Shape::new(g, sheet))
}
