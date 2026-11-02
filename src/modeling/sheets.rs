//! Sheet constructors.

use crate::builders::errors::SheetFromFacesError;
use crate::builders::sheets::add_sheet_from_faces_edit;
use crate::model::{Cell2, Model};
use crate::topology::ModelEdit;
use crate::topology::payload::Payload;
use crate::topology::shape::{FaceTag, Shape, SheetTag};
use crate::topology::shape_keys::SheetKey;
use crate::topology::sheet::Sheet;

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

/// Copies one sheet, with the entities that bound it and nothing else of its
/// model, and returns its key in the model being edited.
///
/// The copy faces the way the view does: it is anchored at the view's own
/// dart, which carries its direction, rather than at the stored root a
/// reversed view runs against.
pub(crate) fn copy_sheet<P: Payload>(edit: &mut ModelEdit<'_, P>, sheet: Sheet<'_, P>) -> SheetKey {
    let dart = edit.merge(sheet);
    let key = edit
        .sheet_key(dart)
        .expect("a copied sheet keeps its registration");
    edit.sheet_attr_mut_unchecked(key).root = dart;
    key
}
