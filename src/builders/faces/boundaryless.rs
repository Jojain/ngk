//! Aggregate references retained when a boundary replaces a closed support's scaffold.

use std::collections::HashSet;

use crate::topology::gmap::{Dart, Dim};
use crate::topology::orientation::Orientation;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{FaceKey, SheetKey, SolidKey};
use crate::topology::{IsolatedDart, ModelEdit, ModelEditError};

/// The sheet and solid references to a boundaryless face's original raw cell.
pub(super) struct BoundarylessReplacement {
    face: FaceKey,
    darts: HashSet<Dart>,
    sheets: Vec<(SheetKey, Orientation)>,
    shells: Vec<(SolidKey, Dart, Orientation)>,
}

impl BoundarylessReplacement {
    /// Reads aggregate references before the face's scaffold is replaced.
    pub(super) fn capture<P: Payload>(edit: &ModelEdit<'_, P>, face: FaceKey) -> Option<Self> {
        let attr = edit.face_attr(face)?;
        if !attr.is_boundaryless() {
            return None;
        }
        let view = edit.face_unchecked(face);
        let darts: HashSet<_> = view.region_darts().into_iter().collect();
        let sense = |dart| {
            edit.cell_orientation_from_seed(view.dart(), dart, Dim::Two)
                .expect("a reference inside the source cell has an orientation")
        };
        let sheets = edit
            .iter_sheets()
            .filter(|(_, attr)| darts.contains(&attr.root))
            .map(|(key, attr)| (key, sense(attr.root)))
            .collect();
        let shells = edit
            .iter_solids()
            .flat_map(|(key, attr)| {
                attr.shells()
                    .filter(|dart| darts.contains(dart))
                    .map(|dart| (key, dart, sense(dart)))
                    .collect::<Vec<_>>()
            })
            .collect();
        Some(Self {
            face,
            darts,
            sheets,
            shells,
        })
    }

    /// Moves aggregate references to the new boundary and discards detached scaffold.
    pub(super) fn apply<P: Payload>(
        self,
        edit: &mut ModelEdit<'_, P>,
    ) -> Result<(), ModelEditError> {
        let anchor = edit.face_unchecked(self.face).dart();
        if self.darts.contains(&anchor) {
            return Ok(());
        }
        let replacement = |sense| match sense {
            Orientation::Same => anchor,
            Orientation::Reversed => edit.alpha(Dim::Zero, anchor),
        };
        let sheets = self
            .sheets
            .iter()
            .map(|&(key, sense)| (key, replacement(sense)))
            .collect::<Vec<_>>();
        let shells = self
            .shells
            .iter()
            .map(|&(key, old, sense)| (key, old, replacement(sense)))
            .collect::<Vec<_>>();
        for (key, root) in sheets {
            edit.sheet_attr_mut_unchecked(key).root = root;
        }
        for (key, old, root) in shells {
            edit.solid_attr_mut_unchecked(key)
                .map_shell_darts(|dart| if dart == old { root } else { dart });
        }

        // A detached polygon carries only the source face's old scaffold.
        // A polygon attached to another shell retains that attachment.
        let detached = self.darts.iter().all(|&dart| {
            [Dim::Zero, Dim::One, Dim::Two, Dim::Three]
                .into_iter()
                .all(|dim| self.darts.contains(&edit.alpha(dim, dart)))
        });
        if detached {
            let mut darts = self.darts.into_iter().collect::<Vec<_>>();
            darts.sort_by_key(|dart| dart.id());
            edit.deferred(|raw| {
                for &dart in &darts {
                    for dim in [Dim::Zero, Dim::One, Dim::Two, Dim::Three] {
                        if !raw.topology().is_free(dart, dim) {
                            raw.unlink(dim, dart)?;
                        }
                    }
                }
                raw.remove_isolated_darts(darts.into_iter().map(IsolatedDart::new).collect());
                Ok::<_, ModelEditError>(())
            })?;
        }
        Ok(())
    }
}
