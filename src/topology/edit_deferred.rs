//! Raw surgery on the map and its labelling, checked when it ends.
//!
//! Some rewirings cannot be written as a sequence of operators, because every
//! intermediate state of them is one no operator would leave: a face is
//! detached from its neighbours, its darts are relinked into two loops, and
//! only then does the labelling describe the map again. [`ModelEdit::deferred`]
//! runs such a sequence against a [`RawEdit`], the edit with its views taken
//! away. A view needs the embedding index, and the index needs the labelling to
//! describe the map, which is exactly what the surgery is allowed to break; so
//! a `RawEdit` offers the primitives and the stored attributes and nothing that
//! walks the map through the labelling. When the block returns, the map's
//! axioms and the labelling are checked, like a database checks its deferred
//! constraints at the end of the statement, and a block that left either broken
//! is an error that names the block rather than a later read.

use super::attributes::{EdgeAttr, FaceAttr, ProfileAttr, SheetAttr, SolidAttr, VertexAttr};
use super::edit::{ModelEdit, ModelEditError};
use super::embedding::{Embedding, EntityOwner};
use super::gmap::{Dart, Dim, GMap};
use super::payload::Payload;
use super::shape_keys::{EdgeKey, FaceKey, ProfileKey, SheetKey, SolidKey, VertexKey};
use super::validation::validate_gmap;

/// The edit while a deferred block runs: map and labelling primitives, and the
/// stored attributes, with no view.
///
/// A `RawEdit` does not dereference to the model, so a view cannot be reached
/// from it:
///
/// ```compile_fail,E0599
/// use ngk::topology::{RawEdit, StandardPayload};
///
/// fn read(raw: &mut RawEdit<'_, '_, StandardPayload>) {
///     let _ = raw.iter_faces();
/// }
/// ```
pub struct RawEdit<'a, 'g, P: Payload> {
    edit: &'a mut ModelEdit<'g, P>,
}

impl<P: Payload> ModelEdit<'_, P> {
    /// Runs raw surgery that may pass through states no operator would leave,
    /// and checks the map and its labelling when it ends.
    ///
    /// Inside the block `raw` offers the primitives (`add_dart`, `link`,
    /// `unlink`, `sew`, `own_cell`, ...) and the stored attributes, but no view:
    /// nothing in it can read the model through a labelling that is allowed to
    /// be wrong. An error from the block, or from the closing check, leaves the
    /// edit as it is and is returned to the caller, whose transaction rolls it
    /// back.
    pub fn deferred<T, E>(
        &mut self,
        surgery: impl FnOnce(&mut RawEdit<'_, '_, P>) -> Result<T, E>,
    ) -> Result<T, E>
    where
        E: From<ModelEditError>,
    {
        let value = surgery(&mut RawEdit { edit: self })?;
        validate_gmap(self.model.topology())
            .map_err(|error| E::from(ModelEditError::InvalidTopology(error)))?;
        self.check_consistent()?;
        Ok(value)
    }
}

impl<P: Payload> RawEdit<'_, '_, P> {
    /// Adds an isolated dart.
    pub fn add_dart(&mut self) -> Dart {
        self.edit.add_dart()
    }

    /// Links two free darts through one involution.
    pub fn link(&mut self, dim: Dim, first: Dart, second: Dart) -> Result<(), ModelEditError> {
        self.edit.link(dim, first, second)
    }

    /// Unlinks the pair containing `dart`, returning its partner.
    pub fn unlink(&mut self, dim: Dim, dart: Dart) -> Result<Dart, ModelEditError> {
        self.edit.unlink(dim, dart)
    }

    /// Sews two cells of dimension `dim`, linking every dart pair of their
    /// orbits.
    pub fn sew(&mut self, dim: Dim, first: Dart, second: Dart) -> Result<(), ModelEditError> {
        self.edit.sew(dim, first, second)
    }

    /// Labels the raw `dimension`-cell containing `dart` as interior to `owner`.
    pub fn own_cell(&mut self, dimension: Dim, dart: Dart, owner: EntityOwner) {
        self.edit.own_cell(dimension, dart, owner);
    }

    /// Unlabels the raw `dimension`-cell containing `dart`.
    pub fn disown_cell(&mut self, dimension: Dim, dart: Dart) {
        self.edit.disown_cell(dimension, dart);
    }

    /// The map as it stands, mid-surgery.
    pub fn topology(&self) -> &GMap {
        self.edit.model().topology()
    }

    /// The stored labels as they stand, mid-surgery. They are records, not the
    /// index derived from them: nothing here claims they describe the map.
    pub fn embedding(&self) -> &Embedding {
        self.edit.model().embedding()
    }

    /// The vertex attribute stored under `key`.
    pub fn vertex_attr(&self, key: VertexKey) -> Option<&VertexAttr<P::V>> {
        self.edit.model().vertex_attr(key)
    }

    /// The edge attribute stored under `key`.
    pub fn edge_attr(&self, key: EdgeKey) -> Option<&EdgeAttr<P::E>> {
        self.edit.model().edge_attr(key)
    }

    /// The face attribute stored under `key`.
    pub fn face_attr(&self, key: FaceKey) -> Option<&FaceAttr<P::F>> {
        self.edit.model().face_attr(key)
    }

    /// The profile attribute stored under `key`.
    pub fn profile_attr(&self, key: ProfileKey) -> Option<&ProfileAttr<P::Profile>> {
        self.edit.model().profile_attr(key)
    }

    /// The sheet attribute stored under `key`.
    pub fn sheet_attr(&self, key: SheetKey) -> Option<&SheetAttr<P::Sheet>> {
        self.edit.model().sheet_attr(key)
    }

    /// The solid attribute stored under `key`.
    pub fn solid_attr(&self, key: SolidKey) -> Option<&SolidAttr<P::S>> {
        self.edit.model().solid_attr(key)
    }

    /// The vertex attribute stored under `key`, for change.
    pub fn vertex_attr_mut(&mut self, key: VertexKey) -> Option<&mut VertexAttr<P::V>> {
        self.edit.vertex_attr_mut(key)
    }

    /// The edge attribute stored under `key`, for change.
    pub fn edge_attr_mut(&mut self, key: EdgeKey) -> Option<&mut EdgeAttr<P::E>> {
        self.edit.edge_attr_mut(key)
    }

    /// The face attribute stored under `key`, for change.
    pub fn face_attr_mut(&mut self, key: FaceKey) -> Option<&mut FaceAttr<P::F>> {
        self.edit.face_attr_mut(key)
    }
}
