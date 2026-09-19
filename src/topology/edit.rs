use std::collections::{HashMap, HashSet};
use std::convert::Infallible;
use std::error::Error;
use std::hash::Hash;
use std::ops::Deref;

use thiserror::Error;

use super::Dart;
use super::attributes::{EdgeAttr, FaceAttr, ProfileAttr, SheetAttr, SolidAttr, VertexAttr};
use super::embedding::{EmbeddingError, EntityOwner};
use super::gmap::Dim;
use super::payload::Payload;
use super::shape_keys::{EdgeKey, FaceKey, ProfileKey, SheetKey, SolidKey, VertexKey};
use super::validation::{
    CellOccupancyError, GMapValidationError, validate_cell_occupancy, validate_gmap,
};
use crate::model::{MergeTopology, Model};

/// Why a transaction created an entity.
///
/// Exhaustive over the ways a builder may explain a creation: an entity that
/// arrived by copy is not among them, since [`Model::merge`] transports
/// payload verbatim and never reaches [`EditPolicy`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Origin {
    /// Nothing in the model before this transaction explains it.
    New,
    /// It carries on from an entity of its own kind.
    Split(EditKey),
    /// It was produced by entities of another kind, in an order the builder
    /// documents — a loft names its sections as it traverses them.
    Derived(Vec<EditKey>),
}

impl Origin {
    /// Builds a [`Origin::Derived`] from a single source.
    ///
    /// The variant always carries a `Vec` because a derived entity may have
    /// several sources; this is the one-source spelling over it, not a second
    /// variant.
    pub fn derived(source: EditKey) -> Self {
        Self::Derived(vec![source])
    }
}

/// Controls how payloads are propagated for explicit semantic edit events.
///
/// The edit layer does not infer merge or split lineage from topology. Builders
/// must declare semantic events through methods such as
/// [`ModelEdit::add_edge_split_from`] and
/// [`ModelEdit::merge_edges_into`].
pub trait EditPolicy<P: Payload> {
    /// Error returned when the policy rejects an edit.
    type Error: Error + Send + Sync + 'static;

    /// Initializes a vertex payload created by splitting an existing vertex.
    fn split_vertex_data(
        &mut self,
        _source: VertexKey,
        source_data: &P::V,
        _created: VertexKey,
        created_data: &mut P::V,
    ) -> Result<(), Self::Error> {
        *created_data = source_data.clone();
        Ok(())
    }

    /// Initializes an edge payload created by splitting an existing edge.
    fn split_edge_data(
        &mut self,
        _source: EdgeKey,
        source_data: &P::E,
        _created: EdgeKey,
        created_data: &mut P::E,
    ) -> Result<(), Self::Error> {
        *created_data = source_data.clone();
        Ok(())
    }

    /// Initializes a profile payload created by splitting an existing profile.
    fn split_profile_data(
        &mut self,
        _source: ProfileKey,
        source_data: &P::Profile,
        _created: ProfileKey,
        created_data: &mut P::Profile,
    ) -> Result<(), Self::Error> {
        *created_data = source_data.clone();
        Ok(())
    }

    /// Initializes a face payload created by splitting an existing face.
    fn split_face_data(
        &mut self,
        _source: FaceKey,
        source_data: &P::F,
        _created: FaceKey,
        created_data: &mut P::F,
    ) -> Result<(), Self::Error> {
        *created_data = source_data.clone();
        Ok(())
    }

    /// Initializes a sheet payload created by splitting an existing sheet.
    fn split_sheet_data(
        &mut self,
        _source: SheetKey,
        source_data: &P::Sheet,
        _created: SheetKey,
        created_data: &mut P::Sheet,
    ) -> Result<(), Self::Error> {
        *created_data = source_data.clone();
        Ok(())
    }

    /// Initializes a solid payload created by splitting an existing solid.
    fn split_solid_data(
        &mut self,
        _source: SolidKey,
        source_data: &P::S,
        _created: SolidKey,
        created_data: &mut P::S,
    ) -> Result<(), Self::Error> {
        *created_data = source_data.clone();
        Ok(())
    }

    /// Merges a removed vertex payload into the surviving vertex payload.
    fn merge_vertex_data(
        &mut self,
        _survivor: VertexKey,
        _survivor_data: &mut P::V,
        _removed: VertexKey,
        _removed_data: P::V,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    /// Merges a removed edge payload into the surviving edge payload.
    fn merge_edge_data(
        &mut self,
        _survivor: EdgeKey,
        _survivor_data: &mut P::E,
        _removed: EdgeKey,
        _removed_data: P::E,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    /// Merges a removed profile payload into the surviving profile payload.
    fn merge_profile_data(
        &mut self,
        _survivor: ProfileKey,
        _survivor_data: &mut P::Profile,
        _removed: ProfileKey,
        _removed_data: P::Profile,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    /// Merges a removed face payload into the surviving face payload.
    fn merge_face_data(
        &mut self,
        _survivor: FaceKey,
        _survivor_data: &mut P::F,
        _removed: FaceKey,
        _removed_data: P::F,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    /// Merges a removed sheet payload into the surviving sheet payload.
    fn merge_sheet_data(
        &mut self,
        _survivor: SheetKey,
        _survivor_data: &mut P::Sheet,
        _removed: SheetKey,
        _removed_data: P::Sheet,
    ) -> Result<(), Self::Error> {
        Ok(())
    }

    /// Merges a removed solid payload into the surviving solid payload.
    fn merge_solid_data(
        &mut self,
        _survivor: SolidKey,
        _survivor_data: &mut P::S,
        _removed: SolidKey,
        _removed_data: P::S,
    ) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// Default edit policy.
#[derive(Debug, Clone, Copy, Default)]
pub struct PreservePayload;

impl<P: Payload> EditPolicy<P> for PreservePayload {
    type Error = Infallible;
}

/// Failure raised while applying a safe model mutation.
#[derive(Debug, Error)]
pub enum ModelEditError {
    #[error("cannot delete dart {dart:?} while it is a registered sheet or solid root")]
    ReferencedDartDeletion { dart: Dart },
    /// A split or merge references an attribute that is not staged.
    #[error("model edit lineage references missing attribute {key:?}")]
    MissingLineageAttribute { key: EditKey },
    /// A transaction-start attribute is gone at commit and no event explains it.
    #[error("{key:?} was removed without a merge or consumption declaring it")]
    UnexplainedRemoval { key: EditKey },
    /// The same attribute was declared consumed more than once.
    #[error("model edit lineage consumes {removed:?} more than once")]
    RepeatedMerge { removed: EditKey },
    /// A merge cannot consume its own survivor.
    #[error("model edit lineage cannot merge {removed:?} into itself")]
    InvalidMerge { survivor: EditKey, removed: EditKey },
    /// Explicit merge declarations contain a cycle.
    #[error("model edit lineage contains a merge cycle through {key:?}")]
    MergeCycle { key: EditKey },
    /// Several transaction-start identities still describe one final cell.
    #[error(
        "{entity} cell {representative:?} retains multiple pre-existing identities {candidates:?}"
    )]
    UnresolvedPreExistingCollision {
        entity: &'static str,
        representative: Dart,
        candidates: Vec<EditKey>,
    },
    /// Explicit lineage selected an identity discarded during reconciliation.
    #[error("explicit lineage survivor {survivor:?} does not survive reconciliation")]
    InvalidLineageSurvivor { survivor: EditKey },
    /// A dart does not exist in the edited map.
    #[error("dart {dart:?} does not exist")]
    MissingDart { dart: Dart },
    /// A face boundary does not have a registered profile identity.
    #[error("face {face:?} boundary at {dart:?} has no registered profile")]
    MissingProfileRegistration { face: FaceKey, dart: Dart },
    /// A solid shell does not have a registered sheet identity.
    #[error("solid {solid:?} shell at {dart:?} has no registered sheet")]
    MissingSheetRegistration { solid: SolidKey, dart: Dart },
    /// An involution cannot link a dart to itself.
    #[error("cannot link dart {dart:?} to itself")]
    SameDart { dart: Dart },
    /// A requested dart is already linked through the selected involution.
    #[error("dart {dart:?} is not free along {dim:?}")]
    DartNotFree { dart: Dart, dim: Dim },
    /// A requested unlink operation targeted a free dart.
    #[error("dart {dart:?} is already free along {dim:?}")]
    DartAlreadyFree { dart: Dart, dim: Dim },
    /// The two cells do not satisfy the sewing constraints.
    #[error("darts {first:?} and {second:?} are not sewable along {dim:?}")]
    NotSewable { dim: Dim, first: Dart, second: Dart },
    /// The edited alpha relations do not satisfy the gmap axioms.
    #[error("this edit produced an invalid generalized map")]
    InvalidTopology(#[source] GMapValidationError),
    /// The embedding labels no longer describe the edited map.
    #[error("this edit produced a embedding that does not describe its map")]
    InvalidEmbedding(#[source] EmbeddingError),
    /// A logical entity does not occupy exactly one raw cell of its own dimension.
    #[error("this edit broke logical cell occupancy")]
    InvalidCellOccupancy(#[source] CellOccupancyError),
    /// More than one attribute key describes the same domain cell.
    #[error("{entity} attributes contain duplicate keys for representative {representative:?}")]
    DuplicateCellAttribute {
        entity: &'static str,
        representative: Dart,
    },
    /// The payload policy rejected a merge or split.
    #[error("topology edit policy rejected the edit")]
    Policy(#[source] Box<dyn Error + Send + Sync>),
}

/// Transaction-scoped capability for reading and mutating a [`Model`].
///
/// All staged mutations and semantic lineage pass through this capability.
/// Validation, policy application, and rollback are owned by
/// [`Model::transaction`](Model::transaction).
pub struct ModelEdit<'g, P: Payload> {
    model: &'g mut Model<P>,
}

/// Identifies a topology-associated attribute in edit-lineage diagnostics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EditKey {
    /// Vertex attribute key.
    Vertex(VertexKey),
    /// Edge attribute key.
    Edge(EdgeKey),
    /// Profile attribute key.
    Profile(ProfileKey),
    /// Face attribute key.
    Face(FaceKey),
    /// Sheet attribute key.
    Sheet(SheetKey),
    /// Solid attribute key.
    Solid(SolidKey),
}

/// A semantic edit declared against the staged model, uniform over [`EditKey`]
/// so a [`Derived`](Origin::Derived) source can name an entity of another kind.
///
/// The typed `add_*_split_from` / `merge_*_into` constructors on [`ModelEdit`]
/// keep the same-kind check at the call site, where the concrete key types
/// make it free; this representation does not repeat it.
#[derive(Debug, Clone)]
pub(crate) enum EditEvent {
    /// A new identity, explained by `origin`.
    Created { key: EditKey, origin: Origin },
    /// `removed`'s identity carries on inside `survivor`.
    Merged { survivor: EditKey, removed: EditKey },
    /// `key` stopped existing and nothing inherits it.
    Consumed { key: EditKey },
    /// `key` was transported verbatim from another model by [`Model::merge`].
    ///
    /// Never routed to [`EditPolicy`]: a copy is not a creation.
    Copied { key: EditKey },
}

impl<'g, P: Payload> ModelEdit<'g, P> {
    /// Opens the mutation capability owned by an active transaction.
    pub(crate) fn new(model: &'g mut Model<P>) -> Self {
        Self { model }
    }

    /// Returns an immutable view of the staged model.
    pub fn model(&self) -> &Model<P> {
        self.model
    }

    /// Copies a topology view into the staged map and returns its remapped handle.
    pub fn merge<T>(&mut self, topology: T) -> Dart
    where
        T: MergeTopology<P>,
    {
        self.model.merge(topology)
    }

    /// Adds an isolated dart inside the transaction.
    pub fn add_dart(&mut self) -> Dart {
        self.model.add_dart()
    }

    /// Removes a dart whose isolation has been proven by the caller.
    pub fn remove_dart(&mut self, dart: super::IsolatedDart) {
        self.model.remove_dart(dart);
    }

    /// Removes several isolated darts and remaps all retained topology and
    /// attribute references in one atomic compaction.
    pub fn remove_isolated_darts(
        &mut self,
        darts: Vec<super::IsolatedDart>,
    ) -> std::collections::HashMap<super::Dart, super::Dart> {
        self.model.remove_isolated_darts(darts)
    }

    /// Returns the current number of dart slots in the staged topology.
    pub fn dart_count(&self) -> usize {
        self.model.dart_count()
    }

    /// Returns the canonical representative of the staged cell containing
    /// `dart`.
    pub fn cell_representative(&self, dart: Dart, dim: Dim) -> Dart {
        self.model.cell_representative(dart, dim)
    }

    /// Links two free darts through exactly one alpha involution.
    pub fn link(&mut self, dim: Dim, first: Dart, second: Dart) -> Result<(), ModelEditError> {
        self.validate_dart(first)?;
        self.validate_dart(second)?;
        if first == second {
            return Err(ModelEditError::SameDart { dart: first });
        }
        for dart in [first, second] {
            if !self.model.is_free(dart, dim) {
                return Err(ModelEditError::DartNotFree { dart, dim });
            }
        }
        self.model.link_raw(dim, first, second);
        Ok(())
    }

    /// Unlinks the alpha pair containing `dart`.
    pub fn unlink(&mut self, dim: Dim, dart: Dart) -> Result<Dart, ModelEditError> {
        self.validate_dart(dart)?;
        if self.model.is_free(dart, dim) {
            return Err(ModelEditError::DartAlreadyFree { dart, dim });
        }
        Ok(self.model.unlink_raw(dim, dart))
    }

    /// Labels the raw `dimension`-cell containing `dart` as interior to `owner`.
    ///
    /// Commit rejects a labelling that does not describe the committed map, so
    /// an ownership record cannot outlive the cell it names.
    pub fn own_cell(&mut self, dimension: Dim, dart: Dart, owner: EntityOwner) {
        self.model.own_cell(dimension, dart, owner);
    }

    /// Unlabels the raw `dimension`-cell containing `dart`.
    ///
    /// The counterpart of [`Self::own_cell`], for a cell that stops being
    /// interior to anything -- a closure point an operation promotes into a
    /// corner of its own.
    pub fn disown_cell(&mut self, dimension: Dim, dart: Dart) {
        self.model.disown_cell(dimension, dart);
    }

    /// Performs a complete sewing operation without exposing intermediate
    /// inconsistent indexes.
    pub fn sew(&mut self, dim: Dim, first: Dart, second: Dart) -> Result<(), ModelEditError> {
        self.validate_dart(first)?;
        self.validate_dart(second)?;
        let Some(darts) = self.model.is_sewable(first, second, dim) else {
            return Err(ModelEditError::NotSewable { dim, first, second });
        };
        for (left, right) in darts.mapping {
            self.model.link_raw(dim, left, right);
        }
        Ok(())
    }

    /// Stages a vertex attribute for reconciliation at commit.
    ///
    /// No ownership is recorded here. Every entity contains the cell its own
    /// anchor sits in, so that part of the classification is read straight off
    /// the attribute and cannot drift from it. [`Self::own_cell`] records the
    /// rest -- the cells an entity contains *besides* its own, such as a
    /// closure point inside an edge or a seam inside a face.
    pub fn add_vertex(&mut self, vertex: VertexAttr<P::V>) -> VertexKey {
        self.add_vertex_with_origin(vertex, Origin::New)
    }

    /// Stages a vertex created by explicitly splitting an existing vertex.
    pub fn add_vertex_split_from(
        &mut self,
        source: VertexKey,
        vertex: VertexAttr<P::V>,
    ) -> VertexKey {
        self.add_vertex_with_origin(vertex, Origin::Split(EditKey::Vertex(source)))
    }

    /// Stages a vertex produced by entities of another kind.
    pub fn add_vertex_derived_from(
        &mut self,
        sources: Vec<EditKey>,
        vertex: VertexAttr<P::V>,
    ) -> VertexKey {
        self.add_vertex_with_origin(vertex, Origin::Derived(sources))
    }

    fn add_vertex_with_origin(
        &mut self,
        mut vertex: VertexAttr<P::V>,
        origin: Origin,
    ) -> VertexKey {
        vertex.dart = self.model.cell_representative(vertex.dart, Dim::Zero);
        let key = self.model.vertices.insert(vertex);
        self.model.invalidate_derived_indexes();
        self.model.record_edit_event(EditEvent::Created {
            key: EditKey::Vertex(key),
            origin,
        });
        key
    }

    /// Stages an edge attribute for reconciliation at commit.
    ///
    /// Its own 1-cell is derived, not recorded; see [`Self::add_vertex`].
    pub fn add_edge(&mut self, edge: EdgeAttr<P::E>) -> EdgeKey {
        self.add_edge_with_origin(edge, Origin::New)
    }

    /// Stages an edge created by explicitly splitting an existing edge.
    pub fn add_edge_split_from(&mut self, source: EdgeKey, edge: EdgeAttr<P::E>) -> EdgeKey {
        self.add_edge_with_origin(edge, Origin::Split(EditKey::Edge(source)))
    }

    /// Stages an edge produced by entities of another kind.
    pub fn add_edge_derived_from(
        &mut self,
        sources: Vec<EditKey>,
        edge: EdgeAttr<P::E>,
    ) -> EdgeKey {
        self.add_edge_with_origin(edge, Origin::Derived(sources))
    }

    fn add_edge_with_origin(&mut self, edge: EdgeAttr<P::E>, origin: Origin) -> EdgeKey {
        let key = self.model.edges.insert(edge);
        self.model.invalidate_derived_indexes();
        self.model.record_edit_event(EditEvent::Created {
            key: EditKey::Edge(key),
            origin,
        });
        key
    }

    /// Stages a profile attribute for reconciliation at commit.
    pub fn add_profile(&mut self, profile: ProfileAttr<P::Profile>) -> ProfileKey {
        self.add_profile_with_origin(profile, Origin::New)
    }

    /// Stages a profile created by explicitly splitting an existing profile.
    pub fn add_profile_split_from(
        &mut self,
        source: ProfileKey,
        profile: ProfileAttr<P::Profile>,
    ) -> ProfileKey {
        self.add_profile_with_origin(profile, Origin::Split(EditKey::Profile(source)))
    }

    /// Stages a profile produced by entities of another kind.
    pub fn add_profile_derived_from(
        &mut self,
        sources: Vec<EditKey>,
        profile: ProfileAttr<P::Profile>,
    ) -> ProfileKey {
        self.add_profile_with_origin(profile, Origin::Derived(sources))
    }

    fn add_profile_with_origin(
        &mut self,
        profile: ProfileAttr<P::Profile>,
        origin: Origin,
    ) -> ProfileKey {
        let key = self.model.profiles.insert(profile);
        self.model.invalidate_derived_indexes();
        self.model.record_edit_event(EditEvent::Created {
            key: EditKey::Profile(key),
            origin,
        });
        key
    }

    /// Stages a face attribute for reconciliation at commit.
    ///
    /// Its own 2-cell is derived, not recorded; see [`Self::add_vertex`].
    pub fn add_face(&mut self, face: FaceAttr<P::F>) -> FaceKey {
        self.add_face_with_origin(face, Origin::New)
    }

    /// Stages a face created by explicitly splitting an existing face.
    pub fn add_face_split_from(&mut self, source: FaceKey, face: FaceAttr<P::F>) -> FaceKey {
        self.add_face_with_origin(face, Origin::Split(EditKey::Face(source)))
    }

    /// Stages a face produced by entities of another kind, such as a sweep's
    /// wall face from the profile edge it runs along, or a loft's from the two
    /// section edges it spans.
    pub fn add_face_derived_from(
        &mut self,
        sources: Vec<EditKey>,
        face: FaceAttr<P::F>,
    ) -> FaceKey {
        self.add_face_with_origin(face, Origin::Derived(sources))
    }

    fn add_face_with_origin(&mut self, face: FaceAttr<P::F>, origin: Origin) -> FaceKey {
        let key = self.model.faces.insert(face);
        self.model.invalidate_derived_indexes();
        self.model.record_edit_event(EditEvent::Created {
            key: EditKey::Face(key),
            origin,
        });
        key
    }

    /// Stages a sheet attribute for reconciliation at commit.
    pub fn add_sheet(&mut self, sheet: SheetAttr<P::Sheet>) -> SheetKey {
        self.add_sheet_with_origin(sheet, Origin::New)
    }

    /// Stages a sheet created by explicitly splitting an existing sheet.
    pub fn add_sheet_split_from(
        &mut self,
        source: SheetKey,
        sheet: SheetAttr<P::Sheet>,
    ) -> SheetKey {
        self.add_sheet_with_origin(sheet, Origin::Split(EditKey::Sheet(source)))
    }

    /// Stages a sheet produced by entities of another kind.
    pub fn add_sheet_derived_from(
        &mut self,
        sources: Vec<EditKey>,
        sheet: SheetAttr<P::Sheet>,
    ) -> SheetKey {
        self.add_sheet_with_origin(sheet, Origin::Derived(sources))
    }

    fn add_sheet_with_origin(&mut self, sheet: SheetAttr<P::Sheet>, origin: Origin) -> SheetKey {
        let key = self.model.sheets.insert(sheet);
        self.model.invalidate_derived_indexes();
        self.model.record_edit_event(EditEvent::Created {
            key: EditKey::Sheet(key),
            origin,
        });
        key
    }

    /// Stages a solid attribute for reconciliation at commit.
    pub fn add_solid(&mut self, solid: SolidAttr<P::S>) -> SolidKey {
        self.add_solid_with_origin(solid, Origin::New)
    }

    /// Stages a solid created by explicitly splitting an existing solid.
    pub fn add_solid_split_from(&mut self, source: SolidKey, solid: SolidAttr<P::S>) -> SolidKey {
        self.add_solid_with_origin(solid, Origin::Split(EditKey::Solid(source)))
    }

    /// Stages a solid produced by entities of another kind.
    pub fn add_solid_derived_from(
        &mut self,
        sources: Vec<EditKey>,
        solid: SolidAttr<P::S>,
    ) -> SolidKey {
        self.add_solid_with_origin(solid, Origin::Derived(sources))
    }

    fn add_solid_with_origin(&mut self, solid: SolidAttr<P::S>, origin: Origin) -> SolidKey {
        let key = self.model.solids.insert(solid);
        self.model.invalidate_derived_indexes();
        self.model.record_edit_event(EditEvent::Created {
            key: EditKey::Solid(key),
            origin,
        });
        key
    }

    /// Declares that `removed` merged into `survivor`.
    ///
    /// Anything `removed` was classified as owning follows it, so a cell it
    /// claimed does not go on naming an identity the transaction has spoken
    /// for. This has to happen here rather than at commit: a later pass of the
    /// same transaction reads the classification back, and two owners on one
    /// orbit is a contradiction there however the transaction ends.
    pub fn merge_vertices_into(&mut self, survivor: VertexKey, removed: VertexKey) {
        self.model
            .retarget_ownership(EntityOwner::Vertex(removed), EntityOwner::Vertex(survivor));
        self.model.record_edit_event(EditEvent::Merged {
            survivor: EditKey::Vertex(survivor),
            removed: EditKey::Vertex(removed),
        });
    }

    /// Declares that `removed` merged into `survivor`, carrying its
    /// classification records across — see [`Self::merge_vertices_into`].
    pub fn merge_edges_into(&mut self, survivor: EdgeKey, removed: EdgeKey) {
        self.model
            .retarget_ownership(EntityOwner::Edge(removed), EntityOwner::Edge(survivor));
        self.model.record_edit_event(EditEvent::Merged {
            survivor: EditKey::Edge(survivor),
            removed: EditKey::Edge(removed),
        });
    }

    /// Declares that `removed` merged into `survivor`.
    pub fn merge_profiles_into(&mut self, survivor: ProfileKey, removed: ProfileKey) {
        self.model.record_edit_event(EditEvent::Merged {
            survivor: EditKey::Profile(survivor),
            removed: EditKey::Profile(removed),
        });
    }

    /// Follows the profile merges declared so far to the identity `profile`
    /// will be reconciled into, which is `profile` itself when none names it.
    ///
    /// A pass that decides what merged by reading the map back sees keys an
    /// earlier pass of the same transaction already spoke for, and a key may be
    /// declared merged only once. Asking where it went, and declaring into that
    /// identity instead, is what lets two removals rejoin overlapping sets of
    /// boundaries without either having to know about the other.
    pub(crate) fn merged_profile_survivor(&self, profile: ProfileKey) -> ProfileKey {
        let mut current = EditKey::Profile(profile);
        // A chain that closes on itself is a mistake, and commit names it
        // `MergeCycle`. This walk only has to reach that report rather than
        // spin, so it stops at the first key it sees twice.
        let mut visited = HashSet::from([current]);
        while let Some(survivor) =
            self.model
                .staged_edit_events()
                .iter()
                .find_map(|event| match event.merge_keys() {
                    Some((survivor, removed)) if removed == current => Some(survivor),
                    _ => None,
                })
        {
            if !visited.insert(survivor) {
                break;
            }
            current = survivor;
        }
        match current {
            EditKey::Profile(key) => key,
            _ => profile,
        }
    }

    /// Declares that `removed` merged into `survivor`, carrying its
    /// classification records across — see [`Self::merge_vertices_into`].
    pub fn merge_faces_into(&mut self, survivor: FaceKey, removed: FaceKey) {
        self.model
            .retarget_ownership(EntityOwner::Face(removed), EntityOwner::Face(survivor));
        self.model.record_edit_event(EditEvent::Merged {
            survivor: EditKey::Face(survivor),
            removed: EditKey::Face(removed),
        });
    }

    /// Declares that `removed` merged into `survivor`.
    pub fn merge_sheets_into(&mut self, survivor: SheetKey, removed: SheetKey) {
        self.model.record_edit_event(EditEvent::Merged {
            survivor: EditKey::Sheet(survivor),
            removed: EditKey::Sheet(removed),
        });
    }

    /// Declares that `removed` merged into `survivor`, carrying its
    /// classification records across — see [`Self::merge_vertices_into`].
    pub fn merge_solids_into(&mut self, survivor: SolidKey, removed: SolidKey) {
        self.model
            .retarget_ownership(EntityOwner::Solid(removed), EntityOwner::Solid(survivor));
        self.model.record_edit_event(EditEvent::Merged {
            survivor: EditKey::Solid(survivor),
            removed: EditKey::Solid(removed),
        });
    }

    /// Deletes face loops and orphaned lower-dimensional cells in one compaction pass.
    /// Sheet/solid registrations rooted in the deleted set must first be removed or moved.
    /// All cached darts are invalid afterwards; resolve surviving cells from their keys.
    pub fn remove_faces(&mut self, faces: &[FaceKey]) -> Result<(), ModelEditError> {
        let mut removed = HashSet::new();
        for &key in faces {
            let face = self
                .model
                .face(key)
                .ok_or(ModelEditError::MissingLineageAttribute {
                    key: EditKey::Face(key),
                })?;
            // Every dart the face covers, not the one per occurrence its
            // boundary walk names: removing a face has to free all of it, and
            // leaving half its darts behind leaves them sewn to neighbours that
            // no longer have anything on the other side.
            removed.extend(face.region_darts());
        }
        for root in self
            .model
            .iter_sheets()
            .map(|(_, attr)| attr.dart())
            .chain(self.model.iter_solids().flat_map(|(_, attr)| attr.shells()))
        {
            if removed.contains(&root) {
                return Err(ModelEditError::ReferencedDartDeletion { dart: root });
            }
        }
        let edges = self
            .model
            .iter_edges()
            .map(|(key, attr)| {
                let start = self.model.cell_key::<crate::model::Cell0>(attr.dart);
                let replacement = self
                    .model
                    .orbit(attr.dart, self.model.orbit_indices(Dim::One))
                    .find(|dart| {
                        !removed.contains(dart)
                            && self.model.cell_key::<crate::model::Cell0>(*dart) == start
                    });
                (key, replacement)
            })
            .collect::<Vec<_>>();
        let vertices = self
            .model
            .iter_vertices()
            .map(|(key, attr)| {
                (
                    key,
                    self.model
                        .orbit(attr.dart, self.model.orbit_indices(Dim::Zero))
                        .find(|dart| !removed.contains(dart)),
                )
            })
            .collect::<Vec<_>>();
        let profiles = self
            .model
            .iter_profiles()
            .filter_map(|(key, attr)| removed.contains(&attr.dart).then_some(key))
            .collect::<Vec<_>>();
        for &key in faces {
            self.remove_face(key);
        }
        for key in profiles {
            self.remove_profile(key);
        }
        for (key, dart) in edges {
            match dart {
                Some(dart) => self.edge_attr_mut_unchecked(key).dart = dart,
                None => {
                    self.remove_edge(key);
                }
            }
        }
        for (key, dart) in vertices {
            match dart {
                Some(dart) => self.vertex_attr_mut_unchecked(key).dart = dart,
                None => {
                    self.remove_vertex(key);
                }
            }
        }
        let mut darts = removed.into_iter().collect::<Vec<_>>();
        darts.sort_by_key(|dart| dart.id());
        for &dart in &darts {
            for dim in [Dim::Zero, Dim::One, Dim::Two, Dim::Three] {
                if !self.is_free(dart, dim) {
                    self.unlink(dim, dart)?;
                }
            }
        }
        self.remove_isolated_darts(darts.into_iter().map(super::IsolatedDart::new).collect());
        Ok(())
    }
    /// Removes a vertex attribute inside the transaction.
    ///
    /// Whatever the vertex claimed is unclassified with it: a label must not
    /// outlive the entity it names.
    pub fn remove_vertex(&mut self, key: VertexKey) -> Option<VertexAttr<P::V>> {
        let removed = self.model.vertices.remove(key);
        if removed.is_some() {
            self.model.disown_entity(EntityOwner::Vertex(key));
            self.model.record_edit_event(EditEvent::Consumed {
                key: EditKey::Vertex(key),
            });
        }
        removed
    }

    /// Removes an edge attribute inside the transaction.
    ///
    /// Whatever the edge claimed is unclassified with it, including any cell
    /// interior to it such as a closure point.
    pub fn remove_edge(&mut self, key: EdgeKey) -> Option<EdgeAttr<P::E>> {
        let removed = self.model.edges.remove(key);
        if removed.is_some() {
            self.model.disown_entity(EntityOwner::Edge(key));
            self.model.record_edit_event(EditEvent::Consumed {
                key: EditKey::Edge(key),
            });
        }
        removed
    }

    /// Removes a profile attribute inside the transaction.
    pub fn remove_profile(&mut self, key: ProfileKey) -> Option<ProfileAttr<P::Profile>> {
        let removed = self.model.profiles.remove(key);
        if removed.is_some() {
            self.model.invalidate_derived_indexes();
            self.model.record_edit_event(EditEvent::Consumed {
                key: EditKey::Profile(key),
            });
        }
        removed
    }

    /// Removes a face attribute inside the transaction.
    ///
    /// Whatever the face claimed is unclassified with it, including any cell
    /// interior to it such as a seam or a bridge.
    pub fn remove_face(&mut self, key: FaceKey) -> Option<FaceAttr<P::F>> {
        let removed = self.model.faces.remove(key);
        if removed.is_some() {
            self.model.disown_entity(EntityOwner::Face(key));
            self.model.record_edit_event(EditEvent::Consumed {
                key: EditKey::Face(key),
            });
        }
        removed
    }

    /// Removes a sheet attribute inside the transaction.
    pub fn remove_sheet(&mut self, key: SheetKey) -> Option<SheetAttr<P::Sheet>> {
        let removed = self.model.sheets.remove(key);
        if removed.is_some() {
            self.model.invalidate_derived_indexes();
            self.model.record_edit_event(EditEvent::Consumed {
                key: EditKey::Sheet(key),
            });
        }
        removed
    }

    /// Removes a solid attribute inside the transaction.
    ///
    /// Whatever the solid claimed is unclassified with it, including buried
    /// faces, edges and corners interior to it.
    pub fn remove_solid(&mut self, key: SolidKey) -> Option<SolidAttr<P::S>> {
        let removed = self.model.solids.remove(key);
        if removed.is_some() {
            self.model.disown_entity(EntityOwner::Solid(key));
            self.model.record_edit_event(EditEvent::Consumed {
                key: EditKey::Solid(key),
            });
        }
        removed
    }

    /// Returns mutable access to a staged vertex attribute.
    pub fn vertex_attr_mut(&mut self, key: VertexKey) -> Option<&mut VertexAttr<P::V>> {
        self.model.invalidate_derived_indexes();
        self.model.vertices.get_mut(key)
    }

    /// Returns mutable access to a staged vertex attribute, or panics if absent.
    pub fn vertex_attr_mut_unchecked(&mut self, key: VertexKey) -> &mut VertexAttr<P::V> {
        self.vertex_attr_mut(key)
            .expect("vertex attribute should be in the map")
    }

    /// Returns mutable access to a staged edge attribute.
    pub fn edge_attr_mut(&mut self, key: EdgeKey) -> Option<&mut EdgeAttr<P::E>> {
        self.model.invalidate_derived_indexes();
        self.model.edges.get_mut(key)
    }

    /// Returns mutable access to a staged edge attribute, or panics if absent.
    pub fn edge_attr_mut_unchecked(&mut self, key: EdgeKey) -> &mut EdgeAttr<P::E> {
        self.edge_attr_mut(key)
            .expect("edge attribute should be in the map")
    }

    /// Returns mutable access to a staged profile attribute.
    pub fn profile_attr_mut(&mut self, key: ProfileKey) -> Option<&mut ProfileAttr<P::Profile>> {
        self.model.invalidate_derived_indexes();
        self.model.profiles.get_mut(key)
    }

    /// Returns mutable access to a staged profile attribute, or panics if absent.
    pub fn profile_attr_mut_unchecked(&mut self, key: ProfileKey) -> &mut ProfileAttr<P::Profile> {
        self.profile_attr_mut(key)
            .expect("profile attribute should be in the map")
    }

    /// Returns mutable access to a staged face attribute.
    pub fn face_attr_mut(&mut self, key: FaceKey) -> Option<&mut FaceAttr<P::F>> {
        self.model.invalidate_derived_indexes();
        self.model.faces.get_mut(key)
    }

    /// Returns mutable access to a staged face attribute, or panics if absent.
    pub fn face_attr_mut_unchecked(&mut self, key: FaceKey) -> &mut FaceAttr<P::F> {
        self.face_attr_mut(key)
            .expect("face attribute should be in the map")
    }

    /// Returns mutable access to a staged sheet attribute.
    pub fn sheet_attr_mut(&mut self, key: SheetKey) -> Option<&mut SheetAttr<P::Sheet>> {
        self.model.invalidate_derived_indexes();
        self.model.sheets.get_mut(key)
    }

    /// Returns mutable access to a staged sheet attribute, or panics if absent.
    pub fn sheet_attr_mut_unchecked(&mut self, key: SheetKey) -> &mut SheetAttr<P::Sheet> {
        self.sheet_attr_mut(key)
            .expect("sheet attribute should be in the map")
    }

    /// Returns mutable access to a staged solid attribute.
    pub fn solid_attr_mut(&mut self, key: SolidKey) -> Option<&mut SolidAttr<P::S>> {
        self.model.invalidate_derived_indexes();
        self.model.solids.get_mut(key)
    }

    /// Returns mutable access to a staged solid attribute, or panics if absent.
    pub fn solid_attr_mut_unchecked(&mut self, key: SolidKey) -> &mut SolidAttr<P::S> {
        self.solid_attr_mut(key)
            .expect("solid attribute should be in the map")
    }

    fn validate_dart(&self, dart: Dart) -> Result<(), ModelEditError> {
        (dart.id() < self.model.dart_count())
            .then_some(())
            .ok_or(ModelEditError::MissingDart { dart })
    }
}

impl<P: Payload> Deref for ModelEdit<'_, P> {
    type Target = Model<P>;

    fn deref(&self) -> &Self::Target {
        self.model
    }
}

fn validate_required_domain_attributes<P: Payload>(g: &Model<P>) -> Result<(), ModelEditError> {
    for (face, attr) in g.faces.iter() {
        for dart in attr.darts() {
            if g.profile_key(dart).is_none() {
                return Err(ModelEditError::MissingProfileRegistration { face, dart });
            }
        }
    }

    for (solid, attr) in g.solids.iter() {
        for dart in attr.shells() {
            if g.sheet_key(dart).is_none() {
                return Err(ModelEditError::MissingSheetRegistration { solid, dart });
            }
        }
    }

    Ok(())
}

/// Validates and reconciles all staged work, then applies net payload events.
///
/// The caller owns rollback, so this function only mutates the staged map and
/// returns the first commit error it encounters.
pub(crate) fn commit_model_transaction<P, Q>(
    g: &mut Model<P>,
    snapshot: &Model<P>,
    events: &[EditEvent],
    policy: &mut Q,
) -> Result<(), ModelEditError>
where
    P: Payload,
    Q: EditPolicy<P>,
{
    validate_gmap(g.topology()).map_err(ModelEditError::InvalidTopology)?;
    validate_required_domain_attributes(g)?;
    validate_edit_events(g, snapshot, events)?;
    let lineage = TransactionLineage::new(g, snapshot, events);
    let structurally_dropped = reconcile_transaction_attributes(g, snapshot, events, &lineage)?;
    canonicalize_vertex_darts(g);
    validate_no_unexplained_removal(g, snapshot, events, &structurally_dropped)?;
    // The classification is checked *after* reconciliation, not before it.
    // A builder that lays down one entity per face corner and lets commit merge
    // the coincident ones is holding several keys on one cell on purpose, and
    // that is exactly what an earlier check would reject. Only once
    // reconciliation has settled which keys survived does a second owner for
    // one orbit mean a contradiction rather than a pending merge.
    g.validate_embedding()
        .map_err(ModelEditError::InvalidEmbedding)?;
    validate_cell_occupancy(g).map_err(ModelEditError::InvalidCellOccupancy)?;
    g.invalidate_derived_indexes();
    let policy_events = resolve_policy_events(g, snapshot, events, &lineage);
    apply_policy_events(g, snapshot, &policy_events, policy)?;
    g.materialize_derived_indexes();
    Ok(())
}

/// Rejects malformed lineage before reconciliation can consume any attributes.
fn validate_edit_events<P: Payload>(
    g: &Model<P>,
    snapshot: &Model<P>,
    events: &[EditEvent],
) -> Result<(), ModelEditError> {
    let mut merges = HashMap::new();

    let created_within = |key: EditKey| {
        events
            .iter()
            .any(|candidate| matches!(candidate, EditEvent::Created { key: created, .. } if *created == key))
    };
    let check_source = |source: EditKey| -> Result<(), ModelEditError> {
        if !contains_edit_key(g, source)
            && !contains_edit_key(snapshot, source)
            && !created_within(source)
        {
            return Err(ModelEditError::MissingLineageAttribute { key: source });
        }
        Ok(())
    };

    for event in events {
        match event {
            // Transaction-local identities may be explicitly discarded by a
            // later builder pass. Their creation record still determines
            // ordering, but they need not survive until commit.
            EditEvent::Created {
                origin: Origin::New,
                ..
            }
            | EditEvent::Copied { .. } => continue,
            EditEvent::Created {
                origin: Origin::Split(source),
                ..
            } => {
                check_source(*source)?;
            }
            EditEvent::Created {
                origin: Origin::Derived(sources),
                ..
            } => {
                for &source in sources {
                    check_source(source)?;
                }
            }
            // Recorded once the attribute is already gone; there is nothing
            // left here to validate against the staged map.
            EditEvent::Consumed { .. } => continue,
            EditEvent::Merged { survivor, removed } => {
                let (survivor, removed) = (*survivor, *removed);
                // A merge both of whose identities are gone was spent: a later
                // pass in the same operation removed the cell they had just
                // come to share, so there is nothing left to reconcile or to
                // hand the payload policy.
                if is_spent_merge(g, survivor, removed) {
                    continue;
                }
                for key in [survivor, removed] {
                    if !contains_edit_key(g, key) {
                        return Err(ModelEditError::MissingLineageAttribute { key });
                    }
                }
                if survivor == removed {
                    return Err(ModelEditError::InvalidMerge { survivor, removed });
                }
                if merges.insert(removed, survivor).is_some() {
                    return Err(ModelEditError::RepeatedMerge { removed });
                }
            }
        }
    }

    for &start in merges.keys() {
        let mut visited = HashSet::new();
        let mut current = start;
        while let Some(&next) = merges.get(&current) {
            if !visited.insert(current) {
                return Err(ModelEditError::MergeCycle { key: current });
            }
            current = next;
        }
    }

    Ok(())
}

impl EditEvent {
    /// Extracts a merge's survivor and consumed identity.
    pub(crate) fn merge_keys(&self) -> Option<(EditKey, EditKey)> {
        match self {
            Self::Merged { survivor, removed } => Some((*survivor, *removed)),
            _ => None,
        }
    }
}

/// Reports whether a merge declaration has nothing left to reconcile.
///
/// Both identities are absent exactly when a later pass removed the cell they
/// were merging into. The declaration is then inert: it names no surviving
/// identity, consumes nothing, and reaches no payload policy.
fn is_spent_merge<P: Payload>(g: &Model<P>, survivor: EditKey, removed: EditKey) -> bool {
    !contains_edit_key(g, survivor) && !contains_edit_key(g, removed)
}

/// Rejects a commit that leaves a transaction-start attribute unaccounted for.
///
/// Every attribute the transaction started with is either still present at
/// commit, named by a `Merged` or `Consumed` event, or dropped by the
/// structural bookkeeping reconciliation performs on its own -- a profile that
/// lost its last edge, which is a redefinition rather than a removal. Since
/// `remove_*` and `merge_*_into` are the only ways to delete an attribute and
/// both record their own event, reaching this error means some other path
/// bypassed them.
fn validate_no_unexplained_removal<P: Payload>(
    g: &Model<P>,
    snapshot: &Model<P>,
    events: &[EditEvent],
    structurally_dropped: &HashSet<EditKey>,
) -> Result<(), ModelEditError> {
    let explained = events
        .iter()
        .filter_map(|event| match event {
            EditEvent::Merged { removed, .. } => Some(*removed),
            EditEvent::Consumed { key } => Some(*key),
            _ => None,
        })
        .collect::<HashSet<_>>();
    for key in current_edit_keys(snapshot) {
        if !contains_edit_key(g, key)
            && !explained.contains(&key)
            && !structurally_dropped.contains(&key)
        {
            return Err(ModelEditError::UnexplainedRemoval { key });
        }
    }
    Ok(())
}

/// Checks the appropriate attribute store for a type-erased edit key.
fn contains_edit_key<P: Payload>(g: &Model<P>, key: EditKey) -> bool {
    match key {
        EditKey::Vertex(key) => g.vertices.contains_key(key),
        EditKey::Edge(key) => g.edges.contains_key(key),
        EditKey::Profile(key) => g.profiles.contains_key(key),
        EditKey::Face(key) => g.faces.contains_key(key),
        EditKey::Sheet(key) => g.sheets.contains_key(key),
        EditKey::Solid(key) => g.solids.contains_key(key),
    }
}

#[derive(Debug, Clone, Copy)]
enum CreationOrigin {
    Fresh,
    SplitFrom(EditKey),
}

struct TransactionLineage {
    origins: HashMap<EditKey, CreationOrigin>,
    creation_order: HashMap<EditKey, usize>,
    merges: HashMap<EditKey, EditKey>,
}

impl TransactionLineage {
    /// Builds origin, creation-order, and merge-chain metadata for the commit.
    ///
    /// Attributes inserted directly on the map are also discovered and treated
    /// as fresh local identities so they participate in deterministic ordering.
    fn new<P: Payload>(g: &Model<P>, snapshot: &Model<P>, events: &[EditEvent]) -> Self {
        let mut origins = HashMap::new();
        let mut creation_order = HashMap::new();
        let mut merges = HashMap::new();

        for (order, event) in events.iter().enumerate() {
            match event {
                EditEvent::Created {
                    key,
                    origin: Origin::Split(source),
                } => {
                    origins.insert(*key, CreationOrigin::SplitFrom(*source));
                    creation_order.entry(*key).or_insert(order);
                }
                // A derived creation has several sources of another kind, so
                // there is no single parent to trace a split chain through.
                // Stage 2's creation hook reads `Origin::Derived` off the
                // event log directly; the reconciliation ordering below only
                // needs a creation order, which "fresh" already supplies.
                EditEvent::Created {
                    key,
                    origin: Origin::New | Origin::Derived(_),
                }
                | EditEvent::Copied { key } => {
                    origins.entry(*key).or_insert(CreationOrigin::Fresh);
                    creation_order.entry(*key).or_insert(order);
                }
                EditEvent::Merged { survivor, removed } => {
                    merges.insert(*removed, *survivor);
                }
                EditEvent::Consumed { .. } => {}
            }
        }

        let mut next_order = events.len();
        for key in current_edit_keys(g) {
            if contains_edit_key(snapshot, key) || creation_order.contains_key(&key) {
                continue;
            }
            origins.insert(key, CreationOrigin::Fresh);
            creation_order.insert(key, next_order);
            next_order += 1;
        }

        Self {
            origins,
            creation_order,
            merges,
        }
    }
}

#[derive(Debug, Clone, Copy)]
enum PolicyEvent {
    Split { source: EditKey, created: EditKey },
    Merge { survivor: EditKey, removed: EditKey },
}

/// Reduces the raw journal to net changes visible outside the transaction.
///
/// Transient local identities are omitted, split ancestry is traced back to the
/// snapshot, and merge chains target their final surviving identity.
fn resolve_policy_events<P: Payload>(
    g: &Model<P>,
    snapshot: &Model<P>,
    events: &[EditEvent],
    lineage: &TransactionLineage,
) -> Vec<PolicyEvent> {
    events
        .iter()
        .filter_map(|event| match event {
            // Only a split names a single source of the created entity's own
            // kind, so only a split has a hook to route to in this vocabulary.
            // `New` and `Derived` reach no policy call until a creation hook
            // exists to receive them.
            EditEvent::Created {
                key: created,
                origin: Origin::Split(source),
            } => {
                if !contains_edit_key(g, *created) {
                    return None;
                }
                transaction_start_origin(snapshot, &lineage.origins, *source).map(|source| {
                    PolicyEvent::Split {
                        source,
                        created: *created,
                    }
                })
            }
            EditEvent::Merged { survivor, removed } => {
                let survivor = final_survivor(&lineage.merges, *survivor);
                let removed = *removed;
                // A spent merge has no surviving payload to merge into.
                (contains_edit_key(snapshot, removed) && contains_edit_key(g, survivor))
                    .then_some(PolicyEvent::Merge { survivor, removed })
            }
            _ => None,
        })
        .collect()
}

/// Collects all topology-associated attribute keys currently stored by the map.
fn current_edit_keys<P: Payload>(g: &Model<P>) -> Vec<EditKey> {
    let mut keys = Vec::new();
    keys.extend(g.vertices.keys().map(EditKey::Vertex));
    keys.extend(g.edges.keys().map(EditKey::Edge));
    keys.extend(g.profiles.keys().map(EditKey::Profile));
    keys.extend(g.faces.keys().map(EditKey::Face));
    keys.extend(g.sheets.keys().map(EditKey::Sheet));
    keys.extend(g.solids.keys().map(EditKey::Solid));
    keys
}

/// Follows split ancestry until it reaches a transaction-start identity.
fn transaction_start_origin<P: Payload>(
    snapshot: &Model<P>,
    origins: &HashMap<EditKey, CreationOrigin>,
    start: EditKey,
) -> Option<EditKey> {
    let mut current = start;
    let mut visited = HashSet::new();

    loop {
        if contains_edit_key(snapshot, current) {
            return Some(current);
        }
        if !visited.insert(current) {
            return None;
        }
        match origins.get(&current) {
            Some(CreationOrigin::SplitFrom(source)) => current = *source,
            Some(CreationOrigin::Fresh) | None => return None,
        }
    }
}

/// Follows an already-validated merge chain to its final survivor.
fn final_survivor(merges: &HashMap<EditKey, EditKey>, start: EditKey) -> EditKey {
    let mut survivor = start;
    while let Some(next) = merges.get(&survivor) {
        survivor = *next;
    }
    survivor
}

/// Collapses duplicate identities that now describe the same final topological cell.
///
/// Explicitly consumed attributes are removed first. Remaining local collisions
/// are resolved per cell type, while ambiguous pre-existing collisions are errors.
///
/// Returns every transaction-start attribute this bookkeeping dropped on its
/// own authority, independent of any declared event -- today, only a profile
/// that lost its last edge. The unexplained-removal check exempts these:
/// dropping them is the profile definition applied, not a removal a builder
/// had to name.
fn reconcile_transaction_attributes<P: Payload>(
    g: &mut Model<P>,
    snapshot: &Model<P>,
    events: &[EditEvent],
    lineage: &TransactionLineage,
) -> Result<HashSet<EditKey>, ModelEditError> {
    let mut spent = events
        .iter()
        .filter_map(|event| event.merge_keys())
        .filter(|&(survivor, removed)| is_spent_merge(g, survivor, removed))
        .map(|(survivor, _)| survivor)
        .collect::<HashSet<_>>();
    let mut structurally_dropped = HashSet::new();
    remove_consumed_attributes(g, events);

    let vertices = g
        .vertices
        .iter()
        .map(|(key, attr)| (key, vec![g.cell_representative(attr.dart, Dim::Zero)]))
        .collect();
    reconcile_components(
        g,
        snapshot,
        lineage,
        "vertex",
        collision_components(vertices),
        EditKey::Vertex,
    )?;

    let edges = g
        .edges
        .iter()
        .map(|(key, attr)| (key, vec![g.cell_representative(attr.dart, Dim::One)]))
        .collect();
    reconcile_components(
        g,
        snapshot,
        lineage,
        "edge",
        collision_components(edges),
        EditKey::Edge,
    )?;

    // A profile is a connected set of *edges*, so it is seeded on one. A
    // boundary that loses its last edge leaves its key behind on the bridge
    // that used to reach it, which is scaffold and belongs to no chain -- and a
    // walk started there leaves along every loop that bridge joins, so the key
    // reads as a second identity on a neighbouring profile. Dropping it here is
    // the definition applied, not a repair.
    let edgeless = g
        .profiles
        .iter()
        .filter(|(_, attr)| g.cell_key::<crate::model::Cell1>(attr.dart).is_none())
        .map(|(key, _)| key)
        .collect::<Vec<_>>();
    for key in edgeless {
        g.profiles.remove(key);
        // A merge declared into a key the definition says does not exist is
        // spent, exactly like one whose identities a later pass consumed.
        spent.insert(EditKey::Profile(key));
        structurally_dropped.insert(EditKey::Profile(key));
    }

    let profiles = g
        .profiles
        .iter()
        .map(|(key, attr)| {
            (
                key,
                vec![crate::topology::profile::Profile::representative(
                    g, attr.dart,
                )],
            )
        })
        .collect();
    reconcile_components(
        g,
        snapshot,
        lineage,
        "profile",
        collision_components(profiles),
        EditKey::Profile,
    )?;

    let faces = g
        .faces
        .iter()
        .map(|(key, attr)| {
            let representatives = attr
                .darts()
                .map(|dart| g.cell_representative(dart, Dim::Two))
                .collect();
            (key, representatives)
        })
        .collect();
    reconcile_components(
        g,
        snapshot,
        lineage,
        "face",
        collision_components(faces),
        EditKey::Face,
    )?;

    let sheets = g
        .sheets
        .iter()
        .map(|(key, attr)| {
            (
                key,
                vec![crate::topology::sheet::Sheet::representative(
                    g,
                    attr.dart(),
                )],
            )
        })
        .collect();
    reconcile_components(
        g,
        snapshot,
        lineage,
        "sheet",
        collision_components(sheets),
        EditKey::Sheet,
    )?;

    let solids = g
        .solids
        .iter()
        .map(|(key, attr)| {
            let representatives = attr
                .shells()
                .map(|dart| g.cell_representative(dart, Dim::Three))
                .collect();
            (key, representatives)
        })
        .collect();
    reconcile_components(
        g,
        snapshot,
        lineage,
        "solid",
        collision_components(solids),
        EditKey::Solid,
    )?;

    let mut checked = HashSet::new();
    for survivor in lineage.merges.values() {
        let survivor = final_survivor(&lineage.merges, *survivor);
        if checked.insert(survivor) && !spent.contains(&survivor) && !contains_edit_key(g, survivor)
        {
            return Err(ModelEditError::InvalidLineageSurvivor { survivor });
        }
    }

    Ok(structurally_dropped)
}

/// Groups keys connected through one or more shared cell representatives.
///
/// Faces and solids can have multiple registered boundaries, so collisions are
/// transitive rather than necessarily sharing a single representative directly.
fn collision_components<K>(items: Vec<(K, Vec<Dart>)>) -> Vec<(Dart, Vec<K>)>
where
    K: Copy + Eq + Hash,
{
    let locations = items.into_iter().collect::<HashMap<_, _>>();
    let mut by_representative = HashMap::<Dart, Vec<K>>::new();
    for (&key, representatives) in &locations {
        for &representative in representatives {
            by_representative
                .entry(representative)
                .or_default()
                .push(key);
        }
    }

    let mut visited = HashSet::new();
    let mut components = Vec::new();
    for &start in locations.keys() {
        if !visited.insert(start) {
            continue;
        }

        let mut stack = vec![start];
        let mut keys = Vec::new();
        while let Some(key) = stack.pop() {
            keys.push(key);
            for representative in &locations[&key] {
                for &neighbor in &by_representative[representative] {
                    if visited.insert(neighbor) {
                        stack.push(neighbor);
                    }
                }
            }
        }

        if keys.len() < 2 {
            continue;
        }
        let representative = keys
            .iter()
            .flat_map(|key| locations[key].iter().copied())
            .filter(|representative| by_representative[representative].len() > 1)
            .min_by_key(|representative| representative.id())
            .expect("a collision component must share a representative");
        components.push((representative, keys));
    }
    components.sort_by_key(|(representative, _)| representative.id());
    components
}

/// Chooses one deterministic survivor in each collision component and drops locals.
fn reconcile_components<P, K, F>(
    g: &mut Model<P>,
    snapshot: &Model<P>,
    lineage: &TransactionLineage,
    entity: &'static str,
    components: Vec<(Dart, Vec<K>)>,
    edit_key: F,
) -> Result<(), ModelEditError>
where
    P: Payload,
    K: Copy,
    F: Fn(K) -> EditKey,
{
    for (representative, keys) in components {
        let keys = keys.into_iter().map(&edit_key).collect::<Vec<_>>();
        let pre_existing = keys
            .iter()
            .copied()
            .filter(|key| contains_edit_key(snapshot, *key))
            .collect::<Vec<_>>();
        if pre_existing.len() > 1 {
            return Err(ModelEditError::UnresolvedPreExistingCollision {
                entity,
                representative,
                candidates: pre_existing,
            });
        }

        let survivor = pre_existing.first().copied().unwrap_or_else(|| {
            keys.iter()
                .copied()
                .min_by_key(|key| {
                    lineage
                        .creation_order
                        .get(key)
                        .copied()
                        .unwrap_or(usize::MAX)
                })
                .expect("a collision component cannot be empty")
        });
        for key in keys {
            if key != survivor {
                remove_edit_key(g, key);
            }
        }
    }
    Ok(())
}

/// Removes a type-erased attribute known to exist during reconciliation.
fn remove_edit_key<P: Payload>(g: &mut Model<P>, key: EditKey) {
    match key {
        EditKey::Vertex(key) => {
            g.vertices
                .remove(key)
                .expect("reconciled vertex key must have an attribute");
        }
        EditKey::Edge(key) => {
            g.edges
                .remove(key)
                .expect("reconciled edge key must have an attribute");
        }
        EditKey::Profile(key) => {
            g.profiles
                .remove(key)
                .expect("reconciled profile key must have an attribute");
        }
        EditKey::Face(key) => {
            g.faces
                .remove(key)
                .expect("reconciled face key must have an attribute");
        }
        EditKey::Sheet(key) => {
            g.sheets
                .remove(key)
                .expect("reconciled sheet key must have an attribute");
        }
        EditKey::Solid(key) => {
            g.solids
                .remove(key)
                .expect("reconciled solid key must have an attribute");
        }
    }
}

/// Removes every identity explicitly consumed by a validated merge declaration.
/// A consumed identity is normally still staged, having been validated above.
/// The exception is a spent merge, whose cell a later pass of the same
/// operation removed outright, taking both identities with it.
fn remove_consumed_attributes<P: Payload>(g: &mut Model<P>, events: &[EditEvent]) {
    for event in events {
        if let EditEvent::Merged { removed, .. } = event {
            match *removed {
                EditKey::Vertex(key) => {
                    g.vertices.remove(key);
                }
                EditKey::Edge(key) => {
                    g.edges.remove(key);
                }
                EditKey::Profile(key) => {
                    g.profiles.remove(key);
                }
                EditKey::Face(key) => {
                    g.faces.remove(key);
                }
                EditKey::Sheet(key) => {
                    g.sheets.remove(key);
                }
                EditKey::Solid(key) => {
                    g.solids.remove(key);
                }
            }
        }
    }
}

/// Applies net split and merge policy calls in journal order using snapshot inputs.
///
/// Source and removed payloads always come from the transaction-start snapshot;
/// only surviving staged payloads are mutated.
fn apply_policy_events<P, Q>(
    g: &mut Model<P>,
    snapshot: &Model<P>,
    events: &[PolicyEvent],
    policy: &mut Q,
) -> Result<(), ModelEditError>
where
    P: Payload,
    Q: EditPolicy<P>,
{
    for event in events {
        match *event {
            PolicyEvent::Split {
                source: EditKey::Vertex(source),
                created: EditKey::Vertex(created),
            } => {
                let source_data = snapshot.vertex_attr_unchecked(source).data.clone();
                let created_data = &mut g.vertex_attr_mut_unchecked(created).data;
                policy
                    .split_vertex_data(source, &source_data, created, created_data)
                    .map_err(|error| ModelEditError::Policy(Box::new(error)))?;
            }
            PolicyEvent::Split {
                source: EditKey::Edge(source),
                created: EditKey::Edge(created),
            } => {
                let source_data = snapshot.edge_attr_unchecked(source).data.clone();
                let created_data = &mut g.edge_attr_mut_unchecked(created).data;
                policy
                    .split_edge_data(source, &source_data, created, created_data)
                    .map_err(|error| ModelEditError::Policy(Box::new(error)))?;
            }
            PolicyEvent::Split {
                source: EditKey::Profile(source),
                created: EditKey::Profile(created),
            } => {
                let source_data = snapshot.profile_attr_unchecked(source).data.clone();
                let created_data = &mut g.profile_attr_mut_unchecked(created).data;
                policy
                    .split_profile_data(source, &source_data, created, created_data)
                    .map_err(|error| ModelEditError::Policy(Box::new(error)))?;
            }
            PolicyEvent::Split {
                source: EditKey::Face(source),
                created: EditKey::Face(created),
            } => {
                let source_data = snapshot.face_attr_unchecked(source).data.clone();
                let created_data = &mut g.face_attr_mut_unchecked(created).data;
                policy
                    .split_face_data(source, &source_data, created, created_data)
                    .map_err(|error| ModelEditError::Policy(Box::new(error)))?;
            }
            PolicyEvent::Split {
                source: EditKey::Sheet(source),
                created: EditKey::Sheet(created),
            } => {
                let source_data = snapshot.sheet_attr_unchecked(source).data.clone();
                let created_data = &mut g.sheet_attr_mut_unchecked(created).data;
                policy
                    .split_sheet_data(source, &source_data, created, created_data)
                    .map_err(|error| ModelEditError::Policy(Box::new(error)))?;
            }
            PolicyEvent::Split {
                source: EditKey::Solid(source),
                created: EditKey::Solid(created),
            } => {
                let source_data = snapshot.solid_attr_unchecked(source).data.clone();
                let created_data = &mut g.solid_attr_mut_unchecked(created).data;
                policy
                    .split_solid_data(source, &source_data, created, created_data)
                    .map_err(|error| ModelEditError::Policy(Box::new(error)))?;
            }
            PolicyEvent::Merge {
                survivor: EditKey::Vertex(survivor),
                removed: EditKey::Vertex(removed),
            } => {
                let removed_data = snapshot.vertex_attr_unchecked(removed).data.clone();
                let survivor_data = &mut g.vertex_attr_mut_unchecked(survivor).data;
                policy
                    .merge_vertex_data(survivor, survivor_data, removed, removed_data)
                    .map_err(|error| ModelEditError::Policy(Box::new(error)))?;
            }
            PolicyEvent::Merge {
                survivor: EditKey::Edge(survivor),
                removed: EditKey::Edge(removed),
            } => {
                let removed_data = snapshot.edge_attr_unchecked(removed).data.clone();
                let survivor_data = &mut g.edge_attr_mut_unchecked(survivor).data;
                policy
                    .merge_edge_data(survivor, survivor_data, removed, removed_data)
                    .map_err(|error| ModelEditError::Policy(Box::new(error)))?;
            }
            PolicyEvent::Merge {
                survivor: EditKey::Profile(survivor),
                removed: EditKey::Profile(removed),
            } => {
                let removed_data = snapshot.profile_attr_unchecked(removed).data.clone();
                let survivor_data = &mut g.profile_attr_mut_unchecked(survivor).data;
                policy
                    .merge_profile_data(survivor, survivor_data, removed, removed_data)
                    .map_err(|error| ModelEditError::Policy(Box::new(error)))?;
            }
            PolicyEvent::Merge {
                survivor: EditKey::Face(survivor),
                removed: EditKey::Face(removed),
            } => {
                let removed_data = snapshot.face_attr_unchecked(removed).data.clone();
                let survivor_data = &mut g.face_attr_mut_unchecked(survivor).data;
                policy
                    .merge_face_data(survivor, survivor_data, removed, removed_data)
                    .map_err(|error| ModelEditError::Policy(Box::new(error)))?;
            }
            PolicyEvent::Merge {
                survivor: EditKey::Sheet(survivor),
                removed: EditKey::Sheet(removed),
            } => {
                let removed_data = snapshot.sheet_attr_unchecked(removed).data.clone();
                let survivor_data = &mut g.sheet_attr_mut_unchecked(survivor).data;
                policy
                    .merge_sheet_data(survivor, survivor_data, removed, removed_data)
                    .map_err(|error| ModelEditError::Policy(Box::new(error)))?;
            }
            PolicyEvent::Merge {
                survivor: EditKey::Solid(survivor),
                removed: EditKey::Solid(removed),
            } => {
                let removed_data = snapshot.solid_attr_unchecked(removed).data.clone();
                let survivor_data = &mut g.solid_attr_mut_unchecked(survivor).data;
                policy
                    .merge_solid_data(survivor, survivor_data, removed, removed_data)
                    .map_err(|error| ModelEditError::Policy(Box::new(error)))?;
            }
            _ => unreachable!("edit lineage always preserves the attribute type"),
        }
    }
    Ok(())
}

/// Stores each surviving vertex attribute on its final canonical 0-cell dart.
fn canonicalize_vertex_darts<P: Payload>(g: &mut Model<P>) {
    let canonical_darts = g
        .vertices
        .iter()
        .map(|(key, attr)| (key, g.cell_representative(attr.dart, Dim::Zero)))
        .collect::<Vec<_>>();
    for (key, dart) in canonical_darts {
        g.vertices[key].dart = dart;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Point3;
    use crate::topology::payload::StandardPayload;

    /// `remove_vertex` and `merge_*_into` are the only ways `ModelEdit` deletes
    /// an attribute, and both record their own event. Reaching this error
    /// therefore requires bypassing them, which this test does directly
    /// through the crate-internal field `ModelEdit` normally keeps private --
    /// standing in for a future path that forgets to call either.
    #[test]
    fn a_removal_no_event_explains_is_rejected() {
        let mut g = Model::<StandardPayload>::new();
        let key = g
            .transaction(|edit| {
                let dart = edit.add_dart();
                Ok::<_, ModelEditError>(edit.add_vertex(VertexAttr::new(
                    dart,
                    Point3::origin(),
                    (),
                )))
            })
            .unwrap();

        let result = g.transaction(|edit| {
            edit.model.vertices.remove(key);
            Ok::<_, ModelEditError>(())
        });

        assert!(matches!(
            result,
            Err(ModelEditError::UnexplainedRemoval { key: EditKey::Vertex(k) }) if k == key
        ));
        assert!(
            g.vertices.contains_key(key),
            "a rejected commit should restore the transaction-start snapshot"
        );
    }
}
