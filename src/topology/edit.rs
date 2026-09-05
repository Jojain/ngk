use std::collections::{HashMap, HashSet};
use std::convert::Infallible;
use std::error::Error;
use std::hash::Hash;
use std::ops::Deref;

use thiserror::Error;

use super::Dart;
use super::attributes::{EdgeAttr, FaceAttr, ProfileAttr, SheetAttr, SolidAttr, VertexAttr};
use super::gmap::{Dim, GMap, MergeTopology};
use super::payload::Payload;
use super::shape_keys::{EdgeKey, FaceKey, ProfileKey, SheetKey, SolidKey, VertexKey};
use super::validation::{GMapValidationError, validate_gmap};

/// Controls how payloads are propagated for explicit semantic edit events.
///
/// The edit layer does not infer merge or split lineage from topology. Builders
/// must declare semantic events through methods such as
/// [`TopologyEdit::add_edge_split_from`] and
/// [`TopologyEdit::merge_edges_into`].
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

/// Failure raised while applying a safe topology mutation.
#[derive(Debug, Error)]
pub enum TopologyEditError {
    #[error("cannot delete dart {dart:?} while it is a registered sheet or solid root")]
    ReferencedDartDeletion { dart: Dart },
    /// A split or merge references an attribute that is not staged.
    #[error("topology edit lineage references missing attribute {key:?}")]
    MissingLineageAttribute { key: EditKey },
    /// The same attribute was declared consumed more than once.
    #[error("topology edit lineage consumes {removed:?} more than once")]
    RepeatedMerge { removed: EditKey },
    /// A merge cannot consume its own survivor.
    #[error("topology edit lineage cannot merge {removed:?} into itself")]
    InvalidMerge { survivor: EditKey, removed: EditKey },
    /// Explicit merge declarations contain a cycle.
    #[error("topology edit lineage contains a merge cycle through {key:?}")]
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
    /// The two cells do not satisfy the GMap sewing constraints.
    #[error("darts {first:?} and {second:?} are not sewable along {dim:?}")]
    NotSewable { dim: Dim, first: Dart, second: Dart },
    /// The edited alpha relations do not satisfy the GMap axioms.
    #[error("topology edit produced an invalid GMap")]
    InvalidTopology(#[source] GMapValidationError),
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

/// Transaction-scoped capability for reading and mutating a [`GMap`].
///
/// All staged mutations and semantic lineage pass through this capability.
/// Validation, policy application, and rollback are owned by
/// [`GMap::transaction`](GMap::transaction).
pub struct TopologyEdit<'g, P: Payload> {
    gmap: &'g mut GMap<P>,
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

#[derive(Debug, Clone, Copy)]
pub(super) enum EditEvent {
    Created {
        key: EditKey,
    },
    VertexSplit {
        source: VertexKey,
        created: VertexKey,
    },
    EdgeSplit {
        source: EdgeKey,
        created: EdgeKey,
    },
    ProfileSplit {
        source: ProfileKey,
        created: ProfileKey,
    },
    FaceSplit {
        source: FaceKey,
        created: FaceKey,
    },
    SheetSplit {
        source: SheetKey,
        created: SheetKey,
    },
    SolidSplit {
        source: SolidKey,
        created: SolidKey,
    },
    VertexMerge {
        survivor: VertexKey,
        removed: VertexKey,
    },
    EdgeMerge {
        survivor: EdgeKey,
        removed: EdgeKey,
    },
    ProfileMerge {
        survivor: ProfileKey,
        removed: ProfileKey,
    },
    FaceMerge {
        survivor: FaceKey,
        removed: FaceKey,
    },
    SheetMerge {
        survivor: SheetKey,
        removed: SheetKey,
    },
    SolidMerge {
        survivor: SolidKey,
        removed: SolidKey,
    },
}

impl<'g, P: Payload> TopologyEdit<'g, P> {
    /// Opens the mutation capability owned by an active transaction.
    pub(super) fn new(gmap: &'g mut GMap<P>) -> Self {
        Self { gmap }
    }

    /// Returns an immutable view of the staged map.
    pub fn map(&self) -> &GMap<P> {
        self.gmap
    }

    /// Copies a topology view into the staged map and returns its remapped handle.
    pub fn merge<T>(&mut self, topology: T) -> Dart
    where
        T: MergeTopology<P>,
    {
        self.gmap.merge(topology)
    }

    /// Adds an isolated dart inside the transaction.
    pub fn add_dart(&mut self) -> Dart {
        self.gmap.add_dart()
    }

    /// Removes a dart whose isolation has been proven by the caller.
    pub fn remove_dart(&mut self, dart: super::IsolatedDart) {
        self.gmap.remove_dart(dart);
    }

    /// Removes several isolated darts and remaps all retained topology and
    /// attribute references in one atomic compaction.
    pub fn remove_isolated_darts(
        &mut self,
        darts: Vec<super::IsolatedDart>,
    ) -> std::collections::HashMap<super::Dart, super::Dart> {
        self.gmap.remove_isolated_darts(darts)
    }

    /// Returns the current number of dart slots in the staged topology.
    pub fn dart_count(&self) -> usize {
        self.gmap.dart_count()
    }

    /// Returns the canonical representative of the staged cell containing
    /// `dart`.
    pub fn cell_representative(&self, dart: Dart, dim: Dim) -> Dart {
        self.gmap.cell_representative(dart, dim)
    }

    /// Links two free darts through exactly one alpha involution.
    pub fn link(&mut self, dim: Dim, first: Dart, second: Dart) -> Result<(), TopologyEditError> {
        self.validate_dart(first)?;
        self.validate_dart(second)?;
        if first == second {
            return Err(TopologyEditError::SameDart { dart: first });
        }
        for dart in [first, second] {
            if !self.gmap.is_free(dart, dim) {
                return Err(TopologyEditError::DartNotFree { dart, dim });
            }
        }
        self.gmap.link_raw(dim, first, second);
        Ok(())
    }

    /// Unlinks the alpha pair containing `dart`.
    pub fn unlink(&mut self, dim: Dim, dart: Dart) -> Result<Dart, TopologyEditError> {
        self.validate_dart(dart)?;
        if self.gmap.is_free(dart, dim) {
            return Err(TopologyEditError::DartAlreadyFree { dart, dim });
        }
        Ok(self.gmap.unlink_raw(dim, dart))
    }

    /// Performs a complete GMap sewing operation without exposing intermediate
    /// inconsistent indexes.
    pub fn sew(&mut self, dim: Dim, first: Dart, second: Dart) -> Result<(), TopologyEditError> {
        self.validate_dart(first)?;
        self.validate_dart(second)?;
        let Some(darts) = self.gmap.is_sewable(first, second, dim) else {
            return Err(TopologyEditError::NotSewable { dim, first, second });
        };
        for (left, right) in darts.mapping {
            self.gmap.link_raw(dim, left, right);
        }
        Ok(())
    }

    /// Stages a vertex attribute for reconciliation at commit.
    pub fn add_vertex(&mut self, mut vertex: VertexAttr<P::V>) -> VertexKey {
        vertex.dart = self.gmap.cell_representative(vertex.dart, Dim::Zero);
        let key = self.gmap.vertices.insert(vertex);
        self.gmap.invalidate_derived_indexes();
        self.gmap.record_edit_event(EditEvent::Created {
            key: EditKey::Vertex(key),
        });
        key
    }

    /// Stages a vertex created by explicitly splitting an existing vertex.
    pub fn add_vertex_split_from(
        &mut self,
        source: VertexKey,
        vertex: VertexAttr<P::V>,
    ) -> VertexKey {
        let created = self.add_vertex(vertex);
        self.gmap
            .record_edit_event(EditEvent::VertexSplit { source, created });
        created
    }

    /// Stages an edge attribute for reconciliation at commit.
    pub fn add_edge(&mut self, edge: EdgeAttr<P::E>) -> EdgeKey {
        let key = self.gmap.edges.insert(edge);
        self.gmap.invalidate_derived_indexes();
        self.gmap.record_edit_event(EditEvent::Created {
            key: EditKey::Edge(key),
        });
        key
    }

    /// Stages an edge created by explicitly splitting an existing edge.
    pub fn add_edge_split_from(&mut self, source: EdgeKey, edge: EdgeAttr<P::E>) -> EdgeKey {
        let created = self.add_edge(edge);
        self.gmap
            .record_edit_event(EditEvent::EdgeSplit { source, created });
        created
    }

    /// Stages a profile attribute for reconciliation at commit.
    pub fn add_profile(&mut self, profile: ProfileAttr<P::Profile>) -> ProfileKey {
        let key = self.gmap.profiles.insert(profile);
        self.gmap.invalidate_derived_indexes();
        self.gmap.record_edit_event(EditEvent::Created {
            key: EditKey::Profile(key),
        });
        key
    }

    /// Stages a profile created by explicitly splitting an existing profile.
    pub fn add_profile_split_from(
        &mut self,
        source: ProfileKey,
        profile: ProfileAttr<P::Profile>,
    ) -> ProfileKey {
        let created = self.add_profile(profile);
        self.gmap
            .record_edit_event(EditEvent::ProfileSplit { source, created });
        created
    }

    /// Stages a face attribute for reconciliation at commit.
    pub fn add_face(&mut self, face: FaceAttr<P::F>) -> FaceKey {
        let key = self.gmap.faces.insert(face);
        self.gmap.invalidate_derived_indexes();
        self.gmap.record_edit_event(EditEvent::Created {
            key: EditKey::Face(key),
        });
        key
    }

    /// Stages a face created by explicitly splitting an existing face.
    pub fn add_face_split_from(&mut self, source: FaceKey, face: FaceAttr<P::F>) -> FaceKey {
        let created = self.add_face(face);
        self.gmap
            .record_edit_event(EditEvent::FaceSplit { source, created });
        created
    }

    /// Stages a sheet attribute for reconciliation at commit.
    pub fn add_sheet(&mut self, sheet: SheetAttr<P::Sheet>) -> SheetKey {
        let key = self.gmap.sheets.insert(sheet);
        self.gmap.invalidate_derived_indexes();
        self.gmap.record_edit_event(EditEvent::Created {
            key: EditKey::Sheet(key),
        });
        key
    }

    /// Stages a sheet created by explicitly splitting an existing sheet.
    pub fn add_sheet_split_from(
        &mut self,
        source: SheetKey,
        sheet: SheetAttr<P::Sheet>,
    ) -> SheetKey {
        let created = self.add_sheet(sheet);
        self.gmap
            .record_edit_event(EditEvent::SheetSplit { source, created });
        created
    }

    /// Stages a solid attribute for reconciliation at commit.
    pub fn add_solid(&mut self, solid: SolidAttr<P::S>) -> SolidKey {
        let key = self.gmap.solids.insert(solid);
        self.gmap.invalidate_derived_indexes();
        self.gmap.record_edit_event(EditEvent::Created {
            key: EditKey::Solid(key),
        });
        key
    }

    /// Stages a solid created by explicitly splitting an existing solid.
    pub fn add_solid_split_from(&mut self, source: SolidKey, solid: SolidAttr<P::S>) -> SolidKey {
        let created = self.add_solid(solid);
        self.gmap
            .record_edit_event(EditEvent::SolidSplit { source, created });
        created
    }

    /// Declares that `removed` merged into `survivor`.
    pub fn merge_vertices_into(&mut self, survivor: VertexKey, removed: VertexKey) {
        self.gmap
            .record_edit_event(EditEvent::VertexMerge { survivor, removed });
    }

    /// Declares that `removed` merged into `survivor`.
    pub fn merge_edges_into(&mut self, survivor: EdgeKey, removed: EdgeKey) {
        self.gmap
            .record_edit_event(EditEvent::EdgeMerge { survivor, removed });
    }

    /// Declares that `removed` merged into `survivor`.
    pub fn merge_profiles_into(&mut self, survivor: ProfileKey, removed: ProfileKey) {
        self.gmap
            .record_edit_event(EditEvent::ProfileMerge { survivor, removed });
    }

    /// Declares that `removed` merged into `survivor`.
    pub fn merge_faces_into(&mut self, survivor: FaceKey, removed: FaceKey) {
        self.gmap
            .record_edit_event(EditEvent::FaceMerge { survivor, removed });
    }

    /// Declares that `removed` merged into `survivor`.
    pub fn merge_sheets_into(&mut self, survivor: SheetKey, removed: SheetKey) {
        self.gmap
            .record_edit_event(EditEvent::SheetMerge { survivor, removed });
    }

    /// Declares that `removed` merged into `survivor`.
    pub fn merge_solids_into(&mut self, survivor: SolidKey, removed: SolidKey) {
        self.gmap
            .record_edit_event(EditEvent::SolidMerge { survivor, removed });
    }

    /// Deletes face loops and orphaned lower-dimensional cells in one compaction pass.
    /// Sheet/solid registrations rooted in the deleted set must first be removed or moved.
    /// All cached darts are invalid afterwards; resolve surviving cells from their keys.
    pub fn remove_faces(&mut self, faces: &[FaceKey]) -> Result<(), TopologyEditError> {
        let mut removed = HashSet::new();
        for &key in faces {
            let face = self
                .gmap
                .face(key)
                .ok_or(TopologyEditError::MissingLineageAttribute {
                    key: EditKey::Face(key),
                })?;
            for boundary in face.loops() {
                removed.extend(boundary.darts());
            }
        }
        for root in self.gmap.iter_sheets().map(|(_, attr)| attr.dart).chain(
            self.gmap.iter_solids().flat_map(|(_, attr)| {
                std::iter::once(attr.outer_shell).chain(attr.inner_shells.iter().flatten().copied())
            }),
        ) {
            if removed.contains(&root) {
                return Err(TopologyEditError::ReferencedDartDeletion { dart: root });
            }
        }
        let edges = self
            .gmap
            .iter_edges()
            .map(|(key, attr)| {
                let start = self.gmap.cell_key::<super::gmap::Cell0>(attr.dart);
                let replacement = self
                    .gmap
                    .orbit(attr.dart, self.gmap.orbit_indices(Dim::One))
                    .find(|dart| {
                        !removed.contains(dart)
                            && self.gmap.cell_key::<super::gmap::Cell0>(*dart) == start
                    });
                (key, replacement)
            })
            .collect::<Vec<_>>();
        let vertices = self
            .gmap
            .iter_vertices()
            .map(|(key, attr)| {
                (
                    key,
                    self.gmap
                        .orbit(attr.dart, self.gmap.orbit_indices(Dim::Zero))
                        .find(|dart| !removed.contains(dart)),
                )
            })
            .collect::<Vec<_>>();
        let profiles = self
            .gmap
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
    pub fn remove_vertex(&mut self, key: VertexKey) -> Option<VertexAttr<P::V>> {
        let removed = self.gmap.vertices.remove(key);
        self.gmap.invalidate_derived_indexes();
        removed
    }

    /// Removes an edge attribute inside the transaction.
    pub fn remove_edge(&mut self, key: EdgeKey) -> Option<EdgeAttr<P::E>> {
        let removed = self.gmap.edges.remove(key);
        self.gmap.invalidate_derived_indexes();
        removed
    }

    /// Removes a profile attribute inside the transaction.
    pub fn remove_profile(&mut self, key: ProfileKey) -> Option<ProfileAttr<P::Profile>> {
        let removed = self.gmap.profiles.remove(key);
        self.gmap.invalidate_derived_indexes();
        removed
    }

    /// Removes a face attribute inside the transaction.
    pub fn remove_face(&mut self, key: FaceKey) -> Option<FaceAttr<P::F>> {
        let removed = self.gmap.faces.remove(key);
        self.gmap.invalidate_derived_indexes();
        removed
    }

    /// Removes a sheet attribute inside the transaction.
    pub fn remove_sheet(&mut self, key: SheetKey) -> Option<SheetAttr<P::Sheet>> {
        let removed = self.gmap.sheets.remove(key);
        self.gmap.invalidate_derived_indexes();
        removed
    }

    /// Removes a solid attribute inside the transaction.
    pub fn remove_solid(&mut self, key: SolidKey) -> Option<SolidAttr<P::S>> {
        let removed = self.gmap.solids.remove(key);
        self.gmap.invalidate_derived_indexes();
        removed
    }

    /// Returns mutable access to a staged vertex attribute.
    pub fn vertex_attr_mut(&mut self, key: VertexKey) -> Option<&mut VertexAttr<P::V>> {
        self.gmap.invalidate_derived_indexes();
        self.gmap.vertices.get_mut(key)
    }

    /// Returns mutable access to a staged vertex attribute, or panics if absent.
    pub fn vertex_attr_mut_unchecked(&mut self, key: VertexKey) -> &mut VertexAttr<P::V> {
        self.vertex_attr_mut(key)
            .expect("vertex attribute should be in the map")
    }

    /// Returns mutable access to a staged edge attribute.
    pub fn edge_attr_mut(&mut self, key: EdgeKey) -> Option<&mut EdgeAttr<P::E>> {
        self.gmap.invalidate_derived_indexes();
        self.gmap.edges.get_mut(key)
    }

    /// Returns mutable access to a staged edge attribute, or panics if absent.
    pub fn edge_attr_mut_unchecked(&mut self, key: EdgeKey) -> &mut EdgeAttr<P::E> {
        self.edge_attr_mut(key)
            .expect("edge attribute should be in the map")
    }

    /// Returns mutable access to a staged profile attribute.
    pub fn profile_attr_mut(&mut self, key: ProfileKey) -> Option<&mut ProfileAttr<P::Profile>> {
        self.gmap.invalidate_derived_indexes();
        self.gmap.profiles.get_mut(key)
    }

    /// Returns mutable access to a staged profile attribute, or panics if absent.
    pub fn profile_attr_mut_unchecked(&mut self, key: ProfileKey) -> &mut ProfileAttr<P::Profile> {
        self.profile_attr_mut(key)
            .expect("profile attribute should be in the map")
    }

    /// Returns mutable access to a staged face attribute.
    pub fn face_attr_mut(&mut self, key: FaceKey) -> Option<&mut FaceAttr<P::F>> {
        self.gmap.invalidate_derived_indexes();
        self.gmap.faces.get_mut(key)
    }

    /// Returns mutable access to a staged face attribute, or panics if absent.
    pub fn face_attr_mut_unchecked(&mut self, key: FaceKey) -> &mut FaceAttr<P::F> {
        self.face_attr_mut(key)
            .expect("face attribute should be in the map")
    }

    /// Returns mutable access to a staged sheet attribute.
    pub fn sheet_attr_mut(&mut self, key: SheetKey) -> Option<&mut SheetAttr<P::Sheet>> {
        self.gmap.invalidate_derived_indexes();
        self.gmap.sheets.get_mut(key)
    }

    /// Returns mutable access to a staged sheet attribute, or panics if absent.
    pub fn sheet_attr_mut_unchecked(&mut self, key: SheetKey) -> &mut SheetAttr<P::Sheet> {
        self.sheet_attr_mut(key)
            .expect("sheet attribute should be in the map")
    }

    /// Returns mutable access to a staged solid attribute.
    pub fn solid_attr_mut(&mut self, key: SolidKey) -> Option<&mut SolidAttr<P::S>> {
        self.gmap.invalidate_derived_indexes();
        self.gmap.solids.get_mut(key)
    }

    /// Returns mutable access to a staged solid attribute, or panics if absent.
    pub fn solid_attr_mut_unchecked(&mut self, key: SolidKey) -> &mut SolidAttr<P::S> {
        self.solid_attr_mut(key)
            .expect("solid attribute should be in the map")
    }

    fn validate_dart(&self, dart: Dart) -> Result<(), TopologyEditError> {
        (dart.id() < self.gmap.dart_count())
            .then_some(())
            .ok_or(TopologyEditError::MissingDart { dart })
    }
}

impl<P: Payload> Deref for TopologyEdit<'_, P> {
    type Target = GMap<P>;

    fn deref(&self) -> &Self::Target {
        self.gmap
    }
}

fn validate_required_domain_attributes<P: Payload>(g: &GMap<P>) -> Result<(), TopologyEditError> {
    for (face, attr) in g.faces.iter() {
        for dart in std::iter::once(attr.outer_loop).chain(attr.inner_loops.iter().copied()) {
            if g.profile_key(dart).is_none() {
                return Err(TopologyEditError::MissingProfileRegistration { face, dart });
            }
        }
    }

    for (solid, attr) in g.solids.iter() {
        for dart in
            std::iter::once(attr.outer_shell).chain(attr.inner_shells.iter().flatten().copied())
        {
            if g.sheet_key(dart).is_none() {
                return Err(TopologyEditError::MissingSheetRegistration { solid, dart });
            }
        }
    }

    Ok(())
}

/// Validates and reconciles all staged work, then applies net payload events.
///
/// The caller owns rollback, so this function only mutates the staged map and
/// returns the first commit error it encounters.
pub(super) fn commit_topology_transaction<P, Q>(
    g: &mut GMap<P>,
    snapshot: &GMap<P>,
    events: &[EditEvent],
    policy: &mut Q,
) -> Result<(), TopologyEditError>
where
    P: Payload,
    Q: EditPolicy<P>,
{
    validate_gmap(g).map_err(TopologyEditError::InvalidTopology)?;
    validate_required_domain_attributes(g)?;
    validate_edit_events(g, snapshot, events)?;
    let lineage = TransactionLineage::new(g, snapshot, events);
    reconcile_transaction_attributes(g, snapshot, events, &lineage)?;
    canonicalize_vertex_darts(g);
    g.invalidate_derived_indexes();
    let policy_events = resolve_policy_events(g, snapshot, events, &lineage);
    apply_policy_events(g, snapshot, &policy_events, policy)?;
    g.materialize_derived_indexes();
    Ok(())
}

/// Rejects malformed lineage before reconciliation can consume any attributes.
fn validate_edit_events<P: Payload>(
    g: &GMap<P>,
    snapshot: &GMap<P>,
    events: &[EditEvent],
) -> Result<(), TopologyEditError> {
    let mut merges = HashMap::new();

    for event in events {
        if matches!(event, EditEvent::Created { .. }) {
            // Transaction-local identities may be explicitly discarded by a
            // later builder pass. Their creation record still determines
            // ordering, but they need not survive until commit.
            continue;
        }

        if let Some((source, created)) = event.split_keys() {
            let source_was_created = events
                .iter()
                .any(|candidate| matches!(candidate, EditEvent::Created { key } if *key == source));
            if !contains_edit_key(g, source)
                && !contains_edit_key(snapshot, source)
                && !source_was_created
            {
                return Err(TopologyEditError::MissingLineageAttribute { key: source });
            }
            // A split identity consumed by a later pass is transient and does
            // not need to remain in the final attribute stores.
            let _ = created;
            continue;
        }

        // A merge both of whose identities are gone was spent: a later pass in
        // the same operation removed the cell they had just come to share, so
        // there is nothing left to reconcile or to hand the payload policy.
        if is_spent_merge(g, *event) {
            continue;
        }

        let (first, second) = event.keys();
        for key in std::iter::once(first).chain(second) {
            if !contains_edit_key(g, key) {
                return Err(TopologyEditError::MissingLineageAttribute { key });
            }
        }

        let Some((survivor, removed)) = event.merge_keys() else {
            continue;
        };
        if survivor == removed {
            return Err(TopologyEditError::InvalidMerge { survivor, removed });
        }
        if merges.insert(removed, survivor).is_some() {
            return Err(TopologyEditError::RepeatedMerge { removed });
        }
    }

    for &start in merges.keys() {
        let mut visited = HashSet::new();
        let mut current = start;
        while let Some(&next) = merges.get(&current) {
            if !visited.insert(current) {
                return Err(TopologyEditError::MergeCycle { key: current });
            }
            current = next;
        }
    }

    Ok(())
}

impl EditEvent {
    /// Returns the generic attribute keys carried by any event variant.
    pub(super) fn keys(self) -> (EditKey, Option<EditKey>) {
        match self {
            Self::Created { key } => (key, None),
            Self::VertexSplit { source, created } => {
                (EditKey::Vertex(source), Some(EditKey::Vertex(created)))
            }
            Self::EdgeSplit { source, created } => {
                (EditKey::Edge(source), Some(EditKey::Edge(created)))
            }
            Self::ProfileSplit { source, created } => {
                (EditKey::Profile(source), Some(EditKey::Profile(created)))
            }
            Self::FaceSplit { source, created } => {
                (EditKey::Face(source), Some(EditKey::Face(created)))
            }
            Self::SheetSplit { source, created } => {
                (EditKey::Sheet(source), Some(EditKey::Sheet(created)))
            }
            Self::SolidSplit { source, created } => {
                (EditKey::Solid(source), Some(EditKey::Solid(created)))
            }
            Self::VertexMerge { survivor, removed } => {
                (EditKey::Vertex(survivor), Some(EditKey::Vertex(removed)))
            }
            Self::EdgeMerge { survivor, removed } => {
                (EditKey::Edge(survivor), Some(EditKey::Edge(removed)))
            }
            Self::ProfileMerge { survivor, removed } => {
                (EditKey::Profile(survivor), Some(EditKey::Profile(removed)))
            }
            Self::FaceMerge { survivor, removed } => {
                (EditKey::Face(survivor), Some(EditKey::Face(removed)))
            }
            Self::SheetMerge { survivor, removed } => {
                (EditKey::Sheet(survivor), Some(EditKey::Sheet(removed)))
            }
            Self::SolidMerge { survivor, removed } => {
                (EditKey::Solid(survivor), Some(EditKey::Solid(removed)))
            }
        }
    }

    /// Extracts a split's source and created identity, independent of cell type.
    fn split_keys(self) -> Option<(EditKey, EditKey)> {
        match self {
            Self::VertexSplit { source, created } => {
                Some((EditKey::Vertex(source), EditKey::Vertex(created)))
            }
            Self::EdgeSplit { source, created } => {
                Some((EditKey::Edge(source), EditKey::Edge(created)))
            }
            Self::ProfileSplit { source, created } => {
                Some((EditKey::Profile(source), EditKey::Profile(created)))
            }
            Self::FaceSplit { source, created } => {
                Some((EditKey::Face(source), EditKey::Face(created)))
            }
            Self::SheetSplit { source, created } => {
                Some((EditKey::Sheet(source), EditKey::Sheet(created)))
            }
            Self::SolidSplit { source, created } => {
                Some((EditKey::Solid(source), EditKey::Solid(created)))
            }
            _ => None,
        }
    }

    /// Extracts a merge's survivor and consumed identity, independent of cell type.
    pub(super) fn merge_keys(self) -> Option<(EditKey, EditKey)> {
        match self {
            Self::VertexMerge { survivor, removed } => {
                Some((EditKey::Vertex(survivor), EditKey::Vertex(removed)))
            }
            Self::EdgeMerge { survivor, removed } => {
                Some((EditKey::Edge(survivor), EditKey::Edge(removed)))
            }
            Self::ProfileMerge { survivor, removed } => {
                Some((EditKey::Profile(survivor), EditKey::Profile(removed)))
            }
            Self::FaceMerge { survivor, removed } => {
                Some((EditKey::Face(survivor), EditKey::Face(removed)))
            }
            Self::SheetMerge { survivor, removed } => {
                Some((EditKey::Sheet(survivor), EditKey::Sheet(removed)))
            }
            Self::SolidMerge { survivor, removed } => {
                Some((EditKey::Solid(survivor), EditKey::Solid(removed)))
            }
            _ => None,
        }
    }
}

/// Reports whether a merge declaration has nothing left to reconcile.
///
/// Both identities are absent exactly when a later pass removed the cell they
/// were merging into. The declaration is then inert: it names no surviving
/// identity, consumes nothing, and reaches no payload policy.
fn is_spent_merge<P: Payload>(g: &GMap<P>, event: EditEvent) -> bool {
    event.merge_keys().is_some_and(|(survivor, removed)| {
        !contains_edit_key(g, survivor) && !contains_edit_key(g, removed)
    })
}

/// Checks the appropriate attribute store for a type-erased edit key.
fn contains_edit_key<P: Payload>(g: &GMap<P>, key: EditKey) -> bool {
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
    fn new<P: Payload>(g: &GMap<P>, snapshot: &GMap<P>, events: &[EditEvent]) -> Self {
        let mut origins = HashMap::new();
        let mut creation_order = HashMap::new();
        let mut merges = HashMap::new();

        for (order, event) in events.iter().enumerate() {
            match *event {
                EditEvent::Created { key } => {
                    origins.entry(key).or_insert(CreationOrigin::Fresh);
                    creation_order.entry(key).or_insert(order);
                }
                _ => {
                    if let Some((source, created)) = event.split_keys() {
                        origins.insert(created, CreationOrigin::SplitFrom(source));
                        creation_order.entry(created).or_insert(order);
                    }
                    if let Some((survivor, removed)) = event.merge_keys() {
                        merges.insert(removed, survivor);
                    }
                }
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
    g: &GMap<P>,
    snapshot: &GMap<P>,
    events: &[EditEvent],
    lineage: &TransactionLineage,
) -> Vec<PolicyEvent> {
    events
        .iter()
        .filter_map(|event| {
            if let Some((source, created)) = event.split_keys() {
                if !contains_edit_key(g, created) {
                    return None;
                }
                return transaction_start_origin(snapshot, &lineage.origins, source)
                    .map(|source| PolicyEvent::Split { source, created });
            }

            event.merge_keys().and_then(|(survivor, removed)| {
                let survivor = final_survivor(&lineage.merges, survivor);
                // A spent merge has no surviving payload to merge into.
                (contains_edit_key(snapshot, removed) && contains_edit_key(g, survivor))
                    .then_some(PolicyEvent::Merge { survivor, removed })
            })
        })
        .collect()
}

/// Collects all topology-associated attribute keys currently stored by the map.
fn current_edit_keys<P: Payload>(g: &GMap<P>) -> Vec<EditKey> {
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
    snapshot: &GMap<P>,
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
fn reconcile_transaction_attributes<P: Payload>(
    g: &mut GMap<P>,
    snapshot: &GMap<P>,
    events: &[EditEvent],
    lineage: &TransactionLineage,
) -> Result<(), TopologyEditError> {
    let spent = events
        .iter()
        .filter(|event| is_spent_merge(g, **event))
        .filter_map(|event| event.merge_keys())
        .map(|(survivor, _)| survivor)
        .collect::<HashSet<_>>();
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

    let profiles = g
        .profiles
        .iter()
        .map(|(key, attr)| (key, vec![g.profile_representative(attr.dart)]))
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
            let representatives = std::iter::once(attr.outer_loop)
                .chain(attr.inner_loops.iter().copied())
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
        .map(|(key, attr)| (key, vec![g.cell_representative(attr.dart, Dim::Three)]))
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
            let representatives = std::iter::once(attr.outer_shell)
                .chain(attr.inner_shells.iter().flatten().copied())
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
            return Err(TopologyEditError::InvalidLineageSurvivor { survivor });
        }
    }

    Ok(())
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
    g: &mut GMap<P>,
    snapshot: &GMap<P>,
    lineage: &TransactionLineage,
    entity: &'static str,
    components: Vec<(Dart, Vec<K>)>,
    edit_key: F,
) -> Result<(), TopologyEditError>
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
            return Err(TopologyEditError::UnresolvedPreExistingCollision {
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
fn remove_edit_key<P: Payload>(g: &mut GMap<P>, key: EditKey) {
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
fn remove_consumed_attributes<P: Payload>(g: &mut GMap<P>, events: &[EditEvent]) {
    for event in events {
        match *event {
            EditEvent::VertexMerge { removed, .. } => {
                g.vertices.remove(removed);
            }
            EditEvent::EdgeMerge { removed, .. } => {
                g.edges.remove(removed);
            }
            EditEvent::ProfileMerge { removed, .. } => {
                g.profiles.remove(removed);
            }
            EditEvent::FaceMerge { removed, .. } => {
                g.faces.remove(removed);
            }
            EditEvent::SheetMerge { removed, .. } => {
                g.sheets.remove(removed);
            }
            EditEvent::SolidMerge { removed, .. } => {
                g.solids.remove(removed);
            }
            _ => {}
        }
    }
}

/// Applies net split and merge policy calls in journal order using snapshot inputs.
///
/// Source and removed payloads always come from the transaction-start snapshot;
/// only surviving staged payloads are mutated.
fn apply_policy_events<P, Q>(
    g: &mut GMap<P>,
    snapshot: &GMap<P>,
    events: &[PolicyEvent],
    policy: &mut Q,
) -> Result<(), TopologyEditError>
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
                    .map_err(|error| TopologyEditError::Policy(Box::new(error)))?;
            }
            PolicyEvent::Split {
                source: EditKey::Edge(source),
                created: EditKey::Edge(created),
            } => {
                let source_data = snapshot.edge_attr_unchecked(source).data.clone();
                let created_data = &mut g.edge_attr_mut_unchecked(created).data;
                policy
                    .split_edge_data(source, &source_data, created, created_data)
                    .map_err(|error| TopologyEditError::Policy(Box::new(error)))?;
            }
            PolicyEvent::Split {
                source: EditKey::Profile(source),
                created: EditKey::Profile(created),
            } => {
                let source_data = snapshot.profile_attr_unchecked(source).data.clone();
                let created_data = &mut g.profile_attr_mut_unchecked(created).data;
                policy
                    .split_profile_data(source, &source_data, created, created_data)
                    .map_err(|error| TopologyEditError::Policy(Box::new(error)))?;
            }
            PolicyEvent::Split {
                source: EditKey::Face(source),
                created: EditKey::Face(created),
            } => {
                let source_data = snapshot.face_attr_unchecked(source).data.clone();
                let created_data = &mut g.face_attr_mut_unchecked(created).data;
                policy
                    .split_face_data(source, &source_data, created, created_data)
                    .map_err(|error| TopologyEditError::Policy(Box::new(error)))?;
            }
            PolicyEvent::Split {
                source: EditKey::Sheet(source),
                created: EditKey::Sheet(created),
            } => {
                let source_data = snapshot.sheet_attr_unchecked(source).data.clone();
                let created_data = &mut g.sheet_attr_mut_unchecked(created).data;
                policy
                    .split_sheet_data(source, &source_data, created, created_data)
                    .map_err(|error| TopologyEditError::Policy(Box::new(error)))?;
            }
            PolicyEvent::Split {
                source: EditKey::Solid(source),
                created: EditKey::Solid(created),
            } => {
                let source_data = snapshot.solid_attr_unchecked(source).data.clone();
                let created_data = &mut g.solid_attr_mut_unchecked(created).data;
                policy
                    .split_solid_data(source, &source_data, created, created_data)
                    .map_err(|error| TopologyEditError::Policy(Box::new(error)))?;
            }
            PolicyEvent::Merge {
                survivor: EditKey::Vertex(survivor),
                removed: EditKey::Vertex(removed),
            } => {
                let removed_data = snapshot.vertex_attr_unchecked(removed).data.clone();
                let survivor_data = &mut g.vertex_attr_mut_unchecked(survivor).data;
                policy
                    .merge_vertex_data(survivor, survivor_data, removed, removed_data)
                    .map_err(|error| TopologyEditError::Policy(Box::new(error)))?;
            }
            PolicyEvent::Merge {
                survivor: EditKey::Edge(survivor),
                removed: EditKey::Edge(removed),
            } => {
                let removed_data = snapshot.edge_attr_unchecked(removed).data.clone();
                let survivor_data = &mut g.edge_attr_mut_unchecked(survivor).data;
                policy
                    .merge_edge_data(survivor, survivor_data, removed, removed_data)
                    .map_err(|error| TopologyEditError::Policy(Box::new(error)))?;
            }
            PolicyEvent::Merge {
                survivor: EditKey::Profile(survivor),
                removed: EditKey::Profile(removed),
            } => {
                let removed_data = snapshot.profile_attr_unchecked(removed).data.clone();
                let survivor_data = &mut g.profile_attr_mut_unchecked(survivor).data;
                policy
                    .merge_profile_data(survivor, survivor_data, removed, removed_data)
                    .map_err(|error| TopologyEditError::Policy(Box::new(error)))?;
            }
            PolicyEvent::Merge {
                survivor: EditKey::Face(survivor),
                removed: EditKey::Face(removed),
            } => {
                let removed_data = snapshot.face_attr_unchecked(removed).data.clone();
                let survivor_data = &mut g.face_attr_mut_unchecked(survivor).data;
                policy
                    .merge_face_data(survivor, survivor_data, removed, removed_data)
                    .map_err(|error| TopologyEditError::Policy(Box::new(error)))?;
            }
            PolicyEvent::Merge {
                survivor: EditKey::Sheet(survivor),
                removed: EditKey::Sheet(removed),
            } => {
                let removed_data = snapshot.sheet_attr_unchecked(removed).data.clone();
                let survivor_data = &mut g.sheet_attr_mut_unchecked(survivor).data;
                policy
                    .merge_sheet_data(survivor, survivor_data, removed, removed_data)
                    .map_err(|error| TopologyEditError::Policy(Box::new(error)))?;
            }
            PolicyEvent::Merge {
                survivor: EditKey::Solid(survivor),
                removed: EditKey::Solid(removed),
            } => {
                let removed_data = snapshot.solid_attr_unchecked(removed).data.clone();
                let survivor_data = &mut g.solid_attr_mut_unchecked(survivor).data;
                policy
                    .merge_solid_data(survivor, survivor_data, removed, removed_data)
                    .map_err(|error| TopologyEditError::Policy(Box::new(error)))?;
            }
            _ => unreachable!("edit lineage always preserves the attribute type"),
        }
    }
    Ok(())
}

/// Stores each surviving vertex attribute on its final canonical 0-cell dart.
fn canonicalize_vertex_darts<P: Payload>(g: &mut GMap<P>) {
    let canonical_darts = g
        .vertices
        .iter()
        .map(|(key, attr)| (key, g.cell_representative(attr.dart, Dim::Zero)))
        .collect::<Vec<_>>();
    for (key, dart) in canonical_darts {
        g.vertices[key].dart = dart;
    }
}
