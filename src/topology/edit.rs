use std::collections::HashSet;
use std::convert::Infallible;
use std::error::Error;

use thiserror::Error;

use super::Dart;
use super::attributes::{EdgeAttr, FaceAttr, ProfileAttr, SheetAttr, SolidAttr, VertexAttr};
use super::gmap::{Dim, GMap};
use super::payload::Payload;
use super::shape_keys::{EdgeKey, FaceKey, ProfileKey, SheetKey, SolidKey, VertexKey};
use super::validation::{GMapValidationError, validate_gmap};

/// Policy boundary for topology edits.
///
/// Automatic payload merge/split inference is intentionally not performed by
/// commit. Future explicit edit APIs will use this policy boundary when a
/// builder declares semantic lineage itself.
pub trait EditPolicy<P: Payload> {
    /// Error returned when the policy rejects an edit.
    type Error: Error + Send + Sync + 'static;
}

/// Default edit policy.
#[derive(Debug, Clone, Copy, Default)]
pub struct PreservePayload;

impl<P: Payload> EditPolicy<P> for PreservePayload {
    type Error = Infallible;
}

/// Failure raised while applying a safe topology mutation.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum TopologyEditError {
    /// A dart does not exist in the edited map.
    #[error("dart {dart:?} does not exist")]
    MissingDart { dart: Dart },
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
}

/// Failure raised while reconciling an edit through a custom policy.
#[derive(Debug, Error)]
pub enum TopologyCommitError<E: Error + Send + Sync + 'static> {
    /// The edited alpha relations do not satisfy the GMap axioms.
    #[error("topology edit produced an invalid GMap")]
    InvalidTopology(#[source] GMapValidationError),
    /// The payload policy rejected a merge or split.
    #[error("topology edit policy rejected the edit")]
    Policy(#[source] E),
}

/// Failure raised while running and committing an edit closure.
#[derive(Debug, Error)]
pub enum TopologyTransactionError<
    E: Error + Send + Sync + 'static,
    P: Error + Send + Sync + 'static,
> {
    /// The edit closure rejected or failed an operation.
    #[error("topology edit operation failed")]
    Operation(#[source] E),
    /// The staged topology or payload reconciliation failed at commit.
    #[error("topology edit commit failed")]
    Commit(#[source] TopologyCommitError<P>),
}

/// Clone-backed topology transaction.
///
/// Dropping this value without a successful [`commit`](Self::commit) restores
/// the complete map snapshot taken by [`GMap::edit`].
pub struct TopologyEdit<'g, P: Payload> {
    gmap: &'g mut GMap<P>,
    backup: Option<GMap<P>>,
}

impl<'g, P: Payload> TopologyEdit<'g, P> {
    pub(super) fn new(gmap: &'g mut GMap<P>) -> Self {
        Self {
            backup: Some(gmap.clone()),
            gmap,
        }
    }

    /// Adds an isolated dart inside the transaction.
    pub fn add_dart(&mut self) -> Dart {
        self.gmap.add_dart()
    }

    /// Removes a dart whose isolation has been proven by the caller.
    pub fn remove_dart(&mut self, dart: super::IsolatedDart) {
        self.gmap.remove_dart(dart);
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
    pub fn add_vertex(&mut self, vertex: VertexAttr<P::V>) -> VertexKey {
        self.gmap.vertices.insert(vertex)
    }

    /// Stages an edge attribute for reconciliation at commit.
    pub fn add_edge(&mut self, edge: EdgeAttr<P::E>) -> EdgeKey {
        self.gmap.edges.insert(edge)
    }

    /// Stages a profile attribute for reconciliation at commit.
    pub fn add_profile(&mut self, profile: ProfileAttr<P::Profile>) -> ProfileKey {
        self.gmap.profiles.insert(profile)
    }

    /// Stages a face attribute for reconciliation at commit.
    pub fn add_face(&mut self, face: FaceAttr<P::F>) -> FaceKey {
        self.gmap.faces.insert(face)
    }

    /// Stages a sheet attribute for reconciliation at commit.
    pub fn add_sheet(&mut self, sheet: SheetAttr<P::Sheet>) -> SheetKey {
        self.gmap.sheets.insert(sheet)
    }

    /// Stages a solid attribute for reconciliation at commit.
    pub fn add_solid(&mut self, solid: SolidAttr<P::S>) -> SolidKey {
        self.gmap.solids.insert(solid)
    }

    /// Removes a vertex attribute inside the transaction.
    pub fn remove_vertex(&mut self, key: VertexKey) -> Option<VertexAttr<P::V>> {
        self.gmap.vertices.remove(key)
    }

    /// Removes an edge attribute inside the transaction.
    pub fn remove_edge(&mut self, key: EdgeKey) -> Option<EdgeAttr<P::E>> {
        self.gmap.edges.remove(key)
    }

    /// Removes a profile attribute inside the transaction.
    pub fn remove_profile(&mut self, key: ProfileKey) -> Option<ProfileAttr<P::Profile>> {
        self.gmap.profiles.remove(key)
    }

    /// Removes a face attribute inside the transaction.
    pub fn remove_face(&mut self, key: FaceKey) -> Option<FaceAttr<P::F>> {
        self.gmap.faces.remove(key)
    }

    /// Removes a sheet attribute inside the transaction.
    pub fn remove_sheet(&mut self, key: SheetKey) -> Option<SheetAttr<P::Sheet>> {
        self.gmap.sheets.remove(key)
    }

    /// Removes a solid attribute inside the transaction.
    pub fn remove_solid(&mut self, key: SolidKey) -> Option<SolidAttr<P::S>> {
        self.gmap.solids.remove(key)
    }

    /// Returns mutable access to a staged vertex attribute.
    pub fn vertex_attr_mut(&mut self, key: VertexKey) -> Option<&mut VertexAttr<P::V>> {
        self.gmap.vertices.get_mut(key)
    }

    /// Returns mutable access to a staged edge attribute.
    pub fn edge_attr_mut(&mut self, key: EdgeKey) -> Option<&mut EdgeAttr<P::E>> {
        self.gmap.edges.get_mut(key)
    }

    /// Returns mutable access to a staged profile attribute.
    pub fn profile_attr_mut(&mut self, key: ProfileKey) -> Option<&mut ProfileAttr<P::Profile>> {
        self.gmap.profiles.get_mut(key)
    }

    /// Returns mutable access to a staged face attribute.
    pub fn face_attr_mut(&mut self, key: FaceKey) -> Option<&mut FaceAttr<P::F>> {
        self.gmap.faces.get_mut(key)
    }

    /// Returns mutable access to a staged sheet attribute.
    pub fn sheet_attr_mut(&mut self, key: SheetKey) -> Option<&mut SheetAttr<P::Sheet>> {
        self.gmap.sheets.get_mut(key)
    }

    /// Returns mutable access to a staged solid attribute.
    pub fn solid_attr_mut(&mut self, key: SolidKey) -> Option<&mut SolidAttr<P::S>> {
        self.gmap.solids.get_mut(key)
    }

    /// Rebuilds derived indexes, then makes the edit permanent.
    pub(super) fn commit<Q>(mut self, _policy: &mut Q) -> Result<(), TopologyCommitError<Q::Error>>
    where
        Q: EditPolicy<P>,
    {
        validate_gmap(self.gmap).map_err(TopologyCommitError::InvalidTopology)?;
        ensure_required_domain_attributes(self.gmap);
        rebuild_vertex_index(self.gmap);
        rebuild_edge_index(self.gmap);
        rebuild_profile_index(self.gmap);
        rebuild_face_index(self.gmap);
        rebuild_sheet_index(self.gmap);
        rebuild_solid_index(self.gmap);
        self.backup = None;
        Ok(())
    }

    fn validate_dart(&self, dart: Dart) -> Result<(), TopologyEditError> {
        (dart.id() < self.gmap.dart_count())
            .then_some(())
            .ok_or(TopologyEditError::MissingDart { dart })
    }
}

fn ensure_required_domain_attributes<P: Payload>(g: &mut GMap<P>) {
    let mut profile_components = g
        .profiles
        .iter()
        .map(|(_, attr)| g.profile_representative(attr.dart))
        .collect::<HashSet<_>>();
    let face_loops = g
        .faces
        .iter()
        .flat_map(|(_, attr)| {
            std::iter::once(attr.outer_loop).chain(attr.inner_loops.iter().copied())
        })
        .collect::<Vec<_>>();
    for dart in face_loops {
        let repr = g.profile_representative(dart);
        if profile_components.insert(repr) {
            g.profiles
                .insert(ProfileAttr::new(dart, P::Profile::default()));
        }
    }

    let mut sheet_components = g
        .sheets
        .iter()
        .map(|(_, attr)| g.cell_representative(attr.dart, Dim::Three))
        .collect::<HashSet<_>>();
    let solid_shells = g
        .solids
        .iter()
        .flat_map(|(_, attr)| {
            std::iter::once(attr.outer_shell).chain(attr.inner_shells.iter().flatten().copied())
        })
        .collect::<Vec<_>>();
    for dart in solid_shells {
        let repr = g.cell_representative(dart, Dim::Three);
        if sheet_components.insert(repr) {
            g.sheets.insert(SheetAttr::new(dart, P::Sheet::default()));
        }
    }
}

impl<P: Payload> Drop for TopologyEdit<'_, P> {
    fn drop(&mut self) {
        if let Some(backup) = self.backup.take() {
            *self.gmap = backup;
        }
    }
}

fn rebuild_vertex_index<P: Payload>(g: &mut GMap<P>) {
    let canonical_darts = g
        .vertices
        .iter()
        .map(|(key, attr)| (key, g.cell_representative(attr.dart, Dim::Zero)))
        .collect::<Vec<_>>();
    for (key, dart) in canonical_darts {
        g.vertices[key].dart = dart;
    }
    g.dart_to_vertex.clear();
    for (key, attr) in g.vertices.iter() {
        g.dart_to_vertex.insert(attr.dart, key);
    }
}

fn rebuild_edge_index<P: Payload>(g: &mut GMap<P>) {
    g.dart_to_edge.clear();
    for (key, attr) in g.edges.iter() {
        let repr = g.cell_representative(attr.dart, Dim::One);
        g.dart_to_edge.insert(repr, key);
    }
}

fn rebuild_profile_index<P: Payload>(g: &mut GMap<P>) {
    g.dart_to_profile.clear();
    for (key, attr) in g.profiles.iter() {
        let repr = g.profile_representative(attr.dart);
        g.dart_to_profile.insert(repr, key);
    }
}

fn rebuild_face_index<P: Payload>(g: &mut GMap<P>) {
    g.dart_to_face.clear();
    for (key, attr) in g.faces.iter() {
        for dart in std::iter::once(attr.outer_loop).chain(attr.inner_loops.iter().copied()) {
            let repr = g.cell_representative(dart, Dim::Two);
            g.dart_to_face.insert(repr, key);
        }
    }
}

fn rebuild_sheet_index<P: Payload>(g: &mut GMap<P>) {
    g.dart_to_sheet.clear();
    for (key, attr) in g.sheets.iter() {
        let repr = g.cell_representative(attr.dart, Dim::Three);
        g.dart_to_sheet.insert(repr, key);
    }
}

fn rebuild_solid_index<P: Payload>(g: &mut GMap<P>) {
    g.dart_to_solid.clear();
    for (key, attr) in g.solids.iter() {
        for shell in
            std::iter::once(attr.outer_shell).chain(attr.inner_shells.iter().flatten().copied())
        {
            let repr = g.cell_representative(shell, Dim::Three);
            g.dart_to_solid.insert(repr, key);
        }
    }
}
