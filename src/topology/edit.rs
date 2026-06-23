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

/// Clone-backed topology transaction.
///
/// Dropping this value without a successful [`commit`](Self::commit) restores
/// the complete map snapshot taken by [`GMap::edit`].
pub struct TopologyEdit<'g, P: Payload> {
    gmap: &'g mut GMap<P>,
    backup: Option<GMap<P>>,
    events: Vec<EditEvent>,
}

#[derive(Debug, Clone, Copy)]
enum EditEvent {
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
    pub(super) fn new(gmap: &'g mut GMap<P>) -> Self {
        Self {
            backup: Some(gmap.clone()),
            gmap,
            events: Vec::new(),
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

    /// Stages a vertex created by explicitly splitting an existing vertex.
    pub fn add_vertex_split_from(
        &mut self,
        source: VertexKey,
        vertex: VertexAttr<P::V>,
    ) -> VertexKey {
        let created = self.add_vertex(vertex);
        self.events.push(EditEvent::VertexSplit { source, created });
        created
    }

    /// Stages an edge attribute for reconciliation at commit.
    pub fn add_edge(&mut self, edge: EdgeAttr<P::E>) -> EdgeKey {
        self.gmap.edges.insert(edge)
    }

    /// Stages an edge created by explicitly splitting an existing edge.
    pub fn add_edge_split_from(&mut self, source: EdgeKey, edge: EdgeAttr<P::E>) -> EdgeKey {
        let created = self.add_edge(edge);
        self.events.push(EditEvent::EdgeSplit { source, created });
        created
    }

    /// Stages a profile attribute for reconciliation at commit.
    pub fn add_profile(&mut self, profile: ProfileAttr<P::Profile>) -> ProfileKey {
        self.gmap.profiles.insert(profile)
    }

    /// Stages a profile created by explicitly splitting an existing profile.
    pub fn add_profile_split_from(
        &mut self,
        source: ProfileKey,
        profile: ProfileAttr<P::Profile>,
    ) -> ProfileKey {
        let created = self.add_profile(profile);
        self.events
            .push(EditEvent::ProfileSplit { source, created });
        created
    }

    /// Stages a face attribute for reconciliation at commit.
    pub fn add_face(&mut self, face: FaceAttr<P::F>) -> FaceKey {
        self.gmap.faces.insert(face)
    }

    /// Stages a face created by explicitly splitting an existing face.
    pub fn add_face_split_from(&mut self, source: FaceKey, face: FaceAttr<P::F>) -> FaceKey {
        let created = self.add_face(face);
        self.events.push(EditEvent::FaceSplit { source, created });
        created
    }

    /// Stages a sheet attribute for reconciliation at commit.
    pub fn add_sheet(&mut self, sheet: SheetAttr<P::Sheet>) -> SheetKey {
        self.gmap.sheets.insert(sheet)
    }

    /// Stages a sheet created by explicitly splitting an existing sheet.
    pub fn add_sheet_split_from(
        &mut self,
        source: SheetKey,
        sheet: SheetAttr<P::Sheet>,
    ) -> SheetKey {
        let created = self.add_sheet(sheet);
        self.events.push(EditEvent::SheetSplit { source, created });
        created
    }

    /// Stages a solid attribute for reconciliation at commit.
    pub fn add_solid(&mut self, solid: SolidAttr<P::S>) -> SolidKey {
        self.gmap.solids.insert(solid)
    }

    /// Stages a solid created by explicitly splitting an existing solid.
    pub fn add_solid_split_from(&mut self, source: SolidKey, solid: SolidAttr<P::S>) -> SolidKey {
        let created = self.add_solid(solid);
        self.events.push(EditEvent::SolidSplit { source, created });
        created
    }

    /// Declares that `removed` merged into `survivor`.
    pub fn merge_vertices_into(&mut self, survivor: VertexKey, removed: VertexKey) {
        self.events
            .push(EditEvent::VertexMerge { survivor, removed });
    }

    /// Declares that `removed` merged into `survivor`.
    pub fn merge_edges_into(&mut self, survivor: EdgeKey, removed: EdgeKey) {
        self.events.push(EditEvent::EdgeMerge { survivor, removed });
    }

    /// Declares that `removed` merged into `survivor`.
    pub fn merge_profiles_into(&mut self, survivor: ProfileKey, removed: ProfileKey) {
        self.events
            .push(EditEvent::ProfileMerge { survivor, removed });
    }

    /// Declares that `removed` merged into `survivor`.
    pub fn merge_faces_into(&mut self, survivor: FaceKey, removed: FaceKey) {
        self.events.push(EditEvent::FaceMerge { survivor, removed });
    }

    /// Declares that `removed` merged into `survivor`.
    pub fn merge_sheets_into(&mut self, survivor: SheetKey, removed: SheetKey) {
        self.events
            .push(EditEvent::SheetMerge { survivor, removed });
    }

    /// Declares that `removed` merged into `survivor`.
    pub fn merge_solids_into(&mut self, survivor: SolidKey, removed: SolidKey) {
        self.events
            .push(EditEvent::SolidMerge { survivor, removed });
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
    pub(super) fn commit<Q>(mut self, policy: &mut Q) -> Result<(), TopologyEditError>
    where
        Q: EditPolicy<P>,
    {
        validate_gmap(self.gmap).map_err(TopologyEditError::InvalidTopology)?;
        ensure_required_domain_attributes(self.gmap);
        apply_edit_events(self.gmap, &self.events, policy)?;
        rebuild_vertex_index(self.gmap)?;
        rebuild_edge_index(self.gmap)?;
        rebuild_profile_index(self.gmap)?;
        rebuild_face_index(self.gmap)?;
        rebuild_sheet_index(self.gmap)?;
        rebuild_solid_index(self.gmap)?;
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

fn apply_edit_events<P, Q>(
    g: &mut GMap<P>,
    events: &[EditEvent],
    policy: &mut Q,
) -> Result<(), TopologyEditError>
where
    P: Payload,
    Q: EditPolicy<P>,
{
    for event in events {
        match *event {
            EditEvent::VertexSplit { source, created } => {
                let source_data = g.vertex_attr_unchecked(source).data.clone();
                let created_data = &mut g.vertex_attr_mut_unchecked(created).data;
                policy
                    .split_vertex_data(source, &source_data, created, created_data)
                    .map_err(|error| TopologyEditError::Policy(Box::new(error)))?;
            }
            EditEvent::EdgeSplit { source, created } => {
                let source_data = g.edge_attr_unchecked(source).data.clone();
                let created_data = &mut g.edge_attr_mut_unchecked(created).data;
                policy
                    .split_edge_data(source, &source_data, created, created_data)
                    .map_err(|error| TopologyEditError::Policy(Box::new(error)))?;
            }
            EditEvent::ProfileSplit { source, created } => {
                let source_data = g.profile_attr_unchecked(source).data.clone();
                let created_data = &mut g.profile_attr_mut_unchecked(created).data;
                policy
                    .split_profile_data(source, &source_data, created, created_data)
                    .map_err(|error| TopologyEditError::Policy(Box::new(error)))?;
            }
            EditEvent::FaceSplit { source, created } => {
                let source_data = g.face_attr_unchecked(source).data.clone();
                let created_data = &mut g.face_attr_mut_unchecked(created).data;
                policy
                    .split_face_data(source, &source_data, created, created_data)
                    .map_err(|error| TopologyEditError::Policy(Box::new(error)))?;
            }
            EditEvent::SheetSplit { source, created } => {
                let source_data = g.sheet_attr_unchecked(source).data.clone();
                let created_data = &mut g.sheet_attr_mut_unchecked(created).data;
                policy
                    .split_sheet_data(source, &source_data, created, created_data)
                    .map_err(|error| TopologyEditError::Policy(Box::new(error)))?;
            }
            EditEvent::SolidSplit { source, created } => {
                let source_data = g.solid_attr_unchecked(source).data.clone();
                let created_data = &mut g.solid_attr_mut_unchecked(created).data;
                policy
                    .split_solid_data(source, &source_data, created, created_data)
                    .map_err(|error| TopologyEditError::Policy(Box::new(error)))?;
            }
            EditEvent::VertexMerge { survivor, removed } => {
                let removed_data = g
                    .vertices
                    .remove(removed)
                    .expect("declared removed vertex key must have an attribute")
                    .data;
                let survivor_data = &mut g.vertex_attr_mut_unchecked(survivor).data;
                policy
                    .merge_vertex_data(survivor, survivor_data, removed, removed_data)
                    .map_err(|error| TopologyEditError::Policy(Box::new(error)))?;
            }
            EditEvent::EdgeMerge { survivor, removed } => {
                let removed_data = g
                    .edges
                    .remove(removed)
                    .expect("declared removed edge key must have an attribute")
                    .data;
                let survivor_data = &mut g.edge_attr_mut_unchecked(survivor).data;
                policy
                    .merge_edge_data(survivor, survivor_data, removed, removed_data)
                    .map_err(|error| TopologyEditError::Policy(Box::new(error)))?;
            }
            EditEvent::ProfileMerge { survivor, removed } => {
                let removed_data = g
                    .profiles
                    .remove(removed)
                    .expect("declared removed profile key must have an attribute")
                    .data;
                let survivor_data = &mut g.profile_attr_mut_unchecked(survivor).data;
                policy
                    .merge_profile_data(survivor, survivor_data, removed, removed_data)
                    .map_err(|error| TopologyEditError::Policy(Box::new(error)))?;
            }
            EditEvent::FaceMerge { survivor, removed } => {
                let removed_data = g
                    .faces
                    .remove(removed)
                    .expect("declared removed face key must have an attribute")
                    .data;
                let survivor_data = &mut g.face_attr_mut_unchecked(survivor).data;
                policy
                    .merge_face_data(survivor, survivor_data, removed, removed_data)
                    .map_err(|error| TopologyEditError::Policy(Box::new(error)))?;
            }
            EditEvent::SheetMerge { survivor, removed } => {
                let removed_data = g
                    .sheets
                    .remove(removed)
                    .expect("declared removed sheet key must have an attribute")
                    .data;
                let survivor_data = &mut g.sheet_attr_mut_unchecked(survivor).data;
                policy
                    .merge_sheet_data(survivor, survivor_data, removed, removed_data)
                    .map_err(|error| TopologyEditError::Policy(Box::new(error)))?;
            }
            EditEvent::SolidMerge { survivor, removed } => {
                let removed_data = g
                    .solids
                    .remove(removed)
                    .expect("declared removed solid key must have an attribute")
                    .data;
                let survivor_data = &mut g.solid_attr_mut_unchecked(survivor).data;
                policy
                    .merge_solid_data(survivor, survivor_data, removed, removed_data)
                    .map_err(|error| TopologyEditError::Policy(Box::new(error)))?;
            }
        }
    }
    Ok(())
}

fn duplicate_cell_error(entity: &'static str, representative: Dart) -> TopologyEditError {
    TopologyEditError::DuplicateCellAttribute {
        entity,
        representative,
    }
}

fn rebuild_vertex_index<P: Payload>(g: &mut GMap<P>) -> Result<(), TopologyEditError> {
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
        if g.dart_to_vertex.contains_key(&attr.dart) {
            return Err(duplicate_cell_error("vertex", attr.dart));
        }
        g.dart_to_vertex.insert(attr.dart, key);
    }
    Ok(())
}

fn rebuild_edge_index<P: Payload>(g: &mut GMap<P>) -> Result<(), TopologyEditError> {
    g.dart_to_edge.clear();
    for (key, attr) in g.edges.iter() {
        let repr = g.cell_representative(attr.dart, Dim::One);
        if g.dart_to_edge.contains_key(&repr) {
            return Err(duplicate_cell_error("edge", repr));
        }
        g.dart_to_edge.insert(repr, key);
    }
    Ok(())
}

fn rebuild_profile_index<P: Payload>(g: &mut GMap<P>) -> Result<(), TopologyEditError> {
    g.dart_to_profile.clear();
    for (key, attr) in g.profiles.iter() {
        let repr = g.profile_representative(attr.dart);
        if g.dart_to_profile.contains_key(&repr) {
            return Err(duplicate_cell_error("profile", repr));
        }
        g.dart_to_profile.insert(repr, key);
    }
    Ok(())
}

fn rebuild_face_index<P: Payload>(g: &mut GMap<P>) -> Result<(), TopologyEditError> {
    g.dart_to_face.clear();
    for (key, attr) in g.faces.iter() {
        for dart in std::iter::once(attr.outer_loop).chain(attr.inner_loops.iter().copied()) {
            let repr = g.cell_representative(dart, Dim::Two);
            if g.dart_to_face.contains_key(&repr) {
                return Err(duplicate_cell_error("face", repr));
            }
            g.dart_to_face.insert(repr, key);
        }
    }
    Ok(())
}

fn rebuild_sheet_index<P: Payload>(g: &mut GMap<P>) -> Result<(), TopologyEditError> {
    g.dart_to_sheet.clear();
    for (key, attr) in g.sheets.iter() {
        let repr = g.cell_representative(attr.dart, Dim::Three);
        if g.dart_to_sheet.contains_key(&repr) {
            return Err(duplicate_cell_error("sheet", repr));
        }
        g.dart_to_sheet.insert(repr, key);
    }
    Ok(())
}

fn rebuild_solid_index<P: Payload>(g: &mut GMap<P>) -> Result<(), TopologyEditError> {
    g.dart_to_solid.clear();
    for (key, attr) in g.solids.iter() {
        for shell in
            std::iter::once(attr.outer_shell).chain(attr.inner_shells.iter().flatten().copied())
        {
            let repr = g.cell_representative(shell, Dim::Three);
            if g.dart_to_solid.contains_key(&repr) {
                return Err(duplicate_cell_error("solid", repr));
            }
            g.dart_to_solid.insert(repr, key);
        }
    }
    Ok(())
}
