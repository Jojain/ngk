//! The owner of a modelled shape: one pure GMap plus everything laid over it.
//!
//! A [`GMap`] is connectivity and nothing else. A `Model<P>` is that map, the
//! logical entities keyed against it, the geometry and payload each entity
//! carries, the subdivision classification saying which entity's interior every
//! raw cell falls in, and the derived indexes that make all of it fast to look
//! up. Mutation happens in one place: [`Model::transaction`] hands out a
//! [`ModelEdit`], which is the only capability that can change any of it, and
//! either the whole edit commits or the model is restored exactly as it was.

use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::OnceLock;

use serde::{Deserialize, Serialize};
use slotmap::SlotMap;

use crate::topology::attributes::{
    EdgeAttr, FaceAttr, ProfileAttr, SheetAttr, ShellRoot, SolidAttr, VertexAttr,
};
use crate::topology::edge::Edge;
use crate::topology::edit::{
    EditEvent, EditKey, EditPolicy, ModelEdit, ModelEditError, PreservePayload,
    commit_model_transaction,
};
use crate::topology::face::Face;
use crate::topology::gmap::{Dart, Dim, GMap, IsolatedDart, SewableDarts};
use crate::topology::orientation::Orientation;
use crate::topology::payload::{Payload, StandardPayload};
use crate::topology::profile::Profile;
use crate::topology::shape_keys::{EdgeKey, FaceKey, ProfileKey, SheetKey, SolidKey, VertexKey};
use crate::topology::sheet::Sheet;
use crate::topology::solid::Solid;
use crate::topology::subdivision::{
    EntityOwner, OwnerRemap, OwnershipIndex, Subdivision, SubdivisionError,
};
use crate::topology::vertex::Vertex;

mod realization;
use realization::RealizationCache;
pub use realization::{FaceRealization, RealizationError, RealizationPurpose};

/// Type marker for vertex attributes.
pub struct Cell0;
/// Type marker for edge attributes.
pub struct Cell1;
/// Type marker for face attributes.
pub struct Cell2;
/// Type marker for solid attributes.
pub struct Cell3;

/// Compile-time mapping from a cell marker to its dimension and key type.
///
/// The association lives here rather than in the map because a raw cell does
/// not name a logical entity by itself: the key comes from this model's stores,
/// which is what lets one entity span several raw cells.
pub trait CellDim {
    /// Dimension represented by this cell marker.
    const DIM: Dim;
    /// Stable key type for this cell dimension.
    type Key: Copy;
}

impl CellDim for Cell0 {
    const DIM: Dim = Dim::Zero;
    type Key = VertexKey;
}
impl CellDim for Cell1 {
    const DIM: Dim = Dim::One;
    type Key = EdgeKey;
}
impl CellDim for Cell2 {
    const DIM: Dim = Dim::Two;
    type Key = FaceKey;
}
impl CellDim for Cell3 {
    const DIM: Dim = Dim::Three;
    type Key = SolidKey;
}

/// Attribute lookup backend for a specific cell dimension.
///
/// Most callers should use [`Model::attribute`] instead of calling this trait
/// directly.
pub trait AttributeStore<D: CellDim> {
    /// Attribute type stored for this dimension.
    type Attr;
    /// Returns the attribute associated with canonical representative `repr`.
    fn get(&self, repr: Dart) -> Option<&Self::Attr>;
}

/// Trait for looking up a cell key from a canonical representative dart.
#[doc(hidden)]
pub trait CellKeyLookup<D: CellDim> {
    fn get_key(&self, repr: Dart) -> Option<D::Key>;
}

impl<P: Payload> CellKeyLookup<Cell0> for Model<P> {
    fn get_key(&self, repr: Dart) -> Option<VertexKey> {
        self.derived_indexes().vertex.get(&repr).copied()
    }
}
impl<P: Payload> CellKeyLookup<Cell1> for Model<P> {
    fn get_key(&self, repr: Dart) -> Option<EdgeKey> {
        self.derived_indexes().edge.get(&repr).copied()
    }
}
impl<P: Payload> CellKeyLookup<Cell2> for Model<P> {
    fn get_key(&self, repr: Dart) -> Option<FaceKey> {
        self.derived_indexes().face.get(&repr).copied()
    }
}
impl<P: Payload> CellKeyLookup<Cell3> for Model<P> {
    fn get_key(&self, repr: Dart) -> Option<SolidKey> {
        self.derived_indexes().solid.get(&repr).copied()
    }
}

impl<P: Payload> AttributeStore<Cell0> for Model<P> {
    type Attr = VertexAttr<P::V>;
    fn get(&self, repr: Dart) -> Option<&VertexAttr<P::V>> {
        let key = self.derived_indexes().vertex.get(&repr).copied()?;
        self.vertices.get(key)
    }
}
impl<P: Payload> AttributeStore<Cell1> for Model<P> {
    type Attr = EdgeAttr<P::E>;
    fn get(&self, repr: Dart) -> Option<&EdgeAttr<P::E>> {
        let key = self.derived_indexes().edge.get(&repr).copied()?;
        self.edges.get(key)
    }
}
impl<P: Payload> AttributeStore<Cell2> for Model<P> {
    type Attr = FaceKey;
    fn get(&self, repr: Dart) -> Option<&FaceKey> {
        self.derived_indexes().face.get(&repr)
    }
}
impl<P: Payload> AttributeStore<Cell3> for Model<P> {
    type Attr = SolidKey;
    fn get(&self, repr: Dart) -> Option<&SolidKey> {
        self.derived_indexes().solid.get(&repr)
    }
}

fn remap_dart(dart_map: &HashMap<Dart, Dart>, dart: Dart) -> Dart {
    *dart_map
        .get(&dart)
        .expect("merged dart reference must have a remapped dart")
}

fn copied_cell_dart<P: Payload>(
    source: &Model<P>,
    copied_darts: &HashSet<Dart>,
    dart: Dart,
    dim: Dim,
) -> Option<Dart> {
    source
        .orbit(dart, source.orbit_indices(dim))
        .find(|candidate| copied_darts.contains(candidate))
}

/// Source topology selected for copying into another [`Model`].
///
/// Construct this from a topology view's owning model, the darts to copy, and
/// the representative dart that should be returned after remapping.
pub struct TopologyMerge<'a, P: Payload> {
    source: &'a Model<P>,
    darts: Vec<Dart>,
    faces: Vec<FaceKey>,
    handle: MergeHandle,
}

/// What a copied topology is reached by in the model it was copied into.
///
/// A copy is normally located by a dart. A boundaryless face has none, so the
/// copy names the new face key instead — the same distinction as
/// [`ShellRoot`](crate::topology::attributes::ShellRoot), one layer up.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MergeHandle {
    /// A dart of the copy, in the target model's numbering.
    Dart(Dart),
    /// The copy is one boundaryless face, under its new key.
    Face(FaceKey),
}

impl MergeHandle {
    /// Returns the handle's dart, or `None` for a boundaryless copy.
    pub fn dart(self) -> Option<Dart> {
        match self {
            Self::Dart(dart) => Some(dart),
            Self::Face(_) => None,
        }
    }

    /// Returns the handle's dart.
    ///
    /// # Panics
    ///
    /// Panics on a boundaryless copy, which has no dart to return.
    pub fn dart_unchecked(self) -> Dart {
        self.dart()
            .expect("a dart-backed copy should return a dart handle")
    }

    /// Returns the handle's face, or `None` for a dart-backed copy.
    pub fn face(self) -> Option<FaceKey> {
        match self {
            Self::Face(face) => Some(face),
            Self::Dart(_) => None,
        }
    }
}

impl<'a, P: Payload> TopologyMerge<'a, P> {
    /// Creates a merge descriptor for a dart-backed topology view.
    pub fn new(source: &'a Model<P>, darts: Vec<Dart>, handle: Dart) -> Self {
        Self {
            source,
            darts,
            faces: Vec::new(),
            handle: MergeHandle::Dart(handle),
        }
    }

    /// Creates a merge descriptor for topology that includes boundaryless
    /// faces, which no dart can name.
    pub fn with_faces(
        source: &'a Model<P>,
        darts: Vec<Dart>,
        faces: Vec<FaceKey>,
        handle: MergeHandle,
    ) -> Self {
        Self {
            source,
            darts,
            faces,
            handle,
        }
    }
}

/// Topological views that can be copied into another [`Model`].
pub trait MergeTopology<P: Payload> {
    /// Returns the topology subset that should be copied.
    fn merge_topology(&self) -> TopologyMerge<'_, P>;

    /// Copy this topology into a fresh [`Model`], returning the copied model and
    /// this topology's representative dart rewritten to the new model.
    ///
    /// Alpha links within the copied topology are preserved. Links leaving the
    /// copied dart set become free in the isolated model.
    fn isolate(self) -> (Model<P>, MergeHandle)
    where
        Self: Sized,
    {
        let mut isolated = Model::new();
        let handle = isolated
            .transaction(|edit| Ok::<_, ModelEditError>(edit.merge(self)))
            .expect("isolating valid topology should produce a valid model");
        (isolated, handle)
    }
}

impl<P, T> MergeTopology<P> for &T
where
    P: Payload,
    T: MergeTopology<P>,
{
    fn merge_topology(&self) -> TopologyMerge<'_, P> {
        (*self).merge_topology()
    }
}

/// One modelled shape: a pure GMap under stable, geometry-carrying entities.
///
/// The map supplies all computational connectivity. Everything that gives that
/// connectivity meaning — which entity a cell belongs to, what curve or surface
/// it carries, what payload the user attached — is stored here, alongside it.
#[derive(Serialize, Deserialize)]
#[serde(bound(
    serialize = "P::V: Serialize, P::E: Serialize, P::Profile: Serialize, P::F: Serialize, P::Sheet: Serialize, P::S: Serialize",
    deserialize = "P::V: Deserialize<'de>, P::E: Deserialize<'de>, P::Profile: Deserialize<'de>, P::F: Deserialize<'de>, P::Sheet: Deserialize<'de>, P::S: Deserialize<'de>"
))]
pub struct Model<P: Payload = StandardPayload> {
    topology: GMap,
    pub(crate) vertices: SlotMap<VertexKey, VertexAttr<P::V>>,
    pub(crate) edges: SlotMap<EdgeKey, EdgeAttr<P::E>>,
    pub(crate) profiles: SlotMap<ProfileKey, ProfileAttr<P::Profile>>,
    pub(crate) faces: SlotMap<FaceKey, FaceAttr<P::F>>,
    pub(crate) sheets: SlotMap<SheetKey, SheetAttr<P::Sheet>>,
    pub(crate) solids: SlotMap<SolidKey, SolidAttr<P::S>>,
    pub(crate) subdivision: Subdivision,
    revision: u64,
    #[serde(skip)]
    derived_indexes: OnceLock<DerivedCellIndexes>,
    #[serde(skip)]
    ownership: OnceLock<OwnershipIndex>,
    #[serde(skip)]
    realizations: RealizationCache,
    #[serde(skip)]
    transaction: Option<Box<TransactionState<P>>>,
}

#[derive(Debug, Clone, Default)]
struct DerivedCellIndexes {
    vertex: HashMap<Dart, VertexKey>,
    edge: HashMap<Dart, EdgeKey>,
    profile: HashMap<Dart, ProfileKey>,
    face: HashMap<Dart, FaceKey>,
    sheet: HashMap<Dart, SheetKey>,
    solid: HashMap<Dart, SolidKey>,
}

struct TransactionState<P: Payload> {
    snapshot: Model<P>,
    events: Vec<EditEvent>,
}

impl<P: Payload> Clone for Model<P> {
    fn clone(&self) -> Self {
        Self {
            topology: self.topology.clone(),
            vertices: self.vertices.clone(),
            edges: self.edges.clone(),
            profiles: self.profiles.clone(),
            faces: self.faces.clone(),
            sheets: self.sheets.clone(),
            solids: self.solids.clone(),
            subdivision: self.subdivision.clone(),
            revision: self.revision,
            derived_indexes: OnceLock::new(),
            ownership: OnceLock::new(),
            realizations: RealizationCache::default(),
            transaction: None,
        }
    }
}

impl<P: Payload> Default for Model<P> {
    fn default() -> Self {
        Self::new()
    }
}

impl<P: Payload> Model<P> {
    /// Creates an empty model with no darts and no entities.
    pub fn new() -> Self {
        Self {
            topology: GMap::new(),
            vertices: SlotMap::with_key(),
            edges: SlotMap::with_key(),
            profiles: SlotMap::with_key(),
            faces: SlotMap::with_key(),
            sheets: SlotMap::with_key(),
            solids: SlotMap::with_key(),
            subdivision: Subdivision::new(),
            revision: 0,
            derived_indexes: OnceLock::new(),
            ownership: OnceLock::new(),
            realizations: RealizationCache::default(),
            transaction: None,
        }
    }

    /// Returns the pure map this model is laid over.
    ///
    /// Read-only on purpose: the map's connectivity and this model's entities
    /// only stay in step because every change goes through one transaction.
    pub fn topology(&self) -> &GMap {
        &self.topology
    }

    /// Returns how many times this model has been committed to.
    ///
    /// External caches can use this to distinguish committed states. Staged
    /// edits retain the revision until commit, so model-owned caches also
    /// invalidate on every mutation.
    pub fn revision(&self) -> u64 {
        self.revision
    }

    /// Returns which logical entity owns each raw cell.
    pub fn subdivision(&self) -> &Subdivision {
        &self.subdivision
    }

    /// Returns the dart-to-owner lookup derived from the subdivision.
    ///
    /// # Panics
    ///
    /// Panics if the labelling does not describe this model's map. Commit
    /// rejects such a labelling, so a committed model always has one.
    pub fn ownership(&self) -> &OwnershipIndex {
        self.ownership.get_or_init(|| {
            self.build_ownership()
                .expect("a committed model's labelling should describe its own map")
        })
    }

    /// Returns where each entity is anchored, as the classification reads it.
    ///
    /// This is the derived half of the subdivision: an entity contains the cell
    /// its own anchor sits in, which its attribute already says. A solid is
    /// anchored through its outer shell, whose dart lies in the volume the
    /// solid is. A boundaryless face has no dart and contributes nothing --
    /// the case that disappears once such a face carries a real scaffold.
    fn entity_anchors(&self) -> impl Iterator<Item = (Dim, Dart, EntityOwner)> + '_ {
        let vertices = self
            .vertices
            .iter()
            .map(|(key, attr)| (Dim::Zero, attr.dart, EntityOwner::Vertex(key)));
        let edges = self
            .edges
            .iter()
            .map(|(key, attr)| (Dim::One, attr.dart, EntityOwner::Edge(key)));
        let faces = self.faces.iter().filter_map(|(key, attr)| {
            attr.seed()
                .map(|seed| (Dim::Two, seed, EntityOwner::Face(key)))
        });
        let solids = self.solids.iter().filter_map(|(key, attr)| {
            attr.outer_shell
                .dart()
                .map(|dart| (Dim::Three, dart, EntityOwner::Solid(key)))
        });
        vertices.chain(edges).chain(faces).chain(solids)
    }

    /// Builds the dart-to-owner lookup from the stored labels and the anchors.
    fn build_ownership(&self) -> Result<OwnershipIndex, SubdivisionError> {
        OwnershipIndex::build_with_anchors(&self.topology, &self.subdivision, self.entity_anchors())
    }

    /// Runs one atomic operation against this model.
    ///
    /// The operation receives the only capability that can mutate staged state.
    /// A returned error or failed commit restores the complete starting
    /// snapshot. Panics are not handled by this API.
    pub fn transaction<T, E, F>(&mut self, operation: F) -> Result<T, E>
    where
        E: From<ModelEditError>,
        F: FnOnce(&mut ModelEdit<'_, P>) -> Result<T, E>,
    {
        self.run_transaction(&mut PreservePayload, operation)
    }

    /// Runs one atomic operation with a caller-provided payload policy.
    ///
    /// Policy event application is performed only after the complete operation
    /// has passed topology validation and identity reconciliation.
    pub fn transaction_with_policy<Q, T, E, F>(
        &mut self,
        policy: &mut Q,
        operation: F,
    ) -> Result<T, E>
    where
        Q: EditPolicy<P>,
        E: From<ModelEditError>,
        F: FnOnce(&mut ModelEdit<'_, P>) -> Result<T, E>,
    {
        self.run_transaction(policy, operation)
    }

    /// Owns the snapshot and commit for one transaction-scoped edit session.
    fn run_transaction<Q, T, E, F>(&mut self, policy: &mut Q, operation: F) -> Result<T, E>
    where
        Q: EditPolicy<P>,
        E: From<ModelEditError>,
        F: FnOnce(&mut ModelEdit<'_, P>) -> Result<T, E>,
    {
        debug_assert!(self.transaction.is_none());

        self.transaction = Some(Box::new(TransactionState {
            snapshot: self.clone(),
            events: Vec::new(),
        }));

        let result = operation(&mut ModelEdit::new(self));
        match result {
            Ok(value) => match self.commit_active_transaction(policy) {
                Ok(()) => {
                    self.transaction = None;
                    Ok(value)
                }
                Err(error) => {
                    self.rollback_transaction();
                    Err(E::from(error))
                }
            },
            Err(error) => {
                self.rollback_transaction();
                Err(error)
            }
        }
    }

    /// Replaces all staged state with the snapshot owned by the transaction.
    fn rollback_transaction(&mut self) {
        if let Some(transaction) = self.transaction.take() {
            *self = transaction.snapshot;
        }
    }

    /// Finalizes the active transaction, restoring its snapshot if commit fails.
    fn commit_active_transaction<Q>(&mut self, policy: &mut Q) -> Result<(), ModelEditError>
    where
        Q: EditPolicy<P>,
    {
        let transaction = self
            .transaction
            .take()
            .expect("an active transaction must exist while committing");
        match commit_model_transaction(self, &transaction.snapshot, &transaction.events, policy) {
            Ok(()) => {
                self.revision = transaction.snapshot.revision + 1;
                self.realizations = RealizationCache::default();
                Ok(())
            }
            Err(error) => {
                *self = transaction.snapshot;
                Err(error)
            }
        }
    }

    /// Records one semantic event in the active edit session.
    pub(crate) fn record_edit_event(&mut self, event: EditEvent) {
        self.transaction
            .as_mut()
            .expect("model events require an active transaction")
            .events
            .push(event);
    }

    /// Records attributes created by internal model-copying operations.
    fn record_created_attribute(&mut self, key: EditKey) {
        self.record_edit_event(EditEvent::Created { key });
    }

    /// Discards cached lookups after topology, attributes or labels change.
    pub(crate) fn invalidate_derived_indexes(&mut self) {
        self.derived_indexes.take();
        self.ownership.take();
        self.realizations = RealizationCache::default();
    }

    /// Forces the lazy indexes to be built, notably as the last commit check.
    pub(crate) fn materialize_derived_indexes(&self) {
        let _ = self.derived_indexes();
    }

    /// Checks that the subdivision describes this model's map.
    pub(crate) fn validate_subdivision(&self) -> Result<(), SubdivisionError> {
        self.build_ownership().map(|_| ())
    }

    /// Returns the cached indexes, rebuilding them from authoritative state if needed.
    fn derived_indexes(&self) -> &DerivedCellIndexes {
        self.derived_indexes
            .get_or_init(|| self.build_derived_indexes())
    }

    /// Reconstructs every dart-to-attribute index from current cells and attributes.
    fn build_derived_indexes(&self) -> DerivedCellIndexes {
        let mut indexes = DerivedCellIndexes::default();

        for (key, attr) in self.vertices.iter() {
            let repr = self.cell_representative(attr.dart, Dim::Zero);
            self.insert_logical_key(&mut indexes.vertex, repr, key, EditKey::Vertex);
        }
        for (key, attr) in self.edges.iter() {
            let repr = self.cell_representative(attr.dart, Dim::One);
            self.insert_logical_key(&mut indexes.edge, repr, key, EditKey::Edge);
        }
        for (key, attr) in self.profiles.iter() {
            let repr = Profile::representative(self, attr.dart);
            self.insert_logical_key(&mut indexes.profile, repr, key, EditKey::Profile);
        }
        for (key, attr) in self.faces.iter() {
            for dart in attr.darts() {
                let repr = self.cell_representative(dart, Dim::Two);
                self.insert_logical_key(&mut indexes.face, repr, key, EditKey::Face);
            }
        }
        // A boundaryless shell has no dart to reach it from, so it registers
        // nothing here. It is found through its key, or through the one face
        // it holds, never by walking the map.
        for (key, attr) in self.sheets.iter() {
            for dart in attr
                .dart()
                .into_iter()
                .flat_map(|root| self.logical_sheet_darts(root, &indexes.face))
            {
                let repr = self.cell_representative(dart, Dim::Three);
                self.insert_logical_key(&mut indexes.sheet, repr, key, EditKey::Sheet);
            }
        }
        for (key, attr) in self.solids.iter() {
            for dart in attr.shell_darts() {
                for shell_dart in self.logical_sheet_darts(dart, &indexes.face) {
                    let repr = self.cell_representative(shell_dart, Dim::Three);
                    self.insert_logical_key(&mut indexes.solid, repr, key, EditKey::Solid);
                }
            }
        }

        indexes
    }

    /// Collects every raw alpha0/alpha1/alpha2 component in a logical sheet.
    ///
    /// A face with holes is represented by several disconnected 2-cell orbits
    /// tied to one face attribute. Crossing between those boundary components
    /// makes the incident raw 3-cell components part of the same domain sheet.
    fn logical_sheet_darts(&self, start: Dart, face_index: &HashMap<Dart, FaceKey>) -> Vec<Dart> {
        let mut pending = VecDeque::from([start]);
        let mut seen_components = HashSet::new();
        let mut seen_faces = HashSet::new();
        let mut darts = Vec::new();

        while let Some(seed) = pending.pop_front() {
            let component = self.cell_representative(seed, Dim::Three);
            if !seen_components.insert(component) {
                continue;
            }

            for dart in self.incident_cells(seed, Dim::Three, Dim::Two) {
                let face_component = self.cell_representative(dart, Dim::Two);
                if let Some(&face_key) = face_index.get(&face_component)
                    && seen_faces.insert(face_key)
                {
                    let face = self.face_attr_unchecked(face_key);
                    pending.extend(face.darts());
                }
            }
            darts.extend(self.orbit(seed, self.orbit_indices(Dim::Three)));
        }

        darts
    }

    /// Returns all darts in the logical sheet containing `start`.
    pub(crate) fn sheet_darts(&self, start: Dart) -> Vec<Dart> {
        self.logical_sheet_darts(start, &self.derived_indexes().face)
    }

    /// Inserts one cell key, resolving staged duplicate identities consistently.
    fn insert_logical_key<K: Copy>(
        &self,
        index: &mut HashMap<Dart, K>,
        representative: Dart,
        candidate: K,
        wrap: fn(K) -> EditKey,
    ) {
        let Some(existing) = index.get(&representative).copied() else {
            index.insert(representative, candidate);
            return;
        };
        if self.prefer_transaction_key(wrap(existing), wrap(candidate)) == wrap(candidate) {
            index.insert(representative, candidate);
        }
    }

    /// Selects the identity a staged lookup should expose before final reconciliation.
    ///
    /// Explicit lineage wins first, then transaction-start identities, then the
    /// earliest-created local identity. Commit uses the same ordering rules.
    fn prefer_transaction_key(&self, first: EditKey, second: EditKey) -> EditKey {
        let Some(transaction) = &self.transaction else {
            return first;
        };

        let final_key = |mut key: EditKey| {
            let mut visited = HashSet::new();
            while visited.insert(key) {
                let Some(next) = transaction.events.iter().find_map(|event| {
                    event
                        .merge_keys()
                        .and_then(|(survivor, removed)| (removed == key).then_some(survivor))
                }) else {
                    break;
                };
                key = next;
            }
            key
        };
        let first_final = final_key(first);
        let second_final = final_key(second);
        if first_final == second {
            return second;
        }
        if second_final == first {
            return first;
        }

        let existed = |key| match key {
            EditKey::Vertex(key) => transaction.snapshot.vertices.contains_key(key),
            EditKey::Edge(key) => transaction.snapshot.edges.contains_key(key),
            EditKey::Profile(key) => transaction.snapshot.profiles.contains_key(key),
            EditKey::Face(key) => transaction.snapshot.faces.contains_key(key),
            EditKey::Sheet(key) => transaction.snapshot.sheets.contains_key(key),
            EditKey::Solid(key) => transaction.snapshot.solids.contains_key(key),
        };
        match (existed(first), existed(second)) {
            (true, false) => return first,
            (false, true) => return second,
            _ => {}
        }

        let creation_order = |key| {
            transaction
                .events
                .iter()
                .position(
                    |event| matches!(event, EditEvent::Created { key: created } if *created == key),
                )
                .unwrap_or(usize::MAX)
        };
        if creation_order(second) < creation_order(first) {
            second
        } else {
            first
        }
    }
}

/// Pure-map queries, answered by the topology this model owns.
impl<P: Payload> Model<P> {
    /// Returns the number of alpha involutions.
    pub fn dimension(&self) -> usize {
        self.topology.dimension()
    }

    /// Returns the number of dart slots in the map.
    pub fn dart_count(&self) -> usize {
        self.topology.dart_count()
    }

    /// Iterates all dart identifiers currently addressable in the map.
    pub fn darts(&self) -> impl Iterator<Item = Dart> + '_ {
        self.topology.darts()
    }

    /// Returns `alpha_d(dart)`.
    pub fn alpha(&self, d: Dim, dart: Dart) -> Dart {
        self.topology.alpha(d, dart)
    }

    /// A dart is `i`-free when `αᵢ(d) = d`, i.e. not sewn along dimension `i`.
    pub fn is_free(&self, dart: Dart, d: Dim) -> bool {
        self.topology.is_free(dart, d)
    }

    /// Iterates the orbit generated from `dart` by the given alpha indices.
    pub fn orbit(
        &self,
        dart: Dart,
        involutions: Vec<usize>,
    ) -> crate::topology::gmap::OrbitIterator<'_> {
        self.topology.orbit(dart, involutions)
    }

    /// Returns the alpha indices that generate a cell orbit of dimension `d`.
    pub fn orbit_indices(&self, d: Dim) -> Vec<usize> {
        self.topology.orbit_indices(d)
    }

    /// Returns the canonical representative dart for the `dim`-cell of `dart`.
    pub fn cell_representative(&self, dart: Dart, dim: Dim) -> Dart {
        self.topology.cell_representative(dart, dim)
    }

    /// Iterates one dart per `target_dim`-cell incident to the `container_dim`-cell of `dart`.
    pub fn incident_cells(
        &self,
        dart: Dart,
        container_dim: Dim,
        target_dim: Dim,
    ) -> impl Iterator<Item = Dart> + '_ {
        self.topology
            .incident_cells(dart, container_dim, target_dim)
    }

    /// Iterates one dart per `dimension`-cell of the whole map.
    pub fn cells(&self, dimension: Dim) -> impl Iterator<Item = Dart> + '_ {
        self.topology.cells(dimension)
    }

    /// Iterates one dart per `d`-cell adjacent to the `d`-cell of `dart`.
    pub fn adjacent_cells(&self, dart: Dart, d: Dim) -> impl Iterator<Item = Dart> + '_ {
        self.topology.adjacent_cells(dart, d)
    }

    /// Returns the canonical representative of the profile chain at `dart`.
    /// Pairs two orbits for sewing along dimension `d`.
    pub(crate) fn is_sewable(&self, d0: Dart, d1: Dart, d: Dim) -> Option<SewableDarts> {
        self.topology.is_sewable(d0, d1, d)
    }
}

/// Staged mutation of the map, always paired with cache invalidation.
impl<P: Payload> Model<P> {
    pub(crate) fn add_dart(&mut self) -> Dart {
        self.invalidate_derived_indexes();
        self.topology.add_dart()
    }

    pub(crate) fn remove_dart(&mut self, dart: IsolatedDart) {
        self.invalidate_derived_indexes();
        self.topology.remove_dart(dart);
    }

    pub(crate) fn link_raw(&mut self, d: Dim, d0: Dart, d1: Dart) {
        self.invalidate_derived_indexes();
        self.topology.link_raw(d, d0, d1);
    }

    pub(crate) fn unlink_raw(&mut self, d: Dim, dart: Dart) -> Dart {
        self.invalidate_derived_indexes();
        self.topology.unlink_raw(d, dart)
    }

    /// Labels the raw `dimension`-cell containing `dart` as owned by `owner`.
    pub(crate) fn own_cell(&mut self, dimension: Dim, dart: Dart, owner: EntityOwner) {
        self.invalidate_derived_indexes();
        self.subdivision.own(dimension, dart, owner);
    }

    /// Unlabels the raw `dimension`-cell containing `dart`.
    ///
    /// An entry sits on whichever dart of the orbit the labeller happened to
    /// hand over, so every dart of the cell is offered rather than only the one
    /// asked about.
    pub(crate) fn disown_cell(&mut self, dimension: Dim, dart: Dart) {
        self.invalidate_derived_indexes();
        let anchors: Vec<Dart> = self
            .topology
            .orbit(dart, self.topology.orbit_indices(dimension))
            .collect();
        for anchor in anchors {
            self.subdivision.disown_at(dimension, anchor);
        }
    }

    /// Drops every label naming `owner`, for an entity being removed.
    pub(crate) fn disown_entity(&mut self, owner: EntityOwner) {
        self.invalidate_derived_indexes();
        self.subdivision.disown(owner);
    }

    /// Returns where `dart` sits in space, whether or not a vertex marks it.
    ///
    /// Most darts sit at a logical vertex and the answer is that vertex's
    /// point. A whole circle with nothing marked on it has no vertex at all --
    /// the place its parameterization closes is inside the edge, not a corner
    /// anything meets at -- and the answer comes from the curve instead.
    ///
    /// Ask this whenever a position is wanted. Ask the vertex store only when
    /// the identity of a logical vertex is what matters.
    pub fn point_at_dart(&self, dart: Dart) -> Option<crate::geometry::Point3> {
        if let Some(vertex) = self.attribute::<Cell0>(dart) {
            return Some(vertex.point);
        }
        let edge = self.attribute::<Cell1>(dart)?;
        Some(edge.curve.point_at(edge.curve.domain().start))
    }

    /// Removes several isolated darts, renumbering every reference held here.
    pub(crate) fn remove_isolated_darts(
        &mut self,
        darts: Vec<IsolatedDart>,
    ) -> HashMap<Dart, Dart> {
        self.invalidate_derived_indexes();
        // An entity attribute is guaranteed by its caller to reference only
        // darts that survive, which is what lets the renumbering below expect a
        // mapping for each. An ownership anchor carries no such guarantee: it
        // names an orbit, and removing the particular dart it happens to sit on
        // does not remove the cell. Pick each label's surviving dart here,
        // while the orbits it names still exist.
        let discarded: HashSet<Dart> = darts.iter().map(IsolatedDart::dart).collect();
        let reanchored: Vec<(Dim, Dart, EntityOwner)> = self
            .subdivision
            .records()
            .filter_map(|record| {
                self.topology
                    .orbit(
                        record.representative,
                        self.topology.orbit_indices(record.dimension),
                    )
                    .find(|dart| !discarded.contains(dart))
                    .map(|survivor| (record.dimension, survivor, record.owner))
            })
            .collect();

        let remap = self.topology.compact(darts);
        let follow = |dart: Dart| remap.get(&dart).copied().unwrap_or(dart);
        let mut subdivision = Subdivision::new();
        for (dimension, survivor, owner) in reanchored {
            subdivision.own(dimension, follow(survivor), owner);
        }
        self.subdivision = subdivision;

        if remap.iter().all(|(old, new)| old == new) {
            return remap;
        }

        let map_dart = |dart: Dart| {
            *remap
                .get(&dart)
                .expect("retained topology must not reference a removed dart")
        };
        for attr in self.vertices.values_mut() {
            attr.dart = map_dart(attr.dart);
        }
        for attr in self.edges.values_mut() {
            attr.dart = map_dart(attr.dart);
        }
        for attr in self.profiles.values_mut() {
            attr.dart = map_dart(attr.dart);
        }
        for attr in self.faces.values_mut() {
            attr.map_darts(map_dart);
            attr.pcurves = std::mem::take(&mut attr.pcurves)
                .into_iter()
                .map(|(dart, pcurve)| (map_dart(dart), pcurve))
                .collect();
        }
        for attr in self.sheets.values_mut() {
            attr.root.map_dart(&map_dart);
        }
        for attr in self.solids.values_mut() {
            attr.map_shell_darts(&map_dart);
        }

        remap
    }
}

/// Entity lookup and typed views.
impl<P: Payload> Model<P> {
    /// Returns the typed vertex view registered under `key`.
    pub fn vertex(&self, key: VertexKey) -> Option<Vertex<'_, P>> {
        self.vertex_attr(key)?;
        Some(Vertex::new(self, key))
    }

    /// Returns the typed vertex view registered under `key`.
    ///
    /// # Panics
    ///
    /// Panics if no vertex is registered under `key`.
    pub fn vertex_unchecked(&self, key: VertexKey) -> Vertex<'_, P> {
        self.vertex(key).expect("vertex should be in the model")
    }

    /// Returns the vertex attribute registered under `key`.
    pub fn vertex_attr(&self, key: VertexKey) -> Option<&VertexAttr<P::V>> {
        self.vertices.get(key)
    }

    /// Returns the vertex attribute registered under `key`.
    ///
    /// # Panics
    ///
    /// Panics if no vertex is registered under `key`.
    pub fn vertex_attr_unchecked(&self, key: VertexKey) -> &VertexAttr<P::V> {
        self.vertex_attr(key).expect("vertex key should be in map")
    }

    pub(crate) fn vertex_attr_mut(&mut self, key: VertexKey) -> Option<&mut VertexAttr<P::V>> {
        self.invalidate_derived_indexes();
        self.vertices.get_mut(key)
    }

    pub(crate) fn vertex_attr_mut_unchecked(&mut self, key: VertexKey) -> &mut VertexAttr<P::V> {
        self.vertex_attr_mut(key)
            .expect("vertex key should be in map")
    }

    /// Iterates every registered vertex with its attribute.
    pub fn iter_vertices(&self) -> impl Iterator<Item = (VertexKey, &VertexAttr<P::V>)> {
        self.vertices.iter()
    }

    /// Returns the typed edge view registered under `key`.
    pub fn edge(&self, key: EdgeKey) -> Option<Edge<'_, P>> {
        self.edge_attr(key)?;
        Some(Edge::new(self, key))
    }

    /// Returns the typed edge view registered under `key`.
    ///
    /// # Panics
    ///
    /// Panics if no edge is registered under `key`.
    pub fn edge_unchecked(&self, key: EdgeKey) -> Edge<'_, P> {
        self.edge(key).expect("edge should be in the model")
    }

    /// Returns the key of the `D`-cell containing `dart`, if one is registered.
    pub fn cell_key<D: CellDim>(&self, dart: Dart) -> Option<D::Key>
    where
        Self: CellKeyLookup<D>,
    {
        let repr = self.cell_representative(dart, D::DIM);
        self.get_key(repr)
    }

    /// Returns the key of the `D`-cell containing `dart`.
    ///
    /// # Panics
    ///
    /// Panics if no key is registered for the cell.
    pub fn cell_key_unchecked<D: CellDim>(&self, dart: Dart) -> D::Key
    where
        Self: CellKeyLookup<D>,
    {
        self.cell_key::<D>(dart)
            .expect("cell key should be in the model")
    }

    /// Returns how `dart` reads the edge `key` relative to its default sense.
    pub fn edge_orientation_at_dart(&self, key: EdgeKey, dart: Dart) -> Orientation {
        let attr = self.edge_attr_unchecked(key);
        self.cell_orientation_from_seed(attr.dart, dart, Dim::One)
            .expect("edge orientation requires dart to belong to edge")
    }

    /// Returns the edge attribute registered under `key`.
    pub fn edge_attr(&self, key: EdgeKey) -> Option<&EdgeAttr<P::E>> {
        self.edges.get(key)
    }

    /// Returns the edge attribute registered under `key`.
    ///
    /// # Panics
    ///
    /// Panics if no edge is registered under `key`.
    pub fn edge_attr_unchecked(&self, key: EdgeKey) -> &EdgeAttr<P::E> {
        self.edge_attr(key).expect("edge key should be in map")
    }

    pub(crate) fn edge_attr_mut(&mut self, key: EdgeKey) -> Option<&mut EdgeAttr<P::E>> {
        self.invalidate_derived_indexes();
        self.edges.get_mut(key)
    }

    pub(crate) fn edge_attr_mut_unchecked(&mut self, key: EdgeKey) -> &mut EdgeAttr<P::E> {
        self.edge_attr_mut(key).expect("edge key should be in map")
    }

    /// Iterates every registered edge with its attribute.
    pub fn iter_edges(&self) -> impl Iterator<Item = (EdgeKey, &EdgeAttr<P::E>)> {
        self.edges.iter()
    }

    /// Returns the typed profile view registered under `key`.
    pub fn profile(&self, key: ProfileKey) -> Option<Profile<'_, P>> {
        self.profile_attr(key)?;
        Some(Profile::new(self, key))
    }

    /// Returns the typed profile view registered under `key`.
    ///
    /// # Panics
    ///
    /// Panics if no profile is registered under `key`.
    pub fn profile_unchecked(&self, key: ProfileKey) -> Profile<'_, P> {
        self.profile(key).expect("profile should be in the model")
    }

    /// Returns the profile key of the chain containing `dart`, if registered.
    pub fn profile_key(&self, dart: Dart) -> Option<ProfileKey> {
        let repr = Profile::representative(self, dart);
        self.derived_indexes().profile.get(&repr).copied()
    }

    /// Returns the profile key of the chain containing `dart`.
    ///
    /// # Panics
    ///
    /// Panics if no profile is registered for the chain.
    pub fn profile_key_unchecked(&self, dart: Dart) -> ProfileKey {
        self.profile_key(dart)
            .expect("profile key should be in map")
    }

    /// Returns the profile attribute registered under `key`.
    pub fn profile_attr(&self, key: ProfileKey) -> Option<&ProfileAttr<P::Profile>> {
        self.profiles.get(key)
    }

    /// Returns the profile attribute registered under `key`.
    ///
    /// # Panics
    ///
    /// Panics if no profile is registered under `key`.
    pub fn profile_attr_unchecked(&self, key: ProfileKey) -> &ProfileAttr<P::Profile> {
        self.profile_attr(key)
            .expect("profile key should be in map")
    }

    pub(crate) fn profile_attr_mut(
        &mut self,
        key: ProfileKey,
    ) -> Option<&mut ProfileAttr<P::Profile>> {
        self.invalidate_derived_indexes();
        self.profiles.get_mut(key)
    }

    pub(crate) fn profile_attr_mut_unchecked(
        &mut self,
        key: ProfileKey,
    ) -> &mut ProfileAttr<P::Profile> {
        self.profile_attr_mut(key)
            .expect("profile key should be in map")
    }

    /// Iterates every registered profile with its attribute.
    pub fn iter_profiles(&self) -> impl Iterator<Item = (ProfileKey, &ProfileAttr<P::Profile>)> {
        self.profiles.iter()
    }

    /// Returns the typed face view registered under `key`.
    pub fn face(&self, key: FaceKey) -> Option<Face<'_, P>> {
        self.face_attr(key)?;
        Some(Face::new(self, key))
    }

    /// Returns the typed face view registered under `key`.
    ///
    /// # Panics
    ///
    /// Panics if no face is registered under `key`.
    pub fn face_unchecked(&self, key: FaceKey) -> Face<'_, P> {
        self.face(key).expect("face should be in the model")
    }

    /// Returns how `dart` reads the face `key` relative to its default sense.
    pub fn face_orientation_at_dart(&self, key: FaceKey, dart: Dart) -> Orientation {
        let attr = self.face_attr_unchecked(key);
        attr.darts()
            .find_map(|seed| self.cell_orientation_from_seed(seed, dart, Dim::Two))
            .expect("face orientation requires dart to belong to face")
    }

    /// Returns whether `dart` reads a cell the way `seed` does, if it is in it.
    ///
    /// A flag change below the cell's own dimension reverses it; an incidence
    /// change above it leaves its intrinsic orientation alone.
    pub(crate) fn cell_orientation_from_seed(
        &self,
        seed: Dart,
        target: Dart,
        dim: Dim,
    ) -> Option<Orientation> {
        let mut orientations = vec![None; self.dart_count()];
        let mut queue = VecDeque::from([seed]);
        orientations[seed.id()] = Some(Orientation::Same);
        let involutions = self.orbit_indices(dim);

        while let Some(dart) = queue.pop_front() {
            let orientation =
                orientations[dart.id()].expect("queued dart must have an orientation");
            if dart == target {
                return Some(orientation);
            }

            for &index in &involutions {
                let linked = self.alpha(Dim::from_index(index), dart);
                if linked == dart || orientations[linked.id()].is_some() {
                    continue;
                }
                let linked_orientation = if index < dim.index() {
                    orientation.flip()
                } else {
                    orientation
                };
                orientations[linked.id()] = Some(linked_orientation);
                queue.push_back(linked);
            }
        }

        None
    }

    /// Returns the face attribute registered under `key`.
    pub fn face_attr(&self, key: FaceKey) -> Option<&FaceAttr<P::F>> {
        self.faces.get(key)
    }

    /// Returns the face attribute registered under `key`.
    ///
    /// # Panics
    ///
    /// Panics if no face is registered under `key`.
    pub fn face_attr_unchecked(&self, key: FaceKey) -> &FaceAttr<P::F> {
        self.face_attr(key).expect("face key should be in map")
    }

    pub(crate) fn face_attr_mut(&mut self, key: FaceKey) -> Option<&mut FaceAttr<P::F>> {
        self.invalidate_derived_indexes();
        self.faces.get_mut(key)
    }

    pub(crate) fn face_attr_mut_unchecked(&mut self, key: FaceKey) -> &mut FaceAttr<P::F> {
        self.face_attr_mut(key).expect("face key should be in map")
    }

    /// Iterates every registered face with its attribute.
    pub fn iter_faces(&self) -> impl Iterator<Item = (FaceKey, &FaceAttr<P::F>)> {
        self.faces.iter()
    }

    /// Returns the typed sheet view registered under `key`.
    pub fn sheet(&self, key: SheetKey) -> Option<Sheet<'_, P>> {
        self.sheet_attr(key)?;
        Some(Sheet::new(self, key))
    }

    /// Returns the typed sheet view registered under `key`.
    ///
    /// # Panics
    ///
    /// Panics if no sheet is registered under `key`.
    pub fn sheet_unchecked(&self, key: SheetKey) -> Sheet<'_, P> {
        self.sheet(key).expect("sheet should be in the model")
    }

    /// Returns the sheet key of the shell containing `dart`, if registered.
    pub fn sheet_key(&self, dart: Dart) -> Option<SheetKey> {
        let repr = self.cell_representative(dart, Dim::Three);
        self.derived_indexes().sheet.get(&repr).copied()
    }

    /// Returns the sheet key of the shell containing `dart`.
    ///
    /// # Panics
    ///
    /// Panics if no sheet is registered for the shell.
    pub fn sheet_key_unchecked(&self, dart: Dart) -> SheetKey {
        self.sheet_key(dart).expect("sheet key should be in map")
    }

    /// Returns the sheet holding `face`, including a boundaryless one.
    pub fn sheet_key_at_face(&self, face: FaceKey) -> Option<SheetKey> {
        self.sheets
            .iter()
            .find(|(_, attr)| attr.root.face() == Some(face))
            .map(|(key, _)| key)
    }

    /// Returns the solid bounded by a shell holding `face`.
    pub fn solid_key_at_face(&self, face: FaceKey) -> Option<SolidKey> {
        self.solids
            .iter()
            .find(|(_, attr)| attr.shells().any(|shell| shell.face() == Some(face)))
            .map(|(key, _)| key)
    }

    /// Returns the solid a merge handle landed in, whichever form it took.
    pub fn solid_key_at(&self, handle: MergeHandle) -> Option<SolidKey> {
        match handle {
            MergeHandle::Dart(dart) => self.solid_key(dart),
            MergeHandle::Face(face) => self.solid_key_at_face(face),
        }
    }

    /// Returns the sheet view anchored at `root`, whichever form it took.
    pub fn shell_sheet(&self, root: ShellRoot) -> Option<Sheet<'_, P>> {
        match root {
            ShellRoot::Dart(dart) => Sheet::from_dart(self, dart),
            ShellRoot::Face { face, .. } => self
                .sheet_key_at_face(face)
                .map(|key| Sheet::new(self, key)),
        }
    }

    /// Returns the sheet attribute registered under `key`.
    pub fn sheet_attr(&self, key: SheetKey) -> Option<&SheetAttr<P::Sheet>> {
        self.sheets.get(key)
    }

    /// Returns the sheet attribute registered under `key`.
    ///
    /// # Panics
    ///
    /// Panics if no sheet is registered under `key`.
    pub fn sheet_attr_unchecked(&self, key: SheetKey) -> &SheetAttr<P::Sheet> {
        self.sheet_attr(key).expect("sheet key should be in map")
    }

    pub(crate) fn sheet_attr_mut(&mut self, key: SheetKey) -> Option<&mut SheetAttr<P::Sheet>> {
        self.invalidate_derived_indexes();
        self.sheets.get_mut(key)
    }

    pub(crate) fn sheet_attr_mut_unchecked(&mut self, key: SheetKey) -> &mut SheetAttr<P::Sheet> {
        self.sheet_attr_mut(key)
            .expect("sheet key should be in map")
    }

    /// Iterates every registered sheet with its attribute.
    pub fn iter_sheets(&self) -> impl Iterator<Item = (SheetKey, &SheetAttr<P::Sheet>)> {
        self.sheets.iter()
    }

    /// Returns the typed solid view registered under `key`.
    pub fn solid(&self, key: SolidKey) -> Option<Solid<'_, P>> {
        self.solid_attr(key)?;
        Some(Solid::new(self, key))
    }

    /// Returns the typed solid view registered under `key`.
    ///
    /// # Panics
    ///
    /// Panics if no solid is registered under `key`.
    pub fn solid_unchecked(&self, key: SolidKey) -> Solid<'_, P> {
        self.solid(key).expect("solid should be in the model")
    }

    /// Returns the solid attribute registered under `key`.
    pub fn solid_attr(&self, key: SolidKey) -> Option<&SolidAttr<P::S>> {
        self.solids.get(key)
    }

    /// Returns the solid attribute registered under `key`.
    ///
    /// # Panics
    ///
    /// Panics if no solid is registered under `key`.
    pub fn solid_attr_unchecked(&self, key: SolidKey) -> &SolidAttr<P::S> {
        self.solid_attr(key).expect("solid key should be in map")
    }

    pub(crate) fn solid_attr_mut(&mut self, key: SolidKey) -> Option<&mut SolidAttr<P::S>> {
        self.invalidate_derived_indexes();
        self.solids.get_mut(key)
    }

    pub(crate) fn solid_attr_mut_unchecked(&mut self, key: SolidKey) -> &mut SolidAttr<P::S> {
        self.solid_attr_mut(key)
            .expect("solid key should be in map")
    }

    /// Returns the solid key of the shell containing `dart`, if registered.
    pub fn solid_key(&self, dart: Dart) -> Option<SolidKey> {
        let repr = self.cell_representative(dart, Dim::Three);
        self.derived_indexes().solid.get(&repr).copied()
    }

    /// Iterates every registered solid with its attribute.
    pub fn iter_solids(&self) -> impl Iterator<Item = (SolidKey, &SolidAttr<P::S>)> {
        self.solids.iter()
    }

    /// Returns the attribute associated with the `D`-cell containing `dart`.
    ///
    /// The lookup first canonicalizes `dart` to the representative of `D::DIM`.
    pub fn attribute<D: CellDim>(&self, dart: Dart) -> Option<&<Self as AttributeStore<D>>::Attr>
    where
        Self: AttributeStore<D>,
    {
        let repr = self.cell_representative(dart, D::DIM);
        self.get(repr)
    }

    /// Returns the attribute associated with the `D`-cell containing `dart`.
    ///
    /// # Panics
    ///
    /// Panics if no attribute is registered for the cell.
    pub fn attribute_unchecked<D: CellDim>(&self, dart: Dart) -> &<Self as AttributeStore<D>>::Attr
    where
        Self: AttributeStore<D>,
    {
        self.attribute::<D>(dart)
            .expect("attribute should be in the model")
    }
}

/// Copying topology between models.
impl<P: Payload> Model<P> {
    /// Copy a topological view into a fresh [`Model`].
    ///
    /// This is the associated-function form of [`MergeTopology::isolate`].
    pub fn isolate<T>(topology: T) -> (Self, MergeHandle)
    where
        T: MergeTopology<P>,
    {
        topology.isolate()
    }

    /// Merge a topological view into this model, returning the view's
    /// representative dart rewritten to the destination.
    ///
    /// All darts in the view are copied. Alpha links within those darts are
    /// preserved; links leaving the view become free. Stored vertex, edge, face,
    /// and solid attributes whose representative darts are part of the view are
    /// cloned with embedded dart references remapped to the new dart ids.
    pub(crate) fn merge<T>(&mut self, topology: T) -> MergeHandle
    where
        T: MergeTopology<P>,
    {
        let topology = topology.merge_topology();
        let source = topology.source;
        let handle = topology.handle;
        let source_faces = topology.faces;
        let mut seen_darts = HashSet::new();
        let source_darts = topology
            .darts
            .into_iter()
            .filter(|dart| seen_darts.insert(*dart))
            .collect::<Vec<_>>();
        let source_dart_set = source_darts.iter().copied().collect::<HashSet<_>>();
        let mut dart_map = HashMap::with_capacity(source_darts.len());

        for old in source_darts.iter().copied() {
            let new = self.add_dart();
            dart_map.insert(old, new);
        }

        for old in source_darts.iter().copied() {
            let new = remap_dart(&dart_map, old);
            for i in 0..self.dimension() {
                let dim = Dim::from_index(i);
                let old_link = source.alpha(dim, old);
                let target = dart_map.get(&old_link).copied().unwrap_or(new);
                self.topology.point_alpha(dim, new, target);
            }
        }

        let mut vertex_map = HashMap::new();
        for (old, attr) in source.vertices.iter() {
            let Some(attribute_dart) =
                copied_cell_dart(source, &source_dart_set, attr.dart, Dim::Zero)
            else {
                continue;
            };
            let mut attr = attr.clone();
            attr.dart = self.cell_representative(remap_dart(&dart_map, attribute_dart), Dim::Zero);
            let new_key = self.vertices.insert(attr);
            self.record_created_attribute(EditKey::Vertex(new_key));
            vertex_map.insert(old, new_key);
        }

        let mut edge_map = HashMap::new();
        for (old, attr) in source.edges.iter() {
            let Some(attribute_dart) =
                copied_cell_dart(source, &source_dart_set, attr.dart, Dim::One)
            else {
                continue;
            };
            let mut attr = attr.clone();
            attr.dart = remap_dart(&dart_map, attribute_dart);
            let new_key = self.edges.insert(attr);
            self.record_created_attribute(EditKey::Edge(new_key));
            edge_map.insert(old, new_key);
        }

        for (_, attr) in source.profiles.iter() {
            if !source
                .orbit(attr.dart, vec![Dim::Zero.index(), Dim::One.index()])
                .all(|dart| source_dart_set.contains(&dart))
            {
                continue;
            }
            let mut attr = attr.clone();
            attr.dart = remap_dart(&dart_map, attr.dart);
            let new_key = self.profiles.insert(attr);
            self.record_created_attribute(EditKey::Profile(new_key));
        }

        let mut face_map = HashMap::new();
        for (old, attr) in source.faces.iter() {
            let Some(seed) = attr.seed() else {
                continue;
            };
            if !source_dart_set.contains(&seed) {
                continue;
            }
            let mut attr = attr.clone();
            attr.retain_mapped(&dart_map);
            if attr.is_empty() {
                continue;
            }
            attr.pcurves = attr
                .pcurves
                .into_iter()
                .filter_map(|(dart, curve)| dart_map.get(&dart).copied().map(|d| (d, curve)))
                .collect();
            let new_key = self.faces.insert(attr);
            self.record_created_attribute(EditKey::Face(new_key));
            face_map.insert(old, new_key);
        }

        // A merge is otherwise defined by the darts it copies, and a
        // boundaryless face has none: it is named outright by the caller, and
        // the map from its old key to its new one is what lets the shells that
        // hold it come across too.
        face_map.reserve(source_faces.len());
        for old in source_faces {
            let Some(attr) = source.faces.get(old) else {
                continue;
            };
            let new_key = self.faces.insert(attr.clone());
            self.record_created_attribute(EditKey::Face(new_key));
            face_map.insert(old, new_key);
        }

        for (_, attr) in source.sheets.iter() {
            let root = match attr.root {
                ShellRoot::Dart(root) => {
                    if !source
                        .orbit(root, vec![0, 1, 2])
                        .all(|dart| source_dart_set.contains(&dart))
                    {
                        continue;
                    }
                    ShellRoot::Dart(remap_dart(&dart_map, root))
                }
                ShellRoot::Face { face, sense } => match face_map.get(&face) {
                    Some(&face) => ShellRoot::Face { face, sense },
                    None => continue,
                },
            };
            let mut attr = attr.clone();
            attr.root = root;
            let new_key = self.sheets.insert(attr);
            self.record_created_attribute(EditKey::Sheet(new_key));
        }

        let copied_shell = |shell: ShellRoot| match shell {
            ShellRoot::Dart(dart) => source
                .orbit(dart, vec![0, 1, 2])
                .all(|dart| source_dart_set.contains(&dart))
                .then(|| ShellRoot::Dart(remap_dart(&dart_map, dart))),
            ShellRoot::Face { face, sense } => face_map
                .get(&face)
                .map(|&face| ShellRoot::Face { face, sense }),
        };
        let mut solid_map = HashMap::new();
        for (old, attr) in source.solids.iter() {
            let Some(outer_shell) = copied_shell(attr.outer_shell) else {
                continue;
            };
            let mut attr = attr.clone();
            attr.outer_shell = outer_shell;
            attr.inner_shells = attr
                .inner_shells
                .map(|shells| shells.into_iter().filter_map(copied_shell).collect());
            let new_key = self.solids.insert(attr);
            self.record_created_attribute(EditKey::Solid(new_key));
            solid_map.insert(old, new_key);
        }

        self.subdivision.extend_remapped(
            &source.subdivision,
            &dart_map,
            &OwnerRemap {
                vertices: &vertex_map,
                edges: &edge_map,
                faces: &face_map,
                solids: &solid_map,
            },
        );

        self.invalidate_derived_indexes();
        match handle {
            MergeHandle::Dart(dart) => MergeHandle::Dart(remap_dart(&dart_map, dart)),
            MergeHandle::Face(face) => MergeHandle::Face(
                face_map
                    .get(&face)
                    .copied()
                    .expect("a face named as the merge handle should be copied"),
            ),
        }
    }
}
