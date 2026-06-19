use std::collections::{HashMap, HashSet, VecDeque};

use slotmap::SlotMap;

use crate::topology::edge::Edge;
use crate::topology::face::Face;
use crate::topology::orientation::Orientation;
use crate::topology::shape_keys::{EdgeKey, FaceKey, ProfileKey, SheetKey, SolidKey, VertexKey};
use crate::topology::sheet::Sheet;
use crate::topology::solid::Solid;
use crate::topology::vertex::Vertex;

use super::attributes::{EdgeAttr, FaceAttr, ProfileAttr, SheetAttr, SolidAttr, VertexAttr};
use super::payload::{Payload, StandardPayload};

pub use super::dart::{Dart, IsolatedDart};

/// Topological cell dimension and matching alpha involution index.
///
/// `Dim::Zero` corresponds to vertices and alpha0, `Dim::One` to edges and
/// alpha1, and so on up to solids/sheets and alpha3.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Dim {
    /// Vertex dimension / alpha0.
    Zero,
    /// Edge dimension / alpha1.
    One,
    /// Face dimension / alpha2.
    Two,
    /// Sheet or solid dimension / alpha3.
    Three,
}

impl Dim {
    /// Returns the alpha index associated with this dimension.
    pub fn index(&self) -> usize {
        match self {
            Dim::Zero => 0,
            Dim::One => 1,
            Dim::Two => 2,
            Dim::Three => 3,
        }
    }

    /// Converts an alpha index in `0..=3` back to a [`Dim`].
    ///
    /// # Panics
    ///
    /// Panics for values outside the supported 3-gmap involution range.
    pub fn from_index(i: usize) -> Self {
        match i {
            0 => Dim::Zero,
            1 => Dim::One,
            2 => Dim::Two,
            3 => Dim::Three,
            _ => panic!("Dim::from_index: invalid index {i}"),
        }
    }
}

/// Number of alpha involutions in this 3-gmap implementation.
pub const GMAP_INVOLUTION_COUNT: usize = 4;
/// Pairing map computed while checking whether two dart orbits can be sewn.
pub struct SewableDarts {
    mapping: HashMap<Dart, Dart>,
}

/// Type marker for vertex attributes.
pub struct Cell0;
/// Type marker for edge attributes.
pub struct Cell1;
/// Type marker for face attributes.
pub struct Cell2;
/// Type marker for solid attributes.
pub struct Cell3;

/// Compile-time mapping from a cell marker to its dimension and key type.
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
/// Most callers should use [`GMap::attribute`] and [`GMap::attribute_mut`]
/// instead of calling this trait directly.
pub trait AttributeStore<D: CellDim> {
    /// Attribute type stored for this dimension.
    type Attr;
    /// Returns the attribute associated with canonical representative `repr`.
    fn get(&self, repr: Dart) -> Option<&Self::Attr>;
    /// Returns the mutable attribute associated with canonical representative
    /// `repr`.
    fn get_mut(&mut self, repr: Dart) -> Option<&mut Self::Attr>;
}

/// Trait for looking up a cell key from a canonical representative dart.
pub(crate) trait CellKeyLookup<D: CellDim> {
    fn get_key(&self, repr: Dart) -> Option<D::Key>;
}

impl<P: Payload> CellKeyLookup<Cell0> for GMap<P> {
    fn get_key(&self, repr: Dart) -> Option<VertexKey> {
        self.dart_to_vertex.get(&repr).copied()
    }
}
impl<P: Payload> CellKeyLookup<Cell1> for GMap<P> {
    fn get_key(&self, repr: Dart) -> Option<EdgeKey> {
        self.dart_to_edge.get(&repr).copied()
    }
}
impl<P: Payload> CellKeyLookup<Cell2> for GMap<P> {
    fn get_key(&self, repr: Dart) -> Option<FaceKey> {
        self.dart_to_face.get(&repr).copied()
    }
}
impl<P: Payload> CellKeyLookup<Cell3> for GMap<P> {
    fn get_key(&self, repr: Dart) -> Option<SolidKey> {
        self.dart_to_solid.get(&repr).copied()
    }
}

fn remap_dart(dart_map: &HashMap<Dart, Dart>, dart: Dart) -> Dart {
    *dart_map
        .get(&dart)
        .expect("merged dart reference must have a remapped dart")
}

fn copied_cell_dart<P: Payload>(
    source: &GMap<P>,
    copied_darts: &HashSet<Dart>,
    dart: Dart,
    dim: Dim,
) -> Option<Dart> {
    source
        .orbit(dart, source.orbit_indices(dim))
        .find(|candidate| copied_darts.contains(candidate))
}

/// Source topology selected for copying into another [`GMap`].
///
/// Construct this from a topology view's owning map, the darts to copy, and the
/// representative dart that should be returned after remapping.
pub struct TopologyMerge<'a, P: Payload> {
    source: &'a GMap<P>,
    darts: Vec<Dart>,
    handle: Dart,
}

impl<'a, P: Payload> TopologyMerge<'a, P> {
    /// Creates a merge descriptor for a topology view.
    pub fn new(source: &'a GMap<P>, darts: Vec<Dart>, handle: Dart) -> Self {
        Self {
            source,
            darts,
            handle,
        }
    }
}

/// Topological views that can be copied into another [`GMap`].
pub trait MergeTopology<P: Payload> {
    /// Returns the topology subset that should be copied.
    fn merge_topology(&self) -> TopologyMerge<'_, P>;

    /// Copy this topology into a fresh [`GMap`], returning the copied map and
    /// this topology's representative dart rewritten to the new map.
    ///
    /// Alpha links within the copied topology are preserved. Links leaving the
    /// copied dart set become free in the isolated map.
    fn isolate(self) -> (GMap<P>, Dart)
    where
        Self: Sized,
    {
        let mut isolated = GMap::new();
        let dart = isolated.merge(self);
        (isolated, dart)
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

impl<P: Payload> AttributeStore<Cell0> for GMap<P> {
    type Attr = VertexAttr<P::V>;
    fn get(&self, repr: Dart) -> Option<&VertexAttr<P::V>> {
        let vid = self.dart_to_vertex.get(&repr)?;
        self.vertices.get(*vid)
    }
    fn get_mut(&mut self, repr: Dart) -> Option<&mut VertexAttr<P::V>> {
        let vid = self.dart_to_vertex.get(&repr)?;
        self.vertices.get_mut(*vid)
    }
}
impl<P: Payload> AttributeStore<Cell1> for GMap<P> {
    type Attr = EdgeAttr<P::E>;
    fn get(&self, repr: Dart) -> Option<&EdgeAttr<P::E>> {
        let eid = self.dart_to_edge.get(&repr)?;
        self.edges.get(*eid)
    }
    fn get_mut(&mut self, repr: Dart) -> Option<&mut EdgeAttr<P::E>> {
        let eid = self.dart_to_edge.get(&repr)?;
        self.edges.get_mut(*eid)
    }
}
impl<P: Payload> AttributeStore<Cell2> for GMap<P> {
    type Attr = FaceKey;
    fn get(&self, repr: Dart) -> Option<&FaceKey> {
        self.dart_to_face.get(&repr)
    }
    fn get_mut(&mut self, repr: Dart) -> Option<&mut FaceKey> {
        self.dart_to_face.get_mut(&repr)
    }
}
impl<P: Payload> AttributeStore<Cell3> for GMap<P> {
    type Attr = SolidKey;
    fn get(&self, repr: Dart) -> Option<&SolidKey> {
        self.dart_to_solid.get(&repr)
    }
    fn get_mut(&mut self, repr: Dart) -> Option<&mut SolidKey> {
        self.dart_to_solid.get_mut(&repr)
    }
}

/// A 3-dimensional generalized map with typed attribute stores.
///
/// The map owns all darts, alpha involutions, and domain attributes. Prefer the
/// typed view objects (`Vertex`, `Edge`, `Face`, `Sheet`, `Solid`) for routine
/// traversal, and use `GMap` methods when implementing lower-level topology
/// algorithms.
pub struct GMap<P: Payload = StandardPayload> {
    alphas: [Vec<Dart>; GMAP_INVOLUTION_COUNT],
    free_slots: VecDeque<usize>,
    pub(crate) dart_to_vertex: HashMap<Dart, VertexKey>,
    pub(crate) dart_to_edge: HashMap<Dart, EdgeKey>,
    pub(crate) dart_to_profile: HashMap<Dart, ProfileKey>,
    pub(crate) dart_to_face: HashMap<Dart, FaceKey>,
    pub(crate) dart_to_sheet: HashMap<Dart, SheetKey>,
    pub(crate) dart_to_solid: HashMap<Dart, SolidKey>,
    pub(crate) vertices: SlotMap<VertexKey, VertexAttr<P::V>>,
    pub(crate) edges: SlotMap<EdgeKey, EdgeAttr<P::E>>,
    pub(crate) profiles: SlotMap<ProfileKey, ProfileAttr<P::Profile>>,
    pub(crate) faces: SlotMap<FaceKey, FaceAttr<P::F>>,
    pub(crate) sheets: SlotMap<SheetKey, SheetAttr<P::Sheet>>,
    pub(crate) solids: SlotMap<SolidKey, SolidAttr<P::S>>,
}

impl<P: Payload> Clone for GMap<P> {
    fn clone(&self) -> Self {
        Self {
            alphas: self.alphas.clone(),
            free_slots: self.free_slots.clone(),
            vertices: self.vertices.clone(),
            dart_to_vertex: self.dart_to_vertex.clone(),
            edges: self.edges.clone(),
            dart_to_edge: self.dart_to_edge.clone(),
            profiles: self.profiles.clone(),
            dart_to_profile: self.dart_to_profile.clone(),
            dart_to_face: self.dart_to_face.clone(),
            sheets: self.sheets.clone(),
            dart_to_sheet: self.dart_to_sheet.clone(),
            dart_to_solid: self.dart_to_solid.clone(),
            faces: self.faces.clone(),
            solids: self.solids.clone(),
        }
    }
}

impl<P: Payload> Default for GMap<P> {
    fn default() -> Self {
        Self::new()
    }
}

impl<P: Payload> GMap<P> {
    /// Creates an empty map with no darts or attributes.
    pub fn new() -> Self {
        let alphas = std::array::from_fn(|_| Vec::new());
        let free_slots = VecDeque::new();
        let vertices = SlotMap::with_key();
        let dart_to_vertex = HashMap::new();
        let edges = SlotMap::with_key();
        let dart_to_edge = HashMap::new();
        let profiles = SlotMap::with_key();
        let dart_to_profile = HashMap::new();
        let dart_to_face = HashMap::new();
        let sheets = SlotMap::with_key();
        let dart_to_sheet = HashMap::new();
        let dart_to_solid = HashMap::new();
        let faces = SlotMap::with_key();
        let solids = SlotMap::with_key();
        Self {
            alphas,
            free_slots,
            vertices,
            dart_to_vertex,
            edges,
            dart_to_edge,
            profiles,
            dart_to_profile,
            dart_to_face,
            sheets,
            dart_to_sheet,
            dart_to_solid,
            faces,
            solids,
        }
    }

    /// Returns the number of alpha involutions.
    ///
    /// This is always [`GMAP_INVOLUTION_COUNT`] for the current 3-gmap.
    pub fn dimension(&self) -> usize {
        GMAP_INVOLUTION_COUNT
    }

    /// Returns the number of dart slots in the map.
    pub fn dart_count(&self) -> usize {
        self.alphas[0].len()
    }

    /// Iterates all dart identifiers currently addressable in the map.
    pub fn darts(&self) -> impl Iterator<Item = Dart> + '_ {
        (0..self.dart_count()).map(Dart::new)
    }

    /// Returns `alpha_d(dart)`.
    ///
    /// # Panics
    ///
    /// Panics if `dart` does not address an existing dart slot.
    pub fn alpha(&self, d: Dim, dart: Dart) -> Dart {
        let i = d.index();
        self.alphas[i][dart.id()]
    }

    /// Adds one isolated dart and returns its identifier.
    ///
    /// All alpha involutions initially map the new dart to itself.
    pub fn add_dart(&mut self) -> Dart {
        let dart = if let Some(slot) = self.free_slots.pop_front() {
            Dart::new(slot)
        } else {
            Dart::new(self.alphas[0].len())
        };
        for alphas in self.alphas.iter_mut() {
            alphas.push(dart);
        }
        dart
    }

    /// Removes a dart that the caller has proven isolated.
    ///
    /// The [`IsolatedDart`] wrapper records the caller's proof obligation.
    pub fn remove_dart(&mut self, dart: IsolatedDart) {
        for alphas in self.alphas.iter_mut() {
            alphas.remove(dart.id());
        }
        self.free_slots.push_back(dart.id());
    }

    /// Iterates the orbit generated from `dart` by the given alpha indices.
    ///
    /// For cell traversals, prefer [`Self::orbit_indices`] and typed view
    /// methods when possible.
    pub fn orbit(&self, dart: Dart, involutions: Vec<usize>) -> OrbitIterator<'_, P> {
        OrbitIterator::new(self, dart, involutions)
    }

    /// A dart is `i`-free when `αᵢ(d) = d`, i.e. not sewn along dimension `i`.
    pub fn is_free(&self, dart: Dart, d: Dim) -> bool {
        self.alphas[d.index()][dart.id()] == dart
    }

    /// Returns the alpha indices used to compare sewing orbits.
    fn sewing_orbit_indices(&self, d: Dim) -> impl Iterator<Item = usize> + '_ {
        let i = d.index();
        (0..self.dimension()).filter(move |&j| j + 2 <= i || j >= i + 2)
    }

    /// Returns the alpha indices that generate a cell orbit of dimension `d`.
    ///
    /// For example, the edge orbit excludes alpha1 and includes every other
    /// alpha index.
    pub fn orbit_indices(&self, d: Dim) -> Vec<usize> {
        let i = d.index();
        (0..self.dimension()).filter(|&idx| idx != i).collect()
    }

    /// Adds a vertex attribute and returns its key.
    ///
    /// # Panics
    ///
    /// Panics if a vertex attribute is already registered for the same 0-cell.
    pub fn add_vertex(&mut self, vertex: VertexAttr<P::V>) -> VertexKey {
        let dart = self.cell_representative(vertex.dart, Dim::Zero);
        assert!(
            self.dart_to_vertex.get(&dart).is_none(),
            "A vertex is already attached to this 0-cell"
        );
        let mut vertex = vertex;
        vertex.dart = dart;
        let key = self.vertices.insert(vertex);
        self.dart_to_vertex.insert(dart, key);
        key
    }

    /// Returns the typed vertex view registered under `key`.
    pub fn vertex(&self, key: VertexKey) -> Option<Vertex<'_, P>> {
        let attr = self.vertex_attr(key)?;
        Some(Vertex::new(self, attr.dart))
    }

    /// Returns the typed vertex view registered under `key`.
    ///
    /// # Panics
    ///
    /// Panics if `key` is not a registered vertex.
    pub fn vertex_unchecked(&self, key: VertexKey) -> Vertex<'_, P> {
        self.vertex(key).expect("vertex should be in the map")
    }

    /// Returns the vertex attribute for `key`, if it exists.
    pub fn vertex_attr(&self, key: VertexKey) -> Option<&VertexAttr<P::V>> {
        self.vertices.get(key)
    }

    /// Returns the vertex attribute for `key`.
    ///
    /// # Panics
    ///
    /// Panics if `key` is not a registered vertex.
    pub fn vertex_attr_unchecked(&self, key: VertexKey) -> &VertexAttr<P::V> {
        self.vertex_attr(key)
            .expect("vertex attribute should be in the map")
    }

    /// Returns the mutable vertex attribute for `key`, if it exists.
    pub fn vertex_attr_mut(&mut self, key: VertexKey) -> Option<&mut VertexAttr<P::V>> {
        self.vertices.get_mut(key)
    }

    /// Returns the mutable vertex attribute for `key`.
    ///
    /// # Panics
    ///
    /// Panics if `key` is not a registered vertex.
    pub fn vertex_attr_mut_unchecked(&mut self, key: VertexKey) -> &mut VertexAttr<P::V> {
        self.vertex_attr_mut(key)
            .expect("vertex attribute should be in the map")
    }

    pub(crate) fn remove_vertex(&mut self, key: VertexKey) -> Option<VertexAttr<P::V>> {
        let vertex = self.vertices.remove(key)?;
        let representative = self.cell_representative(vertex.dart, Dim::Zero);
        if self.dart_to_vertex.get(&representative) == Some(&key) {
            self.dart_to_vertex.remove(&representative);
        }
        Some(vertex)
    }

    /// Iterate every stored 0-cell attribute paired with its slotmap key.
    pub fn iter_vertices(&self) -> impl Iterator<Item = (VertexKey, &VertexAttr<P::V>)> {
        self.vertices.iter()
    }

    /// Adds an edge attribute and returns its key.
    ///
    /// The caller's `edge.dart` is preserved as the orientation-defining locator.
    ///
    /// # Panics
    ///
    /// Panics if an edge attribute is already registered for the same 1-cell.
    pub fn add_edge(&mut self, edge: EdgeAttr<P::E>) -> EdgeKey {
        let repr = self.cell_representative(edge.dart, Dim::One);
        assert!(
            self.dart_to_edge.get(&repr).is_none(),
            "An edge is already attached to this 1-cell"
        );
        let key = self.edges.insert(edge);
        self.dart_to_edge.insert(repr, key);
        key
    }

    /// Returns the typed edge view registered under `key` with default
    /// (`Same`) orientation.
    pub fn edge(&self, key: EdgeKey) -> Option<Edge<'_, P>> {
        self.edge_attr(key)?;
        Some(Edge::new(self, key))
    }

    /// Returns the typed edge view registered under `key` with default
    /// (`Same`) orientation.
    ///
    /// # Panics
    ///
    /// Panics if `key` is not a registered edge.
    pub fn edge_unchecked(&self, key: EdgeKey) -> Edge<'_, P> {
        self.edge(key).expect("edge should be in the map")
    }

    /// Returns the key of the `D`-cell containing `dart`.
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
            .expect("cell key should be in the map")
    }

    /// Returns the orientation of `dart` relative to the edge's default
    /// direction.
    ///
    /// # Panics
    ///
    /// Panics if `key` is not a registered edge or `dart` does not belong to
    /// that edge.
    pub fn edge_orientation_at_dart(&self, key: EdgeKey, dart: Dart) -> Orientation {
        let attr = self.edge_attr_unchecked(key);
        self.cell_orientation_from_seed(attr.dart, dart, Dim::One)
            .expect("edge orientation requires dart to belong to edge")
    }

    /// Returns the edge attribute for `key`, if it exists.
    pub fn edge_attr(&self, key: EdgeKey) -> Option<&EdgeAttr<P::E>> {
        self.edges.get(key)
    }

    /// Returns the edge attribute for `key`.
    ///
    /// # Panics
    ///
    /// Panics if `key` is not a registered edge.
    pub fn edge_attr_unchecked(&self, key: EdgeKey) -> &EdgeAttr<P::E> {
        self.edge_attr(key)
            .expect("edge attribute should be in the map")
    }

    /// Returns the mutable edge attribute for `key`, if it exists.
    pub fn edge_attr_mut(&mut self, key: EdgeKey) -> Option<&mut EdgeAttr<P::E>> {
        self.edges.get_mut(key)
    }

    /// Returns the mutable edge attribute for `key`.
    ///
    /// # Panics
    ///
    /// Panics if `key` is not a registered edge.
    pub fn edge_attr_mut_unchecked(&mut self, key: EdgeKey) -> &mut EdgeAttr<P::E> {
        self.edge_attr_mut(key)
            .expect("edge attribute should be in the map")
    }

    pub(crate) fn remove_edge(&mut self, key: EdgeKey) -> Option<EdgeAttr<P::E>> {
        let edge = self.edges.remove(key)?;
        let representative = self.cell_representative(edge.dart, Dim::One);
        if self.dart_to_edge.get(&representative) == Some(&key) {
            self.dart_to_edge.remove(&representative);
        }
        Some(edge)
    }

    /// Iterate every stored 1-cell attribute paired with its slotmap key.
    pub fn iter_edges(&self) -> impl Iterator<Item = (EdgeKey, &EdgeAttr<P::E>)> {
        self.edges.iter()
    }

    /// Adds a profile identity while preserving the caller's oriented root.
    ///
    /// # Panics
    ///
    /// Panics if the same alpha0/alpha1 component already has a profile key.
    pub fn add_profile(&mut self, profile: ProfileAttr<P::Profile>) -> ProfileKey {
        let repr = self.profile_representative(profile.dart);
        assert!(
            !self.dart_to_profile.contains_key(&repr),
            "A profile is already attached to this alpha0/alpha1 component"
        );
        let key = self.profiles.insert(profile);
        self.dart_to_profile.insert(repr, key);
        key
    }

    /// Returns the profile view registered under `key`.
    pub fn profile(&self, key: ProfileKey) -> Option<crate::topology::profile::Profile<'_, P>> {
        let attr = self.profile_attr(key)?;
        Some(crate::topology::profile::Profile::new(self, attr.dart))
    }

    /// Returns the profile view registered under `key`.
    ///
    /// # Panics
    ///
    /// Panics if `key` is not a registered profile.
    pub fn profile_unchecked(&self, key: ProfileKey) -> crate::topology::profile::Profile<'_, P> {
        self.profile(key).expect("profile should be in the map")
    }

    /// Returns the profile key for the alpha0/alpha1 component containing `dart`.
    pub fn profile_key(&self, dart: Dart) -> Option<ProfileKey> {
        self.dart_to_profile
            .get(&self.profile_representative(dart))
            .copied()
    }

    /// Returns the profile key for the alpha0/alpha1 component containing `dart`.
    ///
    /// # Panics
    ///
    /// Panics if no profile key is registered for the component.
    pub fn profile_key_unchecked(&self, dart: Dart) -> ProfileKey {
        self.profile_key(dart)
            .expect("profile key should be in the map")
    }

    /// Returns the stored profile attribute.
    pub fn profile_attr(&self, key: ProfileKey) -> Option<&ProfileAttr<P::Profile>> {
        self.profiles.get(key)
    }

    /// Returns the stored profile attribute.
    ///
    /// # Panics
    ///
    /// Panics if `key` is not a registered profile.
    pub fn profile_attr_unchecked(&self, key: ProfileKey) -> &ProfileAttr<P::Profile> {
        self.profile_attr(key)
            .expect("profile attribute should be in the map")
    }

    /// Returns the mutable profile attribute.
    pub fn profile_attr_mut(&mut self, key: ProfileKey) -> Option<&mut ProfileAttr<P::Profile>> {
        self.profiles.get_mut(key)
    }

    /// Returns the mutable profile attribute.
    ///
    /// # Panics
    ///
    /// Panics if `key` is not a registered profile.
    pub fn profile_attr_mut_unchecked(&mut self, key: ProfileKey) -> &mut ProfileAttr<P::Profile> {
        self.profile_attr_mut(key)
            .expect("profile attribute should be in the map")
    }

    /// Iterates all stored profiles.
    pub fn iter_profiles(&self) -> impl Iterator<Item = (ProfileKey, &ProfileAttr<P::Profile>)> {
        self.profiles.iter()
    }

    /// Adds a face attribute and returns its key.
    ///
    /// # Panics
    ///
    /// Panics if any boundary 2-cell is already attached to a face.
    pub fn add_face(&mut self, face: FaceAttr<P::F>) -> FaceKey {
        let reprs = std::iter::once(face.outer_loop)
            .chain(face.inner_loops.iter().copied())
            .find_map(|d| {
                let repr = self.cell_representative(d, Dim::Two);
                self.dart_to_face.get(&repr)
            });
        assert!(
            reprs.is_none(),
            "A face is already attached to one of the boundary darts"
        );
        let key = self.faces.insert(face);
        self.index_face_loop_darts(key);
        key
    }

    /// Returns the typed face view registered under `key` with default
    /// (`Same`) orientation.
    pub fn face(&self, key: FaceKey) -> Option<Face<'_, P>> {
        self.face_attr(key)?;
        Some(Face::new(self, key))
    }

    /// Returns the typed face view registered under `key` with default
    /// (`Same`) orientation.
    ///
    /// # Panics
    ///
    /// Panics if `key` is not a registered face.
    pub fn face_unchecked(&self, key: FaceKey) -> Face<'_, P> {
        self.face(key).expect("face should be in the map")
    }

    /// Returns the orientation of the face side traversed at `dart` relative
    /// to the face's default orientation.
    ///
    /// # Panics
    ///
    /// Panics if `key` is not a registered face or `dart` does not belong to
    /// one of that face's boundary components.
    pub fn face_orientation_at_dart(&self, key: FaceKey, dart: Dart) -> Orientation {
        let attr = self.face_attr_unchecked(key);
        std::iter::once(attr.outer_loop)
            .chain(attr.inner_loops.iter().copied())
            .find_map(|seed| self.cell_orientation_from_seed(seed, dart, Dim::Two))
            .expect("face orientation requires dart to belong to face")
    }

    fn index_face_loop_darts(&mut self, key: FaceKey) {
        let attr = &self.faces[key]; // This exists because attr has just been added
        for seed in std::iter::once(attr.outer_loop).chain(attr.inner_loops.iter().copied()) {
            let repr = self.cell_representative(seed, Dim::Two);
            self.dart_to_face.insert(repr, key);
        }
    }

    fn cell_orientation_from_seed(
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
                let linked = self.alphas[index][dart.id()];
                if linked == dart || orientations[linked.id()].is_some() {
                    continue;
                }
                // Lower-dimensional flag changes reverse the cell. Higher-
                // dimensional incidence changes preserve its intrinsic orientation.
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

    /// Returns the face attribute for `key`, if it exists.
    pub fn face_attr(&self, key: FaceKey) -> Option<&FaceAttr<P::F>> {
        self.faces.get(key)
    }

    /// Returns the face attribute for `key`.
    ///
    /// # Panics
    ///
    /// Panics if `key` is not a registered face.
    pub fn face_attr_unchecked(&self, key: FaceKey) -> &FaceAttr<P::F> {
        self.face_attr(key)
            .expect("face attribute should be in the map")
    }

    /// Returns the mutable face attribute for `key`, if it exists.
    pub fn face_attr_mut(&mut self, key: FaceKey) -> Option<&mut FaceAttr<P::F>> {
        self.faces.get_mut(key)
    }

    /// Returns the mutable face attribute for `key`.
    ///
    /// # Panics
    ///
    /// Panics if `key` is not a registered face.
    pub fn face_attr_mut_unchecked(&mut self, key: FaceKey) -> &mut FaceAttr<P::F> {
        self.face_attr_mut(key)
            .expect("face attribute should be in the map")
    }

    pub(crate) fn remove_face(&mut self, key: FaceKey) -> Option<FaceAttr<P::F>> {
        let face = self.faces.remove(key)?;
        for seed in std::iter::once(face.outer_loop).chain(face.inner_loops.iter().copied()) {
            let repr = self.cell_representative(seed, Dim::Two);
            self.dart_to_face.remove(&repr);
        }
        Some(face)
    }

    /// Iterate every stored 2-cell attribute paired with its slotmap key.
    pub fn iter_faces(&self) -> impl Iterator<Item = (FaceKey, &FaceAttr<P::F>)> {
        self.faces.iter()
    }

    /// Adds a sheet identity while preserving the caller's oriented root.
    ///
    /// # Panics
    ///
    /// Panics if the same 3-cell already has a sheet key.
    pub fn add_sheet(&mut self, sheet: SheetAttr<P::Sheet>) -> SheetKey {
        let repr = self.cell_representative(sheet.dart, Dim::Three);
        assert!(
            !self.dart_to_sheet.contains_key(&repr),
            "A sheet is already attached to this 3-cell"
        );
        let key = self.sheets.insert(sheet);
        self.dart_to_sheet.insert(repr, key);
        key
    }

    /// Returns the sheet view registered under `key`.
    pub fn sheet(&self, key: SheetKey) -> Option<Sheet<'_, P>> {
        let attr = self.sheet_attr(key)?;
        Some(crate::topology::sheet::Sheet::new(self, attr.dart))
    }

    /// Returns the sheet view registered under `key`.
    ///
    /// # Panics
    ///
    /// Panics if `key` is not a registered sheet.
    pub fn sheet_unchecked(&self, key: SheetKey) -> Sheet<'_, P> {
        self.sheet(key).expect("sheet should be in the map")
    }

    /// Returns the sheet key for the 3-cell containing `dart`.
    pub fn sheet_key(&self, dart: Dart) -> Option<SheetKey> {
        self.dart_to_sheet
            .get(&self.cell_representative(dart, Dim::Three))
            .copied()
    }

    /// Returns the sheet key for the 3-cell containing `dart`.
    ///
    /// # Panics
    ///
    /// Panics if no sheet key is registered for the 3-cell.
    pub fn sheet_key_unchecked(&self, dart: Dart) -> SheetKey {
        self.sheet_key(dart)
            .expect("sheet key should be in the map")
    }

    /// Returns the stored sheet attribute.
    pub fn sheet_attr(&self, key: SheetKey) -> Option<&SheetAttr<P::Sheet>> {
        self.sheets.get(key)
    }

    /// Returns the stored sheet attribute.
    ///
    /// # Panics
    ///
    /// Panics if `key` is not a registered sheet.
    pub fn sheet_attr_unchecked(&self, key: SheetKey) -> &SheetAttr<P::Sheet> {
        self.sheet_attr(key)
            .expect("sheet attribute should be in the map")
    }

    /// Returns the mutable sheet attribute.
    pub fn sheet_attr_mut(&mut self, key: SheetKey) -> Option<&mut SheetAttr<P::Sheet>> {
        self.sheets.get_mut(key)
    }

    /// Returns the mutable sheet attribute.
    ///
    /// # Panics
    ///
    /// Panics if `key` is not a registered sheet.
    pub fn sheet_attr_mut_unchecked(&mut self, key: SheetKey) -> &mut SheetAttr<P::Sheet> {
        self.sheet_attr_mut(key)
            .expect("sheet attribute should be in the map")
    }

    /// Iterates all stored sheets.
    pub fn iter_sheets(&self) -> impl Iterator<Item = (SheetKey, &SheetAttr<P::Sheet>)> {
        self.sheets.iter()
    }

    /// Adds a solid attribute and returns its key.
    ///
    /// If any shell is already attached to a solid, the existing solid key is
    /// # Panics
    ///
    /// Panics if any shell is already attached to a solid.
    pub fn add_solid(&mut self, solid: SolidAttr<P::S>) -> SolidKey {
        let shell_darts = self.solid_shell_representatives(&solid);
        let existing = shell_darts
            .iter()
            .find_map(|dart| self.dart_to_solid.get(dart));
        assert!(
            existing.is_none(),
            "A solid is already attached to one of the shell darts"
        );
        let key = self.solids.insert(solid);
        for dart in shell_darts {
            self.dart_to_solid.insert(dart, key);
        }
        key
    }

    /// Returns the typed solid view registered under `key`.
    pub fn solid(&self, key: SolidKey) -> Option<Solid<'_, P>> {
        let attr = self.solid_attr(key)?;
        Some(Solid::new(self, attr))
    }

    /// Returns the typed solid view registered under `key`.
    ///
    /// # Panics
    ///
    /// Panics if `key` is not a registered solid.
    pub fn solid_unchecked(&self, key: SolidKey) -> Solid<'_, P> {
        self.solid(key).expect("solid should be in the map")
    }

    /// Returns the solid attribute for `key`, if it exists.
    pub fn solid_attr(&self, key: SolidKey) -> Option<&SolidAttr<P::S>> {
        self.solids.get(key)
    }

    /// Returns the solid attribute for `key`.
    ///
    /// # Panics
    ///
    /// Panics if `key` is not a registered solid.
    pub fn solid_attr_unchecked(&self, key: SolidKey) -> &SolidAttr<P::S> {
        self.solid_attr(key)
            .expect("solid attribute should be in the map")
    }

    /// Iterate every stored 3-cell attribute paired with its slotmap key.
    pub fn iter_solids(&self) -> impl Iterator<Item = (SolidKey, &SolidAttr<P::S>)> {
        self.solids.iter()
    }

    fn solid_shell_representatives(&self, solid: &SolidAttr<P::S>) -> Vec<Dart> {
        std::iter::once(solid.outer_shell)
            .chain(solid.inner_shells.iter().flatten().copied())
            .map(|dart| self.cell_representative(dart, Dim::Three))
            .collect()
    }

    fn profile_representative(&self, dart: Dart) -> Dart {
        self.orbit(dart, vec![Dim::Zero.index(), Dim::One.index()])
            .min()
            .expect("profile orbit cannot be empty")
    }

    /// Copy a topological view into a fresh [`GMap`].
    ///
    /// This is the associated-function form of [`MergeTopology::isolate`].
    pub fn isolate<T>(topology: T) -> (Self, Dart)
    where
        T: MergeTopology<P>,
    {
        topology.isolate()
    }

    /// Merge a topological view into this map, returning the view's representative
    /// dart rewritten to the destination map.
    ///
    /// All darts in the view are copied. Alpha links within those darts are
    /// preserved; links leaving the view become free. Stored vertex, edge, face,
    /// and solid attributes whose representative darts are part of the view are
    /// cloned with embedded dart references remapped to the new dart ids.
    pub fn merge<T>(&mut self, topology: T) -> Dart
    where
        T: MergeTopology<P>,
    {
        let topology = topology.merge_topology();
        let source = topology.source;
        let handle = topology.handle;
        let mut seen_darts = HashSet::new();
        let source_darts = topology
            .darts
            .into_iter()
            .filter(|dart| seen_darts.insert(*dart))
            .collect::<Vec<_>>();
        let source_dart_set = source_darts.iter().copied().collect::<HashSet<_>>();
        let mut dart_map = HashMap::with_capacity(source_darts.len());
        let mut vertex_map = HashMap::with_capacity(source.vertices.len());
        let mut edge_map = HashMap::with_capacity(source.edges.len());
        let mut face_map = HashMap::with_capacity(source.faces.len());

        for old in source_darts.iter().copied() {
            let new = self.add_dart();
            dart_map.insert(old, new);
        }

        for old in source_darts.iter().copied() {
            let new = remap_dart(&dart_map, old);
            for i in 0..self.dimension() {
                let old_link = source.alphas[i][old.id()];
                self.alphas[i][new.id()] = dart_map.get(&old_link).copied().unwrap_or(new);
            }
        }

        for (old_key, attr) in source.vertices.iter() {
            let Some(attribute_dart) =
                copied_cell_dart(source, &source_dart_set, attr.dart, Dim::Zero)
            else {
                continue;
            };
            let mut attr = attr.clone();
            attr.dart = self.cell_representative(remap_dart(&dart_map, attribute_dart), Dim::Zero);
            let new_key = self.vertices.insert(attr);
            vertex_map.insert(old_key, new_key);
        }
        for (old_key, attr) in source.vertices.iter() {
            let Some(&new_key) = vertex_map.get(&old_key) else {
                continue;
            };
            for old_dart in source.orbit(attr.dart, source.orbit_indices(Dim::Zero)) {
                if let Some(&new_dart) = dart_map.get(&old_dart) {
                    self.dart_to_vertex.insert(new_dart, new_key);
                }
            }
        }

        for (old_key, attr) in source.edges.iter() {
            let Some(attribute_dart) =
                copied_cell_dart(source, &source_dart_set, attr.dart, Dim::One)
            else {
                continue;
            };
            let mut attr = attr.clone();
            attr.dart = remap_dart(&dart_map, attribute_dart);
            let new_key = self.edges.insert(attr);
            edge_map.insert(old_key, new_key);
        }
        for (old_key, attr) in source.edges.iter() {
            let Some(&new_key) = edge_map.get(&old_key) else {
                continue;
            };
            for old_dart in source.orbit(attr.dart, source.orbit_indices(Dim::One)) {
                if let Some(&new_dart) = dart_map.get(&old_dart) {
                    self.dart_to_edge.insert(new_dart, new_key);
                }
            }
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
            let repr = self.profile_representative(attr.dart);
            let new_key = self.profiles.insert(attr);
            self.dart_to_profile.insert(repr, new_key);
        }

        for (old_key, attr) in source.faces.iter() {
            if !source_dart_set.contains(&attr.outer_loop) {
                continue;
            }
            let mut attr = attr.clone();
            attr.outer_loop = remap_dart(&dart_map, attr.outer_loop);
            attr.inner_loops = attr
                .inner_loops
                .into_iter()
                .filter_map(|dart| dart_map.get(&dart).copied())
                .collect();
            attr.pcurves = attr
                .pcurves
                .into_iter()
                .filter_map(|(dart, curve)| dart_map.get(&dart).copied().map(|d| (d, curve)))
                .collect();
            let new_key = self.faces.insert(attr);
            self.index_face_loop_darts(new_key);
            face_map.insert(old_key, new_key);
        }

        for (_, attr) in source.sheets.iter() {
            if !source
                .orbit(attr.dart, vec![0, 1, 2])
                .all(|dart| source_dart_set.contains(&dart))
            {
                continue;
            }
            let mut attr = attr.clone();
            attr.dart = remap_dart(&dart_map, attr.dart);
            let repr = self.cell_representative(attr.dart, Dim::Three);
            let new_key = self.sheets.insert(attr);
            self.dart_to_sheet.insert(repr, new_key);
        }

        for (_, attr) in source.solids.iter() {
            if !source
                .orbit(attr.outer_shell, vec![0, 1, 2])
                .all(|dart| source_dart_set.contains(&dart))
            {
                continue;
            }
            let mut attr = attr.clone();
            attr.outer_shell = remap_dart(&dart_map, attr.outer_shell);
            attr.inner_shells = attr.inner_shells.map(|shells| {
                shells
                    .into_iter()
                    .filter_map(|dart| dart_map.get(&dart).copied())
                    .collect()
            });
            let shell_darts = self.solid_shell_representatives(&attr);
            let new_key = self.solids.insert(attr);
            for dart in shell_darts {
                self.dart_to_solid.insert(dart, new_key);
            }
        }

        remap_dart(&dart_map, handle)
    }

    /// Algorithm 19 of the book
    fn is_sewable(&self, d0: Dart, d1: Dart, d: Dim) -> Option<SewableDarts> {
        let i = d.index();
        if i >= self.dimension() || d0 == d1 || !self.is_free(d0, d) || !self.is_free(d1, d) {
            return None;
        }

        let inv: Vec<usize> = self.sewing_orbit_indices(d).collect();
        let mut d0_iterator = self.orbit(d0, inv.clone());
        let mut d1_iterator = self.orbit(d1, inv.clone());
        let mut mapping: HashMap<Dart, Dart> = HashMap::new();

        loop {
            match (d0_iterator.next(), d1_iterator.next()) {
                (Some(a), Some(b)) => {
                    mapping.insert(a, b);
                    if inv.iter().any(|j| {
                        let a_aj = self.alphas[*j][a.id()];
                        let b_aj = self.alphas[*j][b.id()];
                        mapping.get(&a_aj).is_some_and(|&mapped| mapped != b_aj)
                    }) {
                        return None;
                    }
                }
                (None, None) => return Some(SewableDarts { mapping }),
                _ => return None,
            }
        }
    }

    /// Returns the canonical representative dart for the `dim`-cell of `dart`.
    ///
    /// The current canonical representative is the minimum dart id in the cell
    /// orbit. Use this when storing or comparing cells by dart.
    ///
    /// # Panics
    ///
    /// Panics if the cell orbit is empty, which should be impossible for a
    /// valid dart in the map.
    pub fn cell_representative(&self, dart: Dart, dim: Dim) -> Dart {
        self.orbit(dart, self.orbit_indices(dim))
            .min()
            .expect("Orbit cannot be empty")
    }

    /// Algorithm 9 (Damiand & Lienhardt): iterates one dart per `target_dim`-cell
    /// incident to the `container_dim`-cell of `dart`. Requires `container_dim != target_dim`.
    ///
    /// Uses a shared visited-mark, so total work is linear in the darts traversed.
    /// The yielded dart for each cell is the first one reached by BFS of the
    /// container orbit — not necessarily the canonical min-id representative;
    /// call [`Self::cell_representative`] on the result if you need that.
    pub fn incident_cells(
        &self,
        dart: Dart,
        container_dim: Dim,
        target_dim: Dim,
    ) -> impl Iterator<Item = Dart> + '_ {
        assert!(
            container_dim != target_dim,
            "incident_cells requires container_dim != target_dim"
        );
        let target_orbit_indices = self.orbit_indices(target_dim);
        let mut marked = vec![false; self.dart_count()];
        let mut container_orbit = self.orbit(dart, self.orbit_indices(container_dim));
        std::iter::from_fn(move || {
            for e in container_orbit.by_ref() {
                if marked[e.id()] {
                    continue;
                }
                self.mark_orbit(e, &target_orbit_indices, &mut marked);
                return Some(e);
            }
            None
        })
    }

    /// BFS-walks an orbit generated by `involutions` starting at `start`, using `marked`
    /// as the shared visited set. Every dart visited is flagged in `marked`.
    ///
    /// The caller must guarantee that no dart of this orbit is already marked on
    /// entry; our three cell iterators rely on the partition property of cells to
    /// guarantee this (if any dart of cⁱ(start) were marked, then start itself
    /// would be marked, which the outer loop checks beforehand).
    fn mark_orbit(&self, start: Dart, involutions: &[usize], marked: &mut [bool]) {
        let mut queue = VecDeque::new();
        marked[start.id()] = true;
        queue.push_back(start);
        while let Some(d) = queue.pop_front() {
            for &i in involutions {
                let neighbor = self.alphas[i][d.id()];
                if !marked[neighbor.id()] {
                    marked[neighbor.id()] = true;
                    queue.push_back(neighbor);
                }
            }
        }
    }

    /// Algorithm 8 (Damiand & Lienhardt): iterate one dart per `i`-cell of the whole n-Gmap.
    pub fn cells(&self, dimension: Dim) -> impl Iterator<Item = Dart> + '_ {
        let orbit_indices = self.orbit_indices(dimension);
        let n = self.dart_count();
        let mut marked = vec![false; n];
        let mut next_id = 0usize;
        std::iter::from_fn(move || {
            while next_id < n {
                let id = next_id;
                next_id += 1;
                if marked[id] {
                    continue;
                }
                let d = Dart::new(id);
                self.mark_orbit(d, &orbit_indices, &mut marked);
                return Some(d);
            }
            None
        })
    }

    /// Algorithm 10: iterate one dart per `i`-cell adjacent to the `i`-cell of `dart`.
    pub fn adjacent_cells(&self, dart: Dart, d: Dim) -> impl Iterator<Item = Dart> + '_ {
        let orbit_indices = self.orbit_indices(d);
        let mut marked = vec![false; self.dart_count()];
        let mut i_orbit = self.orbit(dart, orbit_indices.clone());
        std::iter::from_fn(move || {
            for e in i_orbit.by_ref() {
                let neighbor = self.alpha(d, e);
                if marked[neighbor.id()] {
                    continue;
                }
                self.mark_orbit(neighbor, &orbit_indices, &mut marked);
                return Some(neighbor);
            }
            None
        })
    }

    fn apply_sew(&mut self, darts: SewableDarts, d: Dim) {
        let _i = d.index();
        for (d0, d1) in darts.mapping {
            self.sew_unchecked(d, d0, d1);
        }
        self.reconcile_attributes_after_sew(d);
    }

    fn reconcile_attributes_after_sew(&mut self, d: Dim) {
        if d.index() <= Dim::One.index() {
            self.reconcile_profile_attributes();
        }
        if d.index() <= Dim::Two.index() {
            self.reconcile_sheet_attributes();
        }
        if d.index() > Dim::Zero.index() {
            self.reconcile_vertex_attributes();
        }
        if d.index() > Dim::One.index() {
            self.reconcile_edge_attributes();
        }
        if d.index() > Dim::Two.index() {
            self.reconcile_face_attributes();
        }
    }

    fn reconcile_vertex_attributes(&mut self) {
        let mut survivor_by_repr = HashMap::new();
        let mut duplicates = Vec::new();
        let vertex_keys = self.vertices.keys().collect::<Vec<_>>();

        for key in vertex_keys {
            let Some(attr) = self.vertices.get(key) else {
                continue;
            };
            let repr = self.cell_representative(attr.dart, Dim::Zero);
            if survivor_by_repr.insert(repr, key).is_some() {
                duplicates.push(key);
            } else {
                self.vertices
                    .get_mut(key)
                    .expect("collected vertex key must remain valid")
                    .dart = repr;
            }
        }

        for key in duplicates {
            self.vertices.remove(key);
        }
        self.rebuild_vertex_index();
    }

    fn rebuild_vertex_index(&mut self) {
        self.dart_to_vertex.clear();
        for (key, attr) in self.vertices.iter() {
            self.dart_to_vertex.insert(attr.dart, key);
        }
    }

    fn reconcile_edge_attributes(&mut self) {
        let mut survivor_by_repr = HashMap::new();
        let mut duplicates = Vec::new();
        let edge_keys = self.edges.keys().collect::<Vec<_>>();

        for key in edge_keys {
            let Some(attr) = self.edges.get(key) else {
                continue;
            };
            let repr = self.cell_representative(attr.dart, Dim::One);
            if survivor_by_repr.insert(repr, key).is_some() {
                duplicates.push(key);
            }
        }

        for key in duplicates {
            self.edges.remove(key);
        }
        self.rebuild_edge_index();
    }

    fn rebuild_edge_index(&mut self) {
        self.dart_to_edge.clear();
        for (key, attr) in self.edges.iter() {
            let repr = self.cell_representative(attr.dart, Dim::One);
            self.dart_to_edge.insert(repr, key);
        }
    }

    fn reconcile_profile_attributes(&mut self) {
        let mut survivor_by_repr = HashMap::new();
        let mut duplicates = Vec::new();
        let profile_keys = self.profiles.keys().collect::<Vec<_>>();

        for key in profile_keys {
            let Some(attr) = self.profiles.get(key) else {
                continue;
            };
            let repr = self.profile_representative(attr.dart);
            if survivor_by_repr.insert(repr, key).is_some() {
                duplicates.push(key);
            }
        }

        for key in duplicates {
            self.profiles.remove(key);
        }
        self.rebuild_profile_index();
    }

    fn rebuild_profile_index(&mut self) {
        self.dart_to_profile.clear();
        for (key, attr) in self.profiles.iter() {
            let repr = self.profile_representative(attr.dart);
            self.dart_to_profile.insert(repr, key);
        }
    }

    fn reconcile_face_attributes(&mut self) {
        let mut survivor_by_repr = HashMap::new();
        let mut duplicates = Vec::new();
        let face_keys = self.faces.keys().collect::<Vec<_>>();

        for key in face_keys {
            let Some(attr) = self.faces.get(key) else {
                continue;
            };
            let repr = self.cell_representative(attr.outer_loop, Dim::Two);
            if survivor_by_repr.insert(repr, key).is_some() {
                duplicates.push(key);
            }
        }

        for key in duplicates {
            self.faces.remove(key);
        }
        self.rebuild_face_index();
    }

    fn rebuild_face_index(&mut self) {
        self.dart_to_face.clear();
        let face_keys: Vec<FaceKey> = self.faces.keys().collect();
        for key in face_keys {
            self.index_face_loop_darts(key);
        }
    }

    fn reconcile_sheet_attributes(&mut self) {
        let mut survivor_by_repr = HashMap::new();
        let mut duplicates = Vec::new();
        let sheet_keys = self.sheets.keys().collect::<Vec<_>>();

        for key in sheet_keys {
            let Some(attr) = self.sheets.get(key) else {
                continue;
            };
            let repr = self.cell_representative(attr.dart, Dim::Three);
            if survivor_by_repr.insert(repr, key).is_some() {
                duplicates.push(key);
            }
        }

        for key in duplicates {
            self.sheets.remove(key);
        }
        self.rebuild_sheet_index();
    }

    fn rebuild_sheet_index(&mut self) {
        self.dart_to_sheet.clear();
        for (key, attr) in self.sheets.iter() {
            let repr = self.cell_representative(attr.dart, Dim::Three);
            self.dart_to_sheet.insert(repr, key);
        }
    }

    /// Sews `d0` and `d1` along dimension `d`.
    ///
    /// Returns an error when the darts are the same, either dart is already
    /// sewn along `d`, or the generated sewing orbits are incompatible.
    pub fn sew(&mut self, d: Dim, d0: Dart, d1: Dart) -> Result<(), &'static str> {
        match self.is_sewable(d0, d1, d) {
            Some(sd) => {
                self.apply_sew(sd, d);
                Ok(())
            }
            None => Err("darts are not i-sewable"),
        }
    }
    pub(crate) fn sew_unchecked(&mut self, d: Dim, d0: Dart, d1: Dart) {
        let i = d.index();
        self.alphas[i][d0.id()] = d1;
        self.alphas[i][d1.id()] = d0;
    }

    pub(crate) fn unsew(&mut self, dart: Dart, d: Dim) {
        let i = d.index();
        let a_i = self.alphas[i][dart.id()];
        self.alphas[i][a_i.id()] = a_i;
        self.alphas[i][dart.id()] = dart;
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
    /// The lookup first canonicalizes `dart` to the representative of `D::DIM`.
    ///
    /// # Panics
    ///
    /// Panics if no attribute is registered for the cell.
    pub fn attribute_unchecked<D: CellDim>(&self, dart: Dart) -> &<Self as AttributeStore<D>>::Attr
    where
        Self: AttributeStore<D>,
    {
        self.attribute::<D>(dart)
            .expect("attribute should be in the map")
    }

    /// Returns the mutable attribute associated with the `D`-cell containing
    /// `dart`.
    ///
    /// The lookup first canonicalizes `dart` to the representative of `D::DIM`.
    pub fn attribute_mut<D: CellDim>(
        &mut self,
        dart: Dart,
    ) -> Option<&mut <Self as AttributeStore<D>>::Attr>
    where
        Self: AttributeStore<D>,
    {
        let repr = self.cell_representative(dart, D::DIM);
        self.get_mut(repr)
    }

    /// Returns the mutable attribute associated with the `D`-cell containing
    /// `dart`.
    ///
    /// The lookup first canonicalizes `dart` to the representative of `D::DIM`.
    ///
    /// # Panics
    ///
    /// Panics if no attribute is registered for the cell.
    pub fn attribute_mut_unchecked<D: CellDim>(
        &mut self,
        dart: Dart,
    ) -> &mut <Self as AttributeStore<D>>::Attr
    where
        Self: AttributeStore<D>,
    {
        self.attribute_mut::<D>(dart)
            .expect("attribute should be in the map")
    }
}

/// Breadth-first iterator over a dart orbit.
///
/// The iterator starts at one dart and follows the configured alpha indices,
/// yielding each reachable dart once.
pub struct OrbitIterator<'a, P: Payload> {
    gmap: &'a GMap<P>,
    involutions: Vec<usize>,
    visited: Vec<bool>,
    queue: VecDeque<Dart>,
}

impl<'a, P: Payload> OrbitIterator<'a, P> {
    /// Creates an orbit iterator rooted at `start`.
    ///
    /// `involutions` contains alpha indices, not [`Dim`] values.
    pub fn new(gmap: &'a GMap<P>, start: Dart, involutions: Vec<usize>) -> Self {
        let dart_count = gmap.dart_count();
        let mut visited = vec![false; dart_count];
        let mut queue = VecDeque::new();

        visited[start.id()] = true;
        queue.push_back(start);

        Self {
            gmap,
            involutions,
            visited,
            queue,
        }
    }
}

impl<'a, P: Payload> Iterator for OrbitIterator<'a, P> {
    type Item = Dart;

    fn next(&mut self) -> Option<Self::Item> {
        let dart = self.queue.pop_front()?;

        for &i in &self.involutions {
            let neighbor = self.gmap.alphas[i][dart.id()];

            if !self.visited[neighbor.id()] {
                self.visited[neighbor.id()] = true;
                self.queue.push_back(neighbor);
            }
        }

        Some(dart)
    }
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use nalgebra::Vector3;

    use super::{Cell0, Cell1, Cell2, Dart, Dim, GMap, MergeTopology};
    use crate::builders::edges::add_edge;
    use crate::builders::faces::add_polygon;
    use crate::builders::profiles::add_rectangle;
    use crate::builders::sheets::add_extruded_profile;
    use crate::geometry::{Curve, Curve2, Line2, Plane, Point2, Point3, Surface};
    use crate::topology::attributes::{FaceAttr, SolidAttr};
    use crate::topology::payload::{Payload, StandardPayload};
    use crate::topology::planar::Planar;
    use crate::topology::profile::Profile;
    use crate::topology::sheet::Sheet;
    use crate::topology::solid::Solid;

    #[derive(Clone)]
    struct DataPayload;

    impl Payload for DataPayload {
        type V = ();
        type E = ();
        type Profile = String;
        type F = ();
        type Sheet = String;
        type S = ();
    }

    #[test]
    fn profile_and_sheet_payloads_are_exposed_and_preserved_by_merge() {
        let mut source = GMap::<DataPayload>::new();
        let profile_key =
            add_rectangle(&mut source, Plane::xy(), 2.0, 1.0).expect("profile should build");
        source.profile_attr_mut(profile_key).unwrap().data = "profile".to_owned();
        let profile_dart = source.profile_attr(profile_key).unwrap().dart;
        let sheet_key = add_extruded_profile(&mut source, profile_dart, Vector3::z())
            .expect("sheet should build");
        source.sheet_attr_mut(sheet_key).unwrap().data = "sheet".to_owned();

        assert_eq!(
            source.profile(profile_key).unwrap().data().unwrap(),
            "profile"
        );
        assert_eq!(source.sheet(sheet_key).unwrap().data().unwrap(), "sheet");

        source.profile_attr_mut(profile_key).unwrap().data = "updated profile".to_owned();
        source.sheet_attr_mut(sheet_key).unwrap().data = "updated sheet".to_owned();

        let mut profile_target = GMap::<DataPayload>::new();
        profile_target.merge(source.profile(profile_key).unwrap());
        let mut sheet_target = GMap::<DataPayload>::new();
        sheet_target.merge(source.sheet(sheet_key).unwrap());

        assert_eq!(
            profile_target.iter_profiles().next().unwrap().1.data,
            "updated profile"
        );
        assert_eq!(
            sheet_target.iter_sheets().next().unwrap().1.data,
            "updated sheet"
        );
    }

    #[test]
    fn merge_edge_copies_topology_and_geometry() {
        let mut target = GMap::<StandardPayload>::new();
        let mut source = GMap::<StandardPayload>::new();
        let edge_key = add_edge(
            &mut source,
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)),
        )
        .expect("source edge should build");

        let edge = source.edge_unchecked(edge_key);
        let merged_dart = target.merge(edge);
        let merged_edge = target.attribute_unchecked::<Cell1>(merged_dart);

        assert_eq!(target.dart_count(), 2);
        assert_eq!(merged_edge.dart, Dart::new(0));
        assert_eq!(target.alpha(Dim::Zero, Dart::new(0)), Dart::new(1));
        assert!(target.attribute::<Cell0>(Dart::new(0)).is_some());
        assert!(target.attribute::<Cell0>(Dart::new(1)).is_some());
    }

    #[test]
    fn merge_face_remaps_stored_darts_and_pcurves() {
        let mut target = GMap::<StandardPayload>::new();
        add_edge(
            &mut target,
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 0.0),
            Curve::line(Point3::new(-1.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0)),
        )
        .expect("target edge should build");

        let mut source = GMap::<StandardPayload>::new();
        let profile_key = add_polygon(
            &mut source,
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
        );
        let loop_dart = source
            .profile_attr(profile_key)
            .expect("polygon profile should exist")
            .dart;
        let mut pcurves = HashMap::new();
        pcurves.insert(
            loop_dart,
            Curve2::Line(Line2::new(Point2::new(0.0, 0.0), Point2::new(1.0, 0.0))),
        );
        let face_key = source.add_face(FaceAttr::with_pcurves(
            Surface::Plane(Plane::from_xy(
                Point3::new(0.0, 0.0, 0.0),
                Vector3::x(),
                Vector3::y(),
            )),
            (),
            loop_dart,
            Vec::new(),
            pcurves,
        ));

        let face = source.face_unchecked(face_key);
        let merged_dart = target.merge(face);
        let merged_key = *target.attribute_unchecked::<Cell2>(merged_dart);
        let merged_face = target.face_attr_unchecked(merged_key);

        assert_eq!(target.dart_count(), 10);
        assert_eq!(merged_face.outer_loop, Dart::new(2));
        assert!(merged_face.pcurves.contains_key(&merged_face.outer_loop));
        assert!(!merged_face.pcurves.contains_key(&loop_dart));
        assert_eq!(target.alpha(Dim::Zero, Dart::new(2)), Dart::new(3));
        assert_eq!(target.alpha(Dim::One, Dart::new(3)), Dart::new(4));
    }

    #[test]
    fn merge_profile_sheet_and_solid_return_remapped_darts() {
        let mut source = GMap::<StandardPayload>::new();
        let profile_key = add_polygon(
            &mut source,
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
        );

        let profile_dart = source.profile_attr_unchecked(profile_key).dart;
        let mut target = GMap::<StandardPayload>::new();
        let merged_profile = target.merge(Profile::new(&source, profile_dart));
        assert_eq!(merged_profile, Dart::new(0));
        assert_eq!(target.dart_count(), 6);

        let mut sheet_target = GMap::<StandardPayload>::new();
        let merged_sheet = sheet_target.merge(Sheet::new(&source, profile_dart));
        assert_eq!(merged_sheet, Dart::new(0));
        assert_eq!(sheet_target.dart_count(), 6);

        let solid_key = source.add_solid(SolidAttr::new((), profile_dart, None));
        let mut second_target = GMap::<StandardPayload>::new();
        let solid = source.solid_unchecked(solid_key);
        let merged_solid = second_target.merge(solid);
        assert_eq!(merged_solid, Dart::new(0));
        assert_eq!(
            second_target
                .iter_solids()
                .next()
                .expect("merged solid should exist")
                .1
                .outer_shell,
            Dart::new(0)
        );
    }

    #[test]
    fn isolate_face_copies_it_into_a_fresh_map() {
        let mut source = GMap::<StandardPayload>::new();
        let profile_key = add_polygon(
            &mut source,
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
        );
        let loop_dart = source.profile_attr_unchecked(profile_key).dart;
        let face_key = source.add_face(FaceAttr::new(
            Surface::Plane(Plane::from_xy(
                Point3::new(0.0, 0.0, 0.0),
                Vector3::x(),
                Vector3::y(),
            )),
            (),
            loop_dart,
            Vec::new(),
        ));
        let face = source.face_unchecked(face_key);

        let (isolated, isolated_dart) = face.isolate();

        assert_eq!(isolated_dart, Dart::new(0));
        assert_eq!(isolated.dart_count(), 8);
        assert_eq!(isolated.iter_faces().count(), 1);
        assert!(isolated.attribute::<Cell2>(isolated_dart).is_some());
        assert_eq!(isolated.alpha(Dim::Zero, Dart::new(0)), Dart::new(1));
        assert_eq!(isolated.alpha(Dim::One, Dart::new(1)), Dart::new(2));
    }

    #[test]
    fn isolate_associated_function_accepts_any_merge_topology() {
        let mut source = GMap::<StandardPayload>::new();
        let profile_key = add_polygon(
            &mut source,
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
        );

        let (isolated, isolated_dart) = GMap::isolate(source.profile_unchecked(profile_key));

        assert_eq!(isolated_dart, Dart::new(0));
        assert_eq!(isolated.dart_count(), 6);
    }

    #[test]
    fn isolate_planar_topology_forwards_to_inner_topology() {
        let mut source = GMap::<StandardPayload>::new();
        let profile_key = add_polygon(
            &mut source,
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
        );
        let planar = Planar::new_unchecked(
            source.profile_unchecked(profile_key),
            Plane::from_xy(Point3::new(0.0, 0.0, 0.0), Vector3::x(), Vector3::y()),
        );

        let (isolated, isolated_dart) = planar.isolate();

        assert_eq!(isolated_dart, Dart::new(0));
        assert_eq!(isolated.dart_count(), 6);
    }
}
