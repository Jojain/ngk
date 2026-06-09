use std::collections::{HashMap, HashSet, VecDeque};

use slotmap::SlotMap;

use crate::topology::shape_keys::{EdgeKey, FaceKey, SolidKey, VertexKey};

use super::attributes::{EdgeAttr, FaceAttr, SolidAttr, VertexAttr};
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
    /// Facet or face dimension / alpha2.
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
/// Type marker for facet/face attributes.
pub struct Cell2;
/// Type marker for solid attributes.
pub struct Cell3;

/// Compile-time mapping from a cell marker to its dimension.
pub trait CellDim {
    /// Dimension represented by this cell marker.
    const DIM: Dim;
}

impl CellDim for Cell0 {
    const DIM: Dim = Dim::Zero;
}
impl CellDim for Cell1 {
    const DIM: Dim = Dim::One;
}
impl CellDim for Cell2 {
    const DIM: Dim = Dim::Two;
}
impl CellDim for Cell3 {
    const DIM: Dim = Dim::Three;
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
    pub(crate) dart_to_face: HashMap<Dart, FaceKey>,
    pub(crate) dart_to_solid: HashMap<Dart, SolidKey>,
    vertices: SlotMap<VertexKey, VertexAttr<P::V>>,
    edges: SlotMap<EdgeKey, EdgeAttr<P::E>>,
    pub(crate) faces: SlotMap<FaceKey, FaceAttr<P::F>>,
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
            dart_to_face: self.dart_to_face.clone(),
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
        let dart_to_face = HashMap::new();
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
            dart_to_face,
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
    /// If a vertex attribute is already registered for the same 0-cell, the
    /// existing key is returned and the new attribute is ignored.
    pub fn add_vertex(&mut self, vertex: VertexAttr<P::V>) -> VertexKey {
        let dart = self.cell_representative(vertex.dart, Dim::Zero);
        if let Some(&key) = self.dart_to_vertex.get(&dart) {
            return key;
        }
        let mut vertex = vertex;
        vertex.dart = dart;
        let key = self.vertices.insert(vertex);
        self.dart_to_vertex.insert(dart, key);
        key
    }
    /// Returns the vertex attribute for `key`, if it exists.
    pub fn vertex(&self, key: VertexKey) -> Option<&VertexAttr<P::V>> {
        self.vertices.get(key)
    }

    /// Returns the mutable vertex attribute for `key`, if it exists.
    pub fn vertex_mut(&mut self, key: VertexKey) -> Option<&mut VertexAttr<P::V>> {
        self.vertices.get_mut(key)
    }

    /// Iterate every stored 0-cell attribute paired with its slotmap key.
    pub fn iter_vertices(&self) -> impl Iterator<Item = (VertexKey, &VertexAttr<P::V>)> {
        self.vertices.iter()
    }

    /// Adds an edge attribute and returns its key.
    ///
    /// If an edge attribute is already registered for the same 1-cell, the
    /// existing key is returned and the new attribute is ignored.
    pub fn add_edge(&mut self, edge: EdgeAttr<P::E>) -> EdgeKey {
        let dart = self.cell_representative(edge.dart, Dim::One);
        if let Some(&key) = self.dart_to_edge.get(&dart) {
            return key;
        }
        let mut edge = edge;
        edge.dart = dart;
        let key = self.edges.insert(edge);
        self.dart_to_edge.insert(dart, key);
        key
    }

    /// Returns the edge attribute for `key`, if it exists.
    pub fn edge(&self, key: EdgeKey) -> Option<&EdgeAttr<P::E>> {
        self.edges.get(key)
    }

    /// Returns the mutable edge attribute for `key`, if it exists.
    pub fn edge_mut(&mut self, key: EdgeKey) -> Option<&mut EdgeAttr<P::E>> {
        self.edges.get_mut(key)
    }

    /// Iterate every stored 1-cell attribute paired with its slotmap key.
    pub fn iter_edges(&self) -> impl Iterator<Item = (EdgeKey, &EdgeAttr<P::E>)> {
        self.edges.iter()
    }

    /// Adds a face attribute and returns its key.
    ///
    /// If any boundary facet is already attached to a face, the existing face
    /// key is returned and the new attribute is ignored.
    pub fn add_face(&mut self, face: FaceAttr<P::F>) -> FaceKey {
        let facet_darts = std::iter::once(face.outer_loop)
            .chain(face.inner_loops.iter().copied())
            .map(|dart| self.cell_representative(dart, Dim::Two))
            .collect::<Vec<_>>();
        if let Some(key) = facet_darts
            .iter()
            .find_map(|dart| self.dart_to_face.get(dart).copied())
        {
            return key;
        }
        let key = self.faces.insert(face);
        for dart in facet_darts {
            self.dart_to_face.insert(dart, key);
        }
        key
    }

    /// Returns the face attribute for `key`, if it exists.
    pub fn face(&self, key: FaceKey) -> Option<&FaceAttr<P::F>> {
        self.faces.get(key)
    }

    /// Returns the mutable face attribute for `key`, if it exists.
    pub fn face_mut(&mut self, key: FaceKey) -> Option<&mut FaceAttr<P::F>> {
        self.faces.get_mut(key)
    }

    pub(crate) fn remove_face(&mut self, key: FaceKey) -> Option<FaceAttr<P::F>> {
        let face = self.faces.remove(key)?;
        for dart in std::iter::once(face.outer_loop).chain(face.inner_loops.iter().copied()) {
            let representative = self.cell_representative(dart, Dim::Two);
            if self.dart_to_face.get(&representative) == Some(&key) {
                self.dart_to_face.remove(&representative);
            }
        }
        Some(face)
    }

    /// Iterate every stored 2-cell attribute paired with its slotmap key.
    pub fn iter_faces(&self) -> impl Iterator<Item = (FaceKey, &FaceAttr<P::F>)> {
        self.faces.iter()
    }

    /// Adds a solid attribute and returns its key.
    ///
    /// If any shell is already attached to a solid, the existing solid key is
    /// returned and the new attribute is ignored.
    pub fn add_solid(&mut self, solid: SolidAttr<P::S>) -> SolidKey {
        let shell_darts = self.solid_shell_representatives(&solid);
        if let Some(key) = shell_darts
            .iter()
            .find_map(|dart| self.dart_to_solid.get(dart).copied())
        {
            return key;
        }

        let key = self.solids.insert(solid);
        for dart in shell_darts {
            self.dart_to_solid.insert(dart, key);
        }
        key
    }

    /// Returns the solid attribute for `key`, if it exists.
    pub fn solid(&self, key: SolidKey) -> Option<&SolidAttr<P::S>> {
        self.solids.get(key)
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
            attr.dart = self.cell_representative(remap_dart(&dart_map, attribute_dart), Dim::One);
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
            let outer_loop = attr.outer_loop;
            let new_key = self.faces.insert(attr);
            self.dart_to_face.insert(outer_loop, new_key);
            face_map.insert(old_key, new_key);
        }
        for (old_dart, old_key) in source.dart_to_face.iter() {
            if let (Some(&new_dart), Some(&new_key)) =
                (dart_map.get(old_dart), face_map.get(old_key))
            {
                self.dart_to_face.insert(new_dart, new_key);
            }
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
            } else {
                self.edges
                    .get_mut(key)
                    .expect("collected edge key must remain valid")
                    .dart = repr;
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
            self.dart_to_edge.insert(attr.dart, key);
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
        for (key, attr) in self.faces.iter() {
            for loop_dart in
                std::iter::once(attr.outer_loop).chain(attr.inner_loops.iter().copied())
            {
                let repr = self.cell_representative(loop_dart, Dim::Two);
                self.dart_to_face.insert(repr, key);
            }
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
    use crate::geometry::{Curve, Curve2, Line2, Plane, Point2, Point3, Surface};
    use crate::topology::attributes::{FaceAttr, SolidAttr};
    use crate::topology::edge::Edge;
    use crate::topology::face::Face;
    use crate::topology::payload::StandardPayload;
    use crate::topology::planar::Planar;
    use crate::topology::profile::Profile;
    use crate::topology::sheet::Sheet;
    use crate::topology::solid::Solid;

    #[test]
    fn merge_edge_copies_topology_and_geometry() {
        let mut target = GMap::<StandardPayload>::new();
        let mut source = GMap::<StandardPayload>::new();
        let (_, edge_key) = add_edge(
            &mut source,
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)),
        )
        .expect("source edge should build");

        let edge = source
            .edge(edge_key)
            .map(|attr| Edge::new(&source, attr.dart))
            .expect("source edge should exist");
        let merged_dart = target.merge(edge);
        let merged_edge = target
            .attribute::<Cell1>(merged_dart)
            .expect("merged edge geometry should exist");

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
        let loop_dart = add_polygon(
            &mut source,
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
        );
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

        let face = source
            .face(face_key)
            .map(|attr| Face::new(&source, attr))
            .expect("source face should exist");
        let merged_dart = target.merge(face);
        let merged_key = *target
            .attribute::<Cell2>(merged_dart)
            .expect("merged face lookup should exist");
        let merged_face = target.face(merged_key).expect("merged face should exist");

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
        let profile_dart = add_polygon(
            &mut source,
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
        );

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
        let solid = source
            .solid(solid_key)
            .map(|attr| Solid::new(&source, attr))
            .expect("source solid should exist");
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
        let loop_dart = add_polygon(
            &mut source,
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
        );
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
        let face = source
            .face(face_key)
            .map(|attr| Face::new(&source, attr))
            .expect("source face should exist");

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
        let profile_dart = add_polygon(
            &mut source,
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
        );

        let (isolated, isolated_dart) = GMap::isolate(Profile::new(&source, profile_dart));

        assert_eq!(isolated_dart, Dart::new(0));
        assert_eq!(isolated.dart_count(), 6);
    }

    #[test]
    fn isolate_planar_topology_forwards_to_inner_topology() {
        let mut source = GMap::<StandardPayload>::new();
        let profile_dart = add_polygon(
            &mut source,
            &[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
        );
        let planar = Planar::new_unchecked(
            Profile::new(&source, profile_dart),
            Plane::from_xy(Point3::new(0.0, 0.0, 0.0), Vector3::x(), Vector3::y()),
        );

        let (isolated, isolated_dart) = planar.isolate();

        assert_eq!(isolated_dart, Dart::new(0));
        assert_eq!(isolated.dart_count(), 6);
    }
}
