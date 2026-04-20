use std::collections::{HashMap, VecDeque};

use slotmap::SlotMap;

use crate::topology::edge::Edge;
use crate::topology::shape_keys::{EdgeKey, FaceKey, SolidKey, VertexKey};
use crate::topology::vertex::Vertex;

use super::attributes::{EdgeAttr, FaceAttr, SolidAttr, VertexAttr};
use super::face::Face;
use super::payload::{Payload, StandardPayload};
use super::solid::Solid;

pub use super::dart::{Dart, IsolatedDart};

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub enum Dim {
    Zero,
    One,
    Two,
    Three,
}

impl Dim {
    pub fn index(&self) -> usize {
        match self {
            Dim::Zero => 0,
            Dim::One => 1,
            Dim::Two => 2,
            Dim::Three => 3,
        }
    }
}

/// Number of involutions α₀…α₃ in a 3-gmap (four involutions).
pub const GMAP_INVOLUTION_COUNT: usize = 4;
pub struct SewableDarts {
    mapping: HashMap<Dart, Dart>,
}

pub struct Cell0;
pub struct Cell1;
pub struct Cell2;
pub struct Cell3;

pub trait CellDim {
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

pub trait AttributeStore<D: CellDim> {
    type Attr;
    fn get(&self, repr: Dart) -> Option<&Self::Attr>;
    fn get_mut(&mut self, repr: Dart) -> Option<&mut Self::Attr>;
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
        self.facets.get(&repr)
    }
    fn get_mut(&mut self, repr: Dart) -> Option<&mut FaceKey> {
        self.facets.get_mut(&repr)
    }
}

pub struct GMap<P: Payload = StandardPayload> {
    alphas: [Vec<Dart>; GMAP_INVOLUTION_COUNT],
    free_slots: VecDeque<usize>,
    vertices: SlotMap<VertexKey, VertexAttr<P::V>>,
    pub(crate) dart_to_vertex: HashMap<Dart, VertexKey>,
    edges: SlotMap<EdgeKey, EdgeAttr<P::E>>,
    pub(crate) dart_to_edge: HashMap<Dart, EdgeKey>,
    facets: HashMap<Dart, FaceKey>,
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
            facets: self.facets.clone(),
            faces: self.faces.clone(),
            solids: self.solids.clone(),
        }
    }
}

impl<P: Payload> GMap<P> {
    pub fn new() -> Self {
        let alphas = std::array::from_fn(|_| Vec::new());
        let free_slots = VecDeque::new();
        let vertices = SlotMap::with_key();
        let dart_to_vertex = HashMap::new();
        let edges = SlotMap::with_key();
        let dart_to_edge = HashMap::new();
        let facets = HashMap::new();
        let faces = SlotMap::with_key();
        let solids = SlotMap::with_key();
        Self {
            alphas,
            free_slots,
            vertices,
            dart_to_vertex,
            edges,
            dart_to_edge,
            facets,
            faces,
            solids,
        }
    }

    /// Number of involutions (α₀…α₃), always [`GMAP_INVOLUTION_COUNT`].
    pub fn dimension(&self) -> usize {
        GMAP_INVOLUTION_COUNT
    }

    pub fn dart_count(&self) -> usize {
        self.alphas[0].len()
    }

    pub fn alpha(&self, d: Dim, dart: Dart) -> Dart {
        let i = d.index();
        self.alphas[i][dart.id()]
    }

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

    pub fn remove_dart(&mut self, dart: IsolatedDart) {
        for alphas in self.alphas.iter_mut() {
            alphas.remove(dart.id());
        }
        self.free_slots.push_back(dart.id());
    }

    pub fn orbit(&self, dart: Dart, involutions: Vec<usize>) -> OrbitIterator<'_, P> {
        OrbitIterator::new(self, dart, involutions)
    }

    /// A dart is `i`-free when `αᵢ(d) = d`, i.e. not sewn along dimension `i`.
    pub fn is_free(&self, dart: Dart, d: Dim) -> bool {
        self.alphas[d.index()][dart.id()] == dart
    }

    /// Involutions generating the orbit ⟨α₀,…,α_{i−2}, α_{i+2},…,α_n⟩ used in the i-sew test.
    fn sewing_orbit_indices(&self, d: Dim) -> impl Iterator<Item = usize> + '_ {
        let i = d.index();
        (0..self.dimension()).filter(move |&j| j + 2 <= i || j >= i + 2)
    }

    pub fn orbit_indices(&self, d: Dim) -> Vec<usize> {
        let i = d.index();
        (0..self.dimension()).filter(|&idx| idx != i).collect()
    }

    pub fn add_vertex(&mut self, vertex: VertexAttr<P::V>) -> VertexKey {
        let dart = vertex.dart;
        let key = self.vertices.insert(vertex);
        self.dart_to_vertex.insert(dart, key);
        key
    }
    pub fn vertex(&self, key: VertexKey) -> Option<&VertexAttr<P::V>> {
        self.vertices.get(key)
    }

    pub fn add_edge(&mut self, edge: EdgeAttr<P::E>) -> EdgeKey {
        let dart = edge.dart;
        let key = self.edges.insert(edge);
        self.dart_to_edge.insert(dart, key);
        key
    }

    pub fn edge(&self, key: EdgeKey) -> Option<&EdgeAttr<P::E>> {
        self.edges.get(key)
    }

    pub fn add_face(&mut self, face: FaceAttr<P::F>) -> FaceKey {
        self.faces.insert(face)
    }

    pub fn face(&self, key: FaceKey) -> Option<&FaceAttr<P::F>> {
        self.faces.get(key)
    }

    pub fn add_solid(&mut self, solid: SolidAttr<P::S>) -> SolidKey {
        self.solids.insert(solid)
    }

    pub fn solid(&self, key: SolidKey) -> Option<&SolidAttr<P::S>> {
        self.solids.get(key)
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

    /// BFS-walk the orbit ⟨α_k : k ∈ involutions⟩ starting at `start`, using `marked`
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
        let i = d.index();
        for (d0, d1) in darts.mapping {
            self.sew_unchecked(d, d0, d1);
        }
    }

    /// i-sew `d0` and `d1`. Fails if the configuration is not sewable
    /// (same dart, already i-sewn, or orbits not compatible).
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

    fn unsew(&mut self, dart: Dart, d: Dim) {
        let i = d.index();
        let a_i = self.alphas[i][dart.id()];
        self.alphas[i][a_i.id()] = a_i;
        self.alphas[i][dart.id()] = dart;
    }

    pub fn attribute<D: CellDim>(&self, dart: Dart) -> Option<&<Self as AttributeStore<D>>::Attr>
    where
        Self: AttributeStore<D>,
    {
        let repr = self.cell_representative(dart, D::DIM);
        self.get(repr)
    }

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

pub struct OrbitIterator<'a, P: Payload> {
    gmap: &'a GMap<P>,
    involutions: Vec<usize>,
    visited: Vec<bool>,
    queue: VecDeque<Dart>,
}

impl<'a, P: Payload> OrbitIterator<'a, P> {
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
