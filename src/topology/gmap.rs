use std::collections::{HashMap, VecDeque};

use slotmap::SlotMap;

use super::attributes::{EdgeAttr, VertexAttr};
use super::face::{Face, FaceId};
use super::payload::{Payload, StandardPayload};
use super::solid::{Solid, SolidId};

type Dim = usize;

/// Number of involutions α₀…α₃ in a 3-gmap (four involutions).
pub const GMAP_INVOLUTION_COUNT: usize = 4;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Ord, PartialOrd)]
pub struct Dart(usize);

impl Dart {
    pub fn new(id: usize) -> Self {
        Self(id)
    }
    pub fn id(&self) -> usize {
        self.0
    }
}

pub struct IsolatedDart(Dart);

impl IsolatedDart {
    pub fn new(dart: Dart) -> Self {
        Self(dart)
    }
    pub fn id(&self) -> usize {
        self.0.id()
    }
}
pub struct SewableDarts {
    mapping: HashMap<Dart, Dart>,
}

pub struct Cell0;
pub struct Cell1;
pub struct Cell2;
pub struct Cell3;

pub trait CellDim {
    const DIM: usize;
}

impl CellDim for Cell0 {
    const DIM: usize = 0;
}
impl CellDim for Cell1 {
    const DIM: usize = 1;
}
impl CellDim for Cell2 {
    const DIM: usize = 2;
}
impl CellDim for Cell3 {
    const DIM: usize = 3;
}

pub trait AttributeStore<D: CellDim> {
    type Attr;
    fn get(&self, repr: Dart) -> Option<&Self::Attr>;
    fn get_mut(&mut self, repr: Dart) -> Option<&mut Self::Attr>;

    /// Ensures `repr` has a stored attribute, then returns `&mut` to it.
    fn get_mut_or_insert_with<F: FnOnce() -> Self::Attr>(
        &mut self,
        repr: Dart,
        create: F,
    ) -> &mut Self::Attr;
}

impl<'a, P: Payload> AttributeStore<Cell0> for GMap<'a, P> {
    type Attr = VertexAttr<P::V>;
    fn get(&self, repr: Dart) -> Option<&VertexAttr<P::V>> {
        self.vertices.get(&repr)
    }
    fn get_mut(&mut self, repr: Dart) -> Option<&mut VertexAttr<P::V>> {
        self.vertices.get_mut(&repr)
    }
    fn get_mut_or_insert_with<F: FnOnce() -> VertexAttr<P::V>>(
        &mut self,
        repr: Dart,
        create: F,
    ) -> &mut VertexAttr<P::V> {
        self.vertices.entry(repr).or_insert_with(create)
    }
}
impl<'a, P: Payload> AttributeStore<Cell1> for GMap<'a, P> {
    type Attr = EdgeAttr<P::E>;
    fn get(&self, repr: Dart) -> Option<&EdgeAttr<P::E>> {
        self.edges.get(&repr)
    }
    fn get_mut(&mut self, repr: Dart) -> Option<&mut EdgeAttr<P::E>> {
        self.edges.get_mut(&repr)
    }
    fn get_mut_or_insert_with<F: FnOnce() -> EdgeAttr<P::E>>(
        &mut self,
        repr: Dart,
        create: F,
    ) -> &mut EdgeAttr<P::E> {
        self.edges.entry(repr).or_insert_with(create)
    }
}
impl<'a, P: Payload> AttributeStore<Cell2> for GMap<'a, P> {
    type Attr = FaceId;
    fn get(&self, repr: Dart) -> Option<&FaceId> {
        self.facets.get(&repr)
    }
    fn get_mut(&mut self, repr: Dart) -> Option<&mut FaceId> {
        self.facets.get_mut(&repr)
    }
    fn get_mut_or_insert_with<F: FnOnce() -> FaceId>(
        &mut self,
        repr: Dart,
        create: F,
    ) -> &mut FaceId {
        self.facets.entry(repr).or_insert_with(create)
    }
}

pub struct GMap<'a, P: Payload = StandardPayload> {
    alphas: [Vec<Dart>; GMAP_INVOLUTION_COUNT],
    free_slots: VecDeque<usize>,
    vertices: HashMap<Dart, VertexAttr<P::V>>,
    edges: HashMap<Dart, EdgeAttr<P::E>>,
    facets: HashMap<Dart, FaceId>,
    faces: SlotMap<FaceId, Face<'a, P>>,
    solids: SlotMap<SolidId, Solid<'a, P>>,
}

impl<'a, P: Payload> Clone for GMap<'a, P> {
    fn clone(&self) -> Self {
        Self {
            alphas: self.alphas.clone(),
            free_slots: self.free_slots.clone(),
            vertices: self.vertices.clone(),
            edges: self.edges.clone(),
            facets: self.facets.clone(),
            faces: self.faces.clone(),
            solids: self.solids.clone(),
        }
    }
}

impl<'a, P: Payload> GMap<'a, P> {
    pub fn new() -> Self {
        let alphas = std::array::from_fn(|_| Vec::new());
        let free_slots = VecDeque::new();
        let vertices = HashMap::new();
        let edges = HashMap::new();
        let facets = HashMap::new();
        let faces = SlotMap::with_key();
        let solids = SlotMap::with_key();
        Self {
            alphas,
            free_slots,
            vertices,
            edges,
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

    pub fn alpha(&self, dim: usize, dart: Dart) -> Dart {
        self.alphas[dim][dart.id()]
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
    pub fn is_free(&self, dart: Dart, i: Dim) -> bool {
        self.alphas[i][dart.id()] == dart
    }

    /// Involutions generating the orbit ⟨α₀,…,α_{i−2}, α_{i+2},…,α_n⟩ used in the i-sew test.
    fn sewing_orbit_indices(&self, i: usize) -> impl Iterator<Item = usize> + '_ {
        (0..self.dimension()).filter(move |&j| j + 2 <= i || j >= i + 2)
    }

    pub fn orbit_indices(&self, i: usize) -> Vec<usize> {
        (0..self.dimension()).filter(|&idx| idx != i).collect()
    }

    pub fn add_face(&mut self, face: Face<'a, P>) -> FaceId {
        self.faces.insert(face)
    }

    pub fn face(&self, face_id: FaceId) -> Option<&Face<'a, P>> {
        self.faces.get(face_id)
    }

    pub fn add_solid(&mut self, solid: Solid<'a, P>) -> SolidId {
        self.solids.insert(solid)
    }

    pub fn solid(&self, solid_id: SolidId) -> Option<&Solid<'a, P>> {
        self.solids.get(solid_id)
    }

    /// Algorithm 19 of the book
    fn is_sewable(&self, d0: Dart, d1: Dart, i: usize) -> Option<SewableDarts> {
        if i >= self.dimension() || d0 == d1 || !self.is_free(d0, i) || !self.is_free(d1, i) {
            return None;
        }

        let inv: Vec<usize> = self.sewing_orbit_indices(i).collect();
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
    pub fn adjacent_cells(&self, dart: Dart, i: usize) -> impl Iterator<Item = Dart> + '_ {
        let orbit_indices = self.orbit_indices(i);
        let mut marked = vec![false; self.dart_count()];
        let mut i_orbit = self.orbit(dart, orbit_indices.clone());
        std::iter::from_fn(move || {
            for e in i_orbit.by_ref() {
                let neighbor = self.alpha(i, e);
                if marked[neighbor.id()] {
                    continue;
                }
                self.mark_orbit(neighbor, &orbit_indices, &mut marked);
                return Some(neighbor);
            }
            None
        })
    }

    fn apply_sew(&mut self, darts: SewableDarts, i: usize) {
        for (d0, d1) in darts.mapping {
            self.alphas[i][d0.id()] = d1;
            self.alphas[i][d1.id()] = d0;
        }
    }

    /// i-sew `d0` and `d1`. Fails if the configuration is not sewable
    /// (same dart, already i-sewn, or orbits not compatible).
    pub fn sew(&mut self, i: usize, d0: Dart, d1: Dart) -> Result<(), &'static str> {
        match self.is_sewable(d0, d1, i) {
            Some(sd) => {
                self.apply_sew(sd, i);
                Ok(())
            }
            None => Err("darts are not i-sewable"),
        }
    }

    fn unsew(&mut self, dart: Dart, i: usize) {
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

    /// Like [`Self::attribute_mut`], but when the `D`-cell has no attribute row
    /// yet (typical right after [`Self::add_dart`]), inserts `create()` at the
    /// cell representative and returns `&mut` to the stored value.
    ///
    /// Works for [`Cell0`] / [`Cell1`] / [`Cell2`]: the closure builds
    /// [`VertexAttr`](super::attributes::VertexAttr),
    /// [`EdgeAttr`](super::attributes::EdgeAttr), or a [`FaceId`](super::face::FaceId).
    pub fn attribute_mut_or_insert_with<D: CellDim, F>(
        &mut self,
        dart: Dart,
        create: F,
    ) -> &mut <Self as AttributeStore<D>>::Attr
    where
        Self: AttributeStore<D>,
        F: FnOnce() -> <Self as AttributeStore<D>>::Attr,
    {
        let repr = self.cell_representative(dart, D::DIM);
        <Self as AttributeStore<D>>::get_mut_or_insert_with(self, repr, create)
    }
}

pub struct OrbitIterator<'a, P: Payload> {
    gmap: &'a GMap<'a, P>,
    involutions: Vec<usize>,
    visited: Vec<bool>,
    queue: VecDeque<Dart>,
}

impl<'a, P: Payload> OrbitIterator<'a, P> {
    pub fn new(gmap: &'a GMap<'a, P>, start: Dart, involutions: Vec<usize>) -> Self {
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
