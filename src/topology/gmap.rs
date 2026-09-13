//! The generalized map itself: darts, alpha involutions, and nothing else.
//!
//! This module knows no geometry, no payload, and no logical entity. A `GMap`
//! is combinatorial connectivity and the operations that read or rewire it. The
//! stores that give a cell a curve, a surface or a stable key live in
//! [`Model`](crate::model::Model), one layer up, and so does every operation
//! that has to keep them in step with an edit.

use std::collections::{HashMap, HashSet, VecDeque};

use serde::{Deserialize, Serialize};

pub use super::dart::{Dart, IsolatedDart};

/// Topological cell dimension and matching alpha involution index.
///
/// `Dim::Zero` corresponds to vertices and alpha0, `Dim::One` to edges and
/// alpha1, and so on up to solids/sheets and alpha3.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
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
    pub(crate) mapping: HashMap<Dart, Dart>,
}

/// A 3-dimensional generalized map.
///
/// The map owns every dart and every alpha involution between them, and answers
/// questions about orbits: which darts make up a cell, which cells are incident
/// to which. Prefer the typed view objects for routine traversal, and reach for
/// `GMap` when implementing a topology algorithm that genuinely needs alphas.
#[derive(Clone, Serialize, Deserialize)]
pub struct GMap {
    alphas: [Vec<Dart>; GMAP_INVOLUTION_COUNT],
    free_slots: VecDeque<usize>,
}

impl Default for GMap {
    fn default() -> Self {
        Self::new()
    }
}

impl GMap {
    /// Creates an empty map with no darts.
    pub fn new() -> Self {
        Self {
            alphas: std::array::from_fn(|_| Vec::new()),
            free_slots: VecDeque::new(),
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
        self.alphas[d.index()][dart.id()]
    }

    /// A dart is `i`-free when `αᵢ(d) = d`, i.e. not sewn along dimension `i`.
    pub fn is_free(&self, dart: Dart, d: Dim) -> bool {
        self.alphas[d.index()][dart.id()] == dart
    }

    /// Iterates the orbit generated from `dart` by the given alpha indices.
    ///
    /// For cell traversals, prefer [`Self::orbit_indices`] and typed view
    /// methods when possible.
    pub fn orbit(&self, dart: Dart, involutions: Vec<usize>) -> OrbitIterator<'_> {
        OrbitIterator::new(self, dart, involutions)
    }

    /// Returns the alpha indices that generate a cell orbit of dimension `d`.
    ///
    /// For example, the edge orbit excludes alpha1 and includes every other
    /// alpha index.
    pub fn orbit_indices(&self, d: Dim) -> Vec<usize> {
        let i = d.index();
        (0..self.dimension()).filter(|&idx| idx != i).collect()
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

    /// Returns the alpha indices used to compare sewing orbits.
    fn sewing_orbit_indices(&self, d: Dim) -> impl Iterator<Item = usize> + '_ {
        let i = d.index();
        (0..self.dimension()).filter(move |&j| j + 2 <= i || j >= i + 2)
    }

    /// Algorithm 19 of the book: pairs two orbits for sewing along dimension `d`.
    ///
    /// Returns `None` when the two orbits are not isomorphic, or when either
    /// dart is already sewn along that dimension.
    pub(crate) fn is_sewable(&self, d0: Dart, d1: Dart, d: Dim) -> Option<SewableDarts> {
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

    /// Adds one isolated dart and returns its identifier.
    ///
    /// All alpha involutions initially map the new dart to itself.
    pub(crate) fn add_dart(&mut self) -> Dart {
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
    pub(crate) fn remove_dart(&mut self, dart: IsolatedDart) {
        for alphas in self.alphas.iter_mut() {
            alphas.remove(dart.id());
        }
        self.free_slots.push_back(dart.id());
    }

    /// Removes several isolated darts and renumbers the rest in one pass.
    ///
    /// Returns the old-to-new dart mapping. Repeated single removals would shift
    /// indexes under every reference held elsewhere; doing it once lets the
    /// caller rewrite all of them against a single map.
    ///
    /// # Panics
    ///
    /// Panics if any named dart is still linked by some alpha.
    pub(crate) fn compact(&mut self, darts: Vec<IsolatedDart>) -> HashMap<Dart, Dart> {
        if darts.is_empty() {
            return self.darts().map(|dart| (dart, dart)).collect();
        }
        let removed = darts
            .into_iter()
            .map(|dart| dart.id())
            .collect::<HashSet<_>>();
        for &id in &removed {
            let dart = Dart::new(id);
            assert!(
                (0..GMAP_INVOLUTION_COUNT).all(|dim| self.alphas[dim][id] == dart),
                "bulk dart removal requires every removed dart to be isolated"
            );
        }

        let mut remap = vec![None; self.dart_count()];
        let mut next = 0;
        for (old, slot) in remap.iter_mut().enumerate() {
            if !removed.contains(&old) {
                *slot = Some(Dart::new(next));
                next += 1;
            }
        }
        let map_dart = |dart: Dart| {
            remap[dart.id()].expect("retained topology must not reference a removed dart")
        };
        self.alphas = std::array::from_fn(|dim| {
            (0..remap.len())
                .filter(|old| !removed.contains(old))
                .map(|old| map_dart(self.alphas[dim][old]))
                .collect()
        });
        self.free_slots.clear();

        remap
            .into_iter()
            .enumerate()
            .filter_map(|(old, new)| new.map(|new| (Dart::new(old), new)))
            .collect()
    }

    /// Links two darts through alpha `d`, in both directions.
    pub(crate) fn link_raw(&mut self, d: Dim, d0: Dart, d1: Dart) {
        let i = d.index();
        self.alphas[i][d0.id()] = d1;
        self.alphas[i][d1.id()] = d0;
    }

    /// Unlinks the alpha `d` pair containing `dart`, returning its old partner.
    pub(crate) fn unlink_raw(&mut self, d: Dim, dart: Dart) -> Dart {
        let i = d.index();
        let a_i = self.alphas[i][dart.id()];
        self.alphas[i][a_i.id()] = a_i;
        self.alphas[i][dart.id()] = dart;
        a_i
    }

    /// Writes one side of an alpha link, leaving the other side alone.
    ///
    /// Copying a map writes each dart's links from its source in turn, so the
    /// pairing is restored by the copy as a whole rather than by any one write.
    /// Everything else should use [`Self::link_raw`], which cannot leave a
    /// one-sided link behind.
    pub(crate) fn point_alpha(&mut self, d: Dim, dart: Dart, target: Dart) {
        self.alphas[d.index()][dart.id()] = target;
    }
}

/// Breadth-first iterator over a dart orbit.
///
/// The iterator starts at one dart and follows the configured alpha indices,
/// yielding each reachable dart once.
pub struct OrbitIterator<'a> {
    gmap: &'a GMap,
    involutions: Vec<usize>,
    visited: Vec<bool>,
    queue: VecDeque<Dart>,
}

impl<'a> OrbitIterator<'a> {
    /// Creates an orbit iterator rooted at `start`.
    ///
    /// `involutions` contains alpha indices, not [`Dim`] values.
    pub fn new(gmap: &'a GMap, start: Dart, involutions: Vec<usize>) -> Self {
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

impl Iterator for OrbitIterator<'_> {
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
