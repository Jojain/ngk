use std::collections::{HashMap, HashSet, VecDeque};

use super::face::{Face, FaceId};
use super::solid::{Solid, SolidId};
use crate::geometry::curve::Curve;
use crate::geometry::point::Point;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Ord, PartialOrd)]
pub struct Dart {
    id: usize,
    pub face_id: Option<FaceId>,
    pub solid_id: Option<SolidId>,
}

impl Dart {
    pub fn new(id: usize) -> Self {
        Self {
            id,
            face_id: None,
            solid_id: None,
        }
    }
}

struct IsolatedDart(Dart);

struct SewableDarts {
    mapping: HashMap<Dart, Dart>,
}

#[derive(Clone)]
pub struct GMap<'a> {
    alphas: Vec<Vec<Dart>>,
    free_slots: VecDeque<usize>,
    vertices: HashMap<Dart, Point>,
    edges: HashMap<Dart, Curve>,
    faces: HashMap<FaceId, Face<'a>>,
    solids: HashMap<SolidId, Solid<'a>>,
}

impl<'a> GMap<'a> {
    pub fn new(dim: usize) -> Self {
        let alphas = (0..dim).map(|_| Vec::new()).collect();
        let free_slots = VecDeque::new();
        let vertices = HashMap::new();
        let edges = HashMap::new();
        let faces = HashMap::new();
        let solids = HashMap::new();
        Self {
            alphas,
            free_slots,
            vertices,
            edges,
            faces,
            solids,
        }
    }

    pub fn dimension(&self) -> usize {
        self.alphas.len()
    }

    pub fn dart_count(&self) -> usize {
        self.alphas[0].len()
    }

    pub fn alpha(&self, dim: usize, dart: Dart) -> Dart {
        self.alphas[dim][dart.id]
    }

    fn add_dart(&mut self) -> Dart {
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

    fn remove_dart(&mut self, dart: IsolatedDart) {
        for alphas in self.alphas.iter_mut() {
            alphas.remove(dart.0.id);
        }
        self.free_slots.push_back(dart.0.id);
    }

    pub fn orbit(&self, dart: Dart, involutions: Vec<usize>) -> OrbitIterator<'_> {
        OrbitIterator::new(self, dart, involutions)
    }

    fn is_free(&self, dart: Dart, i: usize) -> bool {
        self.alphas[dart.0][i] == dart
    }

    /// Involutions generating the orbit ⟨α₀,…,α_{i−2}, α_{i+2},…,α_n⟩ used in the i-sew test.
    fn sewing_orbit_indices(&self, i: usize) -> impl Iterator<Item = usize> + '_ {
        (0..self.dimension()).filter(move |&j| j + 2 <= i || j >= i + 2)
    }

    pub fn orbit_indices(&self, i: usize) -> Vec<usize> {
        (0..self.dimension()).filter(|&idx| idx != i).collect()
    }

    pub fn get_face(&self, face_id: FaceId) -> &Face<'a> {
        &self.faces[&face_id]
    }

    pub fn get_solid(&self, solid_id: SolidId) -> &Solid<'a> {
        &self.solids[&solid_id]
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
                        let a_aj = self.alphas[*j][a.0];
                        let b_aj = self.alphas[*j][b.0];
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

    pub fn cell_representative(&self, dart: Dart, dim: usize) -> Dart {
        self.orbit(dart, self.orbit_indices(dim)).min().unwrap()
    }

    /// Returns an iterator of unique representative darts of dimension `target_dim`
    /// that are incident to the cell defined by `dart` at `container_dim`.
    pub fn incident_cells(
        &self,
        dart: Dart,
        container_dim: usize,
        target_dim: usize,
    ) -> impl Iterator<Item = Dart> + '_ {
        self.orbit(dart, self.orbit_indices(container_dim))
            .filter(move |&d| {
                let rep = self.cell_representative(d, target_dim);
                d == rep
            })
    }

    fn sew(&mut self, darts: SewableDarts, i: usize) {
        for (d0, d1) in darts.mapping {
            self.alphas[i][d0.id] = d1;
            self.alphas[i][d1.id] = d0;
        }
    }

    fn unsew(&mut self, dart: Dart, i: usize) {
        let a_i = self.alphas[i][dart.id];
        self.alphas[i][a_i.id] = a_i;
        self.alphas[i][dart.id] = dart;
    }

    fn increased_dimension(&self) -> Self {
        let mut new_gmap = self.clone();
        let new_dim = (0..self.dart_count()).map(|i| Dart::new(i)).collect();
        new_gmap.alphas.push(new_dim);
        new_gmap
    }

    fn decreased_dimension(&self) -> Self {
        let mut new_gmap = self.clone();
        new_gmap.alphas.pop();
        new_gmap
    }
}

pub struct OrbitIterator<'a> {
    gmap: &'a GMap<'a>,
    involutions: Vec<usize>, // which involution indices to traverse
    visited: Vec<bool>,      // visited[dart_id] = true if already yielded
    queue: VecDeque<Dart>,   // BFS queue
}

fn face_darts(gmap: &'a GMap<'a>, dart: Dart) -> Vec<Dart> {
    if let Some(face_id) = dart.face_id {
        gmap.get_face(face_id)
            .loops
            .iter()
            .map(|loop_| loop_.dart)
            .filter(|d| d.id != dart.id)
            .collect()
    } else {
        Vec::new()
    }
}

fn solid_darts(gmap: &'a GMap<'a>, dart: Dart) -> Vec<Dart> {
    if let Some(solid_id) = dart.solid_id {
        gmap.get_solid(solid_id)
            .shells()
            .iter()
            .map(|shell| shell.dart)
            .filter(|d| d.id != dart.id)
            .collect()
    } else {
        Vec::new()
    }
}

impl<'a> OrbitIterator<'a> {
    pub fn new(gmap: &'a GMap<'a>, start: Dart, involutions: Vec<usize>) -> Self {
        let dart_count = gmap.dart_count();
        let mut visited = vec![false; dart_count];
        let mut queue = VecDeque::new();

        visited[start.id] = true;
        queue.push_back(start);

        let face_darts = face_darts(gmap, start);
        for dart in face_darts {
            visited[dart.id] = true;
            queue.push_back(dart);
        }

        let solid_darts = solid_darts(gmap, start);
        for dart in solid_darts {
            visited[dart.id] = true;
            queue.push_back(dart);
        }

        Self {
            gmap,
            involutions,
            visited,
            queue,
        }
    }
}

impl<'a> Iterator for OrbitIterator<'a> {
    type Item = Dart;

    fn next(&mut self) -> Option<Self::Item> {
        let dart = self.queue.pop_front()?;

        let face_darts = face_darts(self.gmap, dart);
        for dart in face_darts {
            if !self.visited[dart.id] {
                self.visited[dart.id] = true;
                self.queue.push_back(dart);
            }
        }

        let solid_darts = solid_darts(self.gmap, dart);
        for dart in solid_darts {
            if !self.visited[dart.id] {
                self.visited[dart.id] = true;
                self.queue.push_back(dart);
            }
        }

        // apply each alpha in the orbit definition
        for &i in &self.involutions {
            let neighbor = self.gmap.alphas[i][dart.id];

            if !self.visited[neighbor.id] {
                self.visited[neighbor.id] = true;
                self.queue.push_back(neighbor);
            }
        }

        Some(dart)
    }
}
