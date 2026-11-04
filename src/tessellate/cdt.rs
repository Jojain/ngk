//! A constrained Delaunay triangulation of a face's parameter domain.
//!
//! The face's boundary arrives as closed rings of points, each segment of
//! which must appear as an edge: those are the points every face along an
//! edge shares, so none may be moved and none may be split. Inside them the
//! triangulation is Delaunay, built by incremental insertion and edge flips;
//! the boundary is forced in by flipping the edges it crosses (Sloan); and the
//! triangles are then told inside from outside by the parity of the boundary
//! segments crossed to reach them, a segment the boundary runs twice (a slit)
//! counting as none.
//!
//! Every geometric decision -- which side of a line a point is on, whether it
//! is inside a circle -- is taken by Shewchuk's adaptive exact predicates
//! (`robust`), so nearly collinear and nearly cocircular inputs are decided
//! correctly and the triangulation cannot fold over. Only the coordinates of
//! points inserted during refinement are computed in floating point.

use robust::{Coord, incircle, orient2d};

use crate::geometry::Point2;

/// No neighbour: the outside of the enclosing triangle.
const NONE: u32 = u32::MAX;
/// The most steps a point location walk may take before it gives up.
const MAX_WALK: usize = 1 << 22;

#[derive(Debug, Clone, Copy)]
struct Triangle {
    /// Vertices, counter-clockwise.
    v: [u32; 3],
    /// The neighbour across the edge opposite `v[i]`.
    n: [u32; 3],
    /// How many times the boundary runs the edge opposite `v[i]`.
    c: [u8; 3],
    inside: bool,
}

/// Where a point falls.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Location {
    Triangle(u32),
    /// On the edge opposite vertex `i` of a triangle.
    Edge(u32, usize),
    Vertex(u32),
}

/// Why a point was not inserted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Refused {
    /// It is an existing vertex.
    Duplicate(u32),
    /// It lies on a boundary segment, which would split it.
    OnBoundary,
    /// It lies outside the domain.
    Outside,
}

/// Why the boundary could not be built.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum CdtError {
    /// Two boundary segments cross, so the boundary bounds nothing.
    CrossingBoundary,
    /// A point could not be located (a walk ran out of steps).
    Lost,
}

pub(super) struct Cdt {
    points: Vec<Point2>,
    triangles: Vec<Triangle>,
    /// One triangle incident to each vertex.
    incident: Vec<u32>,
    hint: u32,
    seed: u32,
}

fn coord(p: Point2) -> Coord<f64> {
    Coord { x: p.x, y: p.y }
}

impl Cdt {
    /// The first vertex that is a point of the domain; the three before it
    /// enclose everything.
    pub(super) const FIRST: u32 = 3;

    /// An empty triangulation enclosing the box `min`–`max`.
    pub(super) fn new(min: Point2, max: Point2) -> Self {
        let centre = nalgebra::center(&min, &max);
        let size = (max - min).abs().max().max(1e-300) * 64.0;
        let points = vec![
            Point2::new(centre.x - 3.0 * size, centre.y - 2.0 * size),
            Point2::new(centre.x + 3.0 * size, centre.y - 2.0 * size),
            Point2::new(centre.x, centre.y + 3.0 * size),
        ];
        Self {
            points,
            triangles: vec![Triangle {
                v: [0, 1, 2],
                n: [NONE; 3],
                c: [0; 3],
                inside: false,
            }],
            incident: vec![0, 0, 0],
            hint: 0,
            seed: 0x9e37_79b9,
        }
    }

    pub(super) fn point(&self, vertex: u32) -> Point2 {
        self.points[vertex as usize]
    }

    pub(super) fn vertex_count(&self) -> usize {
        self.points.len()
    }

    /// The vertices of every triangle inside the domain.
    pub(super) fn inside(&self) -> impl Iterator<Item = (u32, [u32; 3])> + '_ {
        self.triangles
            .iter()
            .enumerate()
            .filter(|(_, t)| t.inside)
            .map(|(index, t)| (index as u32, t.v))
    }

    pub(super) fn vertices_of(&self, triangle: u32) -> Option<[u32; 3]> {
        let t = self.triangles.get(triangle as usize)?;
        t.inside.then_some(t.v)
    }

    /// Whether the edge opposite vertex `i` of `triangle` is a boundary segment.
    pub(super) fn is_boundary(&self, triangle: u32, i: usize) -> bool {
        self.triangles[triangle as usize].c[i] > 0
    }

    fn orient(&self, a: u32, b: u32, c: u32) -> f64 {
        orient2d(coord(self.point(a)), coord(self.point(b)), coord(self.point(c)))
    }

    fn orient_point(&self, a: u32, b: u32, p: Point2) -> f64 {
        orient2d(coord(self.point(a)), coord(self.point(b)), coord(p))
    }

    fn next_random(&mut self) -> u32 {
        self.seed ^= self.seed << 13;
        self.seed ^= self.seed >> 17;
        self.seed ^= self.seed << 5;
        self.seed
    }

    /// Where `p` falls, by a walk from the last triangle touched.
    pub(super) fn locate(&mut self, p: Point2) -> Result<Location, CdtError> {
        let mut t = self.hint.min(self.triangles.len() as u32 - 1);
        for _ in 0..MAX_WALK {
            let triangle = self.triangles[t as usize];
            let offset = (self.next_random() % 3) as usize;
            let mut moved = false;
            for k in 0..3 {
                let i = (k + offset) % 3;
                let (a, b) = (triangle.v[(i + 1) % 3], triangle.v[(i + 2) % 3]);
                if self.orient_point(a, b, p) < 0.0 {
                    let next = triangle.n[i];
                    if next == NONE {
                        return Err(CdtError::Lost);
                    }
                    t = next;
                    moved = true;
                    break;
                }
            }
            if moved {
                continue;
            }
            self.hint = t;
            for i in 0..3 {
                if self.points[triangle.v[i] as usize] == p {
                    return Ok(Location::Vertex(triangle.v[i]));
                }
            }
            for i in 0..3 {
                let (a, b) = (triangle.v[(i + 1) % 3], triangle.v[(i + 2) % 3]);
                if self.orient_point(a, b, p) == 0.0 {
                    return Ok(Location::Edge(t, i));
                }
            }
            return Ok(Location::Triangle(t));
        }
        Err(CdtError::Lost)
    }

    fn set_neighbour(&mut self, triangle: u32, old: u32, new: u32) {
        if triangle == NONE {
            return;
        }
        let t = &mut self.triangles[triangle as usize];
        for k in 0..3 {
            if t.n[k] == old {
                t.n[k] = new;
                return;
            }
        }
    }

    fn touch(&mut self, triangle: u32) {
        let v = self.triangles[triangle as usize].v;
        for vertex in v {
            self.incident[vertex as usize] = triangle;
        }
    }

    /// Adds `p` as a vertex of the triangulation, wherever it falls.
    ///
    /// Refused on an existing vertex, and on a boundary segment unless
    /// `split_boundary`; the triangles made inherit the side of the domain of
    /// the one they replace.
    pub(super) fn insert(&mut self, p: Point2, inside_only: bool) -> Result<u32, Refused> {
        let location = self.locate(p).map_err(|_| Refused::Outside)?;
        match location {
            Location::Vertex(vertex) => Err(Refused::Duplicate(vertex)),
            Location::Triangle(t) => {
                if inside_only && !self.triangles[t as usize].inside {
                    return Err(Refused::Outside);
                }
                let vertex = self.push_point(p);
                self.split_triangle(t, vertex);
                Ok(vertex)
            }
            Location::Edge(t, i) => {
                if self.triangles[t as usize].c[i] > 0 {
                    return Err(Refused::OnBoundary);
                }
                let across = self.triangles[t as usize].n[i];
                if inside_only
                    && (!self.triangles[t as usize].inside
                        || (across != NONE && !self.triangles[across as usize].inside))
                {
                    return Err(Refused::Outside);
                }
                let vertex = self.push_point(p);
                self.split_edge(t, i, vertex);
                Ok(vertex)
            }
        }
    }

    fn push_point(&mut self, p: Point2) -> u32 {
        self.points.push(p);
        self.incident.push(0);
        (self.points.len() - 1) as u32
    }

    fn split_triangle(&mut self, t: u32, p: u32) {
        let old = self.triangles[t as usize];
        let [a, b, c] = old.v;
        let [na, nb, nc] = old.n;
        let [ca, cb, cc] = old.c;
        let t1 = self.triangles.len() as u32;
        let t2 = t1 + 1;
        self.triangles[t as usize] = Triangle {
            v: [p, b, c],
            n: [na, t1, t2],
            c: [ca, 0, 0],
            inside: old.inside,
        };
        self.triangles.push(Triangle {
            v: [p, c, a],
            n: [nb, t2, t],
            c: [cb, 0, 0],
            inside: old.inside,
        });
        self.triangles.push(Triangle {
            v: [p, a, b],
            n: [nc, t, t1],
            c: [cc, 0, 0],
            inside: old.inside,
        });
        self.set_neighbour(nb, t, t1);
        self.set_neighbour(nc, t, t2);
        for triangle in [t, t1, t2] {
            self.touch(triangle);
        }
        self.hint = t;
        self.legalise(vec![t, t1, t2], p);
    }

    fn split_edge(&mut self, t: u32, i: usize, p: u32) {
        // t = (x, a, b) with the edge a-b opposite x; u = (y, b, a) across it.
        let old = self.triangles[t as usize];
        let x = old.v[i];
        let a = old.v[(i + 1) % 3];
        let b = old.v[(i + 2) % 3];
        let n_xa = old.n[(i + 2) % 3]; // across x-a, opposite b
        let n_bx = old.n[(i + 1) % 3]; // across b-x, opposite a
        let c_xa = old.c[(i + 2) % 3];
        let c_bx = old.c[(i + 1) % 3];
        let u = old.n[i];
        let t1 = self.triangles.len() as u32;
        // t becomes (x, a, p), t1 is (x, p, b).
        self.triangles[t as usize] = Triangle {
            v: [x, a, p],
            n: [NONE, t1, n_xa],
            c: [0, 0, c_xa],
            inside: old.inside,
        };
        self.triangles.push(Triangle {
            v: [x, p, b],
            n: [NONE, n_bx, t],
            c: [0, c_bx, 0],
            inside: old.inside,
        });
        self.set_neighbour(n_bx, t, t1);
        let mut fresh = vec![t, t1];
        if u != NONE {
            let other = self.triangles[u as usize];
            let j = (0..3).find(|&j| other.n[j] == t).expect("neighbours agree");
            let y = other.v[j];
            // u = (y, b, a): across y-b opposite a, across a-y opposite b.
            let n_yb = other.n[(j + 2) % 3];
            let n_ay = other.n[(j + 1) % 3];
            let c_yb = other.c[(j + 2) % 3];
            let c_ay = other.c[(j + 1) % 3];
            let u1 = self.triangles.len() as u32;
            // u becomes (y, b, p), u1 is (y, p, a).
            self.triangles[u as usize] = Triangle {
                v: [y, b, p],
                n: [t1, u1, n_yb],
                c: [0, 0, c_yb],
                inside: other.inside,
            };
            self.triangles.push(Triangle {
                v: [y, p, a],
                n: [t, n_ay, u],
                c: [0, c_ay, 0],
                inside: other.inside,
            });
            self.set_neighbour(n_ay, u, u1);
            self.triangles[t as usize].n[0] = u1;
            self.triangles[t1 as usize].n[0] = u;
            fresh.extend([u, u1]);
        }
        for triangle in fresh.iter().copied() {
            self.touch(triangle);
        }
        self.hint = t;
        self.legalise(fresh, p);
    }

    /// Restores the Delaunay property round a new vertex `p`, flipping edges
    /// opposite it that are not boundary segments.
    fn legalise(&mut self, mut stack: Vec<u32>, p: u32) {
        while let Some(t) = stack.pop() {
            let triangle = self.triangles[t as usize];
            let Some(i) = (0..3).find(|&i| triangle.v[i] == p) else {
                continue;
            };
            if triangle.c[i] > 0 {
                continue;
            }
            let u = triangle.n[i];
            if u == NONE {
                continue;
            }
            let other = self.triangles[u as usize];
            let Some(j) = (0..3).find(|&j| other.n[j] == t) else {
                continue;
            };
            let d = other.v[j];
            let [a, b, c] = triangle.v;
            let inside = incircle(
                coord(self.point(a)),
                coord(self.point(b)),
                coord(self.point(c)),
                coord(self.point(d)),
            );
            if inside > 0.0 {
                let (t, u) = self.flip(t, i);
                stack.push(t);
                stack.push(u);
            }
        }
    }

    /// Flips the edge opposite vertex `i` of `t`. Returns the two triangles,
    /// both now holding `t.v[i]`.
    fn flip(&mut self, t: u32, i: usize) -> (u32, u32) {
        let tt = self.triangles[t as usize];
        let u = tt.n[i];
        let uu = self.triangles[u as usize];
        let j = (0..3).find(|&j| uu.n[j] == t).expect("neighbours agree");
        let p = tt.v[i];
        let a = tt.v[(i + 1) % 3];
        let b = tt.v[(i + 2) % 3];
        let d = uu.v[j];
        let t_na = tt.n[(i + 1) % 3]; // across b-p
        let t_nb = tt.n[(i + 2) % 3]; // across p-a
        let t_ca = tt.c[(i + 1) % 3];
        let t_cb = tt.c[(i + 2) % 3];
        let u_nb = uu.n[(j + 1) % 3]; // across a-d (opposite b in u = (d, b, a))
        let u_na = uu.n[(j + 2) % 3]; // across d-b
        let u_cb = uu.c[(j + 1) % 3];
        let u_ca = uu.c[(j + 2) % 3];
        self.triangles[t as usize] = Triangle {
            v: [p, a, d],
            n: [u_nb, u, t_nb],
            c: [u_cb, 0, t_cb],
            inside: tt.inside,
        };
        self.triangles[u as usize] = Triangle {
            v: [p, d, b],
            n: [u_na, t_na, t],
            c: [u_ca, t_ca, 0],
            inside: uu.inside,
        };
        self.set_neighbour(u_nb, u, t);
        self.set_neighbour(t_na, t, u);
        self.touch(t);
        self.touch(u);
        (t, u)
    }

    /// The triangle holding the edge `x`→`y` counter-clockwise, and the index
    /// of the vertex opposite it.
    fn edge(&self, x: u32, y: u32) -> Option<(u32, usize)> {
        let start = self.incident[x as usize];
        let mut t = start;
        for _ in 0..self.triangles.len() + 1 {
            let triangle = self.triangles[t as usize];
            let ix = (0..3).find(|&k| triangle.v[k] == x)?;
            if triangle.v[(ix + 1) % 3] == y {
                return Some((t, (ix + 2) % 3));
            }
            // Turn round x counter-clockwise: across the edge x-v[ix+2].
            let next = triangle.n[(ix + 1) % 3];
            if next == NONE || next == start {
                break;
            }
            t = next;
        }
        // The fan may be open; turn the other way.
        let mut t = start;
        for _ in 0..self.triangles.len() + 1 {
            let triangle = self.triangles[t as usize];
            let ix = (0..3).find(|&k| triangle.v[k] == x)?;
            if triangle.v[(ix + 1) % 3] == y {
                return Some((t, (ix + 2) % 3));
            }
            let next = triangle.n[(ix + 2) % 3];
            if next == NONE || next == start {
                break;
            }
            t = next;
        }
        None
    }

    /// Whether `p` lies inside the diametral circle of a boundary segment
    /// near it: a point there would crowd the segment, which cannot be split.
    pub(super) fn encroaches(&mut self, p: Point2) -> bool {
        let Ok(location) = self.locate(p) else { return true };
        let start = match location {
            Location::Triangle(t) | Location::Edge(t, _) => t,
            Location::Vertex(_) => return true,
        };
        let mut seen = vec![start];
        let mut frontier = vec![start];
        for _ in 0..2 {
            let mut next = Vec::new();
            for &t in &frontier {
                for k in 0..3 {
                    let u = self.triangles[t as usize].n[k];
                    if u != NONE && !seen.contains(&u) {
                        seen.push(u);
                        next.push(u);
                    }
                }
            }
            frontier = next;
        }
        for &t in &seen {
            let triangle = self.triangles[t as usize];
            for k in 0..3 {
                if triangle.c[k] == 0 {
                    continue;
                }
                let a = self.point(triangle.v[(k + 1) % 3]);
                let b = self.point(triangle.v[(k + 2) % 3]);
                if (a - p).dot(&(b - p)) < 0.0 {
                    return true;
                }
            }
        }
        false
    }

    /// Every triangle round vertex `x`.
    pub(super) fn around(&self, x: u32) -> Vec<u32> {
        let start = self.incident[x as usize];
        let mut out = Vec::new();
        let mut t = start;
        for _ in 0..self.triangles.len() + 1 {
            out.push(t);
            let triangle = self.triangles[t as usize];
            let Some(ix) = (0..3).find(|&k| triangle.v[k] == x) else { break };
            let next = triangle.n[(ix + 1) % 3];
            if next == NONE || next == start {
                return out;
            }
            t = next;
        }
        out
    }

    fn crosses(&self, a: u32, b: u32, p: u32, q: u32) -> bool {
        let o1 = self.orient(a, b, p);
        let o2 = self.orient(a, b, q);
        let o3 = self.orient(p, q, a);
        let o4 = self.orient(p, q, b);
        o1 * o2 < 0.0 && o3 * o4 < 0.0
    }

    fn mark(&mut self, x: u32, y: u32) -> bool {
        let Some((t, i)) = self.edge(x, y).or_else(|| self.edge(y, x)) else {
            return false;
        };
        self.triangles[t as usize].c[i] = self.triangles[t as usize].c[i].saturating_add(1);
        let u = self.triangles[t as usize].n[i];
        if u != NONE {
            if let Some(j) = (0..3).find(|&j| self.triangles[u as usize].n[j] == t) {
                self.triangles[u as usize].c[j] = self.triangles[u as usize].c[j].saturating_add(1);
            }
        }
        true
    }

    /// Forces the segment `a`–`b` into the triangulation as a boundary
    /// segment. A vertex lying on it splits it in two.
    pub(super) fn insert_segment(&mut self, a: u32, b: u32) -> Result<(), CdtError> {
        let mut pending = vec![(a, b)];
        let mut budget = 64 + 8 * self.triangles.len();
        while let Some((a, b)) = pending.pop() {
            if budget == 0 {
                return Err(CdtError::Lost);
            }
            budget -= 1;
            self.insert_segment_once(a, b, &mut pending)?;
        }
        Ok(())
    }

    /// Makes `a`–`b` a boundary segment, or the first piece of it: a vertex
    /// lying on it leaves the two halves in `pending`.
    fn insert_segment_once(
        &mut self,
        a: u32,
        b: u32,
        pending: &mut Vec<(u32, u32)>,
    ) -> Result<(), CdtError> {
        if a == b || self.mark(a, b) {
            return Ok(());
        }
        // Find the triangle round `a` the segment leaves through.
        let fan = self.around(a);
        let mut crossing: Option<(u32, u32)> = None;
        for &t in &fan {
            let triangle = self.triangles[t as usize];
            let Some(ia) = (0..3).find(|&k| triangle.v[k] == a) else { continue };
            let v1 = triangle.v[(ia + 1) % 3];
            let v2 = triangle.v[(ia + 2) % 3];
            let o1 = self.orient(a, v1, b);
            let o2 = self.orient(a, v2, b);
            if o1 == 0.0 && self.between(a, v1, b) {
                pending.push((v1, b));
                pending.push((a, v1));
                return Ok(());
            }
            if o2 == 0.0 && self.between(a, v2, b) {
                pending.push((v2, b));
                pending.push((a, v2));
                return Ok(());
            }
            if o1 > 0.0 && o2 < 0.0 {
                crossing = Some((v1, v2));
                break;
            }
        }
        let Some((mut p, mut q)) = crossing else {
            return Err(CdtError::Lost);
        };
        // Walk to b, collecting every edge the segment crosses.
        let mut crossed = Vec::new();
        loop {
            if self.edge_is_boundary(p, q) {
                return Err(CdtError::CrossingBoundary);
            }
            crossed.push((p, q));
            // The triangle beyond p-q, away from a: it holds q-p counter-clockwise.
            let Some((t, i)) = self.edge(q, p) else {
                return Err(CdtError::Lost);
            };
            let w = self.triangles[t as usize].v[i];
            if w == b {
                break;
            }
            let o = self.orient(a, b, w);
            if o == 0.0 {
                // A vertex on the segment: force each half.
                pending.push((w, b));
                pending.push((a, w));
                return Ok(());
            }
            // `p` is the end of the crossed edge to the right of a→b and `q`
            // the end to its left; the vertex beyond replaces the one on its
            // own side.
            if o > 0.0 {
                q = w;
            } else {
                p = w;
            }
            if crossed.len() > self.triangles.len() {
                return Err(CdtError::Lost);
            }
        }
        self.flip_out(a, b, crossed)?;
        if !self.mark(a, b) {
            return Err(CdtError::Lost);
        }
        Ok(())
    }

    fn between(&self, a: u32, m: u32, b: u32) -> bool {
        let (pa, pm, pb) = (self.point(a), self.point(m), self.point(b));
        let along = (pb - pa).dot(&(pm - pa));
        along > 0.0 && along < (pb - pa).norm_squared()
    }

    fn edge_is_boundary(&self, x: u32, y: u32) -> bool {
        self.edge(x, y)
            .map(|(t, i)| self.triangles[t as usize].c[i] > 0)
            .unwrap_or(false)
    }

    /// Flips every edge in `crossed` until none crosses `a`–`b` (Sloan), then
    /// restores the Delaunay property among the edges made.
    fn flip_out(&mut self, a: u32, b: u32, crossed: Vec<(u32, u32)>) -> Result<(), CdtError> {
        let mut queue: std::collections::VecDeque<(u32, u32)> = crossed.into();
        let mut made = Vec::new();
        let mut stalls = 0usize;
        while let Some((p, q)) = queue.pop_front() {
            let Some((t, i)) = self.edge(p, q).or_else(|| self.edge(q, p)) else {
                return Err(CdtError::Lost);
            };
            let tt = self.triangles[t as usize];
            let u = tt.n[i];
            if u == NONE {
                return Err(CdtError::Lost);
            }
            let x = tt.v[i];
            let uu = self.triangles[u as usize];
            let j = (0..3).find(|&j| uu.n[j] == t).ok_or(CdtError::Lost)?;
            let y = uu.v[j];
            let (e1, e2) = (tt.v[(i + 1) % 3], tt.v[(i + 2) % 3]);
            // The quad x, e1, y, e2 is strictly convex when the new diagonal
            // x-y separates e1 from e2.
            let convex = self.orient(x, y, e1) * self.orient(x, y, e2) < 0.0
                && self.orient(e1, e2, x) * self.orient(e1, e2, y) < 0.0;
            if !convex {
                queue.push_back((p, q));
                stalls += 1;
                if stalls > queue.len() * 4 + 16 {
                    return Err(CdtError::Lost);
                }
                continue;
            }
            stalls = 0;
            self.flip(t, i);
            if (x == a && y == b) || (x == b && y == a) {
                continue;
            }
            if self.crosses(a, b, x, y) {
                queue.push_back((x, y));
            } else {
                made.push((x, y));
            }
        }
        // Delaunay restoration among the new edges.
        let mut changed = true;
        let mut rounds = 0;
        while changed && rounds < 64 {
            changed = false;
            rounds += 1;
            for k in 0..made.len() {
                let (x, y) = made[k];
                if (x == a && y == b) || (x == b && y == a) {
                    continue;
                }
                let Some((t, i)) = self.edge(x, y) else { continue };
                if self.triangles[t as usize].c[i] > 0 {
                    continue;
                }
                let u = self.triangles[t as usize].n[i];
                if u == NONE {
                    continue;
                }
                let uu = self.triangles[u as usize];
                let Some(j) = (0..3).find(|&j| uu.n[j] == t) else { continue };
                let d = uu.v[j];
                let [p0, p1, p2] = self.triangles[t as usize].v;
                if incircle(
                    coord(self.point(p0)),
                    coord(self.point(p1)),
                    coord(self.point(p2)),
                    coord(self.point(d)),
                ) > 0.0
                {
                    let e1 = self.triangles[t as usize].v[(i + 1) % 3];
                    let e2 = self.triangles[t as usize].v[(i + 2) % 3];
                    let w = self.triangles[t as usize].v[i];
                    let convex = self.orient(w, d, e1) * self.orient(w, d, e2) < 0.0;
                    if convex {
                        self.flip(t, i);
                        made[k] = (w, d);
                        changed = true;
                    }
                }
            }
        }
        Ok(())
    }

    /// Tells inside from outside: a triangle is inside when reaching it from
    /// the enclosing triangle crosses an odd number of boundary segments.
    pub(super) fn classify(&mut self) {
        let mut parity = vec![u8::MAX; self.triangles.len()];
        let start = (0..self.triangles.len())
            .find(|&t| self.triangles[t].v.contains(&0))
            .unwrap_or(0);
        let mut stack = vec![(start as u32, 0u8)];
        while let Some((t, side)) = stack.pop() {
            if parity[t as usize] != u8::MAX {
                continue;
            }
            parity[t as usize] = side;
            let triangle = self.triangles[t as usize];
            for k in 0..3 {
                let next = triangle.n[k];
                if next == NONE || parity[next as usize] != u8::MAX {
                    continue;
                }
                stack.push((next, side ^ (triangle.c[k] & 1)));
            }
        }
        for (t, side) in self.triangles.iter_mut().zip(parity) {
            t.inside = side == 1 && t.v.iter().all(|&v| v >= Self::FIRST);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ring(cdt: &mut Cdt, points: &[(f64, f64)]) -> Vec<u32> {
        let ids: Vec<u32> = points
            .iter()
            .map(|&(x, y)| match cdt.insert(Point2::new(x, y), false) {
                Ok(v) | Err(Refused::Duplicate(v)) => v,
                Err(e) => panic!("{e:?}"),
            })
            .collect();
        ids
    }

    fn constrain(cdt: &mut Cdt, ids: &[u32]) {
        for k in 0..ids.len() {
            cdt.insert_segment(ids[k], ids[(k + 1) % ids.len()]).unwrap();
        }
    }

    fn area(cdt: &Cdt) -> f64 {
        cdt.inside()
            .map(|(_, [a, b, c])| {
                let (a, b, c) = (cdt.point(a), cdt.point(b), cdt.point(c));
                0.5 * ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x))
            })
            .sum()
    }

    fn assert_delaunay(cdt: &Cdt) {
        for (t, v) in cdt.inside() {
            for i in 0..3 {
                if cdt.is_boundary(t, i) {
                    continue;
                }
                let u = cdt.triangles[t as usize].n[i];
                if u == NONE {
                    continue;
                }
                let j = (0..3).find(|&j| cdt.triangles[u as usize].n[j] == t).unwrap();
                let d = cdt.triangles[u as usize].v[j];
                let inside = incircle(
                    coord(cdt.point(v[0])),
                    coord(cdt.point(v[1])),
                    coord(cdt.point(v[2])),
                    coord(cdt.point(d)),
                );
                assert!(inside <= 0.0, "edge opposite {i} of {t} is not locally Delaunay");
            }
        }
    }

    fn rand(seed: &mut u64) -> f64 {
        *seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((*seed >> 11) as f64) / ((1u64 << 53) as f64)
    }

    #[test]
    fn a_square_with_random_points_has_its_area() {
        let mut cdt = Cdt::new(Point2::new(0.0, 0.0), Point2::new(1.0, 1.0));
        let ids = ring(&mut cdt, &[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]);
        constrain(&mut cdt, &ids);
        cdt.classify();
        let mut seed = 7;
        for _ in 0..500 {
            let _ = cdt.insert(Point2::new(rand(&mut seed), rand(&mut seed)), true);
        }
        assert!((area(&cdt) - 1.0).abs() < 1e-12);
        assert_delaunay(&cdt);
    }

    #[test]
    fn a_ring_with_a_hole_and_a_slanted_constraint() {
        let n = 64;
        let circle = |r: f64| -> Vec<(f64, f64)> {
            (0..n)
                .map(|k| {
                    let a = std::f64::consts::TAU * k as f64 / n as f64;
                    (r * a.cos(), r * a.sin())
                })
                .collect()
        };
        let mut cdt = Cdt::new(Point2::new(-2.0, -2.0), Point2::new(2.0, 2.0));
        let outer = ring(&mut cdt, &circle(2.0));
        let mut inner_points = circle(1.0);
        inner_points.reverse();
        let inner = ring(&mut cdt, &inner_points);
        constrain(&mut cdt, &outer);
        constrain(&mut cdt, &inner);
        cdt.classify();
        let expected = |n: f64, r: f64| 0.5 * n * r * r * (std::f64::consts::TAU / n).sin();
        let want = expected(n as f64, 2.0) - expected(n as f64, 1.0);
        assert!((area(&cdt) - want).abs() < 1e-9, "{} against {}", area(&cdt), want);
        let mut seed = 3;
        for _ in 0..300 {
            let r = 1.0 + rand(&mut seed);
            let a = std::f64::consts::TAU * rand(&mut seed);
            let _ = cdt.insert(Point2::new(r * a.cos(), r * a.sin()), true);
        }
        assert!((area(&cdt) - want).abs() < 1e-9);
        assert_delaunay(&cdt);
    }

    #[test]
    fn a_boundary_segment_is_kept_when_points_are_inserted_beside_it() {
        let mut cdt = Cdt::new(Point2::new(0.0, 0.0), Point2::new(4.0, 1.0));
        let ids = ring(&mut cdt, &[(0.0, 0.0), (4.0, 0.0), (4.0, 1.0), (0.0, 1.0)]);
        constrain(&mut cdt, &ids);
        cdt.classify();
        for k in 1..40 {
            // Exactly on the boundary, and a hair inside it.
            let x = k as f64 * 0.1;
            assert_eq!(cdt.insert(Point2::new(x, 0.0), true), Err(Refused::OnBoundary));
            let _ = cdt.insert(Point2::new(x, 1e-9), true);
        }
        assert!((area(&cdt) - 4.0).abs() < 1e-12);
        assert_delaunay(&cdt);
    }

    #[test]
    fn a_segment_crossing_many_triangles_is_forced_in_whichever_way_it_runs() {
        for (from, to) in [((0.0, 0.5), (10.0, 0.5)), ((10.0, 9.5), (0.0, 0.5)), ((5.0, 0.0), (5.0, 10.0)), ((5.0, 10.0), (5.0, 0.0))] {
            let mut cdt = Cdt::new(Point2::new(0.0, 0.0), Point2::new(10.0, 10.0));
            let mut seed = 7u64;
            let a = cdt.insert(Point2::new(from.0, from.1), false).unwrap();
            let b = cdt.insert(Point2::new(to.0, to.1), false).unwrap();
            for _ in 0..300 {
                let p = Point2::new(10.0 * rand(&mut seed), 10.0 * rand(&mut seed));
                let _ = cdt.insert(p, false);
            }
            cdt.insert_segment(a, b).unwrap();
            let (t, i) = cdt.edge(a, b).or_else(|| cdt.edge(b, a)).expect("the segment is an edge");
            assert!(cdt.triangles[t as usize].c[i] > 0);
        }
    }
}
