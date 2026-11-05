//! Face tessellation: a constrained Delaunay triangulation of the face's
//! parameter domain, against boundary points every neighbour shares,
//! refined until it follows the surface.
//!
//! - **The boundary is the edges' samples.** Each loop of the face's
//!   [`UnwrappedFaceDomain`](crate::topology::unwrapped_face_domain::UnwrappedFaceDomain) is walked edge by edge, and each edge contributes
//!   exactly the samples [`Discretisation`] cut it into, at their own 3D
//!   points and identities; the face only finds where on its pcurve each
//!   sample lies. Stretches of the domain's boundary that lie on no edge -- a
//!   seam the domain synthesised to cut a periodic support open, a row that
//!   collapses to a pole -- are sampled here, once for both sides of a seam,
//!   and a pole is one point however many times the domain visits it.
//! - **Inside, a constrained Delaunay triangulation** with exact predicates
//!   ([`super::cdt`]), in parameters scaled by the surface's average first
//!   fundamental form so that triangles are close to their 3D shape.
//! - **Refined in 3D.** A triangle whose corners' normals turn more than the
//!   angular step, or whose edges stand off the surface by more than the
//!   matching chord sag, is refined by bisecting interior edges; triangles with an angle
//!   under 20° in 3D are split at their circumcentres where that is possible.
//!   The boundary is never split: a face that needs a finer boundary gets it
//!   from the edge, so its neighbour gets it too.
//! - **Oriented by the loops**: the material lies to the left of every
//!   boundary segment as the loop runs it exactly when the face's normal is the
//!   surface's own.
//!
//! A face that cannot be meshed is refused with a [`TessellateError`] naming
//! why; a mesh that is not the face would be read as the face.

use std::collections::HashMap;
use std::f64::consts::TAU;

use nalgebra::Vector3;

use super::cdt::{Cdt, Refused};
use super::edges::{Discretisation, SharedPoint};
use super::{IndexedMesh, TessellateError, TessellateOpts, TessellateResult};
use crate::geometry::{LINEAR_TOLERANCE, Point2, Point3, Surface, SurfacePeriodicity};
use crate::model::Model;
use crate::topology::LoopKind;
use crate::topology::edge::Edge;
use crate::topology::face::Face;
use crate::topology::orientation::Orientation;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::FaceKey;
use crate::topology::unwrapped_face_domain::UnwrappedFaceDomainCurve;

/// How far past the angular step a triangle may stray before it is split: the
/// boundary is sampled at exactly the step, so the triangles it makes with
/// their neighbours sit on the step itself.
const ANGLE_SLACK: f64 = 1.5;
/// The most cells, in angular steps, a face's scaled plane is cut into before
/// the direction that bends less is stretched.
const MAX_CELLS: f64 = 40_000.0;
/// The most pieces a seam of the parameter domain is cut into.
const MAX_SEAM_SEGMENTS: usize = 4096;
/// The most points refinement may add to one face.
const MAX_REFINEMENT: usize = 400_000;
/// The smallest angle, in 3D, refinement tries to keep.
const MIN_ANGLE: f64 = 20.0 * std::f64::consts::PI / 180.0;

/// A face's mesh, with the identity of each boundary vertex.
#[derive(Debug, Clone, Default)]
pub(super) struct FaceMesh {
    pub(super) mesh: IndexedMesh,
    /// For each position, the shape point it is, where it is on the boundary.
    pub(super) shared: Vec<Option<SharedPoint>>,
}

/// Tessellates a trimmed face into an indexed triangle mesh.
///
/// The boundary is cut where [`Discretisation`] cuts the face's edges, which
/// depends only on the model, so this mesh meets the mesh of every face beside
/// it at the same points.
pub fn tessellate_face<P: Payload>(
    face: &Face<'_, P>,
    opts: TessellateOpts,
) -> TessellateResult<IndexedMesh> {
    let mut discretisation = Discretisation::default();
    mesh_face(face, &mut discretisation, opts).map(|mesh| mesh.mesh)
}

/// Tessellates the face stored at `key` in `g`.
pub fn tessellate_face_key<P: Payload>(
    g: &Model<P>,
    key: FaceKey,
    opts: TessellateOpts,
) -> TessellateResult<IndexedMesh> {
    let face = g.face(key).ok_or(TessellateError::UnreadableBoundary)?;
    tessellate_face(&face, opts)
}

/// One point of a face's boundary as the mesher reads it.
#[derive(Debug, Clone, Copy)]
struct BoundaryPoint {
    uv: Point2,
    position: Point3,
    id: SharedPoint,
}

/// Meshes `face`, its boundary cut as `discretisation` cuts its edges.
pub(super) fn mesh_face<P: Payload>(
    face: &Face<'_, P>,
    discretisation: &mut Discretisation,
    opts: TessellateOpts,
) -> TessellateResult<FaceMesh> {
    let model = face.model();
    let surface = face.surface().clone();
    let boundaryless = !face
        .loops()
        .iter()
        .any(|boundary| !matches!(boundary.kind(), LoopKind::Inner));
    // A face with a boundary states which way it points in how that boundary
    // winds as stored; the sense a shell reads it with says the same thing a
    // second time. A face with no loop at all has only that sense to go by.
    let stored;
    let view = if face.loops().is_empty() {
        face
    } else {
        stored = Face::new(model, face.key());
        &stored
    };
    let domain = discretisation
        .domain(view)
        .cloned()
        .ok_or(TessellateError::UnreadableBoundary)?;

    let mut builder = RingBuilder {
        surface: &surface,
        face: face.key(),
        opts,
        seams: HashMap::new(),
        interior: 0,
    };
    let mut rings = Vec::new();
    let loops = domain.loops();
    if boundaryless {
        let holes = loops
            .iter()
            .filter(|boundary| !boundary.is_empty())
            .map(|boundary| builder.ring(model, discretisation, boundary.curves()))
            .collect::<TessellateResult<Vec<_>>>()?;
        let (outer, holes) = builder.rectangle(holes)?;
        rings.push(outer);
        rings.extend(holes);
    } else {
        for boundary in loops.iter().filter(|boundary| !boundary.is_empty()) {
            rings.push(builder.ring(model, discretisation, boundary.curves())?);
        }
    }
    if rings.is_empty() || rings[0].len() < 3 {
        return Err(TessellateError::UnreadableBoundary);
    }

    // A face nothing encloses is wound by its sense, or against its holes where
    // it has them, since a Boolean flips such a face by rewinding its holes.
    let ccw_hint = boundaryless.then(|| match rings.get(1) {
        Some(hole) => ring_area(hole) < 0.0,
        None => view.sense() == Orientation::Same,
    });
    triangulate(&surface, &rings, ccw_hint, opts)
}

/// Builds a face's boundary rings.
struct RingBuilder<'s> {
    surface: &'s Surface,
    face: FaceKey,
    opts: TessellateOpts,
    /// Seams already sampled, by the shape points at their ends and the
    /// parameter axis they run along.
    seams: HashMap<(SharedPoint, SharedPoint, usize), Vec<(Point3, SharedPoint)>>,
    interior: u32,
}

impl RingBuilder<'_> {
    fn next_interior(&mut self) -> SharedPoint {
        self.interior += 1;
        SharedPoint::Interior(self.face, self.interior)
    }

    /// One loop: each pcurve's samples, joined across the gaps between them.
    fn ring<P: Payload>(
        &mut self,
        model: &Model<P>,
        discretisation: &mut Discretisation,
        curves: &[UnwrappedFaceDomainCurve],
    ) -> TessellateResult<Vec<BoundaryPoint>> {
        let mut pieces = Vec::with_capacity(curves.len());
        for curve in curves {
            pieces.push(self.curve_points(model, discretisation, curve)?);
        }
        let mut ring = Vec::new();
        for (index, curve) in curves.iter().enumerate() {
            let previous = pieces[(index + pieces.len() - 1) % pieces.len()]
                .last()
                .copied()
                .ok_or(TessellateError::UnreadableBoundary)?;
            let first = pieces[index][0];
            self.gap(previous, curve.corners(), first, &mut ring);
            let piece = &pieces[index];
            ring.extend_from_slice(&piece[..piece.len() - 1]);
        }
        dedup_ring(&mut ring);
        Ok(ring)
    }

    /// The samples of the edge under `curve`, placed on its pcurve, in the
    /// order the loop runs them, both ends included.
    fn curve_points<P: Payload>(
        &mut self,
        model: &Model<P>,
        discretisation: &mut Discretisation,
        curve: &UnwrappedFaceDomainCurve,
    ) -> TessellateResult<Vec<BoundaryPoint>> {
        let edge =
            Edge::from_dart(model, curve.dart()).ok_or(TessellateError::UnreadableBoundary)?;
        let samples = discretisation
            .edge(model, edge.key(), self.opts.curve)
            .clone();
        let span = curve.span();
        let interval = span.interval();
        let delta = interval.delta();
        if delta == 0.0 {
            return Err(TessellateError::DegenerateBoundary);
        }
        let periodic = match span.curve().periodicity() {
            crate::geometry::Periodicity::Periodic(period) => {
                (delta.abs() - period).abs() <= 1e-9 * period.max(1.0)
            }
            crate::geometry::Periodicity::None => false,
        };
        let mut placed = samples
            .fractions
            .iter()
            .enumerate()
            .map(|(index, _fraction)| {
                let stored = samples.span.interval();
                let native = stored.start.value() + _fraction * stored.delta();
                let mut along = (native - interval.start.value()) / delta;
                if periodic {
                    along = along.rem_euclid(1.0);
                    if along > 1.0 - 1e-12 {
                        along = 0.0;
                    }
                }
                (along, index)
            })
            .filter(|(along, _)| *along >= -1e-9 && *along <= 1.0 + 1e-9)
            .collect::<Vec<_>>();
        placed.sort_by(|a, b| a.0.total_cmp(&b.0));
        if samples.closed {
            // A closed edge's two ends are one sample; it is visited at the
            // start and again at the end.
            placed.dedup_by(|b, a| samples.ids[a.1] == samples.ids[b.1]);
            if let Some(&(_, first)) = placed.first() {
                placed.push((1.0, first));
            }
        }
        if placed.len() < 2 {
            return Err(TessellateError::UnreadableBoundary);
        }
        placed[0].0 = 0.0;
        let last = placed.len() - 1;
        placed[last].0 = 1.0;

        // Where on the pcurve each sample lies: near the same fraction of it,
        // at the point the surface puts closest to the sample.
        // The pcurve has to lie on the edge it belongs to: a sample the surface
        // cannot reach from it by more than a thousandth of the edge says the
        // two describe different curves, and no mesh of the face follows them
        // both.
        let reach = {
            let length: f64 = samples
                .points
                .windows(2)
                .map(|pair| (pair[1] - pair[0]).norm())
                .sum();
            (1e-3 * length).max(LINEAR_TOLERANCE)
        };
        let mut points = Vec::with_capacity(placed.len());
        let mut floor = 0.0_f64;
        for (k, &(along, index)) in placed.iter().enumerate() {
            let target = samples.points[index];
            let fraction = if k == 0 {
                0.0
            } else if k == last {
                1.0
            } else {
                let low = floor.max(placed[k - 1].0);
                let high = placed[k + 1].0.min(1.0);
                let distance = |f: f64| {
                    let uv = curve.point_at(f);
                    (self.surface.point_at(uv.x, uv.y) - target).norm()
                };
                let guess = along.clamp(low, high);
                if distance(guess) <= LINEAR_TOLERANCE {
                    guess
                } else {
                    nearest_on(distance, low, high)
                }
            };
            floor = fraction;
            let uv = curve.point_at(fraction);
            if (self.surface.point_at(uv.x, uv.y) - target).norm() > reach {
                return Err(TessellateError::UntriangulableBoundary);
            }
            points.push(BoundaryPoint {
                uv,
                position: target,
                id: samples.ids[index],
            });
        }
        Ok(points)
    }

    /// The stretch of boundary from `from` to `to` through `corners` that lies
    /// on no edge: a seam, or a row collapsing to a pole.
    fn gap(
        &mut self,
        from: BoundaryPoint,
        corners: &[Point2],
        to: BoundaryPoint,
        ring: &mut Vec<BoundaryPoint>,
    ) {
        if corners.is_empty() && (from.uv - to.uv).norm() <= 1e-12 {
            return;
        }
        let mut path = Vec::with_capacity(corners.len() + 2);
        path.push((from.uv, Some((from.position, from.id))));
        path.extend(corners.iter().map(|uv| (*uv, None)));
        path.push((to.uv, Some((to.position, to.id))));
        let degenerate = path
            .windows(2)
            .map(|pair| self.is_degenerate(pair[0].0, pair[1].0))
            .collect::<Vec<_>>();
        // Points joined by a collapsed row are one point of the surface: they
        // take the name of whichever of them has one, or a fresh one between
        // them.
        let mut k = 0;
        while k < path.len() {
            let mut end = k;
            while end + 1 < path.len() && degenerate[end] {
                end += 1;
            }
            let named = path[k..=end].iter().find_map(|entry| entry.1);
            let shared = named.unwrap_or_else(|| {
                let uv = path[k].0;
                (self.surface.point_at(uv.x, uv.y), self.next_interior())
            });
            for entry in &mut path[k..=end] {
                entry.1.get_or_insert(shared);
            }
            k = end + 1;
        }
        for k in 0..path.len() - 1 {
            let (a_uv, a) = (path[k].0, path[k].1.expect("every point is named"));
            let (b_uv, b) = (path[k + 1].0, path[k + 1].1.expect("every point is named"));
            if k > 0 {
                ring.push(BoundaryPoint {
                    uv: a_uv,
                    position: a.0,
                    id: a.1,
                });
            }
            if degenerate[k] {
                // A pole: many parameter points, one surface point.
                let count = self.row_count(a_uv, b_uv);
                for step in 1..count {
                    let t = step as f64 / count as f64;
                    ring.push(BoundaryPoint {
                        uv: a_uv + (b_uv - a_uv) * t,
                        position: a.0,
                        id: a.1,
                    });
                }
            } else {
                self.seam(a_uv, a, b_uv, b, ring);
            }
        }
    }

    /// Samples the straight seam from `a` to `b`, sharing its points with the
    /// seam's other side.
    fn seam(
        &mut self,
        a_uv: Point2,
        a: (Point3, SharedPoint),
        b_uv: Point2,
        b: (Point3, SharedPoint),
        ring: &mut Vec<BoundaryPoint>,
    ) {
        let delta = b_uv - a_uv;
        let axis = if delta.x.abs() >= delta.y.abs() { 0 } else { 1 };
        // Both sides of a seam are keyed and sampled from the end that is
        // lower along its axis, whichever way the loop runs them; the shared
        // points are read back in the loop's own direction.
        let forward = delta[axis] > 0.0;
        let (lo, hi) = if forward { (a, b) } else { (b, a) };
        let (lo_uv, hi_uv) = if forward { (a_uv, b_uv) } else { (b_uv, a_uv) };
        let key = (lo.1, hi.1, axis);
        if !self.seams.contains_key(&key) {
            let count = self.seam_count(lo_uv, hi_uv);
            let mut points = Vec::with_capacity(count.saturating_sub(1));
            for step in 1..count {
                let uv = lo_uv + (hi_uv - lo_uv) * (step as f64 / count as f64);
                points.push((self.surface.point_at(uv.x, uv.y), self.next_interior()));
            }
            self.seams.insert(key, points);
        }
        let points = &self.seams[&key];
        let count = points.len() + 1;
        for step in 1..count {
            let k = if forward { step - 1 } else { count - 1 - step };
            let t = step as f64 / count as f64;
            let (position, id) = points[k];
            ring.push(BoundaryPoint {
                uv: a_uv + (b_uv - a_uv) * t,
                position,
                id,
            });
        }
    }

    fn is_degenerate(&self, a: Point2, b: Point2) -> bool {
        let pa = self.surface.point_at(a.x, a.y);
        let pb = self.surface.point_at(b.x, b.y);
        let m = nalgebra::center(&a, &b);
        let pm = self.surface.point_at(m.x, m.y);
        let scale = (pa.coords.norm() + 1.0) * 1e-12 + LINEAR_TOLERANCE;
        (pa - pb).norm() <= scale && (pa - pm).norm() <= scale
    }

    /// How many pieces a collapsed row is cut into in parameters: as many as a
    /// whole turn of the curve sampling for the part of a period it spans.
    fn row_count(&self, a: Point2, b: Point2) -> usize {
        let per_turn = self.opts.curve.segments.max(4) as f64;
        let periods = periods_of(self.surface);
        let mut count = 1.0_f64;
        for axis in 0..2 {
            let run = (b[axis] - a[axis]).abs();
            if let Some(period) = periods[axis] {
                count = count.max((run / period * per_turn).ceil());
            }
        }
        (count as usize).clamp(1, MAX_SEAM_SEGMENTS)
    }

    /// How many pieces a seam is cut into: by how far its tangent turns in 3D.
    fn seam_count(&self, a: Point2, b: Point2) -> usize {
        const PROBES: usize = 64;
        let step = TAU / self.opts.curve.segments.max(1) as f64;
        let at = |t: f64| {
            let uv = a + (b - a) * t;
            self.surface.point_at(uv.x, uv.y)
        };
        let mut turning = 0.0;
        let mut previous: Option<Vector3<f64>> = None;
        for k in 0..PROBES {
            let (p, q) = (
                at(k as f64 / PROBES as f64),
                at((k + 1) as f64 / PROBES as f64),
            );
            let direction = q - p;
            let norm = direction.norm();
            if norm <= 0.0 {
                continue;
            }
            let direction = direction / norm;
            if let Some(previous) = previous {
                turning += previous.dot(&direction).clamp(-1.0, 1.0).acos();
            }
            previous = Some(direction);
        }
        ((turning / step).ceil() as usize).clamp(1, MAX_SEAM_SEGMENTS)
    }

    /// The domain of a face no loop encloses -- a whole sphere, a whole torus,
    /// either with holes -- as a rectangle of its support's parameters, cut
    /// open where no hole lies, with its holes moved inside it.
    fn rectangle(
        &mut self,
        mut holes: Vec<Vec<BoundaryPoint>>,
    ) -> TessellateResult<(Vec<BoundaryPoint>, Vec<Vec<BoundaryPoint>>)> {
        let (u, v) = self.surface.domain();
        if !u.is_finite() || !v.is_finite() {
            return Err(TessellateError::UnboundedDomain);
        }
        let periods = periods_of(self.surface);
        let mut low = [u.start.value(), v.start.value()];
        let mut high = [u.end.value(), v.end.value()];
        for axis in 0..2 {
            let Some(period) = periods[axis] else {
                continue;
            };
            let cut = clear_cut(&holes, axis, period, low[axis]);
            low[axis] = cut;
            high[axis] = cut + period;
            for hole in &mut holes {
                let min = hole
                    .iter()
                    .map(|p| p.uv[axis])
                    .fold(f64::INFINITY, f64::min);
                let turns = ((cut - min) / period).ceil();
                for point in hole.iter_mut() {
                    point.uv[axis] += turns * period;
                }
            }
        }
        let corner = |x: usize, y: usize| Point2::new([low, high][x][0], [low, high][y][1]);
        let corners = [corner(0, 0), corner(1, 0), corner(1, 1), corner(0, 1)];
        let mut named: Vec<(Point3, SharedPoint)> = Vec::with_capacity(4);
        for (k, uv) in corners.iter().enumerate() {
            let position = self.surface.point_at(uv.x, uv.y);
            let existing = named
                .iter()
                .take(k)
                .find(|(p, _)| (*p - position).norm() <= LINEAR_TOLERANCE)
                .copied();
            named.push(existing.unwrap_or_else(|| (position, self.next_interior())));
        }
        let mut ring = Vec::new();
        for k in 0..4 {
            let (a_uv, b_uv) = (corners[k], corners[(k + 1) % 4]);
            let (a, b) = (named[k], named[(k + 1) % 4]);
            ring.push(BoundaryPoint {
                uv: a_uv,
                position: a.0,
                id: a.1,
            });
            if self.is_degenerate(a_uv, b_uv) {
                let count = self.row_count(a_uv, b_uv);
                for step in 1..count {
                    ring.push(BoundaryPoint {
                        uv: a_uv + (b_uv - a_uv) * (step as f64 / count as f64),
                        position: a.0,
                        id: a.1,
                    });
                }
            } else {
                // Opposite sides of the rectangle are one seam when the support
                // closes across them; the corners alone do not tell the two
                // seams of a torus apart, so the axis does.
                self.seam(a_uv, a, b_uv, b, &mut ring);
            }
        }
        dedup_ring(&mut ring);
        Ok((ring, holes))
    }
}

/// Where to cut a periodic axis open so that no hole straddles the cut: the
/// middle of the widest gap the holes leave round it.
fn clear_cut(holes: &[Vec<BoundaryPoint>], axis: usize, period: f64, default: f64) -> f64 {
    let mut spans = holes
        .iter()
        .filter(|hole| !hole.is_empty())
        .map(|hole| {
            let min = hole
                .iter()
                .map(|p| p.uv[axis])
                .fold(f64::INFINITY, f64::min);
            let max = hole
                .iter()
                .map(|p| p.uv[axis])
                .fold(f64::NEG_INFINITY, f64::max);
            let start = (min - default).rem_euclid(period) + default;
            (start, start + (max - min))
        })
        .collect::<Vec<_>>();
    if spans.is_empty() {
        return default;
    }
    spans.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut best = (0.0, default);
    let mut reach = spans[0].1;
    for k in 1..=spans.len() {
        let next = if k < spans.len() {
            spans[k].0
        } else {
            spans[0].0 + period
        };
        if next - reach > best.0 {
            best = (next - reach, 0.5 * (next + reach));
        }
        if k < spans.len() {
            reach = reach.max(spans[k].1);
        }
    }
    best.1
}

fn ring_area(ring: &[BoundaryPoint]) -> f64 {
    0.5 * (0..ring.len())
        .map(|k| {
            let (a, b) = (ring[k].uv, ring[(k + 1) % ring.len()].uv);
            a.x * b.y - b.x * a.y
        })
        .sum::<f64>()
}

/// Drops a point that repeats its neighbour: the same shape point at the
/// same place in parameters, to rounding. Two pcurves meeting at a corner
/// each evaluate it, and the two evaluations need not agree to the last bit.
/// Points of a collapsed row share a shape point but not a place, and stay.
fn dedup_ring(ring: &mut Vec<BoundaryPoint>) {
    let close = |a: &BoundaryPoint, b: &BoundaryPoint| {
        let scale = a.uv.x.abs().max(a.uv.y.abs()).max(1.0);
        a.id == b.id && (a.uv - b.uv).norm() <= 1e-12 * scale
    };
    ring.dedup_by(|b, a| close(a, b));
    while ring.len() > 1 && close(&ring[0], &ring[ring.len() - 1]) {
        ring.pop();
    }
}

/// Reads a support's periods as a parameter-indexed pair.
fn periods_of(surface: &Surface) -> [Option<f64>; 2] {
    match surface.periodicity() {
        SurfacePeriodicity::None => [None, None],
        SurfacePeriodicity::UPeriodic(u) => [Some(u), None],
        SurfacePeriodicity::VPeriodic(v) => [None, Some(v)],
        SurfacePeriodicity::UVPeriodic(u, v) => [Some(u), Some(v)],
    }
}

/// Where `distance` is least on `[low, high]`, by golden-section search.
fn nearest_on(distance: impl Fn(f64) -> f64, low: f64, high: f64) -> f64 {
    const RATIO: f64 = 0.618_033_988_749_894_9;
    const STEPS: usize = 48;
    let (mut low, mut high) = (low.min(high), low.max(high));
    let mut left = high - RATIO * (high - low);
    let mut right = low + RATIO * (high - low);
    let (mut at_left, mut at_right) = (distance(left), distance(right));
    for _ in 0..STEPS {
        if at_left <= at_right {
            high = right;
            right = left;
            at_right = at_left;
            left = high - RATIO * (high - low);
            at_left = distance(left);
        } else {
            low = left;
            left = right;
            at_left = at_right;
            right = low + RATIO * (high - low);
            at_right = distance(right);
        }
    }
    0.5 * (low + high)
}

/// A vertex of the triangulation, as the surface sees it.
#[derive(Debug, Clone, Copy)]
struct Lifted {
    uv: Point2,
    position: Point3,
    /// The surface's unit normal, where it has one.
    normal: Option<Vector3<f64>>,
    shared: Option<SharedPoint>,
}

/// The parameter scaling the triangulation is built in, and its inverse.
///
/// Where the surface bends, a unit of the scaled plane is one angular step of
/// the normal along the parameter's line, and the same in 3D length in both
/// directions: triangles that are well shaped there are cut as finely as the
/// surface bends and no finer, and a direction in which the surface does not
/// bend is cut no finer than its length asks, whatever that length. Where the
/// surface is flat, a unit is a unit of length along the parameter's line
/// (the average first fundamental form).
#[derive(Debug, Clone, Copy)]
struct Scaling {
    origin: Point2,
    scale: Vector3<f64>,
    /// The scaled plane's width and height, in angular steps; `None` where
    /// the surface is flat.
    cells: Option<[f64; 2]>,
    /// Whether the cell count was capped, which stretches triangles in 3D.
    capped: bool,
}

impl Scaling {
    fn of(surface: &Surface, min: Point2, max: Point2, step: f64) -> Self {
        const GRID: usize = 9;
        let span = max - min;
        // The widest the normal turns, and the longest the tangent is, along
        // each parameter's line.
        let (mut turn, mut speed) = ([0.0_f64; 2], [0.0_f64; 2]);
        for i in 0..GRID {
            for j in 0..GRID {
                let uv = Point2::new(
                    min.x + span.x * (i as f64 + 0.5) / GRID as f64,
                    min.y + span.y * (j as f64 + 0.5) / GRID as f64,
                );
                let (_, du, dv) = surface.point_and_derivative_at(uv.x, uv.y);
                for (axis, tangent) in [(0usize, du), (1usize, dv)] {
                    let length = tangent.norm();
                    if length.is_finite() {
                        speed[axis] = speed[axis].max(length);
                    }
                    if let Some(rate) = normal_turn_per_parameter(surface, uv, axis, span[axis]) {
                        turn[axis] = turn[axis].max(rate);
                    }
                }
            }
        }
        let speed = speed.map(|s| if s > 0.0 { s } else { 1.0 });
        let bend = (turn[0] / speed[0]).max(turn[1] / speed[1]);
        if !bend.is_finite() || bend <= 1e-9 {
            return Self {
                origin: min,
                scale: Vector3::new(speed[0], speed[1], 1.0),
                cells: None,
                capped: false,
            };
        }
        let mut scale = [turn[0].max(bend * speed[0]), turn[1].max(bend * speed[1])];
        let mut cells = [span.x * scale[0] / step, span.y * scale[1] / step];
        let mut capped = false;
        if cells[0] * cells[1] > MAX_CELLS {
            // Keep the direction that bends more, and stretch the other.
            let low = if turn[0] / speed[0] >= turn[1] / speed[1] {
                1
            } else {
                0
            };
            let high = 1 - low;
            let allowed = (MAX_CELLS / cells[high].max(1.0)).max(1.0);
            scale[low] *= allowed / cells[low];
            cells[low] = allowed;
            capped = true;
        }
        Self {
            origin: min,
            scale: Vector3::new(scale[0], scale[1], 1.0),
            cells: Some(cells),
            capped,
        }
    }

    fn forward(&self, uv: Point2) -> Point2 {
        Point2::new(
            (uv.x - self.origin.x) * self.scale.x,
            (uv.y - self.origin.y) * self.scale.y,
        )
    }

    fn back(&self, xy: Point2) -> Point2 {
        Point2::new(
            xy.x / self.scale.x + self.origin.x,
            xy.y / self.scale.y + self.origin.y,
        )
    }
}

/// The rate, per unit of the parameter on `axis`, at which `surface`'s normal
/// turns at `uv`, or `None` where it has no normal.
fn normal_turn_per_parameter(surface: &Surface, uv: Point2, axis: usize, span: f64) -> Option<f64> {
    let here = surface_normal(surface, uv)?;
    let eps = 1e-5
        * if span.is_finite() && span > 0.0 {
            span
        } else {
            1.0
        };
    let at = |sign: f64| {
        let mut p = uv;
        p[axis] += sign * eps;
        surface_normal(surface, p)
    };
    let turn = match (at(1.0), at(-1.0)) {
        (Some(a), Some(b)) => a.dot(&b).clamp(-1.0, 1.0).acos() / (2.0 * eps),
        (Some(a), None) | (None, Some(a)) => a.dot(&here).clamp(-1.0, 1.0).acos() / eps,
        (None, None) => return None,
    };
    turn.is_finite().then_some(turn)
}

fn surface_normal(surface: &Surface, uv: Point2) -> Option<Vector3<f64>> {
    if surface.is_degenerate_at(uv.x, uv.y) {
        return None;
    }
    let (_, du, dv) = surface.point_and_derivative_at(uv.x, uv.y);
    let normal = du.cross(&dv);
    let norm = normal.norm();
    (norm.is_finite() && norm > 1e-300).then(|| normal / norm)
}

/// Triangulates the rings and refines the triangulation until it follows the
/// surface.
fn triangulate(
    surface: &Surface,
    rings: &[Vec<BoundaryPoint>],
    ccw_hint: Option<bool>,
    opts: TessellateOpts,
) -> TessellateResult<FaceMesh> {
    let (mut min, mut max) = (
        Point2::new(f64::INFINITY, f64::INFINITY),
        Point2::new(f64::NEG_INFINITY, f64::NEG_INFINITY),
    );
    let mut size3 = (
        Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
        Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
    );
    for point in rings.iter().flatten() {
        min = min.inf(&point.uv);
        max = max.sup(&point.uv);
        size3.0 = size3.0.inf(&point.position);
        size3.1 = size3.1.sup(&point.position);
    }
    if !(min.x.is_finite() && max.x.is_finite() && min.y.is_finite() && max.y.is_finite()) {
        return Err(TessellateError::UnreadableBoundary);
    }
    let extent = (size3.1 - size3.0).norm();
    if (max - min).norm() <= 0.0 || !extent.is_finite() || extent <= 0.0 {
        return Err(TessellateError::DegenerateBoundary);
    }

    let step0 = (TAU / opts.curve.segments.max(1) as f64).max(1e-3);
    let scaling = Scaling::of(surface, min, max, step0);

    let mut cdt = Cdt::new(scaling.forward(min), scaling.forward(max));
    let mut lifted: Vec<Lifted> = (0..Cdt::FIRST)
        .map(|_| Lifted {
            uv: Point2::origin(),
            position: Point3::origin(),
            normal: None,
            shared: None,
        })
        .collect();
    let mut ring_vertices = Vec::with_capacity(rings.len());
    for ring in rings {
        let mut vertices = Vec::with_capacity(ring.len());
        for point in ring {
            let vertex = match cdt.insert(scaling.forward(point.uv), false) {
                Ok(vertex) => {
                    lifted.push(Lifted {
                        uv: point.uv,
                        position: point.position,
                        normal: surface_normal(surface, point.uv),
                        shared: Some(point.id),
                    });
                    vertex
                }
                Err(Refused::Duplicate(vertex)) => vertex,
                Err(_) => return Err(TessellateError::UntriangulableBoundary),
            };
            vertices.push(vertex);
        }
        ring_vertices.push(vertices);
    }
    for vertices in &ring_vertices {
        for k in 0..vertices.len() {
            let (a, b) = (vertices[k], vertices[(k + 1) % vertices.len()]);
            cdt.insert_segment(a, b)
                .map_err(|_| TessellateError::UntriangulableBoundary)?;
        }
    }
    cdt.classify();

    // Which way the face looks: the material is on the left of each boundary
    // segment as its loop runs it, when the face's normal is the surface's.
    let ccw = match ccw_hint {
        Some(ccw) => ccw,
        None => orientation(&cdt, &ring_vertices).ok_or(TessellateError::UntriangulableBoundary)?,
    };
    if cdt.inside().next().is_none() {
        return Err(TessellateError::DegenerateBoundary);
    }

    if let Some(cells) = scaling.cells {
        seed_lattice(surface, &mut cdt, &mut lifted, scaling, cells, (min, max));
    }
    refine(surface, &mut cdt, &mut lifted, scaling, extent, opts);

    // Assemble, dropping only triangles that collapse onto one shape point: a
    // sliver with three distinct corners still pairs its edges with its
    // neighbours', and dropping it would open the surface.
    let mut index_of = vec![u32::MAX; cdt.vertex_count()];
    let mut shared_indices = HashMap::new();
    let mut mesh = FaceMesh::default();
    let mut triangles = Vec::new();
    for (_, [a, b, c]) in cdt.inside() {
        let corners = [a, b, c].map(|v| lifted[v as usize]);
        let same = |x: &Lifted, y: &Lifted| match (x.shared, y.shared) {
            (Some(p), Some(q)) => p == q,
            _ => x.position == y.position,
        };
        if same(&corners[0], &corners[1])
            || same(&corners[1], &corners[2])
            || same(&corners[2], &corners[0])
        {
            continue;
        }
        triangles.push([a, b, c]);
    }
    for triangle in &triangles {
        for &v in triangle {
            if index_of[v as usize] == u32::MAX {
                let point = lifted[v as usize];
                let index = mesh.mesh.positions.len() as u32;
                if let Some(shared) = point.shared {
                    let existing = *shared_indices.entry(shared).or_insert(index);
                    index_of[v as usize] = existing;
                    if existing != index {
                        continue;
                    }
                }
                index_of[v as usize] = index;
                mesh.mesh.positions.push(point.position);
                let normal = point.normal.unwrap_or_else(Vector3::zeros);
                let normal = if ccw { normal } else { -normal };
                mesh.mesh
                    .normals
                    .push(nalgebra::Unit::new_unchecked(normal));
                mesh.shared.push(point.shared);
            }
        }
    }
    // A pole has no normal of its own; give it the mean of its triangles'.
    let mut accumulated = vec![Vector3::zeros(); mesh.mesh.positions.len()];
    for triangle in &triangles {
        let [a, b, c] = triangle.map(|v| index_of[v as usize] as usize);
        let (pa, pb, pc) = (
            mesh.mesh.positions[a],
            mesh.mesh.positions[b],
            mesh.mesh.positions[c],
        );
        let n = (pb - pa).cross(&(pc - pa));
        let n = if ccw { n } else { -n };
        for k in [a, b, c] {
            accumulated[k] += n;
        }
        if ccw {
            mesh.mesh
                .indices
                .extend_from_slice(&[a as u32, b as u32, c as u32]);
        } else {
            mesh.mesh
                .indices
                .extend_from_slice(&[a as u32, c as u32, b as u32]);
        }
    }
    for (k, normal) in mesh.mesh.normals.iter_mut().enumerate() {
        if normal.norm() == 0.0 {
            let n = accumulated[k];
            let norm = n.norm();
            *normal =
                nalgebra::Unit::new_unchecked(if norm > 0.0 { n / norm } else { Vector3::z() });
        }
    }
    if mesh.mesh.indices.is_empty() {
        return Err(TessellateError::DegenerateBoundary);
    }
    Ok(mesh)
}

/// Whether the material lies to the left of the boundary as its loops run.
fn orientation(cdt: &Cdt, rings: &[Vec<u32>]) -> Option<bool> {
    let mut left = 0usize;
    let mut right = 0usize;
    let inside = cdt.inside().collect::<Vec<_>>();
    let mut edges: HashMap<(u32, u32), ()> = HashMap::new();
    for (_, [a, b, c]) in &inside {
        edges.insert((*a, *b), ());
        edges.insert((*b, *c), ());
        edges.insert((*c, *a), ());
    }
    for vertices in rings {
        for k in 0..vertices.len() {
            let (a, b) = (vertices[k], vertices[(k + 1) % vertices.len()]);
            if a == b {
                continue;
            }
            // An inside triangle holding a→b counter-clockwise lies to its left.
            if edges.contains_key(&(a, b)) {
                left += 1;
            } else if edges.contains_key(&(b, a)) {
                right += 1;
            }
        }
    }
    match (left, right) {
        (0, 0) => None,
        _ => Some(left >= right),
    }
}

/// Fills the face with a lattice of points one angular step apart in the
/// scaled plane, so that the triangulation starts out following the surface
/// and refinement has little left to do. A point that falls outside the face,
/// or that would crowd a boundary segment, is left out.
fn seed_lattice(
    surface: &Surface,
    cdt: &mut Cdt,
    lifted: &mut Vec<Lifted>,
    scaling: Scaling,
    cells: [f64; 2],
    (min, max): (Point2, Point2),
) {
    let count = cells.map(|c| (c - 1e-9).ceil().max(1.0) as usize);
    let span = max - min;
    for j in 0..count[1] {
        for i in 0..count[0] {
            let (a, b) = if j % 2 == 0 {
                (i, j)
            } else {
                (count[0] - 1 - i, j)
            };
            let uv = Point2::new(
                min.x + span.x * (a as f64 + 0.5) / count[0] as f64,
                min.y + span.y * (b as f64 + 0.5) / count[1] as f64,
            );
            let at = scaling.forward(uv);
            if cdt.encroaches(at) {
                continue;
            }
            if cdt.insert(at, true).is_ok() {
                lifted.push(Lifted {
                    uv,
                    position: surface.point_at(uv.x, uv.y),
                    normal: surface_normal(surface, uv),
                    shared: None,
                });
            }
        }
    }
}

/// Splits the edges that stray from the surface until every one follows it to
/// the angular step, and improves the worst-shaped triangles on the way.
///
/// An edge strays when the normals at its ends turn more than the step, or
/// when the surface at its midpoint stands off its chord by more than the
/// matching sag. Only such an edge is split, at its midpoint: a cylinder's
/// triangles are cut across its bend and never along its ruling, however long
/// the ruling, which a split of whole triangles at their centres cannot do.
fn refine(
    surface: &Surface,
    cdt: &mut Cdt,
    lifted: &mut Vec<Lifted>,
    scaling: Scaling,
    extent: f64,
    opts: TessellateOpts,
) {
    let step = (TAU / opts.curve.segments.max(1) as f64).max(1e-3) * ANGLE_SLACK;
    let sag_ratio = step / 8.0;
    let tiny = extent * 1e-6;
    let lift = |uv: Point2| Lifted {
        uv,
        position: surface.point_at(uv.x, uv.y),
        normal: surface_normal(surface, uv),
        shared: None,
    };

    // Whether the edge between two vertices strays from the surface.
    let strays = |lifted: &[Lifted], a: u32, b: u32| -> bool {
        let (a, b) = (lifted[a as usize], lifted[b as usize]);
        let length = (b.position - a.position).norm();
        if length <= tiny {
            return false;
        }
        if let (Some(na), Some(nb)) = (a.normal, b.normal)
            && na.dot(&nb).clamp(-1.0, 1.0).acos() > step
        {
            return true;
        }
        let mid_uv = nalgebra::center(&a.uv, &b.uv);
        let mid = surface.point_at(mid_uv.x, mid_uv.y);
        let chord = nalgebra::center(&a.position, &b.position);
        (mid - chord).norm() > length * sag_ratio
    };

    // One worklist for both goals, so that a point added to improve a shape
    // is itself checked against the surface. Following the surface comes
    // first and has no budget but the face's; improving shapes is bounded by a
    // multiple of the boundary.
    let mut queue: Vec<u32> = cdt.inside().map(|(t, _)| t).collect();
    let mut added = 0usize;
    let mut polished = 0usize;
    let polish_budget = 2 * lifted.len();
    while let Some(t) = queue.pop() {
        if added >= MAX_REFINEMENT {
            break;
        }
        let Some(v) = cdt.vertices_of(t) else {
            continue;
        };
        let mut split = false;
        for i in 0..3 {
            if cdt.is_boundary(t, i) {
                continue;
            }
            let (a, b) = (v[(i + 1) % 3], v[(i + 2) % 3]);
            if !strays(lifted, a, b) {
                continue;
            }
            let middle = nalgebra::center(&cdt.point(a), &cdt.point(b));
            if let Ok(vertex) = cdt.insert(middle, true) {
                debug_assert_eq!(vertex as usize, lifted.len());
                lifted.push(lift(scaling.back(middle)));
                added += 1;
                queue.extend(cdt.around(vertex));
                split = true;
                break;
            }
        }
        if split || scaling.capped || polished >= polish_budget {
            continue;
        }
        // Shape: a triangle with a small angle in 3D is split at its
        // circumcentre, where that falls inside and clear of the boundary.
        let corners = v.map(|v| lifted[v as usize].position);
        if (0..3).any(|i| (corners[i] - corners[(i + 1) % 3]).norm() <= tiny) {
            continue;
        }
        if min_angle(corners).is_none_or(|angle| angle >= MIN_ANGLE) {
            continue;
        }
        let Some(candidate) = circumcentre(v.map(|v| cdt.point(v))) else {
            continue;
        };
        if cdt.encroaches(candidate) {
            continue;
        }
        if let Ok(vertex) = cdt.insert(candidate, true) {
            lifted.push(lift(scaling.back(candidate)));
            added += 1;
            polished += 1;
            queue.extend(cdt.around(vertex));
        }
    }
}

fn circumcentre(points: [Point2; 3]) -> Option<Point2> {
    let [a, b, c] = points;
    let (bx, by) = (b.x - a.x, b.y - a.y);
    let (cx, cy) = (c.x - a.x, c.y - a.y);
    let d = 2.0 * (bx * cy - by * cx);
    if d == 0.0 || !d.is_finite() {
        return None;
    }
    let b2 = bx * bx + by * by;
    let c2 = cx * cx + cy * cy;
    let x = (cy * b2 - by * c2) / d;
    let y = (bx * c2 - cx * b2) / d;
    let p = Point2::new(a.x + x, a.y + y);
    (p.x.is_finite() && p.y.is_finite()).then_some(p)
}

/// The smallest angle of a triangle in 3D, or `None` where it has no area.
fn min_angle(corners: [Point3; 3]) -> Option<f64> {
    let mut smallest = f64::INFINITY;
    for i in 0..3 {
        let (p, q, r) = (corners[i], corners[(i + 1) % 3], corners[(i + 2) % 3]);
        let (u, v) = (q - p, r - p);
        let (nu, nv) = (u.norm(), v.norm());
        if nu == 0.0 || nv == 0.0 {
            return None;
        }
        smallest = smallest.min((u.dot(&v) / (nu * nv)).clamp(-1.0, 1.0).acos());
    }
    Some(smallest)
}
