//! Conservative trimmed-face bounds and deterministic BVH candidate generation.

use crate::geometry::{Interval, Point2, Point3, Surface};
use crate::topology::edge::Edge;
use crate::topology::face::Face;
use crate::topology::gmap::GMap;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey};
use slotmap::Key;

#[derive(Clone, Copy)]
struct Bounds {
    min: Point3,
    max: Point3,
}

impl Bounds {
    /// Includes all supplied points; invalid data forces the unbounded fallback.
    fn from_points(points: impl IntoIterator<Item = Point3>, padding: f64) -> Option<Self> {
        let mut points = points.into_iter();
        let first = points.next()?;
        let mut bounds = Self {
            min: first,
            max: first,
        };
        for point in std::iter::once(first).chain(points) {
            if !point.coords.iter().all(|v| v.is_finite()) {
                return None;
            }
            for axis in 0..3 {
                bounds.min[axis] = bounds.min[axis].min(point[axis]);
                bounds.max[axis] = bounds.max[axis].max(point[axis]);
            }
        }
        for axis in 0..3 {
            let roundoff =
                32.0 * f64::EPSILON * bounds.min[axis].abs().max(bounds.max[axis].abs()).max(1.0);
            bounds.min[axis] -= padding + roundoff;
            bounds.max[axis] += padding + roundoff;
        }
        Some(bounds)
    }

    fn overlaps(self, other: Self) -> bool {
        (0..3).all(|axis| self.min[axis] <= other.max[axis] && other.min[axis] <= self.max[axis])
    }

    fn union(self, other: Self) -> Self {
        Self {
            min: Point3::from(self.min.coords.inf(&other.min.coords)),
            max: Point3::from(self.max.coords.sup(&other.max.coords)),
        }
    }
}

/// Conservative model-space bounds of a bounded edge, from its control hull.
///
/// Positive weights make the hull a true bound; anything else keeps the edge
/// unbounded so no candidate pair is dropped on unverified data.
fn edge_bounds<P: Payload>(edge: Edge<'_, P>, padding: f64) -> Option<Bounds> {
    let curve = edge.curve()?.to_nurbs().ok()?;
    let mut points = Vec::with_capacity(curve.control_points().len());
    for point in curve.control_points().as_slice() {
        if !point.weight().is_finite() || point.weight() <= 0.0 {
            return None;
        }
        points.push(point.to_cartesian());
    }
    Bounds::from_points(points, padding)
}

/// Conservative parameter-space bounds of a face's trimmed region.
///
/// Callers converting an unbounded analytic surface to NURBS need this so the
/// patch they build actually covers the trimmed region.
pub(crate) fn face_uv_bounds<P: Payload>(face: &Face<'_, P>) -> Option<(Interval, Interval)> {
    let mut u = (f64::INFINITY, f64::NEG_INFINITY);
    let mut v = (f64::INFINITY, f64::NEG_INFINITY);
    for boundary in face.loops() {
        for edge in boundary.edges() {
            let pcurve = face.pcurve(edge.dart())?.to_nurbs().ok()?;
            for point in pcurve.control_points().as_slice() {
                if !point.weight().is_finite() || point.weight() <= 0.0 {
                    return None;
                }
                let uv = point.to_cartesian();
                if !uv.x.is_finite() || !uv.y.is_finite() {
                    return None;
                }
                u = (u.0.min(uv.x), u.1.max(uv.x));
                v = (v.0.min(uv.y), v.1.max(uv.y));
            }
        }
    }
    (u.0 <= u.1 && v.0 <= v.1).then(|| (Interval::new(u.0, u.1), Interval::new(v.0, v.1)))
}

/// Positive rational trim control hulls bound the whole trimmed parameter domain.
fn face_bounds<P: Payload>(face: Face<'_, P>, padding: f64) -> Option<Bounds> {
    let mut uv_points = Vec::<Point2>::new();
    for edge in face.outer_loop().edges() {
        let curve = face.pcurve(edge.dart())?.to_nurbs().ok()?;
        for point in curve.control_points().as_slice() {
            if !point.weight().is_finite() || point.weight() <= 0.0 {
                return None;
            }
            uv_points.push(point.to_cartesian());
        }
    }
    if uv_points.is_empty()
        || uv_points
            .iter()
            .any(|p| !p.coords.iter().all(|v| v.is_finite()))
    {
        return None;
    }
    let u_min = uv_points.iter().map(|p| p.x).fold(f64::INFINITY, f64::min);
    let u_max = uv_points
        .iter()
        .map(|p| p.x)
        .fold(f64::NEG_INFINITY, f64::max);
    let v_min = uv_points.iter().map(|p| p.y).fold(f64::INFINITY, f64::min);
    let v_max = uv_points
        .iter()
        .map(|p| p.y)
        .fold(f64::NEG_INFINITY, f64::max);

    // Analytic supports bound their own parameterization directly. This avoids
    // pretending that rational-conic NURBS parameters are the original angles.
    if !matches!(face.surface(), Surface::Nurbs(_)) {
        let bbox = face
            .surface()
            .bbox_over(Interval::new(u_min, u_max), Interval::new(v_min, v_max))?;
        return Bounds::from_points(bbox.corners()?, padding);
    }

    let Surface::Nurbs(surface) = face.surface() else {
        unreachable!("non-NURBS surfaces returned above")
    };
    let du = surface.domain_u();
    let dv = surface.domain_v();
    if uv_points
        .iter()
        .any(|uv| uv.x < du.start || uv.x > du.end || uv.y < dv.start || uv.y > dv.end)
    {
        return None;
    }
    let mut points = Vec::new();
    for patch in surface.bezier_spans().ok()? {
        if patch.domain_u().end < u_min
            || patch.domain_u().start > u_max
            || patch.domain_v().end < v_min
            || patch.domain_v().start > v_max
        {
            continue;
        }
        for point in patch.control_points().as_slice() {
            if !point.weight().is_finite() || point.weight() <= 0.0 {
                return None;
            }
            points.push(point.to_cartesian());
        }
    }
    Bounds::from_points(points, padding)
}

struct FaceBvh {
    bounds: Bounds,
    node: Node,
}

enum Node {
    Leaf(FaceKey),
    Branch(Box<FaceBvh>, Box<FaceBvh>),
}

impl FaceBvh {
    /// Median partition on the longest axis, with stable-key tie breaking.
    fn build(mut faces: Vec<(FaceKey, Bounds)>) -> Option<Self> {
        let bounds = faces.iter().map(|(_, b)| *b).reduce(Bounds::union)?;
        if faces.len() == 1 {
            return Some(Self {
                bounds,
                node: Node::Leaf(faces[0].0),
            });
        }
        let axis = (0..3)
            .max_by(|&a, &b| {
                (bounds.max[a] - bounds.min[a]).total_cmp(&(bounds.max[b] - bounds.min[b]))
            })
            .unwrap();
        faces.sort_by(|a, b| {
            (a.1.min[axis] + a.1.max[axis])
                .total_cmp(&(b.1.min[axis] + b.1.max[axis]))
                .then_with(|| a.0.data().as_ffi().cmp(&b.0.data().as_ffi()))
        });
        let right = faces.split_off(faces.len() / 2);
        Some(Self {
            bounds,
            node: Node::Branch(
                Box::new(Self::build(faces).unwrap()),
                Box::new(Self::build(right).unwrap()),
            ),
        })
    }

    /// Visits only overlapping nodes; an absent query bound visits every node.
    fn query(&self, bounds: Option<Bounds>, output: &mut Vec<FaceKey>) {
        if bounds.is_some_and(|bounds| !bounds.overlaps(self.bounds)) {
            return;
        }
        match &self.node {
            Node::Leaf(face) => output.push(*face),
            Node::Branch(left, right) => {
                left.query(bounds, output);
                right.query(bounds, output);
            }
        }
    }
}

pub(crate) struct CandidateSet {
    pub(crate) pairs: Vec<(FaceKey, FaceKey)>,
    pub(crate) pruned: usize,
}

/// Edge/face pairs that survive conservative bounds rejection.
pub(crate) struct EdgeFaceCandidateSet {
    pub(crate) pairs: Vec<(EdgeKey, FaceKey)>,
    pub(crate) pruned: usize,
}

/// Keeps every unbounded operand and every potentially overlapping bounded pair.
///
/// Narrow-phase curve/surface work dominates Boolean cost, and most edge/face
/// pairs of two solids cannot touch, so rejecting them here is what keeps that
/// cost proportional to the contacts that exist.
pub(crate) fn candidate_edge_face_pairs<P: Payload>(
    map: &GMap<P>,
    edges: &[EdgeKey],
    faces: &[FaceKey],
    padding: f64,
) -> EdgeFaceCandidateSet {
    let mut bounded = Vec::new();
    let mut unbounded = Vec::new();
    for &key in faces {
        match face_bounds(map.face_unchecked(key), padding) {
            Some(bounds) => bounded.push((key, bounds)),
            None => unbounded.push(key),
        }
    }
    let tree = FaceBvh::build(bounded);
    let mut pairs = Vec::new();
    for &edge in edges {
        let bounds = edge_bounds(map.edge_unchecked(edge), padding);
        let mut hits = unbounded.clone();
        if let Some(tree) = &tree {
            tree.query(bounds, &mut hits);
        }
        pairs.extend(hits.into_iter().map(|face| (edge, face)));
    }
    pairs.sort_by_key(|(edge, face)| (edge.data().as_ffi(), face.data().as_ffi()));
    pairs.dedup();
    EdgeFaceCandidateSet {
        pruned: edges.len() * faces.len() - pairs.len(),
        pairs,
    }
}

/// Keeps every unbounded face and every potentially overlapping bounded pair.
pub(crate) fn candidate_face_pairs<P: Payload>(
    map: &GMap<P>,
    first: &[FaceKey],
    second: &[FaceKey],
    padding: f64,
) -> CandidateSet {
    let mut bounded = Vec::new();
    let mut unbounded = Vec::new();
    for &key in second {
        match face_bounds(map.face_unchecked(key), padding) {
            Some(bounds) => bounded.push((key, bounds)),
            None => unbounded.push(key),
        }
    }
    let tree = FaceBvh::build(bounded);
    let mut pairs = Vec::new();
    for &key in first {
        let bounds = face_bounds(map.face_unchecked(key), padding);
        let mut hits = unbounded.clone();
        if let Some(tree) = &tree {
            tree.query(bounds, &mut hits);
        }
        pairs.extend(
            hits.into_iter()
                .filter(|&other| key != other)
                .map(|other| (key, other)),
        );
    }
    pairs.sort_by_key(|(a, b)| (a.data().as_ffi(), b.data().as_ffi()));
    pairs.dedup();
    let total =
        first.len() * second.len() - first.iter().filter(|key| second.contains(key)).count();
    CandidateSet {
        pruned: total - pairs.len(),
        pairs,
    }
}
