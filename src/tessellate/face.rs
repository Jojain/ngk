//! Trimmed face tessellation.
//!
//! Real plan: sample the face's pcurves into a UV polygon-with-holes, run a
//! constrained Delaunay triangulation, lift back to 3D via
//! `surface.point_at(u, v)`. That CDT is not in the tree yet — we currently
//! ship a small set of per-surface shortcuts that handle the shapes the
//! exploration scripts actually build:
//!
//! - **Cylinder**: uniform UV grid over the pcurve UV bbox. Triangle winding
//!   matches the pcurve loop's signed area in UV.
//! - **Plane, no inner loops**: triangle fan from the UV centroid of the
//!   sampled outer pcurve, lifted to 3D.
//! - **Plane, exactly one inner loop with matching sample count**: strip
//!   between the outer and inner sampled rings (annulus shape).
//! - Anything else falls back to a single quad over the pcurve UV bbox and is
//!   tagged `// TODO: real CDT`.

use nalgebra::UnitVector3;

use super::{IndexedMesh, TessellateOpts, surface::tessellate_surface_patch};
use crate::geometry::Point2;
use crate::geometry::Surface;
use crate::topology::attributes::FaceAttr;
use crate::topology::gmap::{Dart, GMap};
use crate::topology::payload::Payload;
use crate::topology::profile::Profile;
use crate::topology::shape_keys::FaceKey;

const EPS: f64 = 1.0e-9;

/// Tessellate the face stored at `key` in `g`. Returns `None` if the face is
/// missing or has no pcurves to delineate it.
pub fn tessellate_face<P: Payload>(
    g: &GMap<P>,
    key: FaceKey,
    opts: TessellateOpts,
) -> Option<IndexedMesh> {
    let attr = g.face(key)?;
    let outer_uv = sample_loop_pcurve(g, attr, attr.outer_loop, opts)?;
    if outer_uv.len() < 3 {
        return None;
    }

    let inner_uv: Vec<Vec<Point2>> = attr
        .inner_loops
        .iter()
        .filter_map(|d| sample_loop_pcurve(g, attr, *d, opts))
        .collect();

    let ccw = signed_area(&outer_uv) > 0.0;

    Some(match &attr.surface {
        Surface::Cylinder(_) | Surface::Ruled(_) => {
            surface_grid(&attr.surface, &outer_uv, ccw, opts)
        }
        Surface::Plane(_) => plane_polygon_with_holes(&attr.surface, &outer_uv, &inner_uv, ccw),
        // TODO: real CDT for NURBS surfaces.
        _ => uv_bbox_quad(&attr.surface, &outer_uv, ccw),
    })
}

// ---------- pcurve sampling ----------

fn sample_loop_pcurve<P: Payload>(
    g: &GMap<P>,
    attr: &FaceAttr<P::F>,
    loop_dart: Dart,
    opts: TessellateOpts,
) -> Option<Vec<Point2>> {
    let edge_darts: Vec<Dart> = Profile::new(g, loop_dart).darts().step_by(2).collect();
    if edge_darts.is_empty() {
        return None;
    }
    let segments = opts.curve.segments.max(1);
    let mut points = Vec::new();
    for d in &edge_darts {
        let curve = attr.pcurves.get(d)?;
        let samples = curve.sample(segments);
        if samples.is_empty() {
            return None;
        }
        // Drop the last sample so it doesn't duplicate the next pcurve's
        // first sample. The very last edge re-closes the loop, so its tail
        // matches the first edge's head — also fine to drop.
        let n = samples.len();
        for s in samples.into_iter().take(n.saturating_sub(1)) {
            points.push(s);
        }
    }
    if points.is_empty() {
        None
    } else {
        Some(points)
    }
}

/// Shoelace signed area in UV. Positive ⇒ CCW.
fn signed_area(poly: &[Point2]) -> f64 {
    let n = poly.len();
    if n < 3 {
        return 0.0;
    }
    let mut s = 0.0;
    for i in 0..n {
        let p = poly[i];
        let q = poly[(i + 1) % n];
        s += p.x * q.y - q.x * p.y;
    }
    0.5 * s
}

// ---------- shortcuts ----------

fn surface_grid(
    surface: &Surface,
    outer_uv: &[Point2],
    ccw: bool,
    opts: TessellateOpts,
) -> IndexedMesh {
    let (u_min, u_max, v_min, v_max) = uv_bbox(outer_uv);
    let mut mesh = tessellate_surface_patch(surface, (u_min, u_max), (v_min, v_max), opts.surface);
    if !ccw {
        flip_winding(&mut mesh.indices);
        for n in &mut mesh.normals {
            *n = -*n;
        }
    }
    mesh
}

fn plane_polygon_with_holes(
    surface: &Surface,
    outer_uv: &[Point2],
    inner_uv: &[Vec<Point2>],
    ccw: bool,
) -> IndexedMesh {
    let Some(mut polygon) = build_simple_polygon(outer_uv, inner_uv) else {
        return uv_bbox_quad(surface, outer_uv, ccw);
    };

    if signed_area(&polygon) < 0.0 {
        polygon.reverse();
    }

    let normal = face_normal(surface, polygon[0].x, polygon[0].y, ccw);
    let positions = polygon
        .iter()
        .map(|p| surface.point_at(p.x, p.y))
        .collect::<Vec<_>>();
    let normals = vec![normal; positions.len()];
    let mut indices = ear_clip(&polygon).unwrap_or_default();
    if !ccw {
        flip_winding(&mut indices);
    }
    if indices.is_empty() {
        return uv_bbox_quad(surface, outer_uv, ccw);
    }

    IndexedMesh {
        positions,
        normals,
        indices,
    }
}

fn build_simple_polygon(outer_uv: &[Point2], inner_uv: &[Vec<Point2>]) -> Option<Vec<Point2>> {
    let mut polygon = clean_loop(outer_uv);
    if polygon.len() < 3 {
        return None;
    }
    if signed_area(&polygon) < 0.0 {
        polygon.reverse();
    }

    for hole in inner_uv {
        let mut hole = clean_loop(hole);
        if hole.len() < 3 {
            continue;
        }
        if signed_area(&hole) > 0.0 {
            hole.reverse();
        }
        polygon = bridge_hole(&polygon, &hole)?;
    }

    Some(polygon)
}

fn clean_loop(points: &[Point2]) -> Vec<Point2> {
    let mut cleaned = Vec::new();
    for point in points {
        if cleaned
            .last()
            .is_none_or(|previous: &Point2| !same_point(previous, point))
        {
            cleaned.push(*point);
        }
    }
    if cleaned.len() > 1 && same_point(&cleaned[0], cleaned.last().expect("non-empty")) {
        cleaned.pop();
    }

    let mut changed = true;
    while changed && cleaned.len() >= 3 {
        changed = false;
        let n = cleaned.len();
        let mut next = Vec::with_capacity(n);
        for i in 0..n {
            let prev = cleaned[(i + n - 1) % n];
            let curr = cleaned[i];
            let after = cleaned[(i + 1) % n];
            let collinear = orient(prev, curr, after).abs() <= EPS;
            let between = (curr - prev).dot(&(after - curr)) >= -EPS;
            if collinear && between {
                changed = true;
            } else {
                next.push(curr);
            }
        }
        cleaned = next;
    }

    cleaned
}

fn bridge_hole(polygon: &[Point2], hole: &[Point2]) -> Option<Vec<Point2>> {
    let hole_idx = rightmost_vertex(hole);
    let hole_point = hole[hole_idx];
    let polygon_idx = (0..polygon.len())
        .filter(|idx| bridge_is_visible(hole_point, hole_idx, polygon[*idx], *idx, polygon, hole))
        .min_by(|a, b| {
            let da = (polygon[*a] - hole_point).norm_squared();
            let db = (polygon[*b] - hole_point).norm_squared();
            da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
        })?;

    let mut bridged = Vec::with_capacity(polygon.len() + hole.len() + 2);
    bridged.extend_from_slice(&polygon[..=polygon_idx]);
    for offset in 0..hole.len() {
        bridged.push(hole[(hole_idx + offset) % hole.len()]);
    }
    bridged.push(hole_point);
    bridged.push(polygon[polygon_idx]);
    bridged.extend_from_slice(&polygon[polygon_idx + 1..]);
    Some(bridged)
}

fn rightmost_vertex(points: &[Point2]) -> usize {
    (0..points.len())
        .max_by(|a, b| {
            points[*a]
                .x
                .partial_cmp(&points[*b].x)
                .unwrap_or(std::cmp::Ordering::Equal)
                .then_with(|| {
                    points[*b]
                        .y
                        .partial_cmp(&points[*a].y)
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
        })
        .unwrap_or(0)
}

fn bridge_is_visible(
    a: Point2,
    hole_idx: usize,
    b: Point2,
    polygon_idx: usize,
    polygon: &[Point2],
    hole: &[Point2],
) -> bool {
    if same_point(&a, &b) {
        return false;
    }

    for i in 0..polygon.len() {
        let j = (i + 1) % polygon.len();
        if i == polygon_idx || j == polygon_idx {
            continue;
        }
        if segments_intersect_strict(a, b, polygon[i], polygon[j]) {
            return false;
        }
    }

    for i in 0..hole.len() {
        let j = (i + 1) % hole.len();
        if i == hole_idx || j == hole_idx {
            continue;
        }
        if segments_intersect_strict(a, b, hole[i], hole[j]) {
            return false;
        }
    }

    true
}

fn ear_clip(polygon: &[Point2]) -> Option<Vec<u32>> {
    if polygon.len() < 3 {
        return None;
    }

    let mut vertices = (0..polygon.len()).collect::<Vec<_>>();
    let mut indices = Vec::with_capacity((polygon.len() - 2) * 3);
    let mut guard = 0;

    while vertices.len() > 3 {
        let mut clipped = false;
        let len = vertices.len();
        for i in 0..len {
            let prev = vertices[(i + len - 1) % len];
            let curr = vertices[i];
            let next = vertices[(i + 1) % len];
            let a = polygon[prev];
            let b = polygon[curr];
            let c = polygon[next];

            if orient(a, b, c) <= EPS {
                continue;
            }
            if vertices.iter().any(|idx| {
                *idx != prev
                    && *idx != curr
                    && *idx != next
                    && !same_point(&polygon[*idx], &a)
                    && !same_point(&polygon[*idx], &b)
                    && !same_point(&polygon[*idx], &c)
                    && point_in_triangle(polygon[*idx], a, b, c)
            }) {
                continue;
            }

            indices.extend_from_slice(&[prev as u32, curr as u32, next as u32]);
            vertices.remove(i);
            clipped = true;
            break;
        }

        if !clipped {
            return None;
        }
        guard += 1;
        if guard > polygon.len() * polygon.len() {
            return None;
        }
    }

    indices.extend_from_slice(&[vertices[0] as u32, vertices[1] as u32, vertices[2] as u32]);
    Some(indices)
}

fn point_in_triangle(p: Point2, a: Point2, b: Point2, c: Point2) -> bool {
    orient(a, b, p) >= -EPS && orient(b, c, p) >= -EPS && orient(c, a, p) >= -EPS
}

fn segments_intersect_strict(a: Point2, b: Point2, c: Point2, d: Point2) -> bool {
    if same_point(&a, &c) || same_point(&a, &d) || same_point(&b, &c) || same_point(&b, &d) {
        return false;
    }

    let o1 = orient(a, b, c);
    let o2 = orient(a, b, d);
    let o3 = orient(c, d, a);
    let o4 = orient(c, d, b);

    if o1.abs() <= EPS && on_segment(a, c, b) {
        return true;
    }
    if o2.abs() <= EPS && on_segment(a, d, b) {
        return true;
    }
    if o3.abs() <= EPS && on_segment(c, a, d) {
        return true;
    }
    if o4.abs() <= EPS && on_segment(c, b, d) {
        return true;
    }

    (o1 > EPS) != (o2 > EPS) && (o3 > EPS) != (o4 > EPS)
}

fn on_segment(a: Point2, p: Point2, b: Point2) -> bool {
    p.x >= a.x.min(b.x) - EPS
        && p.x <= a.x.max(b.x) + EPS
        && p.y >= a.y.min(b.y) - EPS
        && p.y <= a.y.max(b.y) + EPS
}

fn orient(a: Point2, b: Point2, c: Point2) -> f64 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}

fn same_point(a: &Point2, b: &Point2) -> bool {
    (a - b).norm_squared() <= EPS * EPS
}

/// TODO: real CDT. Until then, faces we can't trim collapse to a single quad
/// covering the pcurve UV bbox so the user at least sees *something*.
fn uv_bbox_quad(surface: &Surface, outer_uv: &[Point2], ccw: bool) -> IndexedMesh {
    let (u_min, u_max, v_min, v_max) = uv_bbox(outer_uv);
    let normal = face_normal(surface, 0.5 * (u_min + u_max), 0.5 * (v_min + v_max), ccw);
    let positions = vec![
        surface.point_at(u_min, v_min),
        surface.point_at(u_max, v_min),
        surface.point_at(u_max, v_max),
        surface.point_at(u_min, v_max),
    ];
    let normals = vec![normal; 4];
    let indices = if ccw {
        vec![0, 1, 2, 0, 2, 3]
    } else {
        vec![0, 2, 1, 0, 3, 2]
    };
    IndexedMesh {
        positions,
        normals,
        indices,
    }
}

// ---------- helpers ----------

fn uv_bbox(points: &[Point2]) -> (f64, f64, f64, f64) {
    let mut u_min = f64::INFINITY;
    let mut u_max = f64::NEG_INFINITY;
    let mut v_min = f64::INFINITY;
    let mut v_max = f64::NEG_INFINITY;
    for p in points {
        u_min = u_min.min(p.x);
        u_max = u_max.max(p.x);
        v_min = v_min.min(p.y);
        v_max = v_max.max(p.y);
    }
    (u_min, u_max, v_min, v_max)
}

fn face_normal(surface: &Surface, u: f64, v: f64, ccw: bool) -> UnitVector3<f64> {
    let n = surface.normal_at(u, v);
    if ccw { n } else { -n }
}

fn flip_winding(indices: &mut [u32]) {
    for tri in indices.chunks_mut(3) {
        if tri.len() == 3 {
            tri.swap(1, 2);
        }
    }
}
