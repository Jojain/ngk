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
        Surface::Plane(_) => {
            if inner_uv.is_empty() {
                plane_fan(&attr.surface, &outer_uv, ccw)
            } else if inner_uv.len() == 1 && inner_uv[0].len() == outer_uv.len() {
                plane_strip(&attr.surface, &outer_uv, &inner_uv[0], ccw)
            } else {
                // TODO: real CDT (constrained Delaunay with holes).
                uv_bbox_quad(&attr.surface, &outer_uv, ccw)
            }
        }
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

fn plane_fan(surface: &Surface, outer_uv: &[Point2], ccw: bool) -> IndexedMesh {
    let n = outer_uv.len();
    let cx: f64 = outer_uv.iter().map(|p| p.x).sum::<f64>() / n as f64;
    let cy: f64 = outer_uv.iter().map(|p| p.y).sum::<f64>() / n as f64;

    let normal = face_normal(surface, cx, cy, ccw);

    let mut positions = Vec::with_capacity(n + 1);
    let mut normals = Vec::with_capacity(n + 1);
    positions.push(surface.point_at(cx, cy));
    normals.push(normal);
    for p in outer_uv {
        positions.push(surface.point_at(p.x, p.y));
        normals.push(normal);
    }

    let mut indices = Vec::with_capacity(n * 3);
    for i in 0..n {
        let a = 0u32;
        let b = (1 + i) as u32;
        let c = (1 + (i + 1) % n) as u32;
        if ccw {
            indices.extend_from_slice(&[a, b, c]);
        } else {
            indices.extend_from_slice(&[a, c, b]);
        }
    }

    IndexedMesh {
        positions,
        normals,
        indices,
    }
}

fn plane_strip(
    surface: &Surface,
    outer_uv: &[Point2],
    inner_uv: &[Point2],
    ccw: bool,
) -> IndexedMesh {
    debug_assert_eq!(outer_uv.len(), inner_uv.len());
    let n = outer_uv.len();
    let normal = face_normal(surface, outer_uv[0].x, outer_uv[0].y, ccw);

    let mut positions = Vec::with_capacity(n * 2);
    let mut normals = Vec::with_capacity(n * 2);
    for (o, i) in outer_uv.iter().zip(inner_uv.iter()) {
        positions.push(surface.point_at(o.x, o.y));
        normals.push(normal);
        positions.push(surface.point_at(i.x, i.y));
        normals.push(normal);
    }

    let mut indices = Vec::with_capacity(n * 6);
    for i in 0..n {
        let next = (i + 1) % n;
        let outer0 = (2 * i) as u32;
        let inner0 = outer0 + 1;
        let outer1 = (2 * next) as u32;
        let inner1 = outer1 + 1;
        if ccw {
            indices.extend_from_slice(&[outer0, outer1, inner1, outer0, inner1, inner0]);
        } else {
            indices.extend_from_slice(&[outer0, inner1, outer1, outer0, inner0, inner1]);
        }
    }

    IndexedMesh {
        positions,
        normals,
        indices,
    }
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
