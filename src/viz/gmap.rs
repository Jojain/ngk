//! Convert a [`GMap`] into a [`VizScene`] for inspection.
//!
//! Conventions:
//! - **Vertices** → one [`VizPoint`] per 0-cell that has a position attribute.
//! - **Darts** → one [`VizArrow`] per dart. An arrow goes from the dart's
//!   vertex to the edge midpoint (half-edge length), so α0-paired darts meet
//!   tip-to-tip and naturally read as "one edge, two darts". Each arrow is
//!   nudged slightly perpendicular to the edge, toward its face center, so
//!   α2-sewn sibling darts from a neighboring face sit on the opposite side
//!   of the edge instead of overlapping.
//! - **α-involutions** → one [`VizLink`] per (dart, αᵢ(dart)) non-fixed pair,
//!   drawn between the shaft midpoints and tagged with `involution = i`. The
//!   viewer colors/toggles them per index.

use std::collections::HashSet;

use nalgebra::Vector3;

use super::{GMapSnapshot, VertexPointEntry, VizArrow, VizLink, VizPoint, VizScene};
use crate::geometry::utils::Point3;
use crate::topology::gmap::{Cell0, Dart, GMap};
use crate::topology::payload::Payload;

/// How far to push a dart perpendicular to its edge, relative to edge length.
/// Kept small — just enough to separate α2-sibling darts in the neighbor face.
const DART_OFFSET_RATIO: f64 = 0.05;

pub fn scene_from_gmap<P: Payload>(g: &GMap<P>) -> VizScene {
    let mut scene = VizScene::new();
    emit_vertices(g, &mut scene);

    let arrows = compute_arrows(g);
    emit_alpha_links(g, &arrows, &mut scene);
    for (_, arrow) in arrows {
        scene.arrows.push(arrow);
    }

    scene
}

pub fn snapshot_from_gmap<P: Payload>(g: &GMap<P>) -> GMapSnapshot {
    let dim = g.dimension();
    let n = g.dart_count();

    let alphas: Vec<Vec<u32>> = (0..dim)
        .map(|i| {
            (0..n)
                .map(|d| g.alpha(i, Dart::new(d)).id() as u32)
                .collect()
        })
        .collect();

    let vertex_points: Vec<VertexPointEntry> = (0..n)
        .filter_map(|d| {
            let dart = Dart::new(d);
            g.attribute::<Cell0>(dart).map(|v| VertexPointEntry {
                dart: d as u32,
                position: [v.point.x, v.point.y, v.point.z],
            })
        })
        .collect();

    GMapSnapshot {
        dimension: dim as u32,
        dart_count: n as u32,
        alphas,
        vertex_points,
    }
}

// ---------- internals ----------

fn emit_vertices<P: Payload>(g: &GMap<P>, scene: &mut VizScene) {
    for dart in representative_darts(g, 0) {
        if let Some(p) = g.attribute::<Cell0>(dart).map(|v| v.point) {
            scene.points.push(VizPoint {
                position: [p.x, p.y, p.z],
                color: None,
                size: None,
                label: None,
            });
        }
    }
}

/// Computes an arrow per drawable dart. Returns them keyed by dart id so we
/// can later resolve α-link endpoints without recomputing geometry.
fn compute_arrows<P: Payload>(g: &GMap<P>) -> Vec<(usize, VizArrow)> {
    let mut out = Vec::new();
    for id in 0..g.dart_count() {
        let d = Dart::new(id);
        if let Some(arrow) = dart_arrow(g, d) {
            out.push((id, arrow));
        }
    }
    out
}

fn dart_arrow<P: Payload>(g: &GMap<P>, d: Dart) -> Option<VizArrow> {
    let v0 = g.attribute::<Cell0>(d).map(|v| v.point)?;
    let other = g.alpha(0, d);
    let v1 = g.attribute::<Cell0>(other).map(|v| v.point)?;
    let edge = v1 - v0;
    let edge_len = edge.norm();
    if edge_len < 1e-12 {
        return None;
    }

    let offset = face_interior_direction(g, d, edge / edge_len).unwrap_or_else(Vector3::zeros);
    let offset_vec = offset * (DART_OFFSET_RATIO * edge_len);

    let origin = v0 + offset_vec;
    let tip = v0 + edge * 0.5 + offset_vec;

    Some(VizArrow {
        origin: [origin.x, origin.y, origin.z],
        tip: [tip.x, tip.y, tip.z],
        dart: Some(d.id() as u32),
        color: None,
        label: Some(format!("d{}", d.id())),
    })
}

/// Offset direction: perpendicular to the edge, in the plane of the face,
/// pointing toward the face's centroid. For a boundary (α2-free) dart this
/// still returns a sensible side so pairs of sewn darts separate cleanly.
fn face_interior_direction<P: Payload>(
    g: &GMap<P>,
    d: Dart,
    edge_hat: Vector3<f64>,
) -> Option<Vector3<f64>> {
    let v0 = g.attribute::<Cell0>(d).map(|v| v.point)?;
    let face_center = face_centroid(g, d)?;
    let toward: Vector3<f64> = face_center - v0;
    let perp = toward - edge_hat * toward.dot(&edge_hat);
    if perp.norm() > 1e-9 {
        return Some(perp.normalize());
    }

    let up = Vector3::new(0.0, 0.0, 1.0);
    let fallback = edge_hat.cross(&up);
    if fallback.norm() > 1e-9 {
        Some(fallback.normalize())
    } else {
        None
    }
}

/// Centroid of the face 2-cell containing `d`: average of unique vertex
/// positions in the ⟨α₀,α₁⟩ orbit.
fn face_centroid<P: Payload>(g: &GMap<P>, d: Dart) -> Option<Point3> {
    if g.dimension() < 2 {
        return None;
    }
    let face_orbit_indices = vec![0usize, 1];
    let mut seen: HashSet<usize> = HashSet::new();
    let mut sum = Vector3::zeros();
    let mut count = 0usize;
    for dd in g.orbit(d, face_orbit_indices) {
        let rep = g.cell_representative(dd, 0).id();
        if seen.insert(rep) {
            if let Some(p) = g.attribute::<Cell0>(dd).map(|v| v.point) {
                sum += p.coords;
                count += 1;
            }
        }
    }
    if count == 0 {
        None
    } else {
        Some(Point3::from(sum / count as f64))
    }
}

fn emit_alpha_links<P: Payload>(
    g: &GMap<P>,
    arrows: &[(usize, VizArrow)],
    scene: &mut VizScene,
) {
    use std::collections::HashMap;
    let by_id: HashMap<usize, &VizArrow> = arrows.iter().map(|(i, a)| (*i, a)).collect();

    for i in 0..g.dimension() {
        for &(id, _) in arrows {
            let d = Dart::new(id);
            let j = g.alpha(i, d).id();
            if j == id || j < id {
                continue; // skip fixed points and the mirror direction
            }
            let Some(a) = by_id.get(&id) else { continue };
            let Some(b) = by_id.get(&j) else { continue };
            scene.alpha_links.push(VizLink {
                involution: i as u32,
                a: shaft_midpoint(a),
                b: shaft_midpoint(b),
                dart_a: Some(id as u32),
                dart_b: Some(j as u32),
            });
        }
    }
}

fn shaft_midpoint(a: &VizArrow) -> [f64; 3] {
    [
        (a.origin[0] + a.tip[0]) * 0.5,
        (a.origin[1] + a.tip[1]) * 0.5,
        (a.origin[2] + a.tip[2]) * 0.5,
    ]
}

fn representative_darts<P: Payload>(g: &GMap<P>, cell_dim: usize) -> Vec<Dart> {
    (0..g.dart_count())
        .map(Dart::new)
        .filter(|&d| g.cell_representative(d, cell_dim) == d)
        .collect()
}
