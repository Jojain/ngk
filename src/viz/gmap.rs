//! GMap debugging overlay: dart arrows + α-involution links.
//!
//! Every dart that has both endpoints on a stored 1-cell becomes one
//! [`VizDart`]. The arrow's shaft follows the edge's `Curve` from the dart's
//! own vertex parameter to the *midpoint* parameter — so on a straight edge
//! the dart is a chord A → mid, on a circular arc it's a curved arc, on a
//! NURBS edge it's a sampled polyline. The arrow tip points toward the dart's
//! `α0` vertex, exactly as in the user's spec ("if an edge goes A → B then
//! the two darts go A → halfB pointing at B and B → halfA pointing at A").
//!
//! A small perpendicular offset toward the face centroid separates the dart
//! from its α2-paired sibling so the two darts of a sewn edge don't overlap.
//!
//! For each non-trivial αᵢ pair we emit one [`VizAlphaLink`] connecting the
//! two darts' shaft midpoints.

use std::collections::{HashMap, HashSet};

use nalgebra::Vector3;

use super::brep::BrepIndex;
use super::hints::VizHints;
use super::scene::{VizAlphaLink, VizDart, VizScene};
use crate::geometry::{Curve, Point3};
use crate::tessellate::{TessellateOpts, tessellate_curve};
use crate::topology::gmap::{Cell0, Cell1, Dart, Dim, GMap};
use crate::topology::payload::Payload;

/// How far to push a dart perpendicular to its edge, relative to edge length.
/// Just enough to separate α2-sibling darts in the neighbor face.
const DART_OFFSET_RATIO: f64 = 0.05;

/// Emit [`VizDart`] entries for every dart that has a recoverable shaft, plus
/// one [`VizAlphaLink`] per non-trivial αᵢ pair.
pub fn emit_gmap_overlay<P: Payload>(
    g: &GMap<P>,
    hints: &VizHints,
    index: &BrepIndex,
    opts: TessellateOpts,
    scene: &mut VizScene,
) {
    let mut shafts: HashMap<u32, Vec<[f64; 3]>> = HashMap::new();

    for id in 0..g.dart_count() {
        let d = Dart::new(id);
        let Some(arrow) = build_dart(g, d, index, opts) else {
            continue;
        };
        shafts.insert(id as u32, arrow.shaft.clone());
        let style = hints.dart_styles.get(&(id as u32));
        scene.darts.push(VizDart {
            dart_id: id as u32,
            edge_id: arrow.edge_id,
            shaft: arrow.shaft,
            tip_dir: arrow.tip_dir,
            color: style.and_then(|s| s.color.clone()),
            label: style
                .and_then(|s| s.label.clone())
                .or_else(|| Some(format!("d{id}"))),
        });
    }

    emit_alpha_links(g, &shafts, scene);
}

struct DartArrow {
    edge_id: u32,
    shaft: Vec<[f64; 3]>,
    tip_dir: [f64; 3],
}

/// Build the arrow for a single dart by sampling the edge's curve from the
/// dart's vertex parameter to the edge midpoint, with a face-interior offset
/// applied uniformly along the shaft.
fn build_dart<P: Payload>(
    g: &GMap<P>,
    d: Dart,
    index: &BrepIndex,
    opts: TessellateOpts,
) -> Option<DartArrow> {
    let edge_attr = g.attribute::<Cell1>(d)?;
    let edge_key = *index.edge_key_by_dart.get(&(d.id() as u32))?;
    let edge_id = *index.edge_id_by_key.get(&edge_key)?;

    let v0 = g.attribute::<Cell0>(d).map(|v| v.point)?;
    let other = g.alpha(Dim::Zero, d);
    let v1 = g.attribute::<Cell0>(other).map(|v| v.point)?;

    let curve = &edge_attr.curve;
    if matches!(curve, Curve::Nurbs(_)) {
        // TODO: NURBS curves don't yet expose `param_at`. Fall back to a
        // simple chord polyline so we at least see something.
        return chord_arrow(g, d, edge_id, v0, v1);
    }

    let t0 = curve.param_at(v0);
    let t1 = curve.param_at(v1);
    if !(t0.is_finite() && t1.is_finite()) {
        return chord_arrow(g, d, edge_id, v0, v1);
    }
    let t_mid = 0.5 * (t0 + t1);
    let polyline = tessellate_curve(curve, t0, t_mid, opts.curve);
    if polyline.points.len() < 2 {
        return chord_arrow(g, d, edge_id, v0, v1);
    }

    // Edge length used to scale the offset is the (Euclidean) distance
    // between the two endpoint vertices; this is simple and stable on
    // non-degenerate curves.
    let edge_len = (v1 - v0).norm();
    if edge_len < 1e-12 {
        return None;
    }

    let edge_hat = (v1 - v0) / edge_len;
    let offset = face_interior_direction(g, d, edge_hat).unwrap_or_else(Vector3::zeros);
    let offset_vec = offset * (DART_OFFSET_RATIO * edge_len);

    let shaft: Vec<[f64; 3]> = polyline
        .points
        .iter()
        .map(|p| {
            let q = p + offset_vec;
            [q.x, q.y, q.z]
        })
        .collect();

    let tip_dir = tip_tangent(&shaft);
    Some(DartArrow {
        edge_id,
        shaft,
        tip_dir,
    })
}

/// Fallback chord shaft for darts whose curve doesn't support inverse
/// parameterization. Same offset rules as the curved path.
fn chord_arrow<P: Payload>(
    g: &GMap<P>,
    d: Dart,
    edge_id: u32,
    v0: Point3,
    v1: Point3,
) -> Option<DartArrow> {
    let edge_len = (v1 - v0).norm();
    if edge_len < 1e-12 {
        return None;
    }
    let edge_hat = (v1 - v0) / edge_len;
    let offset = face_interior_direction(g, d, edge_hat).unwrap_or_else(Vector3::zeros);
    let offset_vec = offset * (DART_OFFSET_RATIO * edge_len);
    let origin = v0 + offset_vec;
    let tip = v0 + (v1 - v0) * 0.5 + offset_vec;
    let shaft = vec![
        [origin.x, origin.y, origin.z],
        [tip.x, tip.y, tip.z],
    ];
    let tip_dir = tip_tangent(&shaft);
    Some(DartArrow {
        edge_id,
        shaft,
        tip_dir,
    })
}

fn tip_tangent(shaft: &[[f64; 3]]) -> [f64; 3] {
    let n = shaft.len();
    if n < 2 {
        return [0.0, 0.0, 0.0];
    }
    let last = shaft[n - 1];
    let prev = shaft[n - 2];
    let v = Vector3::new(last[0] - prev[0], last[1] - prev[1], last[2] - prev[2]);
    let norm = v.norm();
    if norm < 1e-12 {
        [0.0, 0.0, 0.0]
    } else {
        let u = v / norm;
        [u.x, u.y, u.z]
    }
}

/// Offset direction: perpendicular to the edge, in the plane of the face
/// containing `d`, pointing toward the face's centroid. Falls back to a
/// sensible fixed perpendicular when the face is degenerate.
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
/// positions in the ⟨α₀, α₁⟩ orbit.
fn face_centroid<P: Payload>(g: &GMap<P>, d: Dart) -> Option<Point3> {
    if g.dimension() < 2 {
        return None;
    }
    let face_orbit_indices = vec![0usize, 1];
    let mut seen: HashSet<usize> = HashSet::new();
    let mut sum = Vector3::zeros();
    let mut count = 0usize;
    for dd in g.orbit(d, face_orbit_indices) {
        let rep = g.cell_representative(dd, Dim::Zero).id();
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
    shafts: &HashMap<u32, Vec<[f64; 3]>>,
    scene: &mut VizScene,
) {
    for i in 0..g.dimension() {
        for &dart_id in shafts.keys() {
            let d = Dart::new(dart_id as usize);
            let pair = g.alpha(Dim::from_index(i), d).id() as u32;
            // Skip fixed points (αᵢ(d) = d) and the mirror direction
            // (only emit each pair once; lower-id dart owns the link).
            if pair == dart_id || pair < dart_id {
                continue;
            }
            let Some(a) = shafts.get(&dart_id) else {
                continue;
            };
            let Some(b) = shafts.get(&pair) else { continue };
            scene.alpha_links.push(VizAlphaLink {
                involution: i as u32,
                dart_a: dart_id,
                dart_b: pair,
                a: shaft_midpoint(a),
                b: shaft_midpoint(b),
            });
        }
    }
}

fn shaft_midpoint(shaft: &[[f64; 3]]) -> [f64; 3] {
    if shaft.is_empty() {
        return [0.0, 0.0, 0.0];
    }
    let n = shaft.len();
    if n % 2 == 1 {
        shaft[n / 2]
    } else {
        let a = shaft[n / 2 - 1];
        let b = shaft[n / 2];
        [
            0.5 * (a[0] + b[0]),
            0.5 * (a[1] + b[1]),
            0.5 * (a[2] + b[2]),
        ]
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::FRAC_PI_2;

    use crate::builders::add_edge;
    use crate::geometry::{Circle, Curve, Plane, Point3};
    use crate::topology::StandardPayload;
    use crate::topology::gmap::GMap;
    use crate::viz::{VizHints, scene_from_gmap};
    use nalgebra::Vector3;

    /// A single quarter-arc circular edge produces darts whose shafts are
    /// sampled polylines (not chords) and whose tip tangent is roughly
    /// perpendicular to the radial direction at the shaft's tip.
    #[test]
    fn dart_on_circle_edge_has_curved_shaft() {
        let mut g = GMap::<StandardPayload>::new();
        let plane = Plane::new(Point3::origin(), Vector3::x(), Vector3::z());
        let circle = Circle::new(plane, 1.0);
        let curve = Curve::Circle(circle);
        let start = curve.point_at(0.0);
        let end = curve.point_at(FRAC_PI_2);
        let _ = add_edge(&mut g, start, end, curve);

        let scene = scene_from_gmap(&g, &VizHints::new());
        assert_eq!(scene.darts.len(), 2);
        for d in &scene.darts {
            assert!(
                d.shaft.len() > 2,
                "circular dart shaft should have multiple samples, got {}",
                d.shaft.len()
            );
            let tip = d.shaft[d.shaft.len() - 1];
            // Radial direction at the (offset) tip, projected to xy plane.
            let radial = Vector3::new(tip[0], tip[1], 0.0);
            if radial.norm() < 1e-9 {
                continue;
            }
            let radial = radial.normalize();
            let tangent = Vector3::new(d.tip_dir[0], d.tip_dir[1], d.tip_dir[2]);
            let dot = tangent.dot(&radial).abs();
            assert!(
                dot < 0.4,
                "tip tangent {:?} should be roughly perpendicular to radial {:?} (|dot|={dot})",
                tangent,
                radial,
            );
        }
    }
}

