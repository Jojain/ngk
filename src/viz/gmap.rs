//! GMap debugging overlay: dart arrows + alpha-involution links.
//!
//! Every dart that has both endpoints on a stored 1-cell becomes one
//! [`VizDart`]. The arrow's shaft follows the edge's [`Curve`] from the dart's
//! own vertex parameter to the midpoint parameter. Darts are emitted directly
//! on their underlying edge geometry; no display offset is applied here.

use std::collections::HashMap;

use nalgebra::Vector3;

use super::brep::BrepIndex;
use super::hints::VizHints;
use super::scene::{VizAlphaLink, VizDart, VizScene};
use crate::geometry::{Curve, Point3};
use crate::tessellate::{TessellateOpts, tessellate_curve};
use crate::topology::gmap::{Cell0, Cell1, Dart, Dim, GMap};
use crate::topology::payload::Payload;

const DART_SHAFT_FRACTION: f64 = 0.4;

/// Emit [`VizDart`] entries for every dart that has a recoverable shaft, plus
/// one [`VizAlphaLink`] per non-trivial alpha pair.
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
        return chord_arrow(edge_id, v0, v1);
    }

    let t0 = curve.param_at(v0);
    let t1 = curve.param_at(v1);
    if !(t0.is_finite() && t1.is_finite()) {
        return chord_arrow(edge_id, v0, v1);
    }

    let t_tip = t0 + (t1 - t0) * DART_SHAFT_FRACTION;
    let polyline = tessellate_curve(curve, t0, t_tip, opts.curve);
    if polyline.points.len() < 2 {
        return chord_arrow(edge_id, v0, v1);
    }

    let shaft: Vec<[f64; 3]> = polyline.points.iter().map(|p| [p.x, p.y, p.z]).collect();
    let tip_dir = tip_tangent(&shaft);
    Some(DartArrow {
        edge_id,
        shaft,
        tip_dir,
    })
}

fn chord_arrow(edge_id: u32, v0: Point3, v1: Point3) -> Option<DartArrow> {
    let edge_len = (v1 - v0).norm();
    if edge_len < 1e-12 {
        return None;
    }
    let tip = v0 + (v1 - v0) * DART_SHAFT_FRACTION;
    let shaft = vec![[v0.x, v0.y, v0.z], [tip.x, tip.y, tip.z]];
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

fn emit_alpha_links<P: Payload>(
    g: &GMap<P>,
    shafts: &HashMap<u32, Vec<[f64; 3]>>,
    scene: &mut VizScene,
) {
    for i in 0..g.dimension() {
        for &dart_id in shafts.keys() {
            let d = Dart::new(dart_id as usize);
            let pair = g.alpha(Dim::from_index(i), d).id() as u32;
            if pair == dart_id || pair < dart_id {
                continue;
            }
            let Some(a) = shafts.get(&dart_id) else {
                continue;
            };
            let Some(b) = shafts.get(&pair) else {
                continue;
            };
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

    use nalgebra::Vector3;

    use crate::builders::profiles::add_edge;
    use crate::geometry::{Circle, Curve, Line, Plane, Point3};
    use crate::topology::StandardPayload;
    use crate::topology::gmap::GMap;
    use crate::viz::{VizHints, scene_from_gmap};

    #[test]
    fn dart_on_circle_edge_has_curved_shaft() {
        let mut g = GMap::<StandardPayload>::new();
        let plane = Plane::new(Point3::origin(), Vector3::x(), Vector3::z());
        let curve = Curve::Circle(Circle::new(plane, 1.0));
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
            let radial = Vector3::new(tip[0], tip[1], 0.0);
            if radial.norm() < crate::geometry::LINEAR_TOLERANCE {
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

    #[test]
    fn dart_shaft_is_not_offset_from_edge_start() {
        let mut g = GMap::<StandardPayload>::new();
        let start = Point3::new(1.0, 2.0, 3.0);
        let end = Point3::new(4.0, 2.0, 3.0);
        let _ = add_edge(&mut g, start, end, Curve::Line(Line::new(start, end)));

        let scene = scene_from_gmap(&g, &VizHints::new());
        assert_eq!(scene.darts.len(), 2);
        let starts: Vec<[f64; 3]> = scene.darts.iter().map(|d| d.shaft[0]).collect();
        assert!(starts.contains(&[start.x, start.y, start.z]));
        assert!(starts.contains(&[end.x, end.y, end.z]));
    }
}
