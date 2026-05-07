//! Hollow cylinder: outer + inner cylindrical walls capped by two annular
//! plates, α2-sewn into a closed 2-shell.
//!
//! All face geometry comes from the generic `crate::tessellate` pipeline now.
//! The script's only job is to:
//!
//! 1. Build the GMap (cylinder strips + annular caps).
//! 2. Hand pcurves on every dart (so [`crate::tessellate::tessellate_face`]
//!    can sample the trimmed surface region).
//! 3. Stamp a color/label per face via [`VizHints`].

use std::collections::HashMap;
use std::f64::consts::TAU;

use crate::builders::add_polygon;
use crate::geometry::{Curve2, Cylinder, Line2, Plane, Point2, Point3, Surface};
use crate::topology::StandardPayload;
use crate::topology::attributes::{FaceAttr, SolidAttr};
use crate::topology::gmap::{Dart, Dim, GMap};
use crate::topology::profile::Profile;
use crate::topology::shape_keys::FaceKey;
use crate::viz::{ScriptResult, Style, VizHints};

const RING_SEGMENTS: usize = 4;
const OUTER_RADIUS: f64 = 1.0;
const INNER_RADIUS: f64 = 0.55;
const HEIGHT: f64 = 1.8;

pub fn run() -> Result<ScriptResult, String> {
    let (g, face_styles, _) = build_hollow_cylinder_gmap()?;
    let mut hints = VizHints::new();
    for (key, style) in face_styles {
        hints.face(key, style);
    }
    Ok(ScriptResult::from_gmap_with_hints(&g, &hints))
}

/// Returns the map, per-face style hints, and one dart on the outer
/// cylindrical shell (all boundary darts lie on the same closed 2-sheet).
pub fn build_hollow_cylinder_gmap()
-> Result<(GMap<StandardPayload>, Vec<(FaceKey, Style)>, Dart), String> {
    let mut g = GMap::<StandardPayload>::new();
    let mut styles: Vec<(FaceKey, Style)> = Vec::new();

    let (outer_loops, outer_keys) = add_cylinder_wall_strips(&mut g, OUTER_RADIUS, true)?;
    let (inner_loops, inner_keys) = add_cylinder_wall_strips(&mut g, INNER_RADIUS, false)?;

    sew_cylinder_strip_ring(&mut g, &outer_loops, CylinderStripKind::Outer)?;
    sew_cylinder_strip_ring(&mut g, &inner_loops, CylinderStripKind::Inner)?;

    let top_cap = add_annulus_cap(&mut g, HEIGHT * 0.5, true)?;
    let bottom_cap = add_annulus_cap(&mut g, -HEIGHT * 0.5, false)?;

    sew_top_cap_to_walls(
        &mut g,
        top_cap.outer_loop,
        top_cap.inner_loop,
        &outer_loops,
        &inner_loops,
    )?;
    sew_bottom_cap_to_walls(
        &mut g,
        bottom_cap.outer_loop,
        bottom_cap.inner_loop,
        &outer_loops,
        &inner_loops,
    )?;

    let _solid = g.add_solid(SolidAttr::new((), outer_loops[0], None));

    for k in &outer_keys {
        styles.push((
            *k,
            Style::default()
                .color("#4f8cff")
                .label("outer cylinder wall")
                .double_sided(true),
        ));
    }
    for k in &inner_keys {
        styles.push((
            *k,
            Style::default()
                .color("#2f5d9e")
                .label("inner cylinder wall")
                .double_sided(true),
        ));
    }
    styles.push((
        top_cap.face_key,
        Style::default()
            .color("#75b8ff")
            .label("top annular cap")
            .double_sided(true),
    ));
    styles.push((
        bottom_cap.face_key,
        Style::default()
            .color("#3b74c4")
            .label("bottom annular cap")
            .double_sided(true),
    ));

    Ok((g, styles, outer_loops[0]))
}

#[derive(Clone, Copy, Debug)]
enum CylinderStripKind {
    Outer,
    Inner,
}

fn sew_cylinder_strip_ring(
    g: &mut GMap<StandardPayload>,
    loops: &[Dart],
    kind: CylinderStripKind,
) -> Result<(), String> {
    let n = RING_SEGMENTS;
    for i in 0..n {
        let a = edge_leaving_dart(
            g,
            loops[i],
            match kind {
                CylinderStripKind::Outer => 1,
                CylinderStripKind::Inner => 2,
            },
        );
        let b = edge_leaving_dart(
            g,
            loops[(i + 1) % n],
            match kind {
                CylinderStripKind::Outer => 3,
                CylinderStripKind::Inner => 0,
            },
        );
        g.sew(Dim::Two, a, b)
            .map_err(|_| format!("cylinder strip α2 sew failed at seam {i} ({kind:?})"))?;
    }
    Ok(())
}

struct CapBuild {
    face_key: FaceKey,
    outer_loop: Dart,
    inner_loop: Dart,
}

fn add_cylinder_wall_strips(
    g: &mut GMap<StandardPayload>,
    radius: f64,
    outward: bool,
) -> Result<(Vec<Dart>, Vec<FaceKey>), String> {
    let surface = Surface::Cylinder(Cylinder::new(
        Point3::origin(),
        nalgebra::Vector3::x(),
        nalgebra::Vector3::z(),
        radius,
    ));
    let vmin = -HEIGHT * 0.5;
    let vmax = HEIGHT * 0.5;
    let n = RING_SEGMENTS;
    let mut loops = Vec::with_capacity(n);
    let mut keys = Vec::with_capacity(n);
    for i in 0..n {
        let u0 = TAU * i as f64 / n as f64;
        let u1 = TAU * (i + 1) as f64 / n as f64;
        let uv = if outward {
            vec![
                Point2::new(u0, vmin),
                Point2::new(u1, vmin),
                Point2::new(u1, vmax),
                Point2::new(u0, vmax),
            ]
        } else {
            vec![
                Point2::new(u0, vmin),
                Point2::new(u0, vmax),
                Point2::new(u1, vmax),
                Point2::new(u1, vmin),
            ]
        };
        let corners: Vec<Point3> = uv.iter().map(|p| surface.point_at(p.x, p.y)).collect();
        let loop_dart = add_polygon(g, &corners);
        let pcurves = loop_line_pcurves(g, loop_dart, &uv);
        let key = g.add_face(FaceAttr::with_pcurves(
            surface.clone(),
            (),
            loop_dart,
            Vec::new(),
            pcurves,
        ));
        loops.push(loop_dart);
        keys.push(key);
    }
    Ok((loops, keys))
}

fn add_annulus_cap(g: &mut GMap<StandardPayload>, z: f64, top: bool) -> Result<CapBuild, String> {
    let y_dir = if top {
        nalgebra::Vector3::y()
    } else {
        -nalgebra::Vector3::y()
    };
    let surface = Surface::Plane(Plane::from_xy(
        Point3::new(0.0, 0.0, z),
        nalgebra::Vector3::x(),
        y_dir,
    ));

    let outer_uv = circle_uv(OUTER_RADIUS, !top);
    let inner_uv = circle_uv(INNER_RADIUS, top);
    let outer_loop = add_polygon(g, &points_on_surface(&surface, &outer_uv));
    let inner_loop = add_polygon(g, &points_on_surface(&surface, &inner_uv));

    let mut pcurves = loop_line_pcurves(g, outer_loop, &outer_uv);
    pcurves.extend(loop_line_pcurves(g, inner_loop, &inner_uv));

    let face_key = g.add_face(FaceAttr::with_pcurves(
        surface,
        (),
        outer_loop,
        vec![inner_loop],
        pcurves,
    ));

    Ok(CapBuild {
        face_key,
        outer_loop,
        inner_loop,
    })
}

fn sew_top_cap_to_walls(
    g: &mut GMap<StandardPayload>,
    cap_outer: Dart,
    cap_inner: Dart,
    outer_wall: &[Dart],
    inner_wall: &[Dart],
) -> Result<(), String> {
    let n = RING_SEGMENTS;
    for i in 0..n {
        let cap_o = edge_leaving_dart(g, cap_outer, i);
        let wall_o = edge_leaving_dart(g, outer_wall[i], 2);
        g.sew(Dim::Two, cap_o, wall_o)
            .map_err(|_| format!("top cap outer α2 failed at segment {i}"))?;

        let cap_i = edge_leaving_dart(g, cap_inner, i);
        let widx = (n + n - 2 - i) % n;
        let wall_i = edge_leaving_dart(g, inner_wall[widx], 1);
        g.sew(Dim::Two, cap_i, wall_i)
            .map_err(|_| format!("top cap inner α2 failed at segment {i}"))?;
    }
    Ok(())
}

fn sew_bottom_cap_to_walls(
    g: &mut GMap<StandardPayload>,
    cap_outer: Dart,
    cap_inner: Dart,
    outer_wall: &[Dart],
    inner_wall: &[Dart],
) -> Result<(), String> {
    let n = RING_SEGMENTS;
    for i in 0..n {
        let cap_o = edge_leaving_dart(g, cap_outer, i);
        let widx = (n + n - 2 - i) % n;
        let wall_o = edge_leaving_dart(g, outer_wall[widx], 0);
        g.sew(Dim::Two, cap_o, wall_o)
            .map_err(|_| format!("bottom cap outer α2 failed at segment {i}"))?;

        let cap_i = edge_leaving_dart(g, cap_inner, i);
        let wall_i = edge_leaving_dart(g, inner_wall[i], 3);
        g.sew(Dim::Two, cap_i, wall_i)
            .map_err(|_| format!("bottom cap inner α2 failed at segment {i}"))?;
    }
    Ok(())
}

fn edge_leaving_dart<P: crate::topology::payload::Payload>(
    g: &GMap<P>,
    loop_dart: Dart,
    edge_idx: usize,
) -> Dart {
    let mut v = loop_dart;
    for _ in 0..edge_idx {
        v = g.alpha(Dim::One, g.alpha(Dim::Zero, v));
    }
    g.alpha(Dim::Zero, v)
}

fn loop_line_pcurves(
    g: &GMap<StandardPayload>,
    loop_dart: Dart,
    uv: &[Point2],
) -> HashMap<Dart, Curve2> {
    let darts = loop_edge_darts(g, loop_dart);
    assert_eq!(darts.len(), uv.len());
    let mut pcurves = HashMap::with_capacity(darts.len());
    for i in 0..darts.len() {
        pcurves.insert(
            darts[i],
            Curve2::Line(Line2::new(uv[i], uv[(i + 1) % uv.len()])),
        );
    }
    pcurves
}

fn loop_edge_darts(g: &GMap<StandardPayload>, loop_dart: Dart) -> Vec<Dart> {
    Profile::new(g, loop_dart).darts().step_by(2).collect()
}

fn circle_uv(radius: f64, clockwise: bool) -> Vec<Point2> {
    let mut points = (0..RING_SEGMENTS)
        .map(|i| {
            let u = TAU * i as f64 / RING_SEGMENTS as f64;
            let (sin_u, cos_u) = u.sin_cos();
            Point2::new(radius * cos_u, radius * sin_u)
        })
        .collect::<Vec<_>>();
    if clockwise {
        points.reverse();
    }
    points
}

fn points_on_surface(surface: &Surface, uv: &[Point2]) -> Vec<Point3> {
    uv.iter().map(|p| surface.point_at(p.x, p.y)).collect()
}

#[cfg(test)]
mod tests {
    use super::build_hollow_cylinder_gmap;
    use super::run;
    use crate::topology::closed::Closed;
    use crate::topology::sheet::Sheet;

    #[test]
    fn hollow_cylinder_script_emits_brep_scene() {
        let result = run().expect("hollow cylinder script should run");
        assert!(!result.scene.faces.is_empty());
        assert!(!result.scene.vertices.is_empty());
        assert!(!result.scene.edges.is_empty());
        assert!(!result.scene.darts.is_empty());
        assert!(
            result
                .scene
                .faces
                .iter()
                .all(|m| !m.positions.is_empty() && !m.indices.is_empty())
        );
        assert!(result.scene.edges.iter().all(|e| e.polyline.len() >= 2));
        assert!(result.scene.darts.iter().all(|d| d.shaft.len() >= 2));
    }

    #[test]
    fn hollow_cylinder_boundary_is_closed_shell() {
        let (g, _, shell_dart) = build_hollow_cylinder_gmap().expect("build");
        assert!(
            Closed::new(Sheet::new(&g, shell_dart)).is_some(),
            "hollow cylinder boundary should be a closed 2-shell"
        );
    }
}
