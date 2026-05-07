//! A single cylindrical face strip (one quarter of a cylinder) — the smallest
//! possible scene that exercises the curved-surface tessellation path
//! (`Surface::Cylinder` + pcurves) and the curved-edge dart geometry.

use std::collections::HashMap;
use std::f64::consts::FRAC_PI_2;

use crate::builders::add_polygon;
use crate::geometry::{Curve2, Cylinder, Line2, Point2, Point3, Surface};
use crate::topology::StandardPayload;
use crate::topology::attributes::FaceAttr;
use crate::topology::gmap::{Dart, GMap};
use crate::topology::profile::Profile;
use crate::viz::{ScriptResult, Style, VizHints};

const RADIUS: f64 = 1.0;
const HEIGHT: f64 = 1.5;

pub fn run() -> Result<ScriptResult, String> {
    let mut g = GMap::<StandardPayload>::new();
    let mut hints = VizHints::new();

    let surface = Surface::Cylinder(Cylinder::new(
        Point3::origin(),
        nalgebra::Vector3::x(),
        nalgebra::Vector3::z(),
        RADIUS,
    ));

    let u0 = 0.0;
    let u1 = FRAC_PI_2;
    let v0 = -HEIGHT * 0.5;
    let v1 = HEIGHT * 0.5;
    let uv = vec![
        Point2::new(u0, v0),
        Point2::new(u1, v0),
        Point2::new(u1, v1),
        Point2::new(u0, v1),
    ];
    let corners: Vec<Point3> = uv.iter().map(|p| surface.point_at(p.x, p.y)).collect();

    let loop_dart = add_polygon(&mut g, &corners);
    let pcurves = loop_line_pcurves(&g, loop_dart, &uv);
    let face_key = g.add_face(FaceAttr::with_pcurves(
        surface,
        (),
        loop_dart,
        Vec::new(),
        pcurves,
    ));

    hints.face(
        face_key,
        Style::default()
            .color("#7bd0ff")
            .label("quarter cylinder")
            .double_sided(true),
    );

    Ok(ScriptResult::from_gmap_with_hints(&g, &hints))
}

fn loop_line_pcurves(
    g: &GMap<StandardPayload>,
    loop_dart: Dart,
    uv: &[Point2],
) -> HashMap<Dart, Curve2> {
    let darts: Vec<Dart> = Profile::new(g, loop_dart).darts().step_by(2).collect();
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
