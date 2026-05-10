use nalgebra::Vector3;

use crate::builders::profiles::add_polygon;
use crate::geometry::{LINEAR_TOLERANCE, Point3};
use crate::modeling::sweep::extrude_profile;
use crate::topology::StandardPayload;
use crate::topology::gmap::GMap;
use crate::topology::profile::Profile;
use crate::viz::{ScriptResult, Style, VizHints};

const POLYGON_RADIUS: f64 = 1.6;

pub fn build(point_count: usize, extrusion: Vector3<f64>) -> Result<ScriptResult, String> {
    if point_count < 3 {
        return Err(format!(
            "interactive extrusion needs at least 3 polygon points, got {point_count}"
        ));
    }
    if extrusion.norm_squared() <= LINEAR_TOLERANCE * LINEAR_TOLERANCE {
        return Err("interactive extrusion direction must be non-zero".to_string());
    }

    let mut profile_map = GMap::<StandardPayload>::new();
    let points = regular_polygon(point_count);
    let profile_dart = add_polygon(&mut profile_map, &points);
    let profile = Profile::new(&profile_map, profile_dart);
    let shape = extrude_profile(profile, extrusion)
        .map_err(|err| format!("failed to extrude interactive polygon: {err:?}"))?;

    let mut hints = VizHints::new();
    for (key, _) in shape.map().iter_faces() {
        hints.face(
            key,
            Style::default()
                .color("#4fc3a1")
                .opacity(0.78)
                .double_sided(true),
        );
    }

    Ok(ScriptResult::from_gmap_with_hints(shape.map(), &hints))
}

pub fn run() -> Result<ScriptResult, String> {
    build(5, Vector3::new(0.6, 0.2, 1.8))
}

fn regular_polygon(point_count: usize) -> Vec<Point3> {
    let phase = std::f64::consts::FRAC_PI_2;
    (0..point_count)
        .map(|i| {
            let angle = phase + std::f64::consts::TAU * i as f64 / point_count as f64;
            Point3::new(
                angle.cos() * POLYGON_RADIUS,
                angle.sin() * POLYGON_RADIUS,
                0.0,
            )
        })
        .collect()
}
