//! Hollow cylinder built by extruding one annular face.
//!
//! The source face is a planar circle with a concentric circular hole.  The
//! sweep code then creates the top cap, bottom cap, outer wall, and inner wall
//! as one closed solid shell.

use std::collections::HashMap;

use nalgebra::Vector3;

use crate::builders::profiles::{add_polyline, profile_pcurves};
use crate::geometry::{Circle, Curve, Curve2, Plane, Point3, Surface};
use crate::modeling::sweep::extrude_face;
use crate::topology::StandardPayload;
use crate::topology::attributes::FaceAttr;
use crate::topology::gmap::{Dart, GMap};
use crate::topology::profile::Profile;
use crate::topology::shape::{FaceShape, Shape};
use crate::topology::shape_keys::FaceKey;
use crate::viz::{ScriptResult, Style, VizHints};

const OUTER_RADIUS: f64 = 1.0;
const INNER_RADIUS: f64 = 0.55;
const HEIGHT: f64 = 1.8;

type HollowCylinderBuild = (GMap<StandardPayload>, Vec<(FaceKey, Style)>, Dart);

pub fn run() -> Result<ScriptResult, String> {
    let source = build_source_face()?;
    let solid = extrude_face(source, Vector3::new(0.0, 0.0, HEIGHT))
        .map_err(|err| format!("failed to extrude annular circle face: {err:?}"))?;

    let mut hints = VizHints::new();
    for (key, _) in solid.map().iter_faces() {
        hints.face(
            key,
            Style::default()
                .color("#62a7ff")
                .label("extruded circular annulus")
                .double_sided(true),
        );
    }

    Ok(ScriptResult::from_gmap_with_hints(solid.map(), &hints))
}

/// Returns a solid boundary map built through the same face-extrusion path used
/// by the holed pentagon example, plus one dart on the resulting closed shell.
pub fn build_hollow_cylinder_gmap() -> Result<HollowCylinderBuild, String> {
    let source = build_source_face()?;
    let solid = extrude_face(source, Vector3::new(0.0, 0.0, HEIGHT))
        .map_err(|err| format!("failed to extrude annular circle face: {err:?}"))?;
    let shell_dart = solid.solid().outer_shell().dart;
    let (g, _) = solid.into_map();

    let styles = g
        .iter_faces()
        .map(|(key, _)| {
            (
                key,
                Style::default()
                    .color("#62a7ff")
                    .label("extruded circular annulus")
                    .double_sided(true),
            )
        })
        .collect();

    Ok((g, styles, shell_dart))
}

fn build_source_face() -> Result<Shape<FaceShape, StandardPayload>, String> {
    let mut g = GMap::<StandardPayload>::new();
    let plane = Plane::from_xy(Point3::origin(), Vector3::x(), Vector3::y());

    let outer_loop = add_circle_loop(&mut g, OUTER_RADIUS, true)?;
    let inner_loop = add_circle_loop(&mut g, INNER_RADIUS, false)?;

    let mut pcurves = loop_pcurves(&g, outer_loop, &plane)?;
    pcurves.extend(loop_pcurves(&g, inner_loop, &plane)?);

    let face_key = g.add_face(FaceAttr::with_pcurves(
        Surface::Plane(plane),
        (),
        outer_loop,
        vec![inner_loop],
        pcurves,
    ));

    Ok(Shape::new(g, face_key))
}

fn add_circle_loop(
    g: &mut GMap<StandardPayload>,
    radius: f64,
    counter_clockwise: bool,
) -> Result<Dart, String> {
    let start = Point3::new(radius, 0.0, 0.0);
    let normal = if counter_clockwise {
        Vector3::z()
    } else {
        -Vector3::z()
    };
    let circle = Circle::new(Plane::new(Point3::origin(), Vector3::x(), normal), radius);
    let segments = [(start, start, Curve::Circle(circle))];

    add_polyline(g, &segments).map_err(|err| format!("failed to build circular loop: {err:?}"))
}

fn loop_pcurves(
    g: &GMap<StandardPayload>,
    loop_dart: Dart,
    plane: &Plane,
) -> Result<HashMap<Dart, Curve2>, String> {
    profile_pcurves(g, &Profile::new(g, loop_dart), plane)
        .map_err(|err| format!("failed to project circle loop pcurves: {err:?}"))
}

#[cfg(test)]
mod tests {
    use super::{build_hollow_cylinder_gmap, build_source_face, run};
    use crate::modeling::sweep::extrude_face;
    use crate::tessellate::{TessellateOpts, tessellate_face};
    use crate::topology::closed::Closed;
    use crate::topology::sheet::Sheet;
    use nalgebra::Vector3;

    #[test]
    fn hollow_cylinder_script_emits_brep_scene() {
        let result = run().expect("hollow cylinder script should run");
        assert_eq!(result.scene.faces.len(), 4);
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

    #[test]
    fn source_face_tessellation_keeps_circular_hole_empty() {
        let face = build_source_face().expect("source face");
        let mesh = tessellate_face(face.map(), face.key(), TessellateOpts::default())
            .expect("source annulus face should tessellate");

        for triangle in mesh.indices.chunks_exact(3) {
            let a = mesh.positions[triangle[0] as usize];
            let b = mesh.positions[triangle[1] as usize];
            let c = mesh.positions[triangle[2] as usize];
            let centroid = (a.coords + b.coords + c.coords) / 3.0;

            assert!(
                centroid.xy().norm() >= super::INNER_RADIUS * 0.9,
                "triangle centroid should not be inside the circular hole: {centroid:?}"
            );
        }
    }

    #[test]
    fn hollow_cylinder_uses_face_extrusion_path() {
        let source = build_source_face().expect("source face");
        let solid =
            extrude_face(source, Vector3::new(0.0, 0.0, super::HEIGHT)).expect("extrude annulus");
        let shell_dart = solid.solid().outer_shell().dart;

        assert!(
            Closed::new(Sheet::new(solid.map(), shell_dart)).is_some(),
            "extruding the annular source face should produce a closed shell"
        );
    }
}
