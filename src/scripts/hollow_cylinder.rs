//! Hollow cylinder built by extruding one annular face.
//!
//! The source face is a planar circle with a concentric circular hole. The
//! sweep code then creates the top cap, bottom cap, outer wall, and inner wall
//! as one closed solid shell.

use nalgebra::Vector3;

use crate::geometry::Plane;
use crate::modeling::faces;
use crate::modeling::sweep::extrude_face;
use crate::topology::StandardPayload;
use crate::topology::shape::{FaceTag, Shape, SolidTag};
use crate::viz::{ScriptResult, Style, VizHints};

const OUTER_RADIUS: f64 = 1.0;
const INNER_RADIUS: f64 = 0.55;
const HEIGHT: f64 = 1.8;

pub fn run() -> Result<ScriptResult, String> {
    let solid = build_hollow_cylinder_solid()?;

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

pub fn build_hollow_cylinder_solid() -> Result<Shape<SolidTag, StandardPayload>, String> {
    let source = build_source_face()?;
    extrude_face(source, Vector3::new(0.0, 0.0, HEIGHT))
        .map_err(|err| format!("failed to extrude annular circle face: {err:?}"))
}

fn build_source_face() -> Result<Shape<FaceTag, StandardPayload>, String> {
    faces::annulus(Plane::xy(), OUTER_RADIUS, INNER_RADIUS)
        .map_err(|err| format!("failed to build annular circle face: {err:?}"))
}

#[cfg(test)]
mod tests {
    use super::{build_hollow_cylinder_solid, build_source_face, run};
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
        let solid = build_hollow_cylinder_solid().expect("build");
        let shell_dart = solid.solid().outer_shell().dart;
        assert!(
            Closed::new(Sheet::new(solid.map(), shell_dart)).is_some(),
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
