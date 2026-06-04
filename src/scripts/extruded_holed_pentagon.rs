use nalgebra::Vector3;

use crate::geometry::{Plane, Point3};
use crate::modeling::faces;
use crate::modeling::sweep::extrude_face;
use crate::topology::StandardPayload;
use crate::topology::shape::{FaceTag, Shape};
use crate::viz::{ScriptResult, Style, VizHints};

const HEIGHT: f64 = 1.2;

pub fn run() -> Result<ScriptResult, String> {
    let face = build_source_face()?;
    let solid = extrude_face(face, Vector3::new(0.0, 0.0, HEIGHT))
        .map_err(|err| format!("failed to extrude holed pentagon face: {err:?}"))?;

    let mut hints = VizHints::new();
    for (key, _) in solid.map().iter_faces() {
        hints.face(
            key,
            Style::default()
                .color("#78a6ff")
                .label("extruded holed pentagon")
                .double_sided(true),
        );
    }

    Ok(ScriptResult::from_gmap_with_hints(solid.map(), &hints))
}

fn build_source_face() -> Result<Shape<FaceTag, StandardPayload>, String> {
    let outer = vec![
        Point3::new(0.0, 1.35, 0.0),
        Point3::new(1.28, 0.42, 0.0),
        Point3::new(0.8, -1.1, 0.0),
        Point3::new(-0.8, -1.1, 0.0),
        Point3::new(-1.28, 0.42, 0.0),
    ];
    let inner = vec![
        Point3::new(-0.38, -0.38, 0.0),
        Point3::new(-0.38, 0.38, 0.0),
        Point3::new(0.38, 0.38, 0.0),
        Point3::new(0.38, -0.38, 0.0),
    ];
    let holes: [&[Point3]; 1] = [&inner];
    faces::polygon_with_holes(Plane::xy(), &outer, &holes)
        .map_err(|err| format!("failed to build holed pentagon face: {err:?}"))
}

#[cfg(test)]
mod tests {
    use super::{build_source_face, run};
    use crate::modeling::sweep::extrude_face;
    use crate::tessellate::{TessellateOpts, tessellate_face};
    use crate::topology::closed::Closed;
    use crate::topology::sheet::Sheet;
    use nalgebra::Vector3;

    #[test]
    fn extruded_holed_pentagon_script_runs() {
        let result = run().expect("holed pentagon extrusion script should run");
        assert_eq!(result.scene.faces.len(), 11);
        assert!(
            result
                .scene
                .faces
                .iter()
                .all(|face| { !face.positions.is_empty() && !face.indices.is_empty() })
        );
    }

    #[test]
    fn extruded_holed_pentagon_is_closed_solid_shell() {
        let face = build_source_face().expect("source face");
        let solid = extrude_face(face, Vector3::new(0.0, 0.0, super::HEIGHT)).expect("extrude");
        let shell_dart = solid.solid().outer_shell().dart;

        assert!(
            Closed::new(Sheet::new(solid.map(), shell_dart)).is_some(),
            "extruded holed pentagon should produce a closed shell"
        );
    }

    #[test]
    fn source_face_tessellation_keeps_square_hole_empty() {
        let face = build_source_face().expect("source face");
        let mesh = tessellate_face(&face.face(), TessellateOpts::default())
            .expect("source face should tessellate");

        for triangle in mesh.indices.chunks_exact(3) {
            let a = mesh.positions[triangle[0] as usize];
            let b = mesh.positions[triangle[1] as usize];
            let c = mesh.positions[triangle[2] as usize];
            let centroid = (a.coords + b.coords + c.coords) / 3.0;

            assert!(
                centroid.x.abs() >= 0.38 || centroid.y.abs() >= 0.38,
                "triangle centroid should not be inside the square hole: {centroid:?}"
            );
        }
    }
}
