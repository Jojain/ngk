use std::collections::HashMap;

use nalgebra::Vector3;

use crate::builders::profiles::add_polygon;
use crate::geometry::{Curve2, Line2, Plane, Point2, Point3, Surface};
use crate::modeling::sweep::extrude_face;
use crate::topology::StandardPayload;
use crate::topology::attributes::FaceAttr;
use crate::topology::gmap::{Dart, GMap};
use crate::topology::profile::Profile;
use crate::topology::shape::{FaceShape, Shape};
use crate::topology::shape_keys::FaceKey;
use crate::viz::{ScriptResult, Style, VizHints};

const HEIGHT: f64 = 1.2;

pub fn run() -> Result<ScriptResult, String> {
    let face = build_source_face();
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

fn build_source_face() -> Shape<FaceShape, StandardPayload> {
    let mut g = GMap::<StandardPayload>::new();
    let surface = Surface::Plane(Plane::from_xy(Point3::origin(), Vector3::x(), Vector3::y()));

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

    let outer_loop = add_polygon(&mut g, &outer);
    let inner_loop = add_polygon(&mut g, &inner);

    let mut pcurves = loop_line_pcurves(&g, outer_loop, &outer);
    pcurves.extend(loop_line_pcurves(&g, inner_loop, &inner));

    let face_key = g.add_face(FaceAttr::with_pcurves(
        surface,
        (),
        outer_loop,
        vec![inner_loop],
        pcurves,
    ));

    Shape::new(g, face_key)
}

fn loop_line_pcurves(
    g: &GMap<StandardPayload>,
    loop_dart: Dart,
    points: &[Point3],
) -> HashMap<Dart, Curve2> {
    let darts = Profile::new(g, loop_dart)
        .darts()
        .step_by(2)
        .collect::<Vec<_>>();
    let uv = points
        .iter()
        .map(|point| Point2::new(point.x, point.y))
        .collect::<Vec<_>>();

    let mut pcurves = HashMap::with_capacity(darts.len());
    for i in 0..darts.len() {
        pcurves.insert(
            darts[i],
            Curve2::Line(Line2::new(uv[i], uv[(i + 1) % uv.len()])),
        );
    }
    pcurves
}

#[cfg(test)]
mod tests {
    use super::{build_source_face, run};
    use crate::modeling::sweep::extrude_face;
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
        let face = build_source_face();
        let solid = extrude_face(face, Vector3::new(0.0, 0.0, super::HEIGHT)).expect("extrude");
        let shell_dart = solid.solid().outer_shell().dart;

        assert!(
            Closed::new(Sheet::new(solid.map(), shell_dart)).is_some(),
            "extruded holed pentagon should produce a closed shell"
        );
    }
}
