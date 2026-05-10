use nalgebra::Vector3;
use radians::Rad64;

use crate::builders::profiles::{add_face, add_polygon};
use crate::geometry::axis::Axis3;
use crate::geometry::{Curve, Point3};
use crate::modeling::revolve::revolve_face;
use crate::topology::StandardPayload;
use crate::topology::gmap::GMap;
use crate::topology::shape::{FaceShape, Shape};
use crate::viz::{ScriptResult, Style, VizHints};

pub fn build(angle: Rad64) -> Result<ScriptResult, String> {
    let face = triangle_face()?;
    let shape = revolve_face(face, Axis3::new(Point3::origin(), Vector3::z()), angle)
        .map_err(|err| format!("failed to revolve triangle face: {err:?}"))?;

    let mut hints = VizHints::new();
    for (key, attr) in shape.map().iter_edges() {
        let color = match attr.curve {
            Curve::Circle(_) => "#ffb454",
            _ => "#76d7ea",
        };
        hints.edge(key, Style::default().color(color).width(5.0));
    }
    for (key, _) in shape.map().iter_faces() {
        hints.face(
            key,
            Style::default()
                .color("#45b8ac")
                .opacity(0.34)
                .double_sided(true),
        );
    }

    Ok(ScriptResult::from_gmap_with_hints(shape.map(), &hints))
}

pub fn run() -> Result<ScriptResult, String> {
    build(Rad64::QUARTER_TURN)
}

fn triangle_profile() -> [Point3; 3] {
    [
        Point3::new(0.75, 0.0, -0.85),
        Point3::new(1.85, 0.0, -0.05),
        Point3::new(0.85, 0.0, 0.9),
    ]
}

fn triangle_face() -> Result<Shape<FaceShape>, String> {
    let mut g = GMap::<StandardPayload>::new();
    let points = triangle_profile();

    let loop_dart = add_polygon(&mut g, &points);
    let face_key = add_face(&mut g, loop_dart)
        .map_err(|err| format!("failed to add triangle face: {err:?}"))?;
    Ok(Shape::new(g, face_key))
}

#[cfg(test)]
mod tests {
    use super::{build, run};
    use radians::Rad64;

    #[test]
    fn revolved_triangle_script_runs() {
        let result = run().expect("default revolved triangle should run");
        assert!(!result.scene.edges.is_empty());
        assert!(result.scene.edges.len() >= 6);
    }

    #[test]
    fn revolved_triangle_accepts_live_angle() {
        let result = build(Rad64::HALF_TURN).expect("parameterized revolve should run");
        assert!(!result.scene.edges.is_empty());
        assert!(result.gmap.is_some());
    }
}
