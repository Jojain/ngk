//! Extrudes a circular arc profile.
//!
//! The profile is a two-edge loop: a quarter-circle arc plus its chord. This
//! keeps the `Profile` closed while still exercising the sweep path for a
//! genuinely curved edge (`Curve::Circle` -> `Surface::Ruled`).

use nalgebra::Vector3;

use crate::builders::profiles::add_polyline;
use crate::geometry::{Circle, Curve, Line, Plane, Point3};
use crate::modeling::sweep::extrude_profile;
use crate::topology::StandardPayload;
use crate::topology::gmap::{Dart, GMap};
use crate::topology::profile::Profile;
use crate::viz::{ScriptResult, Style, VizHints};

const RADIUS: f64 = 2.0;
const HEIGHT: f64 = 1.5;

pub fn run() -> Result<ScriptResult, String> {
    let mut profile_map = GMap::<StandardPayload>::new();
    let arc_dart = add_arc_profile(&mut profile_map)?;

    let shape = extrude_profile(
        Profile::new(&profile_map, arc_dart),
        Vector3::new(0.0, 0.0, HEIGHT),
    )
    .map_err(|err| format!("arc extrusion failed: {err:?}"))?;
    let (g, arc_dart) = shape.into_map();

    let mut hints = VizHints::new();
    for (key, attr) in g.iter_faces() {
        let style = if attr.outer_loop == arc_dart {
            Style::default()
                .color("#7bd0ff")
                .label("extruded arc")
                .double_sided(true)
        } else {
            Style::default()
                .color("red")
                .label("arc chord side")
                .double_sided(true)
        };
        hints.face(key, style);
    }

    Ok(ScriptResult::from_gmap_with_hints(&g, &hints))
}

fn add_arc_profile(g: &mut GMap<StandardPayload>) -> Result<Dart, String> {
    let start = Point3::new(RADIUS, 0.0, 0.0);
    let end = Point3::new(0.0, RADIUS, 0.0);
    let arc = Curve::Circle(Circle::new(
        Plane::new(Point3::origin(), Vector3::x(), Vector3::z()),
        RADIUS,
    ));
    let chord = Curve::Line(Line::new(end, start));

    add_polyline(g, &[(start, end, arc), (end, start, chord)])
        .map_err(|err| format!("failed to build arc contour: {err:?}"))
}

#[cfg(test)]
mod tests {
    use super::run;

    #[test]
    fn cylinder_script_extrudes_arc_profile() {
        let result = run().expect("arc extrusion script should run");
        assert!(!result.scene.faces.is_empty());
        assert!(!result.scene.edges.is_empty());
        assert!(
            result
                .scene
                .faces
                .iter()
                .all(|face| !face.positions.is_empty())
        );
    }
}
