//! Extrudes a circular arc profile.
//!
//! The profile is a two-edge loop: a quarter-circle arc plus a closing line. This
//! keeps the `Profile` closed while still exercising the sweep path for a
//! genuinely curved edge (`Curve::Circle` -> `Surface::Ruled`).

use nalgebra::Vector3;

use crate::geometry::Plane;
use crate::modeling::edges;
use crate::modeling::sweep::extrude_profile;
use crate::viz::{ScriptResult, Style, VizHints};

const RADIUS: f64 = 2.0;
const HEIGHT: f64 = 1.5;

pub fn run() -> Result<ScriptResult, String> {
    let arc = edges::arc(Plane::xy(), RADIUS, 0.0, std::f64::consts::FRAC_PI_2)
        .map_err(|err| format!("failed to build arc edge: {err:?}"))?;
    let start = *arc.edge().start().point().ok_or("arc start is missing")?;
    let end = *arc.edge().end().point().ok_or("arc end is missing")?;
    let closing_edge =
        edges::line(end, start).map_err(|err| format!("failed to build closing edge: {err:?}"))?;
    let mut profile = arc.into_profile();
    profile
        .add(&closing_edge)
        .map_err(|err| format!("failed to close arc profile with line: {err:?}"))?;
    let shape = extrude_profile(profile.profile(), Vector3::new(0.0, 0.0, HEIGHT))
        .map_err(|err| format!("arc extrusion failed: {err:?}"))?;
    let (g, sheet_key) = shape.into_map();
    let arc_dart = g
        .sheet_attr(sheet_key)
        .expect("extruded sheet must exist")
        .dart;

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
                .label("arc closing side")
                .double_sided(true)
        };
        hints.face(key, style);
    }

    Ok(ScriptResult::from_gmap_with_hints(&g, &hints))
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
