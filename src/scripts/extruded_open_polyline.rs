use nalgebra::Vector3;

use crate::geometry::{Curve, Line, Point3};
use crate::modeling::profiles::add_polyline;
use crate::modeling::sweep::extrude;
use crate::topology::StandardPayload;
use crate::topology::gmap::GMap;
use crate::topology::profile::Profile;
use crate::viz::{ScriptResult, VizHints};

pub fn run() -> Result<ScriptResult, String> {
    let mut profile_map = GMap::<StandardPayload>::new();
    let points = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 0.75, 0.0),
        Point3::new(1.75, 0.75, 0.0),
    ];
    let segments = [
        (
            points[0],
            points[1],
            Curve::Line(Line::new(points[0], points[1])),
        ),
        (
            points[1],
            points[2],
            Curve::Line(Line::new(points[1], points[2])),
        ),
        (
            points[2],
            points[3],
            Curve::Line(Line::new(points[2], points[3])),
        ),
    ];
    let polyline_dart = add_polyline(&mut profile_map, &segments)
        .map_err(|err| format!("failed to add open polyline profile: {err:?}"))?;

    let profile = Profile::new(&profile_map, polyline_dart);
    let shape = extrude(profile, Vector3::new(0.0, 0.0, 1.0))
        .map_err(|err| format!("failed to extrude open polyline: {err:?}"))?;

    Ok(ScriptResult::from_gmap_with_hints(
        shape.map(),
        &VizHints::new(),
    ))
}

#[cfg(test)]
mod tests {
    use super::run;

    #[test]
    fn extruded_open_polyline_script_runs() {
        let result = run().expect("open polyline extrusion script should run");
        assert!(!result.scene.faces.is_empty());
        assert!(!result.scene.darts.is_empty());
    }
}
