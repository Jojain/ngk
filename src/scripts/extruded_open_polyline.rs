use nalgebra::Vector3;

use crate::geometry::Point3;
use crate::modeling::profiles;
use crate::modeling::sweep::extrude_profile;
use crate::viz::{ScriptResult, VizHints};

pub fn run() -> Result<ScriptResult, String> {
    let points = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 0.75, 0.0),
        Point3::new(1.75, 0.75, 0.0),
    ];
    let profile = profiles::polyline(&points)
        .map_err(|err| format!("failed to build open polyline profile: {err:?}"))?;
    let shape = extrude_profile(profile.profile(), Vector3::new(0.0, 0.0, 1.0))
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
