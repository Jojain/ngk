use nalgebra::Vector3;

use crate::geometry::Plane;
use crate::modeling::profiles;
use crate::modeling::sweep::extrude_profile;
use crate::viz::ScriptResult;

pub fn run() -> Result<ScriptResult, String> {
    let profile = profiles::rectangle(Plane::xy(), 1.0, 1.0)
        .map_err(|err| format!("failed to build rectangle profile: {err:?}"))?;
    let shape = extrude_profile(profile.profile(), Vector3::new(0.0, 0.0, 1.0))
        .map_err(|err| format!("failed to extrude rectangle: {err:?}"))?;
    Ok(ScriptResult::from_gmap(shape.map()))
}
