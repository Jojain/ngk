use nalgebra::Vector3;

use crate::builders::profiles::add_rectangle;
use crate::geometry::Plane;
use crate::modeling::sweep::extrude_profile;
use crate::topology::gmap::GMap;
use crate::topology::profile::Profile;
use crate::viz::ScriptResult;

pub fn run() -> Result<ScriptResult, String> {
    let mut profile_map = GMap::new();
    let rectangle_dart = add_rectangle(&mut profile_map, Plane::xy(), 1.0, 1.0)
        .map_err(|err| format!("failed to add rectangle profile: {err:?}"))?;

    let profile = Profile::new(&profile_map, rectangle_dart);
    let shape = extrude_profile(profile, Vector3::new(0.0, 0.0, 1.0))
        .map_err(|err| format!("failed to extrude rectangle: {err:?}"))?;
    Ok(ScriptResult::from_gmap(shape.map()))
}
