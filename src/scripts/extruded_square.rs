use nalgebra::Vector3;

use crate::modeling::sweep::extrude;
use crate::topology::profile::Profile;
use crate::viz::VizHints;
use crate::{
    geometry::Point3, modeling::profiles::add_square, topology::gmap::GMap, viz::ScriptResult,
};

pub fn run() -> Result<ScriptResult, String> {
    let mut profile_map = GMap::new();
    let square_dart = add_square(
        &mut profile_map,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
    )
    .map_err(|err| format!("failed to add square profile: {err:?}"))?;

    let profile = Profile::new(&profile_map, square_dart);
    let shape = extrude(profile, Vector3::new(0.0, 0.0, 1.0))
        .map_err(|err| format!("failed to extrude square: {err:?}"))?;
    let mut hints = VizHints::new();
    Ok(ScriptResult::from_gmap_with_hints(&shape.map(), &VizHints::new()))
}
