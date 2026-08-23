//! Rectangular face with one corner replaced by a straight chamfer.

use std::collections::HashSet;

use crate::builders::chamfer::chamfer;
use crate::builders::faces::add_face;
use crate::builders::profiles::add_rectangle;
use crate::geometry::Plane;
use crate::topology::StandardPayload;
use crate::topology::gmap::GMap;
use crate::viz::{ScriptResult, Style, VizHints};

const X_SIZE: f64 = 2.4;
const Y_SIZE: f64 = 1.6;
const CHAMFER_DISTANCE: f64 = 0.45;

/// Builds a rectangular planar face with its lower-right corner chamfered.
pub fn run() -> Result<ScriptResult, String> {
    let mut g = GMap::<StandardPayload>::new();
    let profile = add_rectangle(&mut g, Plane::xy(), X_SIZE, Y_SIZE)
        .map_err(|err| format!("failed to build rectangle profile: {err:?}"))?;
    let corner = g.profile_unchecked(profile).edges()[0].end().dart;
    let original_edges = g.iter_edges().map(|(key, _)| key).collect::<HashSet<_>>();
    chamfer(&mut g, corner, CHAMFER_DISTANCE)
        .map_err(|err| format!("failed to chamfer rectangle corner: {err:?}"))?;
    let chamfer_edge = g
        .iter_edges()
        .map(|(key, _)| key)
        .find(|key| !original_edges.contains(key))
        .ok_or("chamfer did not create an edge")?;
    let face = add_face(&mut g, profile)
        .map_err(|err| format!("failed to fill chamfered rectangle: {err:?}"))?;

    let mut hints = VizHints::new();
    hints.face(
        face,
        Style::default()
            .color("#68a5ff")
            .label("chamfered rectangle")
            .double_sided(true),
    );
    hints.edge(
        chamfer_edge,
        Style::default()
            .color("#ffb454")
            .label("chamfer edge")
            .width(9.0),
    );

    Ok(ScriptResult::from_gmap_with_hints(&g, &hints))
}
