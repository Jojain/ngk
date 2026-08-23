//! Block whose already-built solid boundary has one edge chamfered in 3D.

use std::collections::HashSet;

use crate::builders::chamfer::chamfer;
use crate::modeling::solids::block;
use crate::viz::{ScriptResult, Style, VizHints};

const X_SIZE: f64 = 2.4;
const Y_SIZE: f64 = 1.6;
const Z_SIZE: f64 = 1.8;
const CHAMFER_DISTANCE: f64 = 0.45;

/// Builds a block with the vertical edge at its lower-right corner chamfered by
/// `distance`.
pub fn build(distance: f64) -> Result<ScriptResult, String> {
    let shape =
        block(X_SIZE, Y_SIZE, Z_SIZE).map_err(|err| format!("failed to build block: {err:?}"))?;
    let edge = shape
        .solid()
        .edges()
        .into_iter()
        .find(|edge| {
            let start = *edge.start().point().expect("block edge has a start point");
            let end = *edge.end().point().expect("block edge has an end point");
            (start.x - X_SIZE).abs() < 1.0e-9
                && (end.x - X_SIZE).abs() < 1.0e-9
                && start.y.abs() < 1.0e-9
                && end.y.abs() < 1.0e-9
                && (start.z - end.z).abs() > Z_SIZE - 1.0e-9
        })
        .map(|edge| edge.key())
        .ok_or("block did not expose the requested vertical edge")?;
    let (mut g, _) = shape.into_map();
    let original_faces = g.iter_faces().map(|(key, _)| key).collect::<HashSet<_>>();
    chamfer(&mut g, edge, distance)
        .map_err(|err| format!("failed to chamfer block edge: {err:?}"))?;
    let chamfer_face = g
        .iter_faces()
        .map(|(key, _)| key)
        .find(|key| !original_faces.contains(key))
        .ok_or("3D chamfer did not create a face")?;

    let mut hints = VizHints::new();
    for (face, _) in g.iter_faces() {
        hints.face(
            face,
            Style::default()
                .color("#68a5ff")
                .label("chamfered block")
                .double_sided(true),
        );
    }
    hints.face(
        chamfer_face,
        Style::default()
            .color("#ffb454")
            .label("chamfer face")
            .double_sided(true),
    );

    Ok(ScriptResult::from_gmap_with_hints(&g, &hints))
}

/// Builds the default chamfered-block experiment.
pub fn run() -> Result<ScriptResult, String> {
    build(CHAMFER_DISTANCE)
}
