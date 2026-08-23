//! Side-by-side comparison of whole-profile and single-vertex chamfers.

use std::collections::HashSet;

use crate::builders::chamfer::chamfer;
use crate::builders::faces::add_face;
use crate::builders::profiles::add_rectangle;
use crate::geometry::{Plane, Point3};
use crate::topology::StandardPayload;
use crate::topology::gmap::GMap;
use crate::viz::{ScriptResult, Style, VizHints};
use nalgebra::Vector3;

const X_SIZE: f64 = 2.4;
const Y_SIZE: f64 = 1.6;
const CHAMFER_DISTANCE: f64 = 0.45;
const GAP: f64 = 1.2;

/// Builds two planar rectangles: one with every profile vertex chamfered and
/// one with only its lower-right vertex chamfered.
pub fn build(distance: f64) -> Result<ScriptResult, String> {
    let mut g = GMap::<StandardPayload>::new();
    let whole_profile = add_rectangle(&mut g, Plane::xy(), X_SIZE, Y_SIZE)
        .map_err(|err| format!("failed to build whole-profile rectangle: {err:?}"))?;
    let original_edges = g.iter_edges().map(|(key, _)| key).collect::<HashSet<_>>();
    chamfer(&mut g, whole_profile, distance)
        .map_err(|err| format!("failed to chamfer whole rectangle profile: {err:?}"))?;
    let whole_profile_chamfers = g
        .iter_edges()
        .map(|(key, _)| key)
        .filter(|key| !original_edges.contains(key))
        .collect::<Vec<_>>();
    let whole_face = add_face(&mut g, whole_profile)
        .map_err(|err| format!("failed to fill whole-profile rectangle: {err:?}"))?;

    let vertex_plane = Plane::from_xy(
        Point3::new(X_SIZE + GAP, 0.0, 0.0),
        Vector3::x(),
        Vector3::y(),
    );
    let vertex_profile = add_rectangle(&mut g, vertex_plane, X_SIZE, Y_SIZE)
        .map_err(|err| format!("failed to build single-vertex rectangle: {err:?}"))?;
    let corner = g.profile_unchecked(vertex_profile).edges()[0].end().key();
    let edges_before_vertex_chamfer = g.iter_edges().map(|(key, _)| key).collect::<HashSet<_>>();
    chamfer(&mut g, corner, distance)
        .map_err(|err| format!("failed to chamfer one rectangle vertex: {err:?}"))?;
    let vertex_chamfer = g
        .iter_edges()
        .map(|(key, _)| key)
        .find(|key| !edges_before_vertex_chamfer.contains(key))
        .ok_or("single-vertex chamfer did not create an edge")?;
    let vertex_face = add_face(&mut g, vertex_profile)
        .map_err(|err| format!("failed to fill single-vertex rectangle: {err:?}"))?;

    let mut hints = VizHints::new();
    hints.face(
        whole_face,
        Style::default()
            .color("#68a5ff")
            .label("whole profile chamfer")
            .double_sided(true),
    );
    for edge in whole_profile_chamfers {
        hints.edge(
            edge,
            Style::default()
                .color("#ffb454")
                .label("profile chamfer edge")
                .width(9.0),
        );
    }
    hints.face(
        vertex_face,
        Style::default()
            .color("#76c893")
            .label("single vertex chamfer")
            .double_sided(true),
    );
    hints.edge(
        vertex_chamfer,
        Style::default()
            .color("#ffd166")
            .label("vertex chamfer edge")
            .width(9.0),
    );

    Ok(ScriptResult::from_gmap_with_hints(&g, &hints))
}

/// Builds the default whole-profile versus single-vertex comparison.
pub fn run() -> Result<ScriptResult, String> {
    build(CHAMFER_DISTANCE)
}
