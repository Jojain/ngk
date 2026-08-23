//! Side-by-side comparison of whole-profile and single-vertex solid chamfers.

use std::collections::HashSet;

use nalgebra::Vector3;

use crate::builders::chamfer::chamfer;
use crate::builders::faces::add_face;
use crate::builders::profiles::add_rectangle;
use crate::builders::solids::add_extruded_face;
use crate::geometry::{Plane, Point3};
use crate::topology::StandardPayload;
use crate::topology::gmap::GMap;
use crate::viz::{ScriptResult, Style, VizHints};

const X_SIZE: f64 = 2.4;
const Y_SIZE: f64 = 1.6;
const Z_SIZE: f64 = 1.8;
const CHAMFER_DISTANCE: f64 = 0.35;
const GAP: f64 = 1.2;

/// Builds two already-solid blocks: one with its complete top profile
/// chamfered and one with only its lower-front-left vertex chamfered.
pub fn build(distance: f64) -> Result<ScriptResult, String> {
    let mut g = GMap::<StandardPayload>::new();

    let profile_solid = add_block(&mut g, Point3::origin(), "whole-profile chamfer block")?;
    let profile = g
        .solid_unchecked(profile_solid)
        .faces()
        .into_iter()
        .find(|face| {
            face.vertices().iter().all(|vertex| {
                vertex
                    .point()
                    .is_some_and(|point| (point.z - Z_SIZE).abs() < 1.0e-9)
            })
        })
        .ok_or("whole-profile block did not expose its top face")?
        .outer_loop()
        .key();
    let profile_faces_before = g.iter_faces().map(|(key, _)| key).collect::<HashSet<_>>();
    chamfer(&mut g, profile, distance)
        .map_err(|err| format!("failed to chamfer the complete top profile: {err:?}"))?;
    let profile_new_faces = g
        .solid_unchecked(profile_solid)
        .faces()
        .into_iter()
        .map(|face| face.key())
        .filter(|face| !profile_faces_before.contains(face))
        .collect::<HashSet<_>>();

    let vertex_origin = Point3::new(X_SIZE + GAP, 0.0, 0.0);
    let vertex_solid = add_block(&mut g, vertex_origin, "single-vertex chamfer block")?;
    let vertex = g
        .solid_unchecked(vertex_solid)
        .vertices()
        .into_iter()
        .find(|vertex| {
            vertex
                .point()
                .is_some_and(|point| (*point - vertex_origin).norm() <= 1.0e-9)
        })
        .ok_or("single-vertex block did not expose its lower-front-left vertex")?
        .key();
    let vertex_faces_before = g.iter_faces().map(|(key, _)| key).collect::<HashSet<_>>();
    chamfer(&mut g, vertex, distance)
        .map_err(|err| format!("failed to chamfer one solid vertex: {err:?}"))?;
    let vertex_new_faces = g
        .solid_unchecked(vertex_solid)
        .faces()
        .into_iter()
        .map(|face| face.key())
        .filter(|face| !vertex_faces_before.contains(face))
        .collect::<HashSet<_>>();

    let mut hints = VizHints::new();
    for face in g.solid_unchecked(profile_solid).faces() {
        let is_top_cap = face.vertices().iter().all(|vertex| {
            vertex
                .point()
                .is_some_and(|point| (point.z - Z_SIZE).abs() < 1.0e-9)
        });
        let style = if profile_new_faces.contains(&face.key()) && !is_top_cap {
            Style::default()
                .color("#ffb454")
                .label("whole top-profile chamfer")
        } else {
            Style::default().color("#68a5ff").label("complete profile")
        };
        hints.face(face.key(), style.double_sided(true));
    }
    for face in g.solid_unchecked(vertex_solid).faces() {
        let style = if vertex_new_faces.contains(&face.key()) {
            Style::default()
                .color("#ffd166")
                .label("single solid-vertex chamfer")
        } else {
            Style::default().color("#76c893").label("single vertex")
        };
        hints.face(face.key(), style.double_sided(true));
    }

    Ok(ScriptResult::from_gmap_with_hints(&g, &hints))
}

/// Adds one translated rectangular prism to the shared comparison map.
fn add_block(
    g: &mut GMap<StandardPayload>,
    origin: Point3,
    label: &str,
) -> Result<crate::topology::shape_keys::SolidKey, String> {
    let profile = add_rectangle(
        g,
        Plane::from_xy(origin, Vector3::x(), Vector3::y()),
        X_SIZE,
        Y_SIZE,
    )
    .map_err(|err| format!("failed to build {label} base profile: {err:?}"))?;
    let face =
        add_face(g, profile).map_err(|err| format!("failed to fill {label} base face: {err:?}"))?;
    add_extruded_face(g, face, Vector3::new(0.0, 0.0, Z_SIZE))
        .map_err(|err| format!("failed to extrude {label}: {err:?}"))
}

/// Builds the default whole-profile versus single-vertex solid comparison.
pub fn run() -> Result<ScriptResult, String> {
    build(CHAMFER_DISTANCE)
}
