//! Three rounded solids side by side: a block whose top rim is rounded, a
//! block rounded on every edge, and an L whose top rim is rounded through its
//! reflex corner.

use std::collections::HashSet;

use nalgebra::Vector3;

use crate::builders::faces::add_face;
use crate::builders::fillet::fillet;
use crate::builders::profiles::add_polyline;
use crate::builders::solids::add_extruded_face;
use crate::geometry::{Point3, Surface};
use crate::model::Model;
use crate::topology::StandardPayload;
use crate::topology::shape_keys::{FaceKey, SolidKey};
use crate::viz::{ScriptResult, Style, VizHints};

const X_SIZE: f64 = 2.4;
const Y_SIZE: f64 = 1.6;
const Z_SIZE: f64 = 1.8;
const FILLET_RADIUS: f64 = 0.3;
const GAP: f64 = 1.2;

/// Builds the three rounded solids from a live fillet radius.
pub fn build(radius: f64) -> Result<ScriptResult, String> {
    let mut g = Model::<StandardPayload>::new();

    let rim = extrude(&mut g, &rectangle(Point3::origin()), "top-rim block")?;
    let top = face_at_height(&g, rim, Z_SIZE).ok_or("the top-rim block has no top face")?;
    let rim_rounds = rounds(&mut g, rim, top.into(), radius, "the block's top rim")?;

    let every = extrude(
        &mut g,
        &rectangle(Point3::new(X_SIZE + GAP, 0.0, 0.0)),
        "rounded block",
    )?;
    let edges = g
        .solid_unchecked(every)
        .edges()
        .into_iter()
        .map(|edge| edge.key())
        .collect::<Vec<_>>();
    let every_rounds = rounds(&mut g, every, edges.into(), radius, "every block edge")?;

    let origin = Point3::new(2.0 * (X_SIZE + GAP), 0.0, 0.0);
    let l_corners = [
        (0.0, 0.0),
        (X_SIZE, 0.0),
        (X_SIZE, Y_SIZE / 2.0),
        (X_SIZE / 2.0, Y_SIZE / 2.0),
        (X_SIZE / 2.0, Y_SIZE),
        (0.0, Y_SIZE),
    ]
    .map(|(x, y)| origin + Vector3::new(x, y, 0.0));
    let l = extrude(&mut g, &l_corners, "L")?;
    let l_top = face_at_height(&g, l, Z_SIZE).ok_or("the L has no top face")?;
    let l_rounds = rounds(&mut g, l, l_top.into(), radius, "the L's top rim")?;

    let mut hints = VizHints::new();
    for (solid, rounded) in [(rim, &rim_rounds), (every, &every_rounds), (l, &l_rounds)] {
        for face in g.solid_unchecked(solid).faces() {
            let style = if !rounded.contains(&face.key()) {
                Style::default().color("#68a5ff").label("original face")
            } else if matches!(face.surface(), Surface::Sphere(_)) {
                Style::default().color("#ffd166").label("corner ball")
            } else {
                Style::default().color("#ffb454").label("round")
            };
            hints.face(face.key(), style.double_sided(true));
        }
    }
    Ok(ScriptResult::from_model_with_hints(&g, &hints))
}

/// Builds the three rounded solids at the default radius.
pub fn run() -> Result<ScriptResult, String> {
    build(FILLET_RADIUS)
}

fn rectangle(origin: Point3) -> [Point3; 4] {
    [
        origin,
        origin + Vector3::new(X_SIZE, 0.0, 0.0),
        origin + Vector3::new(X_SIZE, Y_SIZE, 0.0),
        origin + Vector3::new(0.0, Y_SIZE, 0.0),
    ]
}

/// Extrudes the closed polygon through `corners` up by the shared height.
fn extrude(
    g: &mut Model<StandardPayload>,
    corners: &[Point3],
    label: &str,
) -> Result<SolidKey, String> {
    let mut closed = corners.to_vec();
    closed.push(corners[0]);
    let profile = add_polyline(g, &closed)
        .map_err(|err| format!("failed to build the {label} profile: {err:?}"))?;
    let face =
        add_face(g, profile).map_err(|err| format!("failed to fill the {label} base: {err:?}"))?;
    add_extruded_face(g, face, Vector3::new(0.0, 0.0, Z_SIZE))
        .map(|extrusion| extrusion.solid)
        .map_err(|err| format!("failed to extrude the {label}: {err:?}"))
}

fn face_at_height(g: &Model<StandardPayload>, solid: SolidKey, z: f64) -> Option<FaceKey> {
    g.solid_unchecked(solid)
        .faces()
        .into_iter()
        .find(|face| {
            face.vertices()
                .iter()
                .all(|vertex| (vertex.point().z - z).abs() < 1.0e-9)
        })
        .map(|face| face.key())
}

/// Rounds `target` on `solid` and returns the faces the fillet added.
fn rounds(
    g: &mut Model<StandardPayload>,
    solid: SolidKey,
    target: crate::builders::blend::BlendTarget,
    radius: f64,
    label: &str,
) -> Result<HashSet<FaceKey>, String> {
    let before = g
        .solid_unchecked(solid)
        .faces()
        .iter()
        .map(|face| face.key())
        .collect::<HashSet<_>>();
    fillet(g, target, radius).map_err(|err| format!("failed to round {label}: {err}"))?;
    Ok(g.solid_unchecked(solid)
        .faces()
        .iter()
        .map(|face| face.key())
        .filter(|face| !before.contains(face))
        .collect())
}
