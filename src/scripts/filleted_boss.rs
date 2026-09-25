//! Three rounded solids with curved edges, side by side: a cylindrical boss on
//! a block rounded at its foot and its top, a boss leaning into its block
//! rounded along the ellipse where they meet, and a slot whose rim of lines
//! and arcs is rounded all the way round.
//!
//! The first is rounded by tori, the second by a skinned NURBS round, and the
//! third by cylinders and tori joined where the rim turns from straight to
//! curved.

use std::f64::consts::PI;

use nalgebra::Vector3;
use radians::Rad64;

use crate::builders::edges::{add_arc, add_line};
use crate::builders::faces::add_face;
use crate::builders::fillet::fillet;
use crate::builders::profiles::add_profile_from_edges;
use crate::builders::solids::add_extruded_face;
use crate::geometry::{Curve, Frame, LINEAR_TOLERANCE, Plane, Point3, Surface};
use crate::model::Model;
use crate::modeling::solids::{block_at, cylinder_at, fuse};
use crate::topology::edge::Edge;
use crate::topology::shape_keys::{EdgeKey, SolidKey};
use crate::topology::{ModelEditError, StandardPayload};
use crate::viz::{ScriptResult, Style, VizHints};

const BLOCK: [f64; 3] = [4.0, 4.0, 2.0];
const BOSS_RADIUS: f64 = 1.0;
const FILLET_RADIUS: f64 = 0.25;
const GAP: f64 = 2.0;

/// Builds the three rounded solids from a live fillet radius.
pub fn build(radius: f64) -> Result<ScriptResult, String> {
    let mut scene = Model::<StandardPayload>::new();
    let mut solids = Vec::with_capacity(3);

    let (mut g, solid) = boss(Point3::origin())?;
    let rims = closed_edges(&g, solid);
    round(&mut g, rims, radius, "the boss's foot and top")?;
    solids.push((g, solid));

    let (mut g, solid) = oblique_boss(Point3::new(BLOCK[0] + GAP, 0.0, 0.0))?;
    let rims = closed_edges(&g, solid)
        .into_iter()
        .filter(|&edge| starts_at_height(&g, edge, BLOCK[2]))
        .collect();
    round(&mut g, rims, radius, "the leaning boss's foot")?;
    solids.push((g, solid));

    let (mut g, solid) = slot(Point3::new(2.0 * (BLOCK[0] + GAP) + 1.0, 2.0, 0.0))?;
    let arc = g
        .solid_unchecked(solid)
        .edges()
        .into_iter()
        .find(|edge| {
            matches!(edge.curve(), Curve::Circle(_)) && starts_at_height(&g, edge.key(), BLOCK[2])
        })
        .ok_or("the slot has no arc on its rim")?
        .key();
    // One arc of the rim is named; the round carries on round the whole rim.
    round(&mut g, vec![arc], radius, "the slot's rim")?;
    solids.push((g, solid));

    scene
        .transaction(|edit| {
            for (g, solid) in &solids {
                edit.merge(g.solid_unchecked(*solid));
            }
            Ok::<(), ModelEditError>(())
        })
        .map_err(|err| format!("failed to gather the rounded solids: {err:?}"))?;

    let mut hints = VizHints::new();
    for (key, attr) in scene.iter_faces() {
        let style = match &attr.surface {
            Surface::Torus(_) => Style::default().color("#ffb454").label("round (torus)"),
            Surface::Nurbs(_) => Style::default().color("#ff7b72").label("round (skinned)"),
            Surface::Cylinder(cylinder) if (cylinder.radius - radius).abs() < LINEAR_TOLERANCE => {
                Style::default().color("#ffd166").label("round (cylinder)")
            }
            _ => Style::default().color("#68a5ff").label("original face"),
        };
        hints.face(key, style.double_sided(true));
    }
    Ok(ScriptResult::from_model_with_hints(&scene, &hints))
}

/// Builds the three rounded solids at the default radius.
pub fn run() -> Result<ScriptResult, String> {
    build(FILLET_RADIUS)
}

/// A block with a cylinder standing on it, one unit proud of its top.
fn boss(origin: Point3) -> Result<(Model<StandardPayload>, SolidKey), String> {
    let base = block_at(Frame::at(origin), BLOCK[0], BLOCK[1], BLOCK[2])
        .map_err(|err| format!("failed to build the block: {err:?}"))?;
    let centre = origin + Vector3::new(BLOCK[0] / 2.0, BLOCK[1] / 2.0, BLOCK[2] - 1.0);
    let boss = cylinder_at(Frame::at(centre), BOSS_RADIUS, 2.0)
        .map_err(|err| format!("failed to build the boss: {err:?}"))?;
    fuse(base, boss)
        .map(|shape| shape.into_model())
        .map_err(|err| format!("failed to fuse the boss: {err:?}"))
}

/// A block with a cylinder leaning 15 degrees into it.
fn oblique_boss(origin: Point3) -> Result<(Model<StandardPayload>, SolidKey), String> {
    let base = block_at(Frame::at(origin), BLOCK[0], BLOCK[1], BLOCK[2])
        .map_err(|err| format!("failed to build the block: {err:?}"))?;
    let tilt = 15.0_f64.to_radians();
    let axis = Vector3::new(tilt.sin(), 0.0, tilt.cos());
    let foot = origin + Vector3::new(1.7, 2.1, 0.5);
    let frame = Frame::from_xz(foot, Vector3::y().cross(&axis), axis);
    let boss = cylinder_at(frame, 0.9, 3.0)
        .map_err(|err| format!("failed to build the leaning boss: {err:?}"))?;
    fuse(base, boss)
        .map(|shape| shape.into_model())
        .map_err(|err| format!("failed to fuse the leaning boss: {err:?}"))
}

/// A slot two units long with ends of radius one, as tall as the blocks,
/// its first end's centre at `centre`.
fn slot(centre: Point3) -> Result<(Model<StandardPayload>, SolidKey), String> {
    let (length, radius) = (2.0, 1.0);
    let mut g = Model::<StandardPayload>::new();
    let at = |x: f64, y: f64| centre + Vector3::new(x, y, 0.0);
    let ends = [at(length, 0.0), centre].map(|end| Plane::from_xy(end, Vector3::x(), Vector3::y()));
    let edges = [
        add_line(&mut g, at(0.0, -radius), at(length, -radius)),
        add_arc(
            &mut g,
            ends[0].clone(),
            radius,
            Rad64::new(-PI / 2.0),
            Rad64::new(PI / 2.0),
        ),
        add_line(&mut g, at(length, radius), at(0.0, radius)),
        add_arc(
            &mut g,
            ends[1].clone(),
            radius,
            Rad64::new(PI / 2.0),
            Rad64::new(1.5 * PI),
        ),
    ]
    .into_iter()
    .collect::<Result<Vec<_>, _>>()
    .map_err(|err| format!("failed to build the slot's outline: {err:?}"))?;
    let profile = add_profile_from_edges(&mut g, &edges)
        .map_err(|err| format!("failed to close the slot's outline: {err:?}"))?;
    let face =
        add_face(&mut g, profile).map_err(|err| format!("failed to fill the slot: {err:?}"))?;
    let solid = add_extruded_face(&mut g, face, Vector3::new(0.0, 0.0, BLOCK[2]))
        .map_err(|err| format!("failed to extrude the slot: {err:?}"))?
        .solid;
    Ok((g, solid))
}

/// Every closed edge of `solid`: its rims.
fn closed_edges(g: &Model<StandardPayload>, solid: SolidKey) -> Vec<EdgeKey> {
    g.solid_unchecked(solid)
        .edges()
        .into_iter()
        .filter(|edge| matches!(edge, Edge::Unmarked(_)))
        .map(|edge| edge.key())
        .collect()
}

/// Whether `edge` starts at height `z`.
fn starts_at_height(g: &Model<StandardPayload>, edge: EdgeKey, z: f64) -> bool {
    (g.edge_unchecked(edge).trimmed_curve().start().z - z).abs() < LINEAR_TOLERANCE
}

/// Rounds `edges` of one solid.
fn round(
    g: &mut Model<StandardPayload>,
    edges: Vec<EdgeKey>,
    radius: f64,
    label: &str,
) -> Result<(), String> {
    fillet(g, edges, radius)
        .map(|_| ())
        .map_err(|err| format!("failed to round {label}: {err}"))
}
