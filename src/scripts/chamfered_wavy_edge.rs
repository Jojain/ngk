//! Extruded planar profile whose wavy NURBS edge is chamfered in 3D.

use nalgebra::Vector3;

use crate::builders::chamfer::chamfer;
use crate::builders::edges::add_edge;
use crate::builders::faces::add_face;
use crate::builders::profiles::{add_polyline, append_edge};
use crate::builders::solids::add_extruded_face;
use crate::geometry::{Curve, NurbsCurve, Point3, Surface};
use crate::topology::StandardPayload;
use crate::topology::gmap::GMap;
use crate::viz::{ScriptResult, Style, VizHints};

const WIDTH: f64 = 4.0;
const HEIGHT: f64 = 3.0;
const DEPTH: f64 = 2.2;
const CHAMFER_DISTANCE: f64 = 0.35;

/// Builds an extruded rectangle-like solid and chamfers its translated wavy
/// NURBS boundary edge by `distance`.
pub fn build(distance: f64) -> Result<ScriptResult, String> {
    let mut g = GMap::<StandardPayload>::new();
    let profile = add_polyline(
        &mut g,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(WIDTH, 0.0, 0.0),
            Point3::new(WIDTH, HEIGHT, 0.0),
        ],
    )
    .map_err(|err| format!("failed to build open profile: {err:?}"))?;

    let wavy_edge = add_edge(
        &mut g,
        Point3::new(WIDTH, HEIGHT, 0.0),
        Point3::new(0.0, HEIGHT, 0.0),
        Curve::Nurbs(
            NurbsCurve::interpolate(&[
                Point3::new(WIDTH, HEIGHT, 0.0),
                Point3::new(3.2, HEIGHT - 0.4, 0.0),
                Point3::new(2.4, HEIGHT + 0.35, 0.0),
                Point3::new(1.6, HEIGHT - 0.4, 0.0),
                Point3::new(0.8, HEIGHT + 0.35, 0.0),
                Point3::new(0.0, HEIGHT, 0.0),
            ])
            .map_err(|err| format!("failed to interpolate wavy NURBS edge: {err:?}"))?,
        ),
    )
    .map_err(|err| format!("failed to build wavy edge: {err:?}"))?;
    append_edge(&mut g, profile, wavy_edge)
        .map_err(|err| format!("failed to append wavy edge: {err:?}"))?;

    let closing_edge = add_edge(
        &mut g,
        Point3::new(0.0, HEIGHT, 0.0),
        Point3::origin(),
        Curve::line(Point3::new(0.0, HEIGHT, 0.0), Point3::origin()),
    )
    .map_err(|err| format!("failed to build closing edge: {err:?}"))?;
    append_edge(&mut g, profile, closing_edge)
        .map_err(|err| format!("failed to close profile: {err:?}"))?;

    let face =
        add_face(&mut g, profile).map_err(|err| format!("failed to fill wavy profile: {err:?}"))?;
    let solid = add_extruded_face(&mut g, face, Vector3::new(0.0, 0.0, DEPTH))
        .map_err(|err| format!("failed to extrude wavy profile: {err:?}"))?;
    let top_wavy_edge = g
        .solid_unchecked(solid)
        .edges()
        .into_iter()
        .find(|edge| {
            matches!(edge.curve(), Some(Curve::Nurbs(_)))
                && edge
                    .start()
                    .point()
                    .is_some_and(|point| (point.z - DEPTH).abs() < 1.0e-9)
        })
        .map(|edge| edge.key())
        .ok_or("extrusion did not expose its translated wavy edge")?;

    chamfer(&mut g, top_wavy_edge, distance)
        .map_err(|err| format!("failed to chamfer wavy NURBS edge: {err:?}"))?;
    let chamfer_face = g
        .solid_unchecked(solid)
        .faces()
        .into_iter()
        .find(|face| {
            matches!(face.surface(), Surface::Ruled(surface) if surface.direction().y.abs() > 1.0e-9)
        })
        .map(|face| face.key())
        .ok_or("curved chamfer did not create a face")?;

    let mut hints = VizHints::new();
    for face in g.solid_unchecked(solid).faces() {
        hints.face(
            face.key(),
            Style::default()
                .color("#68a5ff")
                .label("wavy solid")
                .double_sided(true),
        );
    }
    hints.face(
        chamfer_face,
        Style::default()
            .color("#ffb454")
            .label("ruled NURBS chamfer")
            .double_sided(true),
    );

    Ok(ScriptResult::from_gmap_with_hints(&g, &hints))
}

/// Builds the default wavy-edge chamfer experiment.
pub fn run() -> Result<ScriptResult, String> {
    build(CHAMFER_DISTANCE)
}
