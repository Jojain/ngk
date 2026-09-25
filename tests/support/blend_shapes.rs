//! Small solids for blend tests: extruded polygons and ways to pick their
//! edges and faces by position.

use std::f64::consts::FRAC_PI_4;

use nalgebra::Vector3;
use ngk::builders::edges::add_arc;
use ngk::builders::faces::add_face;
use ngk::builders::profiles::{add_polyline, add_profile_from_edges};
use ngk::builders::solids::add_extruded_face;
use ngk::geometry::{Frame, Plane, Point3};
use ngk::model::Model;
use ngk::modeling::solids::{block, block_at, cut};
use ngk::topology::StandardPayload;
use ngk::topology::shape_keys::{EdgeKey, FaceKey, ProfileKey, SolidKey};
use radians::Rad64;

const TOLERANCE: f64 = 1.0e-9;

/// An L seen from above: five convex corners and one reflex corner at (1, 1).
pub const L_SHAPE: [(f64, f64); 6] = [
    (0.0, 0.0),
    (2.0, 0.0),
    (2.0, 1.0),
    (1.0, 1.0),
    (1.0, 2.0),
    (0.0, 2.0),
];

/// A quadrilateral whose side from (4, 0) to (3, 2) leans away from square.
pub const TRAPEZOID: [(f64, f64); 4] = [(0.0, 0.0), (4.0, 0.0), (3.0, 2.0), (0.0, 2.0)];

/// Extrudes the closed polygon through `points` in the xy plane by `height`.
pub fn prism(points: &[(f64, f64)], height: f64) -> (Model<StandardPayload>, SolidKey) {
    let mut g = Model::<StandardPayload>::new();
    let mut corners = points
        .iter()
        .map(|&(x, y)| Point3::new(x, y, 0.0))
        .collect::<Vec<_>>();
    corners.push(corners[0]);
    let profile = add_polyline(&mut g, &corners).expect("polygon should build");
    let face = add_face(&mut g, profile).expect("polygon face should build");
    let solid = add_extruded_face(&mut g, face, Vector3::new(0.0, 0.0, height))
        .expect("polygon should extrude")
        .solid;
    (g, solid)
}

pub fn edge_between(g: &Model<StandardPayload>, solid: SolidKey, a: Point3, b: Point3) -> EdgeKey {
    g.solid_unchecked(solid)
        .edges()
        .into_iter()
        .find(|edge| {
            let bounded = edge.bounded_unchecked();
            let (start, end) = (*bounded.start().point(), *bounded.end().point());
            ((start - a).norm() < TOLERANCE && (end - b).norm() < TOLERANCE)
                || ((start - b).norm() < TOLERANCE && (end - a).norm() < TOLERANCE)
        })
        .expect("the solid should have this edge")
        .key()
}

pub fn face_at_height(g: &Model<StandardPayload>, solid: SolidKey, z: f64) -> FaceKey {
    g.solid_unchecked(solid)
        .faces()
        .into_iter()
        .find(|face| {
            face.vertices()
                .iter()
                .all(|vertex| (vertex.point().z - z).abs() < TOLERANCE)
        })
        .expect("the solid should have a face at this height")
        .key()
}

/// A square slab of side `size` and height `height` with a centred square
/// hole of side `hole` cut through it.
pub fn prism_with_hole(size: f64, hole: f64, height: f64) -> (Model<StandardPayload>, SolidKey) {
    let slab = block(size, size, height).expect("slab should build");
    let low = (size - hole) / 2.0;
    let tool = block_at(
        Frame::at(Point3::new(low, low, -height / 2.0)),
        hole,
        hole,
        2.0 * height,
    )
    .expect("hole tool should build");
    cut(slab, tool)
        .expect("the hole should cut through")
        .into_model()
}

/// A lens: two arcs of radius sqrt(2) about (0, -1) and (0, 1), meeting at
/// sharp corners at (-1, 0) and (1, 0). Returns the closed profile and the
/// two arcs' centres.
pub fn lens(g: &mut Model<StandardPayload>) -> (ProfileKey, [Point3; 2]) {
    let centres = [Point3::new(0.0, -1.0, 0.0), Point3::new(0.0, 1.0, 0.0)];
    let radius = 2.0_f64.sqrt();
    let upper = add_arc(
        g,
        Plane::from_xy(centres[0], Vector3::x(), Vector3::y()),
        radius,
        Rad64::new(FRAC_PI_4),
        Rad64::new(3.0 * FRAC_PI_4),
    )
    .expect("upper arc should build");
    let lower = add_arc(
        g,
        Plane::from_xy(centres[1], Vector3::x(), Vector3::y()),
        radius,
        Rad64::new(5.0 * FRAC_PI_4),
        Rad64::new(7.0 * FRAC_PI_4),
    )
    .expect("lower arc should build");
    let profile = add_profile_from_edges(g, &[upper, lower]).expect("lens should close");
    (profile, centres)
}
