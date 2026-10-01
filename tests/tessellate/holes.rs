//! A face meshes whatever its number of holes (Jojain/ngk#21).

use std::f64::consts::PI;

use nalgebra::Vector3;
use ngk::geometry::{Frame, Point3, Surface};
use ngk::modeling::solids;
use ngk::tessellate::{Tessellation, tessellate};
use ngk::topology::shape::{Shape, SolidTag};

const RADIUS: f64 = 3.0;

/// A 20 mm wide plate, room for `room` holes along x.
fn blank(room: usize) -> Shape<SolidTag> {
    solids::block(20.0 * room as f64, 20.0, 10.0).expect("block builds")
}

/// Drills through-hole number `hole`, of radius 3, 20 mm from the last.
fn drilled(plate: Shape<SolidTag>, hole: usize) -> Shape<SolidTag> {
    let frame = Frame::from_xz(
        Point3::new(10.0 + 20.0 * hole as f64, 10.0, -1.0),
        Vector3::x(),
        Vector3::z(),
    );
    let tool = solids::cylinder_at(frame, RADIUS, 12.0).expect("cylinder builds");
    solids::cut(plate, tool).expect("cut builds")
}

fn hole_centres(holes: usize) -> impl Iterator<Item = (f64, f64)> {
    (0..holes).map(|hole| (10.0 + 20.0 * hole as f64, 10.0))
}

/// The triangles of the planar face at height `z`.
fn triangles_at(mesh: &Tessellation, shape: &Shape<SolidTag>, z: f64) -> Vec<[Point3; 3]> {
    let range = mesh
        .faces
        .iter()
        .find(|range| {
            let face = shape.model().face(range.key).expect("face exists");
            matches!(face.surface(), Surface::Plane(_))
                && (face.surface_properties().expect("face measures").centroid.z - z).abs() < 1e-9
        })
        .expect("the plate has a face at that height");
    mesh.mesh.indices[range.start..range.start + range.count]
        .chunks_exact(3)
        .map(|t| [0, 1, 2].map(|i| mesh.mesh.positions[t[i] as usize]))
        .collect()
}

#[test]
fn a_plate_with_any_number_of_holes_tessellates() {
    let mut plate = blank(12);
    for hole in 0..12 {
        plate = drilled(plate, hole);
        if let Err(error) = tessellate(&plate.solid()) {
            panic!("{} holes: {error}", hole + 1);
        }
    }
}

#[test]
fn a_holed_face_is_meshed_around_every_hole_and_nowhere_else() {
    let holes = 5;
    let shape = (0..holes).fold(blank(holes), drilled);
    let mesh = tessellate(&shape.solid()).expect("the plate tessellates");
    for z in [0.0, 10.0] {
        let triangles = triangles_at(&mesh, &shape, z);
        let area: f64 = triangles
            .iter()
            .map(|[a, b, c]| (b - a).cross(&(c - a)).norm() / 2.0)
            .sum();
        let expected = 20.0 * holes as f64 * 20.0 - holes as f64 * PI * RADIUS * RADIUS;
        // A hole is a 64-gon inscribed in its circle, so the mesh keeps a sliver
        // of each: about 0.16% of the hole's area.
        assert!(
            (area - expected).abs() < 0.01 * holes as f64 * PI * RADIUS * RADIUS,
            "face at z = {z}: area {area}, expected {expected}"
        );
        for [a, b, c] in &triangles {
            let centroid = (a.coords + b.coords + c.coords) / 3.0;
            for (x, y) in hole_centres(holes) {
                let distance = ((centroid.x - x).powi(2) + (centroid.y - y).powi(2)).sqrt();
                assert!(
                    distance > RADIUS * (1.0 - 1e-3),
                    "face at z = {z}: a triangle sits inside the hole at ({x}, {y})"
                );
            }
        }
    }
}
