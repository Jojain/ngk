//! The six faces of an axis-aligned cube, built as loose polygons.

use ngk::geometry::Point3;
use ngk::modeling::faces;
use ngk::topology::shape::{FaceTag, Shape};

/// Which way the cube's faces are wound.
#[derive(Clone, Copy)]
pub enum Winding {
    Outward,
    Inward,
}

/// The six faces of the cube with corner `min` and edge `size`.
pub fn cube_faces(min: Point3, size: f64, winding: Winding) -> Vec<Shape<FaceTag>> {
    let corner = |i: f64, j: f64, k: f64| min + size * nalgebra::Vector3::new(i, j, k);
    // Each list runs counter-clockwise seen from outside the cube.
    let outward = [
        [(0., 0., 0.), (0., 1., 0.), (1., 1., 0.), (1., 0., 0.)],
        [(0., 0., 1.), (1., 0., 1.), (1., 1., 1.), (0., 1., 1.)],
        [(0., 0., 0.), (1., 0., 0.), (1., 0., 1.), (0., 0., 1.)],
        [(0., 1., 0.), (0., 1., 1.), (1., 1., 1.), (1., 1., 0.)],
        [(0., 0., 0.), (0., 0., 1.), (0., 1., 1.), (0., 1., 0.)],
        [(1., 0., 0.), (1., 1., 0.), (1., 1., 1.), (1., 0., 1.)],
    ];
    outward
        .iter()
        .map(|quad| {
            let mut points = quad
                .iter()
                .map(|&(i, j, k)| corner(i, j, k))
                .collect::<Vec<_>>();
            if let Winding::Inward = winding {
                points.reverse();
            }
            faces::polygon(&points).expect("a cube face should build")
        })
        .collect()
}

/// Borrows every face, as the constructors take them.
pub fn refs(faces: &[Shape<FaceTag>]) -> Vec<&Shape<FaceTag>> {
    faces.iter().collect()
}
