use nalgebra::Vector3;
use ngk::builders::boolean::{BooleanError, SplitSide};
use ngk::geometry::Fraction;
use ngk::geometry::Plane;
use ngk::geometry::Point3;
use ngk::geometry::Surface;
use ngk::model::{Cell2, Model};
use ngk::modeling::profiles;
use ngk::modeling::{faces, sheets, solids, split};
use ngk::topology::shape::{FaceTag, Shape};

use crate::quad_sheet::quad_sheet;

#[test]
fn rectangle_returns_owned_face_shape() {
    let shape = faces::rectangle(Plane::xy(), 2.0, 3.0).expect("face should build");

    assert_eq!(shape.model().iter_faces().count(), 1);
    assert_eq!(
        shape
            .face()
            .outer_loop()
            .expect("face should have an outer loop")
            .edges()
            .len(),
        4
    );
}

#[test]
fn circle_returns_owned_face_shape() {
    let shape = faces::circle(Plane::xy(), 2.0).expect("face should build");

    assert_eq!(shape.model().iter_faces().count(), 1);
    assert_eq!(
        shape
            .face()
            .outer_loop()
            .expect("face should have an outer loop")
            .edges()
            .len(),
        1
    );
    assert_eq!(shape.face().inner_loops().len(), 0);
}

#[test]
fn annulus_returns_owned_face_shape_with_circular_hole() {
    let shape = faces::annulus(Plane::xy(), 2.0, 1.0).expect("face should build");

    assert_eq!(shape.model().iter_faces().count(), 1);
    assert_eq!(
        shape
            .face()
            .outer_loop()
            .expect("face should have an outer loop")
            .edges()
            .len(),
        1
    );
    assert_eq!(shape.face().inner_loops().len(), 1);
}

#[test]
fn from_profile_returns_owned_face_shape() {
    let profile = profiles::rectangle(Plane::xy(), 2.0, 3.0).expect("profile should build");

    let shape = faces::from_profile(&profile).expect("face should build");

    assert_eq!(shape.model().iter_faces().count(), 1);
    assert_eq!(
        shape
            .face()
            .outer_loop()
            .expect("face should have an outer loop")
            .edges()
            .len(),
        4
    );
    assert_eq!(profile.model().iter_faces().count(), 0);
}

#[test]
fn polygon_with_holes_returns_owned_face_shape() {
    let outer = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Point3::new(2.0, 2.0, 0.0),
        Point3::new(0.0, 2.0, 0.0),
    ];
    let hole = [
        Point3::new(0.75, 0.75, 0.0),
        Point3::new(0.75, 1.25, 0.0),
        Point3::new(1.25, 1.25, 0.0),
        Point3::new(1.25, 0.75, 0.0),
    ];

    let shape =
        faces::polygon_with_holes(Plane::xy(), &outer, &[&hole]).expect("face should build");

    assert_eq!(shape.model().iter_faces().count(), 1);
    assert_eq!(
        shape
            .face()
            .outer_loop()
            .expect("face should have an outer loop")
            .edges()
            .len(),
        4
    );
    assert_eq!(shape.face().inner_loops().len(), 1);
}

#[test]
fn face_edges_and_vertices_include_inner_loops() {
    let outer = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Point3::new(2.0, 2.0, 0.0),
        Point3::new(0.0, 2.0, 0.0),
    ];
    let hole = [
        Point3::new(0.75, 0.75, 0.0),
        Point3::new(0.75, 1.25, 0.0),
        Point3::new(1.25, 1.25, 0.0),
        Point3::new(1.25, 0.75, 0.0),
    ];

    let shape =
        faces::polygon_with_holes(Plane::xy(), &outer, &[&hole]).expect("face should build");
    let face = shape.face();

    assert_eq!(face.key(), shape.key());
    assert_eq!(face.loops().len(), 2);
    assert_eq!(
        face.loops()
            .into_iter()
            .map(|loop_| loop_.edges().len())
            .collect::<Vec<_>>(),
        vec![4, 4]
    );
    assert_eq!(face.edges().len(), 8);
    assert_eq!(face.vertices().len(), 8);
}

#[test]
fn face_intersection_returns_one_model_and_all_result_handles() {
    let first = faces::square(Plane::xy(), 2.0).expect("first face");
    let second = faces::square(
        Plane::new(
            Point3::new(1.0, 1.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
        ),
        2.0,
    )
    .expect("second face");

    let result = faces::intersect(first, [&second]).expect("intersection");
    assert_eq!(result.face_keys().len(), 1);
    assert_eq!(result.model().iter_faces().count(), 1);
    assert!((result.faces()[0].area().expect("area") - 1.0).abs() < 1e-9);
}

#[test]
fn face_boundary_split_keeps_the_face_and_new_edges_in_one_model() {
    let face = faces::rectangle(Plane::xy(), 2.0, 2.0).expect("face");
    let edge = face.face().edges()[0].key();
    let result = faces::split_boundary_edge(face, edge, Fraction::new(0.5)).expect("split");
    assert_eq!(result.edge_keys().len(), 2);
    assert_eq!(
        result
            .model()
            .face(result.face_key().expect("face key"))
            .unwrap()
            .edges()
            .len(),
        5
    );
}

fn area_of(face: &Shape<FaceTag>) -> f64 {
    face.face().area().expect("the part should measure")
}

#[test]
fn split_cuts_a_face_along_a_crossing_plane() {
    let target = faces::rectangle(Plane::xy(), 2.0, 2.0).expect("the rectangle should build");
    let wall = quad_sheet(&[[
        (0.5, -1.0, -1.0),
        (0.5, 3.0, -1.0),
        (0.5, 3.0, 1.0),
        (0.5, -1.0, 1.0),
    ]]);

    let parts = split(&target, &wall).expect("the plane should split the face");

    assert!((area_of(&parts.front) - 3.0).abs() <= 1e-9);
    assert!((area_of(&parts.back) - 1.0).abs() <= 1e-9);
    assert!((area_of(&target) - 4.0).abs() <= 1e-9);
}

#[test]
fn split_cuts_a_disc_into_halves() {
    let target = faces::circle(Plane::xy(), 1.0).expect("the disc should build");
    let wall = quad_sheet(&[[
        (0.0, -2.0, -1.0),
        (0.0, 2.0, -1.0),
        (0.0, 2.0, 1.0),
        (0.0, -2.0, 1.0),
    ]]);

    let parts = split(&target, &wall).expect("the plane should split the disc");

    let half = std::f64::consts::FRAC_PI_2;
    assert!((area_of(&parts.front) - half).abs() <= 1e-6);
    assert!((area_of(&parts.back) - half).abs() <= 1e-6);
    let front = parts
        .front
        .face()
        .bounding_box()
        .expect("the half should bound");
    assert!(front.min().x >= -1e-9);
}

#[test]
fn split_cuts_a_disc_out_where_a_tube_crosses_the_face() {
    let cylinder = solids::cylinder(1.0, 2.0).expect("the cylinder should build");
    let side = cylinder
        .solid()
        .faces()
        .into_iter()
        .find(|face| !matches!(face.surface(), Surface::Plane(_)))
        .expect("the cylinder should have a curved side");
    let (model, dart) = Model::isolate(side);
    let key = model.cell_key_unchecked::<Cell2>(dart);
    let tube =
        sheets::from_faces(&[&Shape::<FaceTag>::new(model, key)]).expect("one face is a sheet");
    let target = faces::polygon(&[
        Point3::new(-2.0, -2.0, 1.0),
        Point3::new(2.0, -2.0, 1.0),
        Point3::new(2.0, 2.0, 1.0),
        Point3::new(-2.0, 2.0, 1.0),
    ])
    .expect("the square should build");

    let parts = split(&target, &tube).expect("the tube should split the face");

    // The tube faces outwards: the disc it encloses is behind it.
    let disc = std::f64::consts::PI;
    assert!((area_of(&parts.back) - disc).abs() <= 1e-6);
    assert!((area_of(&parts.front) - (16.0 - disc)).abs() <= 1e-6);
}

#[test]
fn split_refuses_a_side_that_falls_in_two() {
    let target = faces::rectangle(Plane::xy(), 3.0, 1.0).expect("the rectangle should build");
    // A channel over 1 <= x <= 2, its faces turned in: the channel is the
    // front, and what is behind lies on both sides of it.
    let channel = quad_sheet(&[
        [
            (1.0, -1.0, -1.0),
            (1.0, 2.0, -1.0),
            (1.0, 2.0, 1.0),
            (1.0, -1.0, 1.0),
        ],
        [
            (1.0, -1.0, 1.0),
            (1.0, 2.0, 1.0),
            (2.0, 2.0, 1.0),
            (2.0, -1.0, 1.0),
        ],
        [
            (2.0, -1.0, -1.0),
            (2.0, -1.0, 1.0),
            (2.0, 2.0, 1.0),
            (2.0, 2.0, -1.0),
        ],
    ]);

    assert!(matches!(
        split(&target, &channel),
        Err(BooleanError::SplitSideDisconnected {
            side: SplitSide::Back,
            components: 2
        })
    ));
}
