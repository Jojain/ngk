use nalgebra::Vector3;
use ngk::geometry::Fraction;
use ngk::geometry::Plane;
use ngk::geometry::Point3;
use ngk::modeling::faces;
use ngk::modeling::profiles;

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

    let result = faces::intersect(first, second).expect("intersection");
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
