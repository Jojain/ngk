use ngk::geometry::Plane;
use ngk::geometry::Point3;
use ngk::modeling::faces;

#[test]
fn rectangle_returns_owned_face_shape() {
    let shape = faces::rectangle(Plane::xy(), 2.0, 3.0).expect("face should build");

    assert_eq!(shape.map().iter_faces().count(), 1);
    assert_eq!(shape.face().outer_loop().edges().len(), 4);
}

#[test]
fn circle_returns_owned_face_shape() {
    let shape = faces::circle(Plane::xy(), 2.0).expect("face should build");

    assert_eq!(shape.map().iter_faces().count(), 1);
    assert_eq!(shape.face().outer_loop().edges().len(), 1);
    assert_eq!(shape.face().inner_loops().len(), 0);
}

#[test]
fn annulus_returns_owned_face_shape_with_circular_hole() {
    let shape = faces::annulus(Plane::xy(), 2.0, 1.0).expect("face should build");

    assert_eq!(shape.map().iter_faces().count(), 1);
    assert_eq!(shape.face().outer_loop().edges().len(), 1);
    assert_eq!(shape.face().inner_loops().len(), 1);
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

    assert_eq!(shape.map().iter_faces().count(), 1);
    assert_eq!(shape.face().outer_loop().edges().len(), 4);
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
