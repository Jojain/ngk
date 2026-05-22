use ngk::geometry::Plane;
use ngk::modeling::faces;

#[test]
fn rectangle_returns_owned_face_shape() {
    let shape = faces::rectangle(Plane::xy(), 2.0, 3.0).expect("face should build");

    assert_eq!(shape.map().iter_faces().count(), 1);
    assert_eq!(shape.face().outer_loop().edges().len(), 4);
}
