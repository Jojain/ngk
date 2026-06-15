use ngk::builders::faces::{add_face, add_polygon, add_polygon_with_holes};
use ngk::geometry::{LINEAR_TOLERANCE, Plane, Point3, PointCoincidence};
use ngk::modeling::faces;
use ngk::topology::attributes::FaceAttr;
use ngk::topology::facet::Facet;
use ngk::topology::gmap::{Dim, GMap};
use ngk::topology::payload::StandardPayload;
use ngk::topology::sheet::Sheet;

#[test]
fn face_point_at_evaluates_its_support_surface() {
    let shape = faces::rectangle(Plane::xy(), 2.0, 3.0).expect("face should build");
    let point = shape.face().point_at(0.5, 1.25);

    assert!(point.coincides(&Point3::new(0.5, 1.25, 0.0), LINEAR_TOLERANCE));
}

#[test]
fn face_point_at_is_defined_inside_a_trimmed_hole() {
    let shape = faces::annulus(Plane::xy(), 2.0, 1.0).expect("face should build");
    let point = shape.face().point_at(0.0, 0.0);

    assert!(point.coincides(&Point3::origin(), LINEAR_TOLERANCE));
}

#[test]
fn face_key_resolves_stored_oriented_connectivity() {
    let mut g = GMap::<StandardPayload>::new();
    let loop_dart = add_polygon(
        &mut g,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
    );
    let key = add_face(&mut g, loop_dart).expect("face should build");
    let face = g.face(key).expect("face key should resolve");

    assert_eq!(face.key(), key);
    assert_eq!(face.outer_loop().dart, loop_dart);
}

#[test]
fn alpha3_shared_faces_have_distinct_keys_and_one_facet_key() {
    let mut g = GMap::<StandardPayload>::new();
    let corners = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    ];
    let first_loop = add_polygon(&mut g, &corners);
    let first_key = add_face(&mut g, first_loop).expect("face should build");
    let facet_key = g.face(first_key).expect("face should resolve").facet_key();
    let second_loop = add_polygon(&mut g, &corners);
    g.sew(Dim::Three, first_loop, second_loop)
        .expect("matching face sides should be alpha3-sewable");
    let second_key = g.add_face_use(FaceAttr::new(facet_key, second_loop, Vec::new()));

    let first = g.face(first_key).expect("first face should resolve");
    let second = g.face(second_key).expect("second face should resolve");

    assert_ne!(first.key(), second.key());
    assert_eq!(first.facet_key(), second.facet_key());
    assert!(first.normal_at(0.5, 0.5).dot(&second.normal_at(0.5, 0.5)) < -0.999);
    assert!(
        second
            .outer_loop()
            .edges()
            .into_iter()
            .all(|edge| second.pcurve(edge.dart).is_some())
    );

    assert_eq!(Sheet::new(&g, first_loop).faces()[0].key(), first_key);
    assert_eq!(Sheet::new(&g, second_loop).faces()[0].key(), second_key);
}

#[test]
fn shared_face_keys_store_oriented_inner_loops() {
    let mut g = GMap::<StandardPayload>::new();
    let outer = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(4.0, 0.0, 0.0),
        Point3::new(4.0, 4.0, 0.0),
        Point3::new(0.0, 4.0, 0.0),
    ];
    let hole = [
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(1.0, 3.0, 0.0),
        Point3::new(3.0, 3.0, 0.0),
        Point3::new(3.0, 1.0, 0.0),
    ];
    let first_key = add_polygon_with_holes(&mut g, Plane::xy(), &outer, &[&hole])
        .expect("face with a hole should build");
    let first = g.face(first_key).expect("face should resolve");
    let first_outer = first.outer_loop().dart;
    let first_inner = first.inner_loops()[0].dart;
    let facet_key = first.facet_key();
    let second_outer = add_polygon(&mut g, &outer);
    let second_inner = add_polygon(&mut g, &hole);

    g.sew(Dim::Three, first_outer, second_outer)
        .expect("outer loops should be alpha3-sewable");
    g.sew(Dim::Three, first_inner, second_inner)
        .expect("inner loops should be alpha3-sewable");
    let second_key = g.add_face_use(FaceAttr::new(facet_key, second_outer, vec![second_inner]));

    let second = g.face(second_key).expect("second face should resolve");
    assert_eq!(second.inner_loops()[0].dart, second_inner);

    let face_from_inner_facet = Facet::new(&g, second_inner)
        .face()
        .expect("inner facet should resolve its oriented face");
    assert_eq!(face_from_inner_facet.key(), second_key);
}
