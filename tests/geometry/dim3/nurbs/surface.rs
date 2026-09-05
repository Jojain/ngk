use nalgebra::Vector3;
use ngk::geometry::{Cylinder, Point3, Surface};

#[test]
fn cylinder_decomposes_into_exact_rational_bezier_spans() {
    let surface = Surface::Cylinder(Cylinder::new(
        Point3::origin(),
        Vector3::x(),
        Vector3::z(),
        1.0,
    ))
    .to_nurbs()
    .unwrap();

    let spans = surface.bezier_spans().unwrap();

    assert_eq!(spans.len(), 4);
    for span in spans {
        let domain_u = span.domain_u();
        let domain_v = span.domain_v();
        for local_u in [0.0, 0.25, 0.5, 0.75, 1.0] {
            for local_v in [0.0, 0.5, 1.0] {
                let u = domain_u.start + domain_u.length() * local_u;
                let v = domain_v.start + domain_v.length() * local_v;
                let expected = surface.point_at(u, v);
                let actual = span.point_at(u, v);
                assert!((actual - expected).norm() <= 1.0e-9);
                assert!(span.bbox().contains_point(expected, 1.0e-9));
            }
        }
    }
}

#[test]
fn bezier_patch_halves_agree_with_the_parent_patch() {
    // A doubly curved rational patch: subdivision must reproduce the parent
    // exactly in parent parameters, which is what makes hull rejection sound.
    let cylinder = Cylinder::new(Point3::origin(), Vector3::x(), Vector3::z(), 2.0);
    let surface = cylinder.to_nurbs().unwrap();
    let patch = surface.bezier_spans().unwrap().remove(0);

    let mid_u = patch.domain_u().start + patch.domain_u().length() * 0.5;
    let mid_v = patch.domain_v().start + patch.domain_v().length() * 0.5;
    let (left, right) = patch.subdivide_u(mid_u).unwrap();
    let (lower, upper) = patch.subdivide_v(mid_v).unwrap();

    assert_eq!(left.domain_u().end, mid_u);
    assert_eq!(right.domain_u().start, mid_u);
    assert_eq!(lower.domain_v().end, mid_v);
    assert_eq!(upper.domain_v().start, mid_v);

    for half in [&left, &right, &lower, &upper] {
        for su in [0.0, 0.5, 1.0] {
            for sv in [0.0, 0.5, 1.0] {
                let u = half.domain_u().start + half.domain_u().length() * su;
                let v = half.domain_v().start + half.domain_v().length() * sv;
                let expected = patch.point_at(u, v);
                assert!((half.point_at(u, v) - expected).norm() <= 1.0e-9, "{u} {v}");
                assert!(half.bbox().contains_point(expected, 1.0e-9));
            }
        }
    }
}

#[test]
fn bezier_patch_halves_bound_more_tightly_than_the_parent() {
    // Rejection only converges because each split shrinks the hull.
    let cylinder = Cylinder::new(Point3::origin(), Vector3::x(), Vector3::z(), 2.0);
    let patch = cylinder
        .to_nurbs()
        .unwrap()
        .bezier_spans()
        .unwrap()
        .remove(0);
    let mid_u = patch.domain_u().start + patch.domain_u().length() * 0.5;

    let (left, right) = patch.subdivide_u(mid_u).unwrap();

    assert!(left.bbox().diagonal_length() < patch.bbox().diagonal_length());
    assert!(right.bbox().diagonal_length() < patch.bbox().diagonal_length());
}

#[test]
fn bezier_patch_rejects_subdivision_outside_its_domain() {
    let cylinder = Cylinder::new(Point3::origin(), Vector3::x(), Vector3::z(), 2.0);
    let patch = cylinder
        .to_nurbs()
        .unwrap()
        .bezier_spans()
        .unwrap()
        .remove(0);

    assert!(patch.subdivide_u(patch.domain_u().start).is_err());
    assert!(patch.subdivide_v(patch.domain_v().end).is_err());
}
