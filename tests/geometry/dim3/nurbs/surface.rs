use nalgebra::Vector3;
use ngk::geometry::{
    ControlNet, Cylinder, Degree, HPoint, KnotVector, LINEAR_TOLERANCE, NurbsSurface, Point3,
    Surface,
};

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

/// An unclamped patch, unclamped in `u` only and deliberately not square.
///
/// A surface clamped one direction at a time can get the control net's stride
/// wrong without any count disagreeing, and a square net hides that.
fn unclamped_surface() -> NurbsSurface {
    let (nu, nv) = (5, 4);
    let points = (0..nv)
        .flat_map(|v| (0..nu).map(move |u| (u, v)))
        .map(|(u, v)| {
            HPoint::from_cartesian(
                Point3::new(u as f64, v as f64, (u as f64 * 0.7).sin() + v as f64 * 0.3),
                1.0,
            )
        })
        .collect();
    NurbsSurface::new(
        Degree::new(2).expect("degree 2"),
        Degree::new(3).expect("degree 3"),
        ControlNet::new(points, nu, nv).expect("a 5 by 4 net"),
        KnotVector::new((0..8).map(|i| i as f64).collect()).expect("eight u knots"),
        KnotVector::new(vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]).expect("eight v knots"),
    )
    .expect("5 + 2 + 1 and 4 + 3 + 1 knots")
}

#[test]
fn clamping_keeps_the_surface_and_its_domain() {
    let surface = unclamped_surface();
    assert!(!surface.knots_u().is_clamped(surface.degree_u()));
    assert!(surface.knots_v().is_clamped(surface.degree_v()));

    let clamped = surface
        .clamped()
        .expect("an unclamped surface should clamp");
    assert!(clamped.knots_u().is_clamped(clamped.degree_u()));
    assert!(clamped.knots_v().is_clamped(clamped.degree_v()));
    assert_eq!(clamped.domain_u(), surface.domain_u());
    assert_eq!(clamped.domain_v(), surface.domain_v());

    let (du, dv) = (surface.domain_u(), surface.domain_v());
    for iu in 0..=12 {
        for iv in 0..=12 {
            let (u, v) = (du.at(f64::from(iu) / 12.0), dv.at(f64::from(iv) / 12.0));
            let (before, after) = (surface.point_at(u, v), clamped.point_at(u, v));
            assert!(
                (before - after).norm() <= LINEAR_TOLERANCE,
                "at ({u}, {v}): {before:?} became {after:?}",
            );
        }
    }
}
