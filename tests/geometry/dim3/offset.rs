use nalgebra::{UnitVector3, Vector3};
use ngk::geometry::{Axis3, ControlPolygon};
use ngk::geometry::{
    Circle, Cone, ControlNet, Curve, Cylinder, Degree, Ellipse, Frame, HPoint, Helix, Interval,
    KnotVector, LINEAR_TOLERANCE, NativeParam, NurbsCurve, NurbsSurface, OFFSET_FIT_TOLERANCE,
    OffsetError, Plane, Point3, RuledSurface, Sphere, Surface, SurfaceOfRevolution, Torus,
};
use std::f64::consts::TAU;

/// Checks the defining property at the source's own parameters: the offset
/// point is the source point moved `distance` along the source normal.
fn assert_surface_offset(
    source: &Surface,
    offset: &Surface,
    distance: f64,
    u: Interval,
    v: Interval,
    tolerance: f64,
) {
    for i in 0..=10 {
        for j in 0..=10 {
            let s = u.start.value() + u.length() * (0.05 + 0.09 * i as f64);
            let t = v.start.value() + v.length() * (0.05 + 0.09 * j as f64);
            let expected = source.point_at(s, t) + *source.normal_at(s, t) * distance;
            let actual = offset.point_at(s, t);
            assert!(
                (actual - expected).norm() <= tolerance,
                "at ({s}, {t}): expected {expected:?}, got {actual:?}"
            );
        }
    }
}

fn assert_curve_offset(
    source: &Curve,
    offset: &Curve,
    normal: UnitVector3<f64>,
    distance: f64,
    window: Interval,
    tolerance: f64,
) {
    for i in 0..=40 {
        let t = NativeParam::new(window.start.value() + window.length() * i as f64 / 40.0);
        let tangent = source.derivative_at(t, 1).normalize();
        let expected = source.point_at(t) + tangent.cross(&normal) * distance;
        let actual = offset.point_at(t);
        assert!(
            (actual - expected).norm() <= tolerance,
            "at {t:?}: expected {expected:?}, got {actual:?}"
        );
    }
}

fn unit(u: f64, v: f64) -> (Interval, Interval) {
    (Interval::new(0.0, u), Interval::new(0.0, v))
}

#[test]
fn a_plane_moves_along_its_normal_keeping_its_parameters() {
    let plane = Surface::Plane(Plane::new(
        Point3::new(1.0, 2.0, 3.0),
        Vector3::x(),
        Vector3::new(0.0, 1.0, 1.0),
    ));
    let offset = plane.offset(-2.5).unwrap();
    assert!(matches!(offset, Surface::Plane(_)));
    let (u, v) = unit(4.0, 4.0);
    assert_surface_offset(&plane, &offset, -2.5, u, v, LINEAR_TOLERANCE);
}

#[test]
fn cylinders_spheres_and_tori_change_radius_exactly() {
    let cylinder = Surface::Cylinder(Cylinder::new(
        Point3::origin(),
        Vector3::x(),
        Vector3::z(),
        2.0,
    ));
    let sphere = Surface::Sphere(Sphere::new(Frame::xyz(), 2.0));
    let torus = Surface::Torus(Torus::new(Frame::xyz(), 5.0, 1.0));
    for (surface, v) in [
        (&cylinder, Interval::new(-1.0, 1.0)),
        (&sphere, Interval::new(-1.5, 1.5)),
        (&torus, Interval::new(0.0, TAU)),
    ] {
        for distance in [0.5, -0.5] {
            let offset = surface.offset(distance).unwrap();
            assert_eq!(
                std::mem::discriminant(&offset),
                std::mem::discriminant(surface)
            );
            assert_surface_offset(
                surface,
                &offset,
                distance,
                Interval::new(0.0, TAU),
                v,
                1e-12,
            );
        }
    }
}

#[test]
fn a_radius_the_distance_consumes_is_refused() {
    let sphere = Surface::Sphere(Sphere::new(Frame::xyz(), 2.0));
    assert!(matches!(
        sphere.offset(-2.0),
        Err(OffsetError::Collapsed { radius, .. }) if radius == 2.0
    ));
    let torus = Surface::Torus(Torus::new(Frame::xyz(), 5.0, 1.0));
    assert!(matches!(
        torus.offset(-1.5),
        Err(OffsetError::Collapsed { .. })
    ));
}

#[test]
fn a_cone_moves_its_reference_circle_and_keeps_its_angle() {
    let cone = Surface::Cone(Cone::new(Frame::xyz(), 2.0, 0.4));
    for distance in [0.3, -0.3] {
        let offset = cone.offset(distance).unwrap();
        let Surface::Cone(ref moved) = offset else {
            panic!("a cone offsets to a cone, got {offset:?}");
        };
        assert_eq!(moved.half_angle(), 0.4);
        assert_surface_offset(
            &cone,
            &offset,
            distance,
            Interval::new(0.0, TAU),
            Interval::new(0.0, 3.0),
            1e-12,
        );
    }
}

#[test]
fn an_extruded_circle_offsets_its_profile_and_stays_exact() {
    let profile = Curve::Circle(Circle::new(Plane::xy(), 3.0));
    let extrusion = Surface::Ruled(RuledSurface::new(profile, Vector3::new(0.0, 0.0, 2.0)));
    let offset = extrusion.offset(0.5).unwrap();
    let Surface::Ruled(ref ruled) = offset else {
        panic!("an extrusion offsets to an extrusion, got {offset:?}");
    };
    assert!(
        matches!(ruled.curve(), Curve::Circle(circle) if (circle.radius() - 3.5).abs() < 1e-12)
    );
    assert_surface_offset(
        &extrusion,
        &offset,
        0.5,
        Interval::new(0.0, TAU),
        Interval::new(0.0, 1.0),
        1e-12,
    );
}

#[test]
fn a_revolved_meridian_arc_offsets_its_arc_and_stays_exact() {
    // A circle in the xz plane, off the z axis: the revolution is a torus.
    let arc = Curve::Circle(Circle::new(
        Plane::new(Point3::new(4.0, 0.0, 0.0), Vector3::x(), Vector3::y()),
        1.0,
    ));
    let revolution = Surface::Revolution(SurfaceOfRevolution::new(arc, Axis3::z()));
    for distance in [0.25, -0.25] {
        let offset = revolution.offset(distance).unwrap();
        let Surface::Revolution(ref revolved) = offset else {
            panic!("a revolution offsets to a revolution, got {offset:?}");
        };
        assert!(matches!(revolved.curve(), Curve::Circle(_)));
        assert_surface_offset(
            &revolution,
            &offset,
            distance,
            Interval::new(0.0, TAU),
            Interval::new(0.0, TAU),
            1e-12,
        );
    }
}

#[test]
fn a_free_form_surface_is_fitted_within_tolerance_at_its_own_parameters() {
    let mut points = Vec::new();
    for j in 0..4 {
        for i in 0..4 {
            let height = if (1..=2).contains(&i) && (1..=2).contains(&j) {
                1.0
            } else {
                0.0
            };
            points.push(HPoint::from_cartesian(
                Point3::new(i as f64, j as f64, height),
                1.0,
            ));
        }
    }
    let degree = Degree::new(3).unwrap();
    let knots = KnotVector::uniform_clamped(4, degree);
    let bump = Surface::Nurbs(
        NurbsSurface::new(
            degree,
            degree,
            ControlNet::new(points, 4, 4).unwrap(),
            knots.clone(),
            knots,
        )
        .unwrap(),
    );
    let offset = bump.offset(0.2).unwrap();
    assert!(matches!(offset, Surface::Nurbs(_)));
    let (u, v) = unit(1.0, 1.0);
    assert_surface_offset(&bump, &offset, 0.2, u, v, OFFSET_FIT_TOLERANCE);
}

#[test]
fn a_free_form_offset_past_its_curvature_radius_folds() {
    let sphere = Surface::Sphere(Sphere::new(Frame::xyz(), 1.0));
    let patch = Surface::Nurbs(
        sphere
            .to_nurbs_over(Interval::new(0.2, 1.2), Interval::new(-0.5, 0.5))
            .unwrap(),
    );
    assert!(matches!(
        patch.offset(-1.2),
        Err(OffsetError::Folded { .. })
    ));
}

#[test]
fn a_line_moves_to_its_right_seen_from_the_normal() {
    let line = Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0));
    let offset = line.offset_in_plane(Vector3::z_axis(), 2.0).unwrap();
    assert!(matches!(offset, Curve::Line(_)));
    let start = offset.point_at(NativeParam::new(0.0));
    assert!((start - Point3::new(0.0, -2.0, 0.0)).norm() <= LINEAR_TOLERANCE);
    assert_curve_offset(
        &line,
        &offset,
        Vector3::z_axis(),
        2.0,
        Interval::new(-3.0, 3.0),
        1e-12,
    );

    let rising = Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 1.0));
    assert_eq!(
        rising.offset_in_plane(Vector3::z_axis(), 1.0),
        Err(OffsetError::NotPlanar)
    );
}

#[test]
fn a_circle_grows_counter_clockwise_and_shrinks_clockwise() {
    let circle = Curve::Circle(Circle::new(Plane::xy(), 2.0));
    let up = circle.offset_in_plane(Vector3::z_axis(), 0.5).unwrap();
    let down = circle.offset_in_plane(-Vector3::z_axis(), 0.5).unwrap();
    assert!(matches!(up, Curve::Circle(ref c) if (c.radius() - 2.5).abs() < 1e-12));
    assert!(matches!(down, Curve::Circle(ref c) if (c.radius() - 1.5).abs() < 1e-12));
    let whole = Interval::new(0.0, TAU);
    assert_curve_offset(&circle, &up, Vector3::z_axis(), 0.5, whole, 1e-12);
    assert_curve_offset(&circle, &down, -Vector3::z_axis(), 0.5, whole, 1e-12);
    assert!(matches!(
        circle.offset_in_plane(Vector3::z_axis(), -2.0),
        Err(OffsetError::Collapsed { .. })
    ));
}

#[test]
fn an_ellipse_is_fitted_and_folds_past_its_tightest_curvature() {
    let ellipse = Curve::Ellipse(Ellipse::new(Frame::xyz(), 2.0, 1.0));
    let whole = Interval::new(0.0, TAU);
    let outward = ellipse.offset_in_plane(Vector3::z_axis(), 0.4).unwrap();
    assert_curve_offset(
        &ellipse,
        &outward,
        Vector3::z_axis(),
        0.4,
        whole,
        OFFSET_FIT_TOLERANCE,
    );
    // Inward, still short of the tightest radius of curvature b²/a = 0.5.
    let inward = ellipse.offset_in_plane(Vector3::z_axis(), -0.45).unwrap();
    assert_curve_offset(
        &ellipse,
        &inward,
        Vector3::z_axis(),
        -0.45,
        whole,
        OFFSET_FIT_TOLERANCE,
    );
    assert!(matches!(
        ellipse.offset_in_plane(Vector3::z_axis(), -0.6),
        Err(OffsetError::Folded { .. })
    ));
}

#[test]
fn a_planar_nurbs_curve_is_fitted_and_a_helix_is_refused() {
    let points = [
        Point3::new(0.0, 0.0, 1.0),
        Point3::new(1.0, 2.0, 1.0),
        Point3::new(3.0, -1.0, 1.0),
        Point3::new(4.0, 1.0, 1.0),
    ];
    let degree = Degree::new(3).unwrap();
    let curve = Curve::Nurbs(
        NurbsCurve::with_uniform_knots(
            degree,
            ControlPolygon::from_cartesian(points.to_vec(), &[1.0; 4]).unwrap(),
        )
        .unwrap(),
    );
    let offset = curve.offset_in_plane(Vector3::z_axis(), 0.1).unwrap();
    assert_curve_offset(
        &curve,
        &offset,
        Vector3::z_axis(),
        0.1,
        curve.domain(),
        OFFSET_FIT_TOLERANCE,
    );
    assert_eq!(
        curve.offset_in_plane(Vector3::x_axis(), 0.1),
        Err(OffsetError::NotPlanar)
    );

    let helix = Curve::Helix(Helix::new(Frame::xyz(), 1.0, 0.5));
    assert_eq!(
        helix.offset_in_plane(Vector3::z_axis(), 0.1),
        Err(OffsetError::NotPlanar)
    );
}
