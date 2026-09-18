use nalgebra::Vector3;
use ngk::geometry::{
    ControlNet, Cylinder, Degree, Fraction, HPoint, KnotVector, LINEAR_TOLERANCE, NurbsSurface,
    Point3, Surface,
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
                let u = domain_u.at(Fraction::new(local_u)).value();
                let v = domain_v.at(Fraction::new(local_v)).value();
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
    // A doubly curved rational patch: embedding must reproduce the parent
    // exactly in parent parameters, which is what makes hull rejection sound.
    let cylinder = Cylinder::new(Point3::origin(), Vector3::x(), Vector3::z(), 2.0);
    let surface = cylinder.to_nurbs().unwrap();
    let patch = surface.bezier_spans().unwrap().remove(0);

    let mid_u = patch.domain_u().start + patch.domain_u().length() * 0.5;
    let mid_v = patch.domain_v().start + patch.domain_v().length() * 0.5;
    let (left, right) = patch.subdivide_u(mid_u.value()).unwrap();
    let (lower, upper) = patch.subdivide_v(mid_v.value()).unwrap();

    assert_eq!(left.domain_u().end, mid_u);
    assert_eq!(right.domain_u().start, mid_u);
    assert_eq!(lower.domain_v().end, mid_v);
    assert_eq!(upper.domain_v().start, mid_v);

    for half in [&left, &right, &lower, &upper] {
        for su in [0.0, 0.5, 1.0] {
            for sv in [0.0, 0.5, 1.0] {
                let u = half.domain_u().at(Fraction::new(su)).value();
                let v = half.domain_v().at(Fraction::new(sv)).value();
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

    let (left, right) = patch.subdivide_u(mid_u.value()).unwrap();

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

    assert!(patch.subdivide_u(patch.domain_u().start.value()).is_err());
    assert!(patch.subdivide_v(patch.domain_v().end.value()).is_err());
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
            let (u, v) = (
                du.at(Fraction::new(f64::from(iu) / 12.0)),
                dv.at(Fraction::new(f64::from(iv) / 12.0)),
            );
            let (u, v) = (u.value(), v.value());
            let (before, after) = (surface.point_at(u, v), clamped.point_at(u, v));
            assert!(
                (before - after).norm() <= LINEAR_TOLERANCE,
                "at ({u}, {v}): {before:?} became {after:?}",
            );
        }
    }
}

mod skinning {
    use std::f64::consts::FRAC_1_SQRT_2;

    use ngk::geometry::{
        ControlPolygon, Degree, HPoint, KnotVector, NurbsCurve, NurbsError, NurbsSurface, Point3,
        SkinningIncompatibility, make_compatible,
    };

    /// A polyline through `points`, as a degree-1 NURBS over `[0, 1]`.
    fn polyline(points: &[Point3]) -> NurbsCurve {
        let degree = Degree::new(1).unwrap();
        let control = ControlPolygon::new(
            points
                .iter()
                .map(|point| HPoint::from_cartesian(*point, 1.0))
                .collect(),
        )
        .unwrap();
        let knots = KnotVector::uniform_clamped(points.len(), degree);
        NurbsCurve::new(degree, control, knots).unwrap()
    }

    /// A rational quarter circle of `radius` at height `z`, in the xy plane.
    fn quarter_arc(radius: f64, z: f64) -> NurbsCurve {
        NurbsCurve::new(
            Degree::new(2).unwrap(),
            ControlPolygon::new(vec![
                HPoint::from_cartesian(Point3::new(radius, 0.0, z), 1.0),
                HPoint::from_cartesian(Point3::new(radius, radius, z), FRAC_1_SQRT_2),
                HPoint::from_cartesian(Point3::new(0.0, radius, z), 1.0),
            ])
            .unwrap(),
            KnotVector::new(vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]).unwrap(),
        )
        .unwrap()
    }

    /// Asserts `S(u, v_k)` reproduces section `k`, for every `k`.
    fn assert_reproduces(surface: &NurbsSurface, sections: &[NurbsCurve], tolerance: f64) {
        let parameters = NurbsSurface::skinning_parameters(sections).unwrap();
        for ((index, section), v) in sections.iter().enumerate().zip(parameters) {
            for step in 0..=16 {
                let u = step as f64 / 16.0;
                let expected = section.point_at(u);
                let actual = surface.point_at(u, v);
                let error = (actual - expected).norm();
                assert!(
                    error <= tolerance,
                    "section {index} at u={u}: expected {expected:?}, got {actual:?} (v={v}, error {error})"
                );
            }
        }
    }

    #[test]
    fn two_sections_at_degree_one_give_a_ruled_surface() {
        let bottom = polyline(&[Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)]);
        let top = polyline(&[Point3::new(0.0, 1.0, 3.0), Point3::new(2.0, 1.0, 3.0)]);
        let surface = NurbsSurface::skinned(&[bottom, top], Degree::new(1).unwrap()).unwrap();

        // Linear in v between the two sections is exactly the midpoint at 0.5.
        let middle = surface.point_at(0.5, 0.5);
        assert!((middle - Point3::new(1.0, 0.5, 1.5)).norm() <= 1e-12);
    }

    #[test]
    fn a_skin_passes_through_every_intermediate_section() {
        let sections = [0.0, 1.0, 2.0, 3.0]
            .iter()
            .enumerate()
            .map(|(index, &z)| {
                let bulge = if index % 2 == 0 { 0.0 } else { 0.8 };
                polyline(&[
                    Point3::new(0.0, 0.0, z),
                    Point3::new(1.0, bulge, z),
                    Point3::new(2.0, 0.0, z),
                ])
            })
            .collect::<Vec<_>>();

        let surface = NurbsSurface::skinned(&sections, Degree::new(3).unwrap()).unwrap();
        assert_eq!(surface.degree_v().get(), 3);
        assert_reproduces(&surface, &sections, 1e-9);
    }

    #[test]
    fn a_skin_through_rational_sections_stays_on_them() {
        let sections = vec![
            quarter_arc(1.0, 0.0),
            quarter_arc(2.0, 1.0),
            quarter_arc(1.5, 2.0),
        ];
        let surface = NurbsSurface::skinned(&sections, Degree::new(2).unwrap()).unwrap();

        assert!(surface.is_rational());
        assert_reproduces(&surface, &sections, 1e-9);
        // Every point of the v = 0 row is one radius from the axis, which a
        // skin through the cartesian control points would not manage.
        for step in 0..=16 {
            let point = surface.point_at(step as f64 / 16.0, 0.0);
            assert!((point.coords.xy().norm() - 1.0).abs() <= 1e-9);
        }
    }

    #[test]
    fn a_skin_of_unlike_sections_needs_them_made_compatible_first() {
        let line = polyline(&[Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)]);
        let arc = quarter_arc(1.0, 2.0);

        assert!(matches!(
            NurbsSurface::skinned(&[line.clone(), arc.clone()], Degree::new(1).unwrap()),
            Err(NurbsError::IncompatibleSkinningSection {
                index: 1,
                reason: SkinningIncompatibility::Degree,
            })
        ));

        let mut sections = vec![line, arc];
        make_compatible(&mut sections).unwrap();
        let surface = NurbsSurface::skinned(&sections, Degree::new(1).unwrap()).unwrap();
        assert_reproduces(&surface, &sections, 1e-9);
    }

    #[test]
    fn skinning_refuses_a_degree_the_section_count_cannot_carry() {
        let sections = vec![
            polyline(&[Point3::origin(), Point3::new(1.0, 0.0, 0.0)]),
            polyline(&[Point3::new(0.0, 0.0, 1.0), Point3::new(1.0, 0.0, 1.0)]),
        ];
        assert!(matches!(
            NurbsSurface::skinned(&sections, Degree::new(3).unwrap()),
            Err(NurbsError::SkinningDegreeTooHigh {
                degree: 3,
                sections: 2
            })
        ));
        assert!(matches!(
            NurbsSurface::skinned(&sections[..1], Degree::new(1).unwrap()),
            Err(NurbsError::InsufficientSkinningSections { minimum: 2, got: 1 })
        ));
    }

    #[test]
    fn isocurves_lie_exactly_on_the_surface() {
        let sections = vec![
            quarter_arc(1.0, 0.0),
            quarter_arc(2.0, 1.0),
            quarter_arc(1.5, 2.0),
        ];
        let surface = NurbsSurface::skinned(&sections, Degree::new(2).unwrap()).unwrap();

        for step in 0..=8 {
            let at = step as f64 / 8.0;
            let rail = surface.isocurve_u(at).unwrap();
            let row = surface.isocurve_v(at).unwrap();
            for sample in 0..=16 {
                let t = sample as f64 / 16.0;
                let v = rail.domain().at(ngk::geometry::Fraction::new(t)).value();
                let u = row.domain().at(ngk::geometry::Fraction::new(t)).value();
                assert!((rail.point_at(v) - surface.point_at(at, v)).norm() <= 1e-12);
                assert!((row.point_at(u) - surface.point_at(u, at)).norm() <= 1e-12);
            }
        }
    }

    #[test]
    fn the_v_zero_isocurve_is_the_first_section() {
        let sections = vec![
            polyline(&[
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(2.0, 0.0, 0.0),
            ]),
            polyline(&[
                Point3::new(0.0, 0.0, 1.0),
                Point3::new(1.0, 2.0, 1.0),
                Point3::new(2.0, 0.0, 1.0),
            ]),
        ];
        let surface = NurbsSurface::skinned(&sections, Degree::new(1).unwrap()).unwrap();

        for (v, section) in [(0.0, &sections[0]), (1.0, &sections[1])] {
            let isocurve = surface.isocurve_v(v).unwrap();
            for step in 0..=16 {
                let u = step as f64 / 16.0;
                assert!((isocurve.point_at(u) - section.point_at(u)).norm() <= 1e-12);
            }
        }
    }
}
