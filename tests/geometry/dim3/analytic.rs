//! The analytic table agrees with the general solver, and says when it declines.
//!
//! Every case is checked two ways: the section is verified against both
//! supports directly, and the answer is compared with what the NURBS solver
//! finds for the same pair. The solver is the oracle rather than the authority
//! -- it is the code the table exists to replace -- so the direct checks come
//! first and the differential ones confirm the two agree.

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI, TAU};

use nalgebra::Vector3;
use ngk::geometry::{
    AnalyticSurfaceIntersection, ControlNet, Curve, Cylinder, Degree, Frame, HPoint,
    IntersectionOptions, KnotVector, LINEAR_TOLERANCE, NurbsSurface, PcurveFidelity, Plane, Point3,
    Sphere, Surface, SurfaceSurfaceIntersection, intersect_analytic_surfaces, intersect_surfaces,
};

fn options() -> IntersectionOptions {
    IntersectionOptions::default()
}

fn analytic(a: &Surface, b: &Surface) -> AnalyticSurfaceIntersection {
    intersect_analytic_surfaces(a, b, options())
        .expect("the pair should be in the analytic table")
        .expect("the analytic table should not fail on a well-posed pair")
}

/// Asserts that every section lies on both supports and that its pcurves lift
/// back onto it, which is what "the pcurves are synchronized" has to mean.
fn assert_sections_are_consistent(
    intersection: &AnalyticSurfaceIntersection,
    a: &Surface,
    b: &Surface,
) {
    let sections = intersection.sections();
    assert!(
        !sections.is_empty(),
        "expected sections, got {intersection:?}"
    );
    for section in sections {
        for index in 0..=32 {
            let t = index as f64 / 32.0;
            let point = section.curve.point_at(t);
            for surface in [a, b] {
                let uv = surface.param_at(point).expect("closest parameter");
                let distance = (surface.point_at(uv.x, uv.y) - point).norm();
                assert!(
                    distance <= LINEAR_TOLERANCE * 10.0,
                    "section left its support by {distance} at t={t}"
                );
            }
            let uv_a = section.pcurve_a.point_at(t);
            let uv_b = section.pcurve_b.point_at(t);
            let tolerance = (section.fidelity.deviation() + LINEAR_TOLERANCE).max(1.0e-6) * 10.0;
            assert!(
                (a.point_at(uv_a.x, uv_a.y) - point).norm() <= tolerance,
                "pcurve on a missed the section at t={t}"
            );
            assert!(
                (b.point_at(uv_b.x, uv_b.y) - point).norm() <= tolerance,
                "pcurve on b missed the section at t={t}"
            );
        }
    }
}

/// Every branch the NURBS solver reports, sampled.
fn solver_points(a: &Surface, b: &Surface) -> Vec<Point3> {
    let results = intersect_surfaces(a, b).expect("solver");
    let mut points = Vec::new();
    for result in results.intersections() {
        match result {
            SurfaceSurfaceIntersection::Branch(branch) => {
                for index in 0..=32 {
                    points.push(branch.curve_3d.point_at(index as f64 / 32.0));
                }
            }
            SurfaceSurfaceIntersection::Point(point) => points.push(point.point),
            SurfaceSurfaceIntersection::OverlapCandidate(_) => {}
        }
    }
    points
}

/// Asserts that the solver's point set lies on the analytic sections.
///
/// One-directional on purpose: the solver realizes unbounded supports over a
/// finite patch, so it legitimately reports less than the table does.
fn assert_solver_agrees(intersection: &AnalyticSurfaceIntersection, a: &Surface, b: &Surface) {
    let sections = intersection.sections();
    for point in solver_points(a, b) {
        // Projected rather than sampled: a sampled comparison measures the
        // sample spacing, not the disagreement, and would pass or fail on the
        // sample count alone.
        let closest = sections
            .iter()
            .map(|section| {
                let parameter = section.curve.param_at(point).clamp(0.0, 1.0);
                (section.curve.point_at(parameter) - point).norm()
            })
            .fold(f64::INFINITY, f64::min);
        assert!(
            closest <= 1.0e-5,
            "the solver found {point:?}, which is {closest} from every analytic section"
        );
    }
}

fn unit_sphere_at(centre: Point3, radius: f64) -> Surface {
    Surface::Sphere(Sphere::new(
        Frame::from_xy(centre, Vector3::x(), Vector3::y()),
        radius,
    ))
}

fn upright_cylinder(radius: f64) -> Surface {
    Surface::Cylinder(Cylinder::new(
        Point3::origin(),
        Vector3::x(),
        Vector3::z(),
        radius,
    ))
}

#[test]
fn two_crossing_planes_meet_in_one_exact_line() {
    let a = Surface::Plane(Plane::xy());
    let b = Surface::Plane(Plane::xz());
    let intersection = analytic(&a, &b);
    let sections = intersection.sections();
    assert_eq!(sections.len(), 1);
    assert_eq!(sections[0].fidelity, PcurveFidelity::Exact);
    assert!(matches!(sections[0].curve, Curve::Line(_)));
    assert_sections_are_consistent(&intersection, &a, &b);
}

#[test]
fn parallel_planes_are_empty_or_coincident_rather_than_unanswered() {
    let base = Surface::Plane(Plane::xy());
    let offset = Surface::Plane(Plane::from_xy(
        Point3::new(0.0, 0.0, 3.0),
        Vector3::x(),
        Vector3::y(),
    ));
    assert_eq!(analytic(&base, &offset), AnalyticSurfaceIntersection::Empty);

    let same = Surface::Plane(Plane::from_xy(
        Point3::new(4.0, 5.0, 0.0),
        Vector3::x(),
        Vector3::y(),
    ));
    assert_eq!(
        analytic(&base, &same),
        AnalyticSurfaceIntersection::Coincident
    );
}

#[test]
fn a_plane_cuts_a_sphere_in_a_circle_exact_in_the_plane() {
    let plane = Surface::Plane(Plane::from_xy(
        Point3::new(0.0, 0.0, 0.5),
        Vector3::x(),
        Vector3::y(),
    ));
    let sphere = unit_sphere_at(Point3::origin(), 1.0);
    let intersection = analytic(&plane, &sphere);
    let sections = intersection.sections();
    assert_eq!(sections.len(), 1);
    assert_sections_are_consistent(&intersection, &plane, &sphere);
    assert_solver_agrees(&intersection, &plane, &sphere);

    // The plane is square to the sphere's axis, so the section is one latitude
    // and both pcurves have closed forms.
    assert_eq!(sections[0].fidelity, PcurveFidelity::Exact);
}

#[test]
fn a_tilted_plane_through_a_sphere_fits_its_spherical_pcurve_and_measures_it() {
    let plane = Surface::Plane(Plane::from_xy(
        Point3::new(0.0, 0.0, 0.2),
        Vector3::x(),
        Vector3::new(0.0, 1.0, 0.6),
    ));
    let sphere = unit_sphere_at(Point3::origin(), 1.0);
    let intersection = analytic(&plane, &sphere);
    assert_sections_are_consistent(&intersection, &plane, &sphere);
    assert_solver_agrees(&intersection, &plane, &sphere);

    // A circle on a sphere is transcendental in longitude and latitude, so the
    // spherical pcurve is a fit -- and says so, with its deviation measured
    // rather than assumed.
    let worst = intersection
        .sections()
        .iter()
        .map(|section| section.fidelity.deviation())
        .fold(0.0_f64, f64::max);
    assert!(worst > 0.0, "a fitted pcurve should report a deviation");
    assert!(worst <= options().fit_tolerance, "deviation {worst}");
}

#[test]
fn a_plane_missing_a_sphere_is_certified_empty() {
    let plane = Surface::Plane(Plane::from_xy(
        Point3::new(0.0, 0.0, 5.0),
        Vector3::x(),
        Vector3::y(),
    ));
    let sphere = unit_sphere_at(Point3::origin(), 1.0);
    assert_eq!(
        analytic(&plane, &sphere),
        AnalyticSurfaceIntersection::Empty
    );
}

#[test]
fn a_plane_tangent_to_a_sphere_reports_the_touch_point() {
    let plane = Surface::Plane(Plane::from_xy(
        Point3::new(0.0, 0.0, 1.0),
        Vector3::x(),
        Vector3::y(),
    ));
    let sphere = unit_sphere_at(Point3::origin(), 1.0);
    let AnalyticSurfaceIntersection::TangentPoint(point) = analytic(&plane, &sphere) else {
        panic!("a tangent plane should report its touch point");
    };
    assert!((point - Point3::new(0.0, 0.0, 1.0)).norm() <= LINEAR_TOLERANCE);
}

#[test]
fn a_plane_square_to_a_cylinder_cuts_a_circle_exact_on_both_supports() {
    let plane = Surface::Plane(Plane::from_xy(
        Point3::new(0.0, 0.0, 2.0),
        Vector3::x(),
        Vector3::y(),
    ));
    let cylinder = upright_cylinder(1.5);
    let intersection = analytic(&plane, &cylinder);
    assert_eq!(intersection.sections().len(), 1);
    assert_eq!(intersection.sections()[0].fidelity, PcurveFidelity::Exact);
    assert_sections_are_consistent(&intersection, &plane, &cylinder);
    assert_solver_agrees(&intersection, &plane, &cylinder);
}

#[test]
fn a_tilted_plane_cuts_a_cylinder_in_an_ellipse() {
    let plane = Surface::Plane(Plane::from_xy(
        Point3::new(0.0, 0.0, 1.0),
        Vector3::x(),
        Vector3::new(0.0, 1.0, 0.5),
    ));
    let cylinder = upright_cylinder(1.0);
    let intersection = analytic(&plane, &cylinder);
    let sections = intersection.sections();
    assert!(!sections.is_empty());
    assert_sections_are_consistent(&intersection, &plane, &cylinder);
    assert_solver_agrees(&intersection, &plane, &cylinder);
}

#[test]
fn a_plane_along_a_cylinder_axis_cuts_two_rulings() {
    let plane = Surface::Plane(Plane::from_xy(
        Point3::new(0.4, 0.0, 0.0),
        Vector3::y(),
        Vector3::z(),
    ));
    let cylinder = upright_cylinder(1.0);
    let intersection = analytic(&plane, &cylinder);
    assert_eq!(
        intersection.sections().len(),
        2,
        "a chord plane cuts a cylinder in two rulings"
    );
    for section in intersection.sections() {
        assert!(matches!(section.curve, Curve::Line(_)));
        assert_eq!(section.fidelity, PcurveFidelity::Exact);
    }
    assert_sections_are_consistent(&intersection, &plane, &cylinder);
}

#[test]
fn a_plane_clear_of_a_cylinder_is_certified_empty() {
    let plane = Surface::Plane(Plane::from_xy(
        Point3::new(5.0, 0.0, 0.0),
        Vector3::y(),
        Vector3::z(),
    ));
    assert_eq!(
        analytic(&plane, &upright_cylinder(1.0)),
        AnalyticSurfaceIntersection::Empty
    );
}

#[test]
fn two_overlapping_spheres_meet_in_a_circle() {
    let a = unit_sphere_at(Point3::origin(), 1.0);
    let b = unit_sphere_at(Point3::new(1.2, 0.0, 0.0), 1.0);
    let intersection = analytic(&a, &b);
    assert!(!intersection.sections().is_empty());
    assert_sections_are_consistent(&intersection, &a, &b);
    assert_solver_agrees(&intersection, &a, &b);
}

#[test]
fn two_spheres_apart_or_nested_or_identical_are_answered_without_a_search() {
    let a = unit_sphere_at(Point3::origin(), 1.0);
    assert_eq!(
        analytic(&a, &unit_sphere_at(Point3::new(5.0, 0.0, 0.0), 1.0)),
        AnalyticSurfaceIntersection::Empty
    );
    assert_eq!(
        analytic(&a, &unit_sphere_at(Point3::origin(), 0.5)),
        AnalyticSurfaceIntersection::Empty
    );
    assert_eq!(
        analytic(&a, &unit_sphere_at(Point3::origin(), 1.0)),
        AnalyticSurfaceIntersection::Coincident
    );
    let AnalyticSurfaceIntersection::TangentPoint(point) =
        analytic(&a, &unit_sphere_at(Point3::new(2.0, 0.0, 0.0), 1.0))
    else {
        panic!("touching spheres should report their touch point");
    };
    assert!((point - Point3::new(1.0, 0.0, 0.0)).norm() <= LINEAR_TOLERANCE);
}

#[test]
fn a_section_crossing_a_seam_is_split_inside_one_period() {
    // The plane's own x direction points away from the sphere's, so the
    // section starts mid-period and runs across longitude zero.
    let plane = Surface::Plane(Plane::from_xy(
        Point3::new(0.0, 0.0, 0.3),
        Vector3::new(-1.0, -0.2, 0.0),
        Vector3::new(0.2, -1.0, 0.0),
    ));
    let sphere = unit_sphere_at(Point3::origin(), 1.0);
    let intersection = analytic(&plane, &sphere);
    assert_sections_are_consistent(&intersection, &plane, &sphere);
    for section in intersection.sections() {
        for index in 0..=32 {
            let u = section.pcurve_b.point_at(index as f64 / 32.0).x;
            assert!(
                (-LINEAR_TOLERANCE..=TAU + LINEAR_TOLERANCE).contains(&u),
                "a spherical pcurve left its period at u={u}"
            );
        }
    }
}

#[test]
fn a_free_form_pair_is_declined_rather_than_guessed() {
    let paraboloid = square_paraboloid(0.5);
    let plane = Surface::Plane(Plane::xy());
    assert!(
        intersect_analytic_surfaces(&paraboloid, &plane, options()).is_none(),
        "a NURBS support is not in the table and must be declined"
    );
}

#[test]
fn a_cone_pair_is_declined_until_its_table_entry_exists() {
    let cone = Surface::Cone(ngk::geometry::Cone::new(Frame::xyz(), 1.0, FRAC_PI_4));
    let plane = Surface::Plane(Plane::xy());
    assert!(
        intersect_analytic_surfaces(&cone, &plane, options()).is_none(),
        "declining is the contract for a pair with no entry"
    );
}

#[test]
fn a_sphere_seam_meridian_section_stays_inside_its_period() {
    // A plane containing the sphere's axis cuts a great circle through both
    // poles, so its spherical pcurve runs along two opposite meridians.
    let plane = Surface::Plane(Plane::from_xy(Point3::origin(), Vector3::x(), Vector3::z()));
    let sphere = unit_sphere_at(Point3::origin(), 1.0);
    let intersection = analytic(&plane, &sphere);
    assert_sections_are_consistent(&intersection, &plane, &sphere);
    for section in intersection.sections() {
        for index in 0..=32 {
            let uv = section.pcurve_b.point_at(index as f64 / 32.0);
            assert!(
                (-LINEAR_TOLERANCE..=TAU + LINEAR_TOLERANCE).contains(&uv.x),
                "a pole-crossing pcurve left its period at u={}",
                uv.x
            );
            assert!(
                (-FRAC_PI_2 - LINEAR_TOLERANCE..=FRAC_PI_2 + LINEAR_TOLERANCE).contains(&uv.y),
                "a pole-crossing pcurve left the latitude range at v={}",
                uv.y
            );
        }
    }
}

#[test]
fn a_plane_through_a_sphere_centre_matches_the_solver_at_every_orientation() {
    let sphere = unit_sphere_at(Point3::new(0.3, -0.2, 0.1), 1.4);
    for angle in [0.0, 0.31, 0.77, FRAC_PI_4, 1.3, FRAC_PI_2 - 0.05, PI * 0.75] {
        let plane = Surface::Plane(Plane::from_xy(
            Point3::new(0.3, -0.2, 0.4),
            Vector3::new(angle.cos(), angle.sin(), 0.0),
            Vector3::new(-angle.sin(), angle.cos(), 0.35),
        ));
        let intersection = analytic(&plane, &sphere);
        assert_sections_are_consistent(&intersection, &plane, &sphere);
        assert_solver_agrees(&intersection, &plane, &sphere);
    }
}

/// Exact biquadratic patch `z = x^2 + y^2 - radius^2` over `[-1, 1]^2`.
fn square_paraboloid(radius: f64) -> Surface {
    let coordinates = [-1.0, 0.0, 1.0];
    let square_coefficients = [1.0, -1.0, 1.0];
    let points = (0..3)
        .flat_map(|v| {
            (0..3).map(move |u| {
                HPoint::from_cartesian(
                    Point3::new(
                        coordinates[u],
                        coordinates[v],
                        square_coefficients[u] + square_coefficients[v] - radius * radius,
                    ),
                    1.0,
                )
            })
        })
        .collect();
    let knots = KnotVector::new(vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]).unwrap();
    Surface::Nurbs(
        NurbsSurface::new(
            Degree::new(2).unwrap(),
            Degree::new(2).unwrap(),
            ControlNet::new(points, 3, 3).unwrap(),
            knots.clone(),
            knots,
        )
        .unwrap(),
    )
}
