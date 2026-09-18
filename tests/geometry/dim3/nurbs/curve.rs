use std::f64::consts::FRAC_1_SQRT_2;

use nalgebra::Vector3;
use ngk::geometry::{
    Circle, ControlPolygon, Degree, Fraction, HPoint, InterpolationSystem, KnotVector,
    LINEAR_TOLERANCE, NurbsCurve, NurbsError, Plane, Point3, interpolate_with_knots,
    make_compatible,
};

fn assert_vector_near(actual: Vector3<f64>, expected: Vector3<f64>, tol: f64) {
    let error = (actual - expected).norm();
    assert!(
        error <= tol,
        "expected {expected:?}, got {actual:?}, error {error}"
    );
}

fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

#[test]
fn quadratic_bezier_midpoint() {
    let cps = vec![
        HPoint::from_cartesian(Point3::new(0.0, 0.0, 0.0), 1.0),
        HPoint::from_cartesian(Point3::new(1.0, 1.0, 0.0), 1.0),
        HPoint::from_cartesian(Point3::new(2.0, 0.0, 0.0), 1.0),
    ];
    let cp = ControlPolygon::new(cps).unwrap();
    let knots = KnotVector::new(vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]).unwrap();
    let curve = NurbsCurve::new(Degree::new(2).unwrap(), cp, knots).unwrap();
    let p = curve.point_at(0.5);
    assert!(approx_eq(p.x, 1.0, 1e-10), "x = {}", p.x);
    assert!(approx_eq(p.y, 0.5, 1e-10), "y = {}", p.y);
    assert!(approx_eq(p.z, 0.0, 1e-10), "z = {}", p.z);
}

#[test]
fn cubic_bezier_endpoints() {
    let pts = vec![
        Point3::new(-2.0, 0.0, 0.0),
        Point3::new(-1.0, 2.0, 0.0),
        Point3::new(1.0, -2.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
    ];
    let cp = ControlPolygon::from_cartesian(pts.clone(), &[1.0, 1.0, 1.0, 1.0]).unwrap();
    let curve = NurbsCurve::with_uniform_knots(Degree::new(3).unwrap(), cp).unwrap();

    let p0 = curve.point_at(0.0);
    assert!((p0 - pts[0]).norm() < 1e-10);

    let p1 = curve.point_at(1.0);
    assert!((p1 - pts[3]).norm() < 1e-10);
}

#[test]
fn line_nurbs_derivative_matches_line_direction() {
    let start = Point3::new(1.0, 2.0, 3.0);
    let end = Point3::new(4.0, 6.0, 8.0);
    let cp = ControlPolygon::from_cartesian(vec![start, end], &[1.0, 1.0]).unwrap();
    let curve = NurbsCurve::with_uniform_knots(Degree::new(1).unwrap(), cp).unwrap();

    assert_vector_near(curve.derivative_at(0.35, 1), end - start, 1e-12);
    assert_vector_near(curve.derivative_at(0.35, 2), Vector3::zeros(), 1e-12);
}

#[test]
fn insert_knot_preserves_shape() {
    let pts = vec![
        Point3::new(-2.0, 0.0, 0.0),
        Point3::new(-1.0, 2.0, 0.0),
        Point3::new(1.0, -2.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
    ];
    let cp = ControlPolygon::from_cartesian(pts, &[1.0, 1.0, 1.0, 1.0]).unwrap();
    let mut curve = NurbsCurve::with_uniform_knots(Degree::new(3).unwrap(), cp).unwrap();

    let orig_samples: Vec<_> = (0..=20).map(|i| curve.point_at(i as f64 / 20.0)).collect();
    let orig_cp_count = curve.control_points().len();
    let orig_knot_count = curve.knots().len();

    curve.insert_knot(0.5);

    assert_eq!(curve.control_points().len(), orig_cp_count + 1);
    assert_eq!(curve.knots().len(), orig_knot_count + 1);

    for (i, orig) in orig_samples.iter().enumerate() {
        let p = curve.point_at(i as f64 / 20.0);
        let err = (p - orig).norm();
        assert!(err < LINEAR_TOLERANCE, "sample {} deviates by {}", i, err);
    }
}

#[test]
fn insert_knot_quadratic_s1() {
    let pts = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(2.0, -1.0, 0.0),
        Point3::new(3.0, 0.0, 0.0),
    ];
    let cp = ControlPolygon::from_cartesian(pts, &[1.0, 1.0, 1.0, 1.0]).unwrap();
    let mut curve = NurbsCurve::with_uniform_knots(Degree::new(2).unwrap(), cp).unwrap();

    assert_eq!(curve.knots().len(), 7);
    assert_eq!(curve.control_points().len(), 4);

    let orig_samples: Vec<_> = (0..=20).map(|i| curve.point_at(i as f64 / 20.0)).collect();

    curve.insert_knot(0.5);

    assert_eq!(
        curve.knots().len(),
        8,
        "knots after insert: {:?}",
        curve.knots().as_slice()
    );
    assert_eq!(curve.control_points().len(), 5);

    for (i, orig) in orig_samples.iter().enumerate() {
        let p = curve.point_at(i as f64 / 20.0);
        let err = (p - orig).norm();
        assert!(err < LINEAR_TOLERANCE, "sample {} deviates by {}", i, err);
    }
}

#[test]
fn rational_circle_quarter() {
    let w = FRAC_1_SQRT_2;
    let cps = vec![
        HPoint::from_cartesian(Point3::new(1.0, 0.0, 0.0), 1.0),
        HPoint::from_cartesian(Point3::new(1.0, 1.0, 0.0), w),
        HPoint::from_cartesian(Point3::new(0.0, 1.0, 0.0), 1.0),
    ];
    let cp = ControlPolygon::new(cps).unwrap();
    let knots = KnotVector::new(vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]).unwrap();
    let curve = NurbsCurve::new(Degree::new(2).unwrap(), cp, knots).unwrap();

    for i in 0..=16 {
        let t = i as f64 / 16.0;
        let p = curve.point_at(t);
        let r = (p.x * p.x + p.y * p.y).sqrt();
        assert!((r - 1.0).abs() < 1e-10, "r={} at t={}", r, t);
    }

    let _ = Vector3::new(1.0, 0.0, 0.0);
}

#[test]
fn rational_quarter_circle_length_matches_arc_length() {
    let w = FRAC_1_SQRT_2;
    let cps = vec![
        HPoint::from_cartesian(Point3::new(1.0, 0.0, 0.0), 1.0),
        HPoint::from_cartesian(Point3::new(1.0, 1.0, 0.0), w),
        HPoint::from_cartesian(Point3::new(0.0, 1.0, 0.0), 1.0),
    ];
    let cp = ControlPolygon::new(cps).unwrap();
    let knots = KnotVector::new(vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]).unwrap();
    let curve = NurbsCurve::new(Degree::new(2).unwrap(), cp, knots).unwrap();
    let length = curve.length(0.0, 1.0);
    assert!((length - std::f64::consts::FRAC_PI_2).abs() < 1e-8);
}

#[test]
fn bezier_spans_returns_single_span_for_bezier_curve() {
    let pts = vec![
        Point3::new(-2.0, 0.0, 0.0),
        Point3::new(-1.0, 2.0, 0.0),
        Point3::new(1.0, -2.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
    ];
    let cp = ControlPolygon::from_cartesian(pts, &[1.0, 1.0, 1.0, 1.0]).unwrap();
    let curve = NurbsCurve::with_uniform_knots(Degree::new(3).unwrap(), cp).unwrap();

    let spans = curve.bezier_spans().unwrap();

    assert_eq!(spans.len(), 1);
    assert_eq!(spans[0].domain(), curve.domain());
}

#[test]
fn bezier_spans_splits_curve_at_interior_knots() {
    let pts = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(2.0, -1.0, 0.0),
        Point3::new(3.0, 0.0, 0.0),
    ];
    let cp = ControlPolygon::from_cartesian(pts, &[1.0, 1.0, 1.0, 1.0]).unwrap();
    let curve = NurbsCurve::with_uniform_knots(Degree::new(2).unwrap(), cp).unwrap();

    let spans = curve.bezier_spans().unwrap();

    assert_eq!(spans.len(), 2);
    for span in spans {
        let domain = span.domain();
        for i in 0..=8 {
            let u = domain.start + (domain.end - domain.start) * i as f64 / 8.0;
            let error = (span.point_at(u.value()) - curve.point_at(u.value())).norm();
            assert!(error <= LINEAR_TOLERANCE, "u={u}, error={error}");
        }
    }
}

#[test]
fn bezier_spans_extracts_four_quadratic_circle_arcs() {
    let circle = Circle::new(Plane::xy(), 1.0);
    let curve = circle.to_nurbs().unwrap();

    let spans = curve.bezier_spans().unwrap();

    assert_eq!(spans.len(), 4);
    assert!(
        spans
            .iter()
            .all(|span| span.degree() == Degree::new(2).unwrap())
    );
    for span in spans {
        let domain = span.domain();
        for i in 0..=8 {
            let u = domain.start + (domain.end - domain.start) * i as f64 / 8.0;
            let point = span.point_at(u.value());
            let radius = (point.x * point.x + point.y * point.y).sqrt();
            assert!((radius - 1.0).abs() <= 1.0e-10);
        }
    }
}

/// An unclamped knot vector of the shape a periodic spline arrives in.
///
/// Degree 2, six control points, and knots that repeat nowhere: the curve runs
/// on either side of its own domain, which is what clamping has to cut away
/// without moving the part in between.
fn unclamped_curve() -> NurbsCurve {
    let points = (0..6)
        .map(|i| {
            let angle = i as f64 * std::f64::consts::FRAC_PI_3;
            HPoint::from_cartesian(Point3::new(angle.cos(), angle.sin(), i as f64 * 0.25), 1.0)
        })
        .collect();
    NurbsCurve::new(
        Degree::new(2).expect("degree 2"),
        ControlPolygon::new(points).expect("six control points"),
        KnotVector::new((0..9).map(|i| i as f64).collect()).expect("nine knots"),
    )
    .expect("6 + 2 + 1 knots")
}

#[test]
fn clamping_keeps_the_curve_and_its_domain() {
    // The whole point set has to survive: knot insertion does not move a
    // curve, and dropping the control points that only reach outside the
    // domain does not either. A clamp that got the trim wrong slides the
    // curve along itself, which looks plausible and is not the same shape.
    let curve = unclamped_curve();
    assert!(!curve.knots().is_clamped(curve.degree()));

    let clamped = curve.clamped().expect("an unclamped curve should clamp");
    assert!(clamped.knots().is_clamped(clamped.degree()));

    let domain = curve.domain();
    assert_eq!(clamped.domain(), domain);
    for step in 0..=32 {
        let u = domain.at(Fraction::new(f64::from(step) / 32.0));
        let (before, after) = (curve.point_at(u.value()), clamped.point_at(u.value()));
        assert!(
            (before - after).norm() <= LINEAR_TOLERANCE,
            "at {u}: {before:?} became {after:?}",
        );
    }
}

#[test]
fn clamping_an_already_clamped_curve_changes_nothing() {
    let curve = NurbsCurve::with_uniform_knots(
        Degree::new(3).expect("degree 3"),
        ControlPolygon::new(
            (0..5)
                .map(|i| HPoint::from_cartesian(Point3::new(i as f64, 0.0, 0.0), 1.0))
                .collect(),
        )
        .expect("five control points"),
    )
    .expect("a clamped uniform curve");

    assert_eq!(curve.clamped().expect("already clamped"), curve);
}

#[test]
fn trimming_a_polyline_an_ulp_off_a_knot_lands_on_the_knot() {
    // An edge's span end is recovered from its vertex, so it reaches an
    // interior knot a rounding step to one side of it. Splitting where it
    // literally asked would leave a span narrower than the parameter can
    // resolve, which no subdivision search can take apart.
    let knot = 0.25;
    let polyline = NurbsCurve::new(
        Degree::new(1).unwrap(),
        ControlPolygon::new(vec![
            HPoint::from_cartesian(Point3::origin(), 1.0),
            HPoint::from_cartesian(Point3::new(1.0, 0.0, 0.0), 1.0),
            HPoint::from_cartesian(Point3::new(1.0, 3.0, 0.0), 1.0),
        ])
        .unwrap(),
        KnotVector::new(vec![0.0, 0.0, knot, 1.0, 1.0]).unwrap(),
    )
    .unwrap();

    let section = polyline
        .trimmed(f64::from_bits(knot.to_bits() - 1), 1.0)
        .unwrap();

    // Two knots a rounding step apart would be a Bezier piece with no extent.
    let knots = section.knots().as_slice().to_vec();
    for pair in knots.windows(2) {
        let gap = pair[1] - pair[0];
        assert!(
            gap <= 0.0 || gap > LINEAR_TOLERANCE,
            "knot span [{}, {}] is narrower than the tolerance",
            pair[0],
            pair[1]
        );
    }
}

/// Samples both curves over `[0, 1]` and asserts they trace the same points.
///
/// This is the whole contract of normalizing, refining and elevating: each
/// changes the representation and nothing else.
fn assert_traces_same(actual: &NurbsCurve, expected: &NurbsCurve, tolerance: f64) {
    for step in 0..=64 {
        let t = step as f64 / 64.0;
        let a = actual.point_at(actual.domain().at(Fraction::new(t)).value());
        let b = expected.point_at(expected.domain().at(Fraction::new(t)).value());
        let error = (a - b).norm();
        assert!(
            error <= tolerance,
            "at fraction {t}: expected {b:?}, got {a:?}, error {error}"
        );
    }
}

fn wavy_cubic() -> NurbsCurve {
    NurbsCurve::interpolate(&[
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 2.0, 0.5),
        Point3::new(3.0, -1.0, 1.0),
        Point3::new(4.5, 0.5, -0.5),
        Point3::new(6.0, 0.0, 0.0),
    ])
    .unwrap()
}

/// A rational quadratic quarter circle of radius 1 in the xy plane.
fn quarter_arc() -> NurbsCurve {
    NurbsCurve::new(
        Degree::new(2).unwrap(),
        ControlPolygon::new(vec![
            HPoint::from_cartesian(Point3::new(1.0, 0.0, 0.0), 1.0),
            HPoint::from_cartesian(Point3::new(1.0, 1.0, 0.0), FRAC_1_SQRT_2),
            HPoint::from_cartesian(Point3::new(0.0, 1.0, 0.0), 1.0),
        ])
        .unwrap(),
        KnotVector::new(vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]).unwrap(),
    )
    .unwrap()
}

#[test]
fn normalized_maps_the_domain_without_moving_the_curve() {
    let source = NurbsCurve::new(
        Degree::new(2).unwrap(),
        ControlPolygon::new(vec![
            HPoint::from_cartesian(Point3::new(0.0, 0.0, 0.0), 1.0),
            HPoint::from_cartesian(Point3::new(1.0, 2.0, 0.0), 1.0),
            HPoint::from_cartesian(Point3::new(3.0, 0.0, 0.0), 1.0),
            HPoint::from_cartesian(Point3::new(4.0, 1.0, 0.0), 1.0),
        ])
        .unwrap(),
        KnotVector::new(vec![2.0, 2.0, 2.0, 5.0, 7.0, 7.0, 7.0]).unwrap(),
    )
    .unwrap();

    let normalized = source.normalized().unwrap();
    assert!(approx_eq(normalized.domain().start.value(), 0.0, 1e-15));
    assert!(approx_eq(normalized.domain().end.value(), 1.0, 1e-15));
    assert_traces_same(&normalized, &source, 1e-12);
}

#[test]
fn refined_inserts_every_knot_without_moving_the_curve() {
    let source = wavy_cubic();
    let refined = source.refined(&[0.1, 0.25, 0.25, 0.9]).unwrap();

    assert_eq!(
        refined.control_points().len(),
        source.control_points().len() + 4
    );
    for knot in [0.1, 0.9] {
        assert_eq!(refined.knots().multiplicity(knot), 1);
    }
    assert_eq!(refined.knots().multiplicity(0.25), 2);
    assert_traces_same(&refined, &source, 1e-12);
}

#[test]
fn refined_matches_repeated_single_insertion() {
    let source = wavy_cubic();
    let mut one_at_a_time = source.clone();
    for knot in [0.2, 0.55, 0.55] {
        one_at_a_time.insert_knot(knot);
    }
    let at_once = source.refined(&[0.2, 0.55, 0.55]).unwrap();

    assert_eq!(at_once.knots().as_slice(), one_at_a_time.knots().as_slice());
    for (a, b) in at_once
        .control_points()
        .iter()
        .zip(one_at_a_time.control_points().iter())
    {
        assert!((a.to_cartesian() - b.to_cartesian()).norm() <= 1e-12);
    }
}

#[test]
fn refined_refuses_a_knot_outside_the_domain() {
    let source = wavy_cubic();
    assert!(matches!(
        source.refined(&[1.5]),
        Err(NurbsError::ParameterOutOfRange { .. })
    ));
    assert!(matches!(
        source.refined(&[0.0]),
        Err(NurbsError::ParameterOutOfRange { .. })
    ));
}

#[test]
fn elevated_degree_raises_the_degree_without_moving_the_curve() {
    let source = wavy_cubic();
    for target in [4, 5, 6] {
        let elevated = source
            .elevated_degree(Degree::new(target).unwrap())
            .unwrap();
        assert_eq!(elevated.degree().get(), target);
        assert_traces_same(&elevated, &source, 1e-10);
    }
}

#[test]
fn elevated_degree_keeps_a_rational_arc_on_its_circle() {
    let source = quarter_arc();
    let elevated = source.elevated_degree(Degree::new(4).unwrap()).unwrap();

    assert_eq!(elevated.degree().get(), 4);
    assert!(elevated.is_rational());
    for step in 0..=32 {
        let point = elevated.point_at(step as f64 / 32.0);
        assert!(
            (point.coords.norm() - 1.0).abs() <= 1e-12,
            "elevated arc left the unit circle at {point:?}"
        );
    }
    assert_traces_same(&elevated, &source, 1e-12);
}

#[test]
fn elevated_degree_refuses_to_lower_a_degree() {
    let source = wavy_cubic();
    assert!(matches!(
        source.elevated_degree(Degree::new(2).unwrap()),
        Err(NurbsError::DegreeReductionRefused { from: 3, to: 2 })
    ));
}

#[test]
fn make_compatible_agrees_a_line_with_an_arc_and_moves_neither() {
    let line = NurbsCurve::new(
        Degree::new(1).unwrap(),
        ControlPolygon::new(vec![
            HPoint::from_cartesian(Point3::new(0.0, 0.0, 1.0), 1.0),
            HPoint::from_cartesian(Point3::new(0.0, 1.0, 1.0), 1.0),
        ])
        .unwrap(),
        KnotVector::new(vec![0.0, 0.0, 1.0, 1.0]).unwrap(),
    )
    .unwrap();
    let arc = quarter_arc();
    let sources = [line.clone(), arc.clone()];

    let mut curves = vec![line, arc];
    make_compatible(&mut curves).unwrap();

    assert_eq!(curves[0].degree(), curves[1].degree());
    assert_eq!(curves[0].degree().get(), 2);
    assert_eq!(
        curves[0].control_points().len(),
        curves[1].control_points().len()
    );
    assert_eq!(curves[0].knots().as_slice(), curves[1].knots().as_slice());
    for curve in &curves {
        assert!(approx_eq(curve.domain().start.value(), 0.0, 1e-15));
        assert!(approx_eq(curve.domain().end.value(), 1.0, 1e-15));
    }
    for (compatible, source) in curves.iter().zip(&sources) {
        assert_traces_same(compatible, source, 1e-12);
    }
}

#[test]
fn make_compatible_takes_the_union_of_interior_knots() {
    let first = wavy_cubic().refined(&[0.3]).unwrap();
    let second = wavy_cubic().refined(&[0.7]).unwrap();
    let sources = [first.clone(), second.clone()];

    let mut curves = vec![first, second];
    make_compatible(&mut curves).unwrap();

    assert_eq!(curves[0].knots().as_slice(), curves[1].knots().as_slice());
    for curve in &curves {
        assert!(curve.knots().multiplicity(0.3) >= 1);
        assert!(curve.knots().multiplicity(0.7) >= 1);
    }
    for (compatible, source) in curves.iter().zip(&sources) {
        assert_traces_same(compatible, source, 1e-12);
    }
}

#[test]
fn interpolate_with_knots_passes_through_rational_samples() {
    let points = vec![
        HPoint::from_cartesian(Point3::new(0.0, 0.0, 0.0), 1.0),
        HPoint::from_cartesian(Point3::new(1.0, 1.0, 0.0), 0.5),
        HPoint::from_cartesian(Point3::new(2.0, 0.0, 0.0), 2.0),
        HPoint::from_cartesian(Point3::new(3.0, 1.0, 0.0), 1.0),
    ];
    let parameters = [0.0, 0.3, 0.7, 1.0];
    let degree = Degree::new(2).unwrap();
    let knots = KnotVector::averaged(&parameters, degree).unwrap();

    let curve = interpolate_with_knots(&points, &parameters, degree, &knots).unwrap();

    for (parameter, point) in parameters.iter().zip(&points) {
        let traced = curve.point_at(*parameter);
        let expected = point.to_cartesian();
        assert!(
            (traced - expected).norm() <= 1e-10,
            "at {parameter}: expected {expected:?}, got {traced:?}"
        );
    }
}

#[test]
fn interpolation_system_solves_many_rows_from_one_factorization() {
    let parameters = [0.0, 0.25, 0.75, 1.0];
    let degree = Degree::new(3).unwrap();
    let knots = KnotVector::averaged(&parameters, degree).unwrap();
    let system = InterpolationSystem::new(&parameters, degree, &knots).unwrap();

    for offset in 0..3 {
        let points = (0..4)
            .map(|index| {
                HPoint::from_cartesian(
                    Point3::new(index as f64, (offset + index) as f64, offset as f64),
                    1.0,
                )
            })
            .collect::<Vec<_>>();
        let control_points = system.solve(&points).unwrap();
        let curve = NurbsCurve::new(degree, control_points, knots.clone()).unwrap();
        for (parameter, point) in parameters.iter().zip(&points) {
            assert!((curve.point_at(*parameter) - point.to_cartesian()).norm() <= 1e-10);
        }
    }
}
