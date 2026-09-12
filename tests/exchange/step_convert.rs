//! The geometry mapping, with no map and no topology anywhere.
//!
//! This is where the parameterization bugs live, and they are the kind that
//! never crash: a surface read with the wrong `v` scale, or with its two
//! directions transposed, produces a face that is exactly the right shape and
//! inside out, and the symptom arrives much later as a failed orientation
//! validation over the whole solid. So every surface here is checked against
//! the ISO 10303-42 formula itself, term for term, at a spread of parameters —
//! not against another NGK type, which would only prove the two agree.
//!
//! The placement is deliberately oblique in every case. An axis-aligned frame
//! hides a transposed or mis-signed axis, because so many of its components
//! are zero.

use std::f64::consts::{FRAC_PI_2, FRAC_PI_3, FRAC_PI_4, PI, TAU};

use ngk::exchange::step::convert::pcurve::lift_pcurve;
use ngk::exchange::step::convert::surfaces::{MappedSurface, read_surface};
use ngk::exchange::step::convert::uv_map::UvMap;
use ngk::exchange::step::part21::{EntityId, parse_exchange};
use ngk::exchange::step::schema::resolver::{Origin, Resolver};
use ngk::geometry::{
    Circle, Curve, Curve2, Cylinder, Interval, LINEAR_TOLERANCE, Line, Plane, Point2, Point3,
    Surface, TrimmedCurve2, Vector2,
};

use nalgebra::Vector3;

/// The oblique placement every surface below is built on.
///
/// `#10` is the location, `#11` the axis and `#12` the reference direction,
/// so the local frame is x = `#12` made perpendicular to `#11`, z = `#11`.
const PLACEMENT: &str = "\
#10 = CARTESIAN_POINT('',(1.0,-2.0,3.0));
#11 = DIRECTION('',(0.0,0.6,0.8));
#12 = DIRECTION('',(1.0,0.0,0.0));
#13 = AXIS2_PLACEMENT_3D('',#10,#11,#12);
";

/// The frame `#13` names, rebuilt here independently of the reader.
fn frame() -> (Point3, Vector3<f64>, Vector3<f64>, Vector3<f64>) {
    let location = Point3::new(1.0, -2.0, 3.0);
    let axis = Vector3::new(0.0, 0.6, 0.8).normalize();
    let reference = Vector3::new(1.0, 0.0, 0.0);
    // STEP takes the reference direction's component in the plane normal to
    // the axis, which is what `Frame::from_xz` does.
    let x = (reference - axis * reference.dot(&axis)).normalize();
    (location, x, axis.cross(&x), axis)
}

/// Reads the surface named `#1` out of a data section carrying `PLACEMENT`.
fn surface(instances: &str) -> MappedSurface {
    let source = format!(
        "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n{PLACEMENT}{instances}\nENDSEC;\nEND-ISO-10303-21;\n"
    );
    let exchange = parse_exchange(&source).expect("the exchange structure should parse");
    let resolver = Resolver::new(&exchange, None).expect("a file with no unit block is metric");
    let from = Origin {
        id: EntityId(1),
        line: 1,
    };
    read_surface(&resolver, from, EntityId(1)).expect("the surface should read")
}

/// Asserts that the NGK surface, read through the map, is the ISO formula.
///
/// `iso` is handed the parameters *as the STEP entity states them*; the
/// surface is asked at the parameters the map turns them into. A map that is
/// wrong in any component moves the two apart at some sample, which is the
/// whole point of sampling a spread rather than one convenient corner.
fn agrees_with_iso(
    mapped: &MappedSurface,
    samples: &[(f64, f64)],
    iso: impl Fn(f64, f64) -> Point3,
) {
    for &(u, v) in samples {
        let ngk = mapped.map.apply(Point2::new(u, v));
        let got = mapped.surface.point_at(ngk.x, ngk.y);
        let want = iso(u, v);
        assert!(
            (got - want).norm() <= 1.0e-9,
            "at STEP ({u}, {v}) -> NGK ({}, {}): got {got:?}, ISO says {want:?}",
            ngk.x,
            ngk.y,
        );
    }
}

/// A dozen parameters spread over an angular direction and a linear one.
fn angular_and_linear() -> Vec<(f64, f64)> {
    let mut samples = Vec::new();
    for step in 0..6 {
        let u = TAU * step as f64 / 6.0;
        for v in [-3.5, 7.25] {
            samples.push((u, v));
        }
    }
    samples
}

/// A dozen parameters spread over two angular directions.
fn two_angles(v_range: (f64, f64)) -> Vec<(f64, f64)> {
    let mut samples = Vec::new();
    for step in 0..6 {
        let u = TAU * step as f64 / 6.0;
        for part in [0.25_f64, 0.75] {
            samples.push((u, v_range.0 + (v_range.1 - v_range.0) * part));
        }
    }
    samples
}

#[test]
fn a_plane_is_the_iso_plane_and_needs_no_map() {
    let mapped = surface("#1 = PLANE('',#13);");
    let (origin, x, y, _) = frame();

    assert_eq!(mapped.map, UvMap::IDENTITY);
    agrees_with_iso(&mapped, &angular_and_linear(), |u, v| {
        origin + x * u + y * v
    });
}

#[test]
fn a_cylindrical_surface_is_the_iso_cylinder_and_needs_no_map() {
    let mapped = surface("#1 = CYLINDRICAL_SURFACE('',#13,4.5);");
    let (origin, x, y, z) = frame();

    assert_eq!(mapped.map, UvMap::IDENTITY);
    agrees_with_iso(&mapped, &angular_and_linear(), |u, v| {
        origin + (x * u.cos() + y * u.sin()) * 4.5 + z * v
    });
}

#[test]
fn a_spherical_surface_is_the_iso_sphere_and_needs_no_map() {
    let mapped = surface("#1 = SPHERICAL_SURFACE('',#13,4.5);");
    let (origin, x, y, z) = frame();

    assert_eq!(mapped.map, UvMap::IDENTITY);
    agrees_with_iso(&mapped, &two_angles((-PI / 2.0, PI / 2.0)), |u, v| {
        origin + (x * u.cos() + y * u.sin()) * (4.5 * v.cos()) + z * (4.5 * v.sin())
    });
}

#[test]
fn a_toroidal_surface_is_the_iso_torus_and_needs_no_map() {
    // The decision this pins: a torus is an *exact* identity, which is why it
    // is the one headline periodic primitive that carries no parameter map.
    let mapped = surface("#1 = TOROIDAL_SURFACE('',#13,10.0,2.5);");
    let (origin, x, y, z) = frame();

    assert_eq!(mapped.map, UvMap::IDENTITY);
    agrees_with_iso(&mapped, &two_angles((0.0, TAU)), |u, v| {
        origin + (x * u.cos() + y * u.sin()) * (10.0 + 2.5 * v.cos()) + z * (2.5 * v.sin())
    });
}

#[test]
fn a_conical_surface_is_the_iso_cone_once_its_v_is_rescaled() {
    // The one surface here whose parameterization differs: STEP measures `v`
    // along the axis and NGK along the generatrix. Nothing else about the cone
    // moves, so a test that only checked the point *set* would pass with the
    // scale missing entirely.
    let half_angle = FRAC_PI_4;
    let mapped = surface(&format!(
        "#1 = CONICAL_SURFACE('',#13,20.0,{half_angle:?});"
    ));
    let (origin, x, y, z) = frame();

    assert_ne!(mapped.map, UvMap::IDENTITY);
    agrees_with_iso(&mapped, &angular_and_linear(), |u, v| {
        origin + (x * u.cos() + y * u.sin()) * (20.0 + v * half_angle.tan()) + z * v
    });
}

#[test]
fn a_cone_in_degrees_arrives_with_its_angle_in_radians() {
    // A plane angle is the one quantity a STEP file routinely states in a
    // non-SI unit, and `semi_angle` is where it reaches the geometry. Read as
    // degrees the cone would be almost flat, which still writes a plausible
    // file.
    let source = format!(
        "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n{PLACEMENT}\
#1 = CONICAL_SURFACE('',#13,20.0,45.0);
#20 = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );
#21 = MEASURE_WITH_UNIT(PLANE_ANGLE_MEASURE(0.017453292519943295),#20);
#22 = ( CONVERSION_BASED_UNIT('DEGREE',#21) NAMED_UNIT(#23) PLANE_ANGLE_UNIT() );
#23 = DIMENSIONAL_EXPONENTS(0.0,0.0,0.0,0.0,0.0,0.0,0.0);
#24 = ( NAMED_UNIT(*) LENGTH_UNIT() SI_UNIT($,.METRE.) );
#25 = UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#24,'distance accuracy value','');
#26 = (GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#25))GLOBAL_UNIT_ASSIGNED_CONTEXT((#24,#22))REPRESENTATION_CONTEXT('',''));
\nENDSEC;\nEND-ISO-10303-21;\n"
    );
    let exchange = parse_exchange(&source).expect("the exchange structure should parse");
    let resolver = Resolver::new(&exchange, None).expect("the unit block should read");
    let from = Origin {
        id: EntityId(1),
        line: 1,
    };
    let mapped = read_surface(&resolver, from, EntityId(1)).expect("the cone should read");

    let Surface::Cone(cone) = &mapped.surface else {
        panic!("a CONICAL_SURFACE should read as a cone");
    };
    assert!((cone.half_angle() - FRAC_PI_4).abs() <= 1.0e-12);
    // Metres, so the reference radius is twenty thousand millimetres.
    assert!((cone.reference_radius() - 20_000.0).abs() <= 1.0e-9);
}

#[test]
fn a_cone_that_is_really_a_cylinder_is_refused_by_name() {
    // A zero semi-angle violates the schema's own rule, and the substitution
    // relating the two `v` parameters has no finite answer there — so the
    // surface is refused rather than read as a cone of infinite reach.
    let source = format!(
        "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n{PLACEMENT}\
#1 = CONICAL_SURFACE('',#13,20.0,1.5707963267948966);\nENDSEC;\nEND-ISO-10303-21;\n"
    );
    let exchange = parse_exchange(&source).expect("the exchange structure should parse");
    let resolver = Resolver::new(&exchange, None).expect("a file with no unit block is metric");
    let from = Origin {
        id: EntityId(1),
        line: 1,
    };

    let error = read_surface(&resolver, from, EntityId(1))
        .expect_err("a right-angled cone has no STEP reading");
    assert!(
        error.to_string().contains("half angle"),
        "the message should name the half angle: {error}",
    );
}

// ------------------------------------------------------------------ the map

#[test]
fn the_identity_map_changes_nothing() {
    let point = Point2::new(0.3, -1.25);
    assert_eq!(UvMap::IDENTITY.apply(point), point);
    assert_eq!(UvMap::IDENTITY.unapply(point), point);
    assert!(!UvMap::IDENTITY.reverses_orientation());
}

#[test]
fn every_map_undoes_itself() {
    // The property that lets the writer be the reader run backwards: one
    // implementation, inverted, rather than two that can disagree.
    let maps = [
        UvMap::IDENTITY,
        UvMap::TRANSPOSED,
        UvMap::scaled(1.0, 2.5),
        UvMap::scaled(-1.0, -0.5).shifted(Vector2::new(TAU, 3.0)),
        UvMap::TRANSPOSED.shifted(Vector2::new(-1.0, 0.25)),
    ];
    for map in maps {
        for point in [Point2::new(0.0, 0.0), Point2::new(1.25, -3.5)] {
            let round_tripped = map.unapply(map.apply(point));
            assert!(
                (round_tripped - point).norm() <= 1.0e-12,
                "{map:?} did not undo itself at {point:?}: got {round_tripped:?}",
            );
        }
    }
}

#[test]
fn orientation_reverses_with_the_sign_of_the_determinant() {
    // Stated as the sign of a determinant rather than a case analysis, so
    // that a transposition and a negative scale cancel without anyone having
    // to notice that they should.
    assert!(!UvMap::IDENTITY.reverses_orientation());
    assert!(UvMap::TRANSPOSED.reverses_orientation());
    assert!(UvMap::scaled(1.0, -2.0).reverses_orientation());
    assert!(!UvMap::scaled(-1.0, -2.0).reverses_orientation());
}

#[test]
fn a_line_pcurve_keeps_its_type_and_its_endpoints_under_every_map() {
    let pcurve = TrimmedCurve2::segment(Point2::new(1.0, 2.0), Point2::new(4.0, -1.0));
    for map in [
        UvMap::IDENTITY,
        UvMap::TRANSPOSED,
        UvMap::scaled(1.0, 2.5),
        UvMap::scaled(3.0, -0.5).shifted(Vector2::new(1.0, 1.0)),
    ] {
        let mapped = map.map_pcurve(&pcurve).expect("a line always maps");
        assert!(
            matches!(mapped.curve(), Curve2::Line(_)),
            "{map:?} demoted a line",
        );
        for fraction in [0.0, 0.25, 0.5, 1.0] {
            let want = map.apply(pcurve.point_at(fraction));
            let got = mapped.point_at(fraction);
            assert!(
                (got - want).norm() <= LINEAR_TOLERANCE,
                "{map:?} moved the line at {fraction}: {got:?} vs {want:?}",
            );
        }
    }
}

#[test]
fn a_circular_pcurve_survives_a_similarity_and_demotes_under_a_stretch() {
    // The consequence the cone's scale carries: a non-uniform scale turns a
    // circle into an ellipse whose axes are not the mapped ones, so the
    // support cannot keep its type. It keeps its *point set*, which is the
    // contract that matters — and a demotion that moved the curve would be a
    // silent corruption, so both halves are asserted.
    let pcurve = TrimmedCurve2::arc(
        Point2::new(2.0, 1.0),
        Vector2::new(1.0, 0.0),
        3.0,
        FRAC_PI_3,
    );

    let similarity = UvMap::scaled(2.0, 2.0);
    let kept = similarity.map_pcurve(&pcurve).expect("a similarity maps");
    assert!(matches!(kept.curve(), Curve2::Circle(_)));

    let stretch = UvMap::scaled(1.0, 2.0);
    let demoted = stretch.map_pcurve(&pcurve).expect("a stretch still maps");
    assert!(
        matches!(demoted.curve(), Curve2::Nurbs(_)),
        "a stretched circle is not a circle",
    );

    // A similarity keeps the conic's own angular parameter, so fraction for
    // fraction is the right assertion there.
    for fraction in [0.0, 0.3, 0.6, 1.0] {
        let want = similarity.apply(pcurve.point_at(fraction));
        let got = kept.point_at(fraction);
        assert!(
            (got - want).norm() <= LINEAR_TOLERANCE,
            "the similarity moved the arc at {fraction}: {got:?} vs {want:?}",
        );
    }

    // The demotion is the case where speed is *not* preserved: a NURBS's
    // parameter is projective where a conic's is angular. What it does keep is
    // the point set and the two ends, which is everything a face asks of a
    // parameter curve — so that is what is asserted rather than a fraction
    // correspondence the type cannot carry.
    assert!((demoted.start() - stretch.apply(pcurve.start())).norm() <= LINEAR_TOLERANCE);
    assert!((demoted.end() - stretch.apply(pcurve.end())).norm() <= LINEAR_TOLERANCE);
    for fraction in [0.0, 0.2, 0.4, 0.6, 0.8, 1.0] {
        let on_arc = stretch.apply(pcurve.point_at(fraction));
        assert!(
            demoted.contains(on_arc, LINEAR_TOLERANCE),
            "the demoted arc lost the point at {fraction}: {on_arc:?}",
        );
    }
}

#[test]
fn a_transposed_pcurve_winds_the_other_way() {
    // The flip that makes a transposing surface work at all: transposing the
    // chart negates the boundary's signed area, and NGK multiplies that by the
    // surface normal to get the face normal — so the two flips cancel only if
    // this one actually happens.
    let square = [
        Point2::new(0.0, 0.0),
        Point2::new(1.0, 0.0),
        Point2::new(1.0, 1.0),
        Point2::new(0.0, 1.0),
    ];
    let area = |points: &[Point2]| {
        let mut total = 0.0;
        for index in 0..points.len() {
            let current = points[index];
            let next = points[(index + 1) % points.len()];
            total += current.x * next.y - next.x * current.y;
        }
        total / 2.0
    };

    let mapped: Vec<Point2> = square.iter().map(|p| UvMap::TRANSPOSED.apply(*p)).collect();
    assert!(area(&square) > 0.0);
    assert!(area(&mapped) < 0.0);
    assert!(UvMap::TRANSPOSED.reverses_orientation());
}

// -------------------------------------------------------------- rebuilding

/// A parameter curve rebuilt on a support, with nothing else involved.
fn lift(surface: &Surface, curve: &Curve, span: Interval) -> TrimmedCurve2 {
    lift_pcurve(surface, curve, span, 1.0e-7)
        .expect("the curve should lift")
        .pcurve
}

/// Asserts that a parameter curve's image is the curve it was rebuilt from.
fn follows(surface: &Surface, pcurve: &TrimmedCurve2, section: impl Fn(f64) -> Point3) {
    for step in 0..=8 {
        let fraction = step as f64 / 8.0;
        let uv = pcurve.point_at(fraction);
        let got = surface.point_at(uv.x, uv.y);
        let want = section(fraction);
        assert!(
            (got - want).norm() <= 1.0e-7,
            "at {fraction}: the pcurve is at {got:?}, the curve at {want:?}",
        );
    }
}

#[test]
fn a_circle_projected_onto_a_plane_facing_it_keeps_its_sense() {
    // The plane and the circle agree on which way is up, so a quarter turn of
    // the circle is a quarter turn counter-clockwise in the plane's chart.
    let plane = Surface::Plane(Plane::new(Point3::origin(), Vector3::x(), Vector3::z()));
    let circle = Curve::Circle(Circle::new(
        Plane::new(Point3::origin(), Vector3::x(), Vector3::z()),
        2.0,
    ));
    let span = Interval::new(0.0, FRAC_PI_2);

    let pcurve = lift(&plane, &circle, span);
    assert!(
        matches!(pcurve.curve(), Curve2::Circle(_)),
        "a circle in a plane is a circle"
    );
    follows(&plane, &pcurve, |fraction| {
        let angle = span.start + (span.end - span.start) * fraction;
        Point3::new(2.0 * angle.cos(), 2.0 * angle.sin(), 0.0)
    });
}

#[test]
fn a_circle_projected_onto_a_plane_facing_away_reverses() {
    // The silent one. Seen from behind, the same circle turns the other way,
    // so a support left counter-clockwise would trace the complementary arc
    // over the very same interval — a boundary of the right length in the
    // wrong place, which no arity or count check can catch.
    let plane = Surface::Plane(Plane::new(Point3::origin(), Vector3::x(), -Vector3::z()));
    let circle = Curve::Circle(Circle::new(
        Plane::new(Point3::origin(), Vector3::x(), Vector3::z()),
        2.0,
    ));
    let span = Interval::new(0.0, FRAC_PI_2);

    let pcurve = lift(&plane, &circle, span);
    follows(&plane, &pcurve, |fraction| {
        let angle = span.start + (span.end - span.start) * fraction;
        Point3::new(2.0 * angle.cos(), 2.0 * angle.sin(), 0.0)
    });
}

#[test]
fn a_seam_lifted_onto_a_cylinder_is_a_straight_parameter_line() {
    // A cut runs along a parameter direction, which is what lets it come back
    // in closed form rather than as a fit — so the support's *type* is the
    // assertion, not just the points.
    let cylinder = Surface::Cylinder(Cylinder::new(
        Point3::origin(),
        Vector3::x(),
        Vector3::z(),
        4.0,
    ));
    let seam = Curve::Line(Line::through(
        Point3::new(4.0, 0.0, 0.0),
        Point3::new(4.0, 0.0, 1.0),
    ));
    let span = Interval::new(0.0, 10.0);

    let pcurve = lift(&cylinder, &seam, span);
    assert!(
        matches!(pcurve.curve(), Curve2::Line(_)),
        "a seam should lift to a parameter line, got {:?}",
        pcurve.curve(),
    );
    follows(&cylinder, &pcurve, |fraction| {
        Point3::new(4.0, 0.0, 10.0 * fraction)
    });
}

#[test]
fn a_rim_lifted_onto_a_cylinder_crosses_the_fold_without_jumping() {
    // A full turn folds: inversion answers within one period, so the samples
    // come back with a whole-period jump in them unless they are unwrapped.
    // A pcurve that jumped would run backwards over most of its length while
    // still starting and ending in the right places.
    let cylinder = Surface::Cylinder(Cylinder::new(
        Point3::origin(),
        Vector3::x(),
        Vector3::z(),
        4.0,
    ));
    let rim = Curve::Circle(Circle::new(
        Plane::new(Point3::origin(), Vector3::x(), Vector3::z()),
        4.0,
    ));
    let span = Interval::new(0.0, TAU);

    let pcurve = lift(&cylinder, &rim, span);
    follows(&cylinder, &pcurve, |fraction| {
        let angle = TAU * fraction;
        Point3::new(4.0 * angle.cos(), 4.0 * angle.sin(), 0.0)
    });

    // Monotone in `u` over the whole turn: that is what "did not jump" means.
    let samples: Vec<f64> = (0..=16)
        .map(|step| pcurve.point_at(step as f64 / 16.0).x)
        .collect();
    assert!(
        samples.windows(2).all(|pair| pair[1] > pair[0]),
        "the rim folded back on itself: {samples:?}",
    );
}
