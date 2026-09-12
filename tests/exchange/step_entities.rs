//! The entity model: one type per STEP entity, read and written in one order.
//!
//! Every test here is the same property — `read(x.record())` is `x` — applied
//! to one entity. That property is the whole reason the entity types exist:
//! a Part 21 file is positional and carries no field names, so a reader and a
//! writer that each spell out an attribute order independently can disagree
//! without anything noticing. Reading back what was just written is what makes
//! disagreeing impossible to do quietly.
//!
//! It catches a transposed pair, an attribute read from the wrong position, a
//! miscounted skip, and a `record` that writes its fields in an order its own
//! `read` does not expect.

use ngk::exchange::step::part21::{EntityId, Record, Value};
use ngk::exchange::step::schema::entities::{self, Entity, Measure, SurfaceCurveKind};
use ngk::exchange::step::schema::resolver::{Attributes, Origin, SchemaError};

fn origin() -> Origin {
    Origin {
        id: EntityId(1),
        line: 1,
    }
}

/// Writes an entity, reads the result back, and returns what came back.
fn round_trip<T: Entity + std::fmt::Debug + PartialEq>(entity: &T) -> T {
    let record = entity.record();
    assert!(
        T::KEYWORDS.iter().any(|keyword| record.is(keyword)),
        "{record:?} is not spelled with any of {:?}",
        T::KEYWORDS,
    );
    let read = T::read(Attributes::new(origin(), &record))
        .unwrap_or_else(|error| panic!("{record:?} should read back: {error}"));
    assert_eq!(&read, entity, "round trip changed the entity");
    read
}

fn id(name: u64) -> EntityId {
    EntityId(name)
}

#[test]
fn a_cartesian_point_round_trips() {
    round_trip(&entities::CartesianPoint {
        coordinates: [1.5, -2.0, 0.25],
    });
}

#[test]
fn a_direction_round_trips() {
    round_trip(&entities::Direction {
        direction_ratios: [0.0, 0.0, -1.0],
    });
}

#[test]
fn a_vector_round_trips() {
    round_trip(&entities::Vector {
        orientation: id(7),
        magnitude: 12.5,
    });
}

#[test]
fn an_axis_placement_round_trips_with_and_without_its_optional_directions() {
    // Both directions are optional, and a reader that loses track of which
    // one is missing builds a frame rotated about its own axis.
    round_trip(&entities::Axis2Placement3d {
        location: id(1),
        axis: Some(id(2)),
        ref_direction: Some(id(3)),
    });
    round_trip(&entities::Axis2Placement3d {
        location: id(1),
        axis: None,
        ref_direction: Some(id(3)),
    });
    round_trip(&entities::Axis2Placement3d {
        location: id(1),
        axis: Some(id(2)),
        ref_direction: None,
    });
    round_trip(&entities::Axis2Placement3d {
        location: id(1),
        axis: None,
        ref_direction: None,
    });
}

#[test]
fn an_axis1_placement_round_trips_with_and_without_its_optional_direction() {
    round_trip(&entities::Axis1Placement {
        location: id(1),
        axis: Some(id(2)),
    });
    round_trip(&entities::Axis1Placement {
        location: id(1),
        axis: None,
    });
}

#[test]
fn a_surface_of_revolution_round_trips_with_its_curve_before_its_axis() {
    // Two adjacent references of different kinds that the type system cannot
    // tell apart: transposed, the profile becomes the axis and the surface is
    // a different shape entirely.
    let revolution = round_trip(&entities::SurfaceOfRevolution {
        swept_curve: id(40),
        axis_position: id(41),
    });
    assert_eq!(revolution.swept_curve, id(40));
    assert_eq!(revolution.axis_position, id(41));
}

#[test]
fn a_line_round_trips() {
    round_trip(&entities::Line {
        pnt: id(4),
        dir: id(5),
    });
}

#[test]
fn a_circle_round_trips() {
    round_trip(&entities::Circle {
        position: id(6),
        radius: 12.5,
    });
}

#[test]
fn an_ellipse_round_trips_with_its_two_semi_axes_the_right_way_round() {
    // Deliberately unequal, and deliberately *not* in descending order: the
    // schema puts the axis along the reference direction first whether or not
    // it is the longer one, so equal radii would hide a transposition.
    let ellipse = round_trip(&entities::Ellipse {
        position: id(6),
        semi_axis_1: 3.0,
        semi_axis_2: 7.0,
    });
    assert_eq!(ellipse.semi_axis_1, 3.0);
    assert_eq!(ellipse.semi_axis_2, 7.0);
}

#[test]
fn a_plane_round_trips() {
    round_trip(&entities::Plane { position: id(9) });
}

#[test]
fn a_cylindrical_surface_round_trips() {
    round_trip(&entities::CylindricalSurface {
        position: id(9),
        radius: 4.0,
    });
}

#[test]
fn a_spherical_surface_round_trips() {
    round_trip(&entities::SphericalSurface {
        position: id(9),
        radius: 4.0,
    });
}

#[test]
fn a_conical_surface_round_trips_with_its_radius_before_its_angle() {
    // Both are bare reals in adjacent positions, so a transposition parses
    // cleanly and yields a cone of the wrong shape. Values that could not be
    // each other are what catches it.
    let cone = round_trip(&entities::ConicalSurface {
        position: id(9),
        radius: 20.0,
        semi_angle: 0.5,
    });
    assert_eq!(cone.radius, 20.0);
    assert_eq!(cone.semi_angle, 0.5);
}

#[test]
fn a_toroidal_surface_round_trips_with_its_radii_the_right_way_round() {
    let torus = round_trip(&entities::ToroidalSurface {
        position: id(9),
        major_radius: 10.0,
        minor_radius: 2.0,
    });
    assert_eq!(torus.major_radius, 10.0);
    assert_eq!(torus.minor_radius, 2.0);
}

#[test]
fn a_surface_curve_round_trips_in_each_of_its_three_spellings() {
    // The three are one entity with three keywords. Writing one back as
    // another would silently reclassify a seam as an ordinary curve.
    for kind in [
        SurfaceCurveKind::Surface,
        SurfaceCurveKind::Seam,
        SurfaceCurveKind::Intersection,
    ] {
        let curve = entities::SurfaceCurve {
            kind,
            curve_3d: id(11),
            associated_geometry: vec![id(12), id(13)],
            master_representation: "CURVE_3D".to_string(),
        };
        round_trip(&curve);
    }
}

#[test]
fn a_vertex_point_round_trips() {
    round_trip(&entities::VertexPoint {
        vertex_geometry: id(20),
    });
}

#[test]
fn an_edge_curve_round_trips_with_its_corners_the_right_way_round() {
    // Two adjacent references of the same type: a transposition here is
    // invisible to the type system and reverses every edge in the file.
    let edge = round_trip(&entities::EdgeCurve {
        edge_start: id(21),
        edge_end: id(22),
        edge_geometry: id(23),
        same_sense: true,
    });
    assert_eq!(edge.edge_start, id(21));
    assert_eq!(edge.edge_end, id(22));
}

#[test]
fn an_oriented_edge_round_trips_past_its_two_derived_attributes() {
    // Its two vertex attributes are redeclared and derived, so they are
    // written `*` and skipped — and a skip of the wrong width would read the
    // edge reference out of the orientation flag.
    for orientation in [true, false] {
        let oriented = round_trip(&entities::OrientedEdge {
            edge_element: id(30),
            orientation,
        });
        assert_eq!(oriented.orientation, orientation);
    }
}

#[test]
fn an_oriented_edge_writes_its_derived_vertices_as_asterisks() {
    // A reader elsewhere may reject a file that puts anything else there.
    let record = entities::OrientedEdge {
        edge_element: id(30),
        orientation: true,
    }
    .record();
    assert_eq!(record.param(1), Some(&Value::Derived));
    assert_eq!(record.param(2), Some(&Value::Derived));
}

#[test]
fn an_edge_loop_round_trips_keeping_its_order() {
    let edge_loop = round_trip(&entities::EdgeLoop {
        edge_list: vec![id(1), id(2), id(3), id(4)],
    });
    assert_eq!(edge_loop.edge_list, vec![id(1), id(2), id(3), id(4)]);
}

#[test]
fn a_face_bound_round_trips_in_both_its_spellings() {
    // `FACE_OUTER_BOUND` and `FACE_BOUND` carry identical attributes and
    // differ only in saying whether the loop encloses the face or a hole, so
    // the spelling is the only thing distinguishing them.
    for outer in [true, false] {
        for orientation in [true, false] {
            let bound = round_trip(&entities::FaceBound {
                bound: id(40),
                orientation,
                outer,
            });
            assert_eq!(bound.outer, outer);
            assert_eq!(bound.orientation, orientation);
        }
    }
}

#[test]
fn a_face_bound_is_spelled_outer_only_when_it_is_one() {
    let outer = entities::FaceBound {
        bound: id(40),
        orientation: true,
        outer: true,
    };
    let inner = entities::FaceBound {
        outer: false,
        ..outer
    };
    assert_eq!(outer.record().keyword, "FACE_OUTER_BOUND");
    assert_eq!(inner.record().keyword, "FACE_BOUND");
}

#[test]
fn an_advanced_face_round_trips() {
    round_trip(&entities::AdvancedFace {
        bounds: vec![id(50), id(51)],
        face_geometry: id(52),
        same_sense: false,
    });
}

#[test]
fn a_closed_shell_round_trips() {
    round_trip(&entities::ClosedShell {
        cfs_faces: vec![id(60), id(61), id(62)],
    });
}

#[test]
fn a_manifold_solid_brep_round_trips() {
    round_trip(&entities::ManifoldSolidBrep { outer: id(70) });
}

#[test]
fn an_oriented_closed_shell_round_trips_over_its_derived_face_set() {
    // The one topological entity whose file record carries an attribute it
    // does not model: the face set is derived from the shell it names, so `*`
    // stands in its place and everything after it is one position along from
    // where a reader counting fields would look.
    round_trip(&entities::OrientedClosedShell {
        closed_shell_element: id(71),
        orientation: false,
    });
}

#[test]
fn a_brep_with_voids_round_trips() {
    round_trip(&entities::BrepWithVoids {
        outer: id(72),
        voids: vec![id(73), id(74)],
    });
}

#[test]
fn an_si_unit_round_trips_with_and_without_a_prefix() {
    // `SI_UNIT` is one of the few entities with no decorative name in front
    // of its data, so a read that skips one loses the prefix and silently
    // rescales the whole document.
    round_trip(&entities::SiUnit {
        prefix: Some("MILLI".to_string()),
        name: "METRE".to_string(),
    });
    round_trip(&entities::SiUnit {
        prefix: None,
        name: "RADIAN".to_string(),
    });
}

#[test]
fn a_conversion_based_unit_round_trips() {
    round_trip(&entities::ConversionBasedUnit {
        name: "INCH".to_string(),
        conversion_factor: id(80),
    });
}

#[test]
fn a_measure_with_unit_round_trips() {
    round_trip(&entities::MeasureWithUnit {
        value_component: Measure::length(25.4),
        unit_component: id(81),
    });
}

#[test]
fn an_uncertainty_measure_with_unit_round_trips() {
    round_trip(&entities::UncertaintyMeasureWithUnit {
        value_component: Measure::length(1.0e-7),
        unit_component: id(82),
        name: "distance_accuracy_value".to_string(),
        description: "confusion accuracy".to_string(),
    });
}

#[test]
fn a_measure_keeps_the_type_it_was_written_as() {
    // The measure type is what says whether a unit block is talking about a
    // length or an angle, so reading it back as a bare number would make a
    // degree indistinguishable from a millimetre.
    let angle = Measure {
        kind: "PLANE_ANGLE_MEASURE".to_string(),
        value: 0.017453292519943295,
    };
    let read = round_trip(&entities::MeasureWithUnit {
        value_component: angle,
        unit_component: id(83),
    });
    assert_eq!(read.value_component.kind, "PLANE_ANGLE_MEASURE");
}

#[test]
fn an_entity_declines_a_record_that_is_not_its_own() {
    // Declining dispatch: `None` means "not mine, try the next one", and is
    // what lets a surface reader work through plane, cylinder and cone in
    // turn without each one knowing about the others.
    let plane = entities::Plane { position: id(9) }.record();
    let attributes = Attributes::new(origin(), &plane);

    assert!(attributes.decode::<entities::Plane>().is_some());
    assert!(attributes.decode::<entities::Line>().is_none());
}

#[test]
fn a_malformed_record_of_the_right_keyword_is_an_error_not_a_decline() {
    // The other half of the convention: a record that *is* this entity but
    // is broken must not be mistaken for one belonging to something else, or
    // the dispatch falls through and reports the wrong problem.
    let broken = Record::new("PLANE", vec![Value::Text(String::new()), Value::Integer(3)]);
    let attributes = Attributes::new(origin(), &broken);

    let decoded = attributes
        .decode::<entities::Plane>()
        .expect("the keyword matches, so this must not decline");
    assert!(matches!(decoded, Err(SchemaError::BadAttribute { .. })));
}

#[test]
fn an_error_names_the_entity_and_the_line_it_was_on() {
    // A STEP error a user cannot find in their file is not actionable.
    let broken = Record::new("PLANE", vec![Value::Text(String::new())]);
    let origin = Origin {
        id: EntityId(1234),
        line: 99,
    };
    let error = entities::Plane::read(Attributes::new(origin, &broken))
        .expect_err("a plane with no position cannot be read");

    let message = error.to_string();
    assert!(message.contains("#1234"), "got {message}");
    assert!(message.contains("line 99"), "got {message}");
    assert!(message.contains("PLANE"), "got {message}");
}

#[test]
fn reading_a_2d_point_as_a_3d_one_fails_on_its_arity() {
    // The dimension is a type parameter precisely so that a `PCURVE`'s 2D
    // geometry cannot be read into model space as though it were a position.
    let flat = entities::CartesianPoint {
        coordinates: [1.0, 2.0],
    }
    .record();
    let attributes = Attributes::new(origin(), &flat);

    assert!(
        entities::CartesianPoint::<3>::read(attributes).is_err(),
        "a 2D point must not read as a 3D one",
    );
    assert!(entities::CartesianPoint::<2>::read(attributes).is_ok());
}

// ------------------------------------------------------ B-splines, both spellings

use ngk::exchange::step::part21::Instance;
use ngk::exchange::step::schema::bspline::{
    BSplineCurve, BSplineSurface, CurveKnots, CurveSpline, CurveWeights, SurfaceKnots,
    SurfaceSpline, SurfaceWeights,
};

/// Writes a B-spline, reads the instance back, and returns what came back.
///
/// The same property the rest of this file asserts, on the two types that
/// cannot be `Entity`: a rational B-spline is a complex instance, so what a
/// round trip has to survive is a *set* of records rather than one.
fn round_trip_curve(curve: &BSplineCurve) -> BSplineCurve {
    let instance = Instance {
        id: id(1),
        records: curve.records(),
        line: 1,
    };
    let read = BSplineCurve::read(origin(), &instance)
        .expect("a written B-spline curve should be recognized")
        .expect("it should read back");
    assert_eq!(&read, curve, "round trip changed the curve");
    read
}

fn round_trip_surface(surface: &BSplineSurface) -> BSplineSurface {
    let instance = Instance {
        id: id(1),
        records: surface.records(),
        line: 1,
    };
    let read = BSplineSurface::read(origin(), &instance)
        .expect("a written B-spline surface should be recognized")
        .expect("it should read back");
    assert_eq!(&read, surface, "round trip changed the surface");
    read
}

fn polynomial_curve() -> BSplineCurve {
    BSplineCurve {
        spline: CurveSpline {
            degree: 3,
            control_points: vec![id(8), id(9), id(10), id(11)],
            closed: false,
        },
        knots: CurveKnots {
            multiplicities: vec![4, 4],
            knots: vec![0.0, 1.0],
        },
        weights: None,
    }
}

#[test]
fn a_polynomial_b_spline_curve_round_trips_as_one_record() {
    let curve = polynomial_curve();
    round_trip_curve(&curve);

    let records = curve.records();
    assert_eq!(records.len(), 1, "the leaf type is a simple instance");
    assert!(records[0].is("B_SPLINE_CURVE_WITH_KNOTS"));
}

#[test]
fn a_rational_b_spline_curve_round_trips_as_a_complex_instance() {
    // The weights have nowhere to go in the simple record, so the same entity
    // has to be written as the intersection of its supertypes instead.
    let curve = BSplineCurve {
        weights: Some(CurveWeights {
            weights: vec![1.0, 0.8, 0.8, 1.0],
        }),
        ..polynomial_curve()
    };
    round_trip_curve(&curve);

    let records = curve.records();
    let keywords: Vec<&str> = records
        .iter()
        .map(|record| record.keyword.as_str())
        .collect();
    assert_eq!(
        keywords,
        [
            "BOUNDED_CURVE",
            "B_SPLINE_CURVE",
            "B_SPLINE_CURVE_WITH_KNOTS",
            "CURVE",
            "GEOMETRIC_REPRESENTATION_ITEM",
            "RATIONAL_B_SPLINE_CURVE",
            "REPRESENTATION_ITEM",
        ],
        "Part 21 orders a complex instance's records by keyword",
    );
}

fn polynomial_surface() -> BSplineSurface {
    // Deliberately not square: a transposed control net is invisible when the
    // two degrees and the two counts agree.
    BSplineSurface {
        spline: SurfaceSpline {
            degree_u: 1,
            degree_v: 2,
            control_points: vec![vec![id(20), id(21), id(22)], vec![id(23), id(24), id(25)]],
            closed_u: false,
            closed_v: true,
        },
        knots: SurfaceKnots {
            multiplicities_u: vec![2, 2],
            multiplicities_v: vec![3, 3],
            knots_u: vec![0.0, 1.0],
            knots_v: vec![0.0, 2.0],
        },
        weights: None,
    }
}

#[test]
fn a_polynomial_b_spline_surface_round_trips_as_one_record() {
    let surface = polynomial_surface();
    round_trip_surface(&surface);

    let records = surface.records();
    assert_eq!(records.len(), 1);
    assert!(records[0].is("B_SPLINE_SURFACE_WITH_KNOTS"));
}

#[test]
fn a_rational_b_spline_surface_round_trips_as_a_complex_instance() {
    let surface = BSplineSurface {
        weights: Some(SurfaceWeights {
            weights: vec![vec![1.0, 0.5, 1.0], vec![1.0, 0.5, 1.0]],
        }),
        ..polynomial_surface()
    };
    round_trip_surface(&surface);
    assert_eq!(surface.records().len(), 7);
}

#[test]
fn a_b_spline_declines_an_instance_that_is_not_one() {
    let instance = Instance {
        id: id(1),
        records: vec![Record::new("CIRCLE", vec![Value::Text(String::new())])],
        line: 1,
    };
    assert!(BSplineCurve::read(origin(), &instance).is_none());
    assert!(BSplineSurface::read(origin(), &instance).is_none());
}

#[test]
fn both_spellings_of_a_b_spline_write_the_same_attributes_in_the_same_order() {
    // The property the whole shape of `bspline.rs` exists for. A leaf type's
    // record is its name followed by every supertype's attributes end to end;
    // a complex instance is the same attributes, each under the keyword of the
    // supertype declaring it. If the two ever disagreed, one spelling would
    // read back as a different curve from the other — and nothing downstream
    // would notice, because each is well formed on its own.
    let polynomial = polynomial_curve();
    let rational = BSplineCurve {
        weights: Some(CurveWeights {
            weights: vec![1.0, 0.8, 0.8, 1.0],
        }),
        ..polynomial_curve()
    };

    let simple = polynomial.records();
    let [leaf] = simple.as_slice() else {
        panic!("a polynomial B-spline is one record");
    };
    let complex = rational.records();

    let mut slices = Vec::new();
    for keyword in ["B_SPLINE_CURVE", "B_SPLINE_CURVE_WITH_KNOTS"] {
        let record = complex
            .iter()
            .find(|record| record.is(keyword))
            .unwrap_or_else(|| panic!("a complex instance carries a {keyword} record"));
        slices.extend(record.params.iter().cloned());
    }

    assert_eq!(
        &leaf.params[1..],
        slices.as_slice(),
        "the leaf record past its name should be the complex instance's slices",
    );
}

#[test]
fn the_same_holds_for_a_b_spline_surface() {
    let polynomial = polynomial_surface();
    let rational = BSplineSurface {
        weights: Some(SurfaceWeights {
            weights: vec![vec![1.0, 0.5, 1.0], vec![1.0, 0.5, 1.0]],
        }),
        ..polynomial_surface()
    };

    let simple = polynomial.records();
    let [leaf] = simple.as_slice() else {
        panic!("a polynomial B-spline surface is one record");
    };
    let complex = rational.records();

    let mut slices = Vec::new();
    for keyword in ["B_SPLINE_SURFACE", "B_SPLINE_SURFACE_WITH_KNOTS"] {
        let record = complex
            .iter()
            .find(|record| record.is(keyword))
            .unwrap_or_else(|| panic!("a complex instance carries a {keyword} record"));
        slices.extend(record.params.iter().cloned());
    }

    assert_eq!(&leaf.params[1..], slices.as_slice());
}
