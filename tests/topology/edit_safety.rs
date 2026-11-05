//! A builder that meets an inconsistent intermediate state answers with an
//! error and leaves the model as it found it; it never panics and never
//! commits a solid whose shell is not outward.

use nalgebra::Vector3;

use ngk::geometry::{Axis3, Frame, Plane, Point3};
use ngk::modeling::{edges, faces, revolve, solids, sweep};
use ngk::topology::shape::{Shape, SolidTag};
use ngk::topology::validation::validate_solid_orientation;
use radians::Rad64;

fn assert_outward_or_error<E: std::fmt::Debug>(result: Result<Shape<SolidTag>, E>) {
    eprintln!("outcome: {:?}", result.as_ref().map(|_| ()));
    if let Ok(solid) = result {
        validate_solid_orientation(solid.model(), solid.key())
            .expect("a committed solid has an outward shell");
    }
}

#[test]
fn sweeping_a_disc_along_a_line_does_not_panic() {
    let section = faces::circle(Plane::xy(), 2.0).expect("disc should build");
    let path = edges::line(Point3::origin(), Point3::new(0.0, 0.0, 6.0)).expect("line");
    assert_outward_or_error(sweep::sweep_face(section, &path.edge(), Default::default()));
}

#[test]
fn cutting_a_pulley_body_by_a_revolved_groove_does_not_panic() {
    let cylinder = |z0: f64, z1: f64, r: f64| {
        solids::cylinder_at(Frame::at(Point3::new(0.0, 0.0, z0)), r, z1 - z0).expect("cylinder")
    };
    let body = solids::fuse(cylinder(-14.0, 14.0, 12.0), [&cylinder(-9.0, 9.0, 28.0)])
        .expect("hub and rim fuse");
    let body = solids::cut(body, [&cylinder(-15.0, 15.0, 6.0)]).expect("bore");
    let triangle = faces::polygon(&[
        Point3::new(22.0, 0.0, 0.0),
        Point3::new(29.0, 0.0, 5.0),
        Point3::new(29.0, 0.0, -5.0),
    ])
    .expect("triangle");
    let groove = revolve::revolve_face(triangle, Axis3::z(), Rad64::new(std::f64::consts::TAU))
        .expect("groove");
    assert_outward_or_error(solids::cut(body, [&groove]));
}

#[test]
fn revolving_a_face_by_a_negative_angle_never_commits_an_inward_solid() {
    for angle in [-0.5, -std::f64::consts::PI, -std::f64::consts::TAU] {
        let triangle = faces::polygon(&[
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 1.0),
        ])
        .expect("triangle");
        let result = revolve::revolve_face(triangle, Axis3::z(), Rad64::new(angle));
        eprintln!("angle {angle}: {:?}", result.as_ref().map(|_| ()));
        assert_outward_or_error(result);
    }
}

#[test]
fn sweeping_a_disc_along_a_cubic_does_not_panic() {
    use ngk::builders::edges::add_edge;
    use ngk::geometry::{ControlPolygon, Degree, Fraction, KnotVector, NurbsCurve};
    let points = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(12.0, -16.0, 0.0),
        Point3::new(32.0, -16.0, 0.0),
        Point3::new(44.0, 0.0, 0.0),
    ];
    let curve = NurbsCurve::new(
        Degree::new(3).expect("degree"),
        ControlPolygon::from_cartesian(points, &[1.0; 4]).expect("polygon"),
        KnotVector::new(vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]).expect("knots"),
    )
    .expect("cubic");
    let (a, b) = (
        curve.point_at(curve.domain().start.value()),
        curve.point_at(curve.domain().end.value()),
    );
    let path = Shape::<ngk::topology::shape::EdgeTag>::build(|model| {
        add_edge(model, a, b, ngk::geometry::Curve::Nurbs(curve))
    })
    .expect("edge");
    let trimmed = path.edge().trimmed_curve();
    let tangent: Vector3<f64> = trimmed.derivative_at(Fraction::START, 1);
    let z = tangent.normalize();
    let helper = if z.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let x = (helper - z * helper.dot(&z)).normalize();
    let frame = Frame::from_xz(trimmed.start(), x, z);
    let section = faces::circle(Plane::from_frame(frame), 20.0).expect("disc");
    assert_outward_or_error(sweep::sweep_face(section, &path.edge(), Default::default()));
}

mod random_builders {
    //! Random sequences of builders on random inputs never panic, and every
    //! solid they return passes the orientation check.

    use nalgebra::Vector3;
    use proptest::prelude::*;

    use ngk::geometry::{Axis3, Plane, Point3};
    use ngk::modeling::{edges, faces, revolve, solids, sweep, transform};
    use ngk::topology::shape::{Shape, SolidTag};
    use ngk::topology::validation::validate_solid_orientation;
    use radians::Rad64;

    #[derive(Debug, Clone)]
    enum Primitive {
        Block {
            x: f64,
            y: f64,
            z: f64,
        },
        Cylinder {
            radius: f64,
            height: f64,
        },
        Sphere {
            radius: f64,
        },
        Torus {
            major: f64,
            minor: f64,
        },
        Prism {
            sides: usize,
            radius: f64,
            distance: f64,
        },
        Revolved {
            offset: f64,
            width: f64,
            height: f64,
            angle: f64,
        },
        SweptDisc {
            radius: f64,
            length: f64,
            tilt: f64,
        },
    }

    #[derive(Debug, Clone, Copy)]
    enum Combine {
        Fuse,
        Cut,
        Intersect,
    }

    #[derive(Debug, Clone)]
    struct Step {
        primitive: Primitive,
        offset: [f64; 3],
        turn: f64,
        combine: Combine,
    }

    fn snapped() -> impl Strategy<Value = f64> {
        prop_oneof![
            3 => (-8i32..=8).prop_map(|quarter| f64::from(quarter) * 0.25),
            1 => -2.0f64..2.0,
        ]
    }

    fn primitive() -> impl Strategy<Value = Primitive> {
        let size = || 0.5f64..4.0;
        prop_oneof![
            (size(), size(), size()).prop_map(|(x, y, z)| Primitive::Block { x, y, z }),
            (size(), size()).prop_map(|(radius, height)| Primitive::Cylinder { radius, height }),
            size().prop_map(|radius| Primitive::Sphere { radius }),
            (1.5f64..4.0, 0.3f64..1.2).prop_map(|(major, minor)| Primitive::Torus { major, minor }),
            (3usize..7, size(), -3.0f64..3.0).prop_map(|(sides, radius, distance)| {
                Primitive::Prism {
                    sides,
                    radius,
                    distance,
                }
            }),
            (0.5f64..3.0, 0.2f64..2.0, 0.2f64..2.0, -7.0f64..7.0).prop_map(
                |(offset, width, height, angle)| Primitive::Revolved {
                    offset,
                    width,
                    height,
                    angle
                }
            ),
            (size(), -4.0f64..4.0, 0.0f64..1.0).prop_map(|(radius, length, tilt)| {
                Primitive::SweptDisc {
                    radius,
                    length,
                    tilt,
                }
            }),
        ]
    }

    fn step() -> impl Strategy<Value = Step> {
        (
            primitive(),
            [snapped(), snapped(), snapped()],
            (0u8..8).prop_map(|eighth| f64::from(eighth) * std::f64::consts::FRAC_PI_4),
            prop_oneof![
                Just(Combine::Fuse),
                Just(Combine::Cut),
                Just(Combine::Intersect)
            ],
        )
            .prop_map(|(primitive, offset, turn, combine)| Step {
                primitive,
                offset,
                turn,
                combine,
            })
    }

    /// The solid a step describes, or the refusal the builder answered with.
    fn build(step: &Step) -> Option<Shape<SolidTag>> {
        let shape = match &step.primitive {
            Primitive::Block { x, y, z } => solids::block(*x, *y, *z).ok()?,
            Primitive::Cylinder { radius, height } => solids::cylinder(*radius, *height).ok()?,
            Primitive::Sphere { radius } => solids::sphere(*radius).ok()?,
            Primitive::Torus { major, minor } => solids::torus(*major, *minor).ok()?,
            Primitive::Prism {
                sides,
                radius,
                distance,
            } => {
                let corners: Vec<Point3> = (0..*sides)
                    .map(|index| {
                        let angle = std::f64::consts::TAU * index as f64 / *sides as f64;
                        Point3::new(radius * angle.cos(), radius * angle.sin(), 0.0)
                    })
                    .collect();
                let face = faces::polygon(&corners).ok()?;
                solids::extruded(face, Vector3::z_axis(), *distance).ok()?
            }
            Primitive::Revolved {
                offset,
                width,
                height,
                angle,
            } => {
                let face = faces::polygon(&[
                    Point3::new(*offset, 0.0, 0.0),
                    Point3::new(offset + width, 0.0, 0.0),
                    Point3::new(offset + width, 0.0, *height),
                ])
                .ok()?;
                revolve::revolve_face(face, Axis3::z(), Rad64::new(*angle)).ok()?
            }
            Primitive::SweptDisc {
                radius,
                length,
                tilt,
            } => {
                let face = faces::circle(Plane::xy(), *radius).ok()?;
                let end = Point3::new(length * tilt, 0.0, *length);
                let path = edges::line(Point3::origin(), end).ok()?;
                sweep::sweep_face(face, &path.edge(), Default::default()).ok()?
            }
        };
        let turned = transform::rotated(shape, Axis3::z(), Rad64::new(step.turn));
        Some(transform::translated(
            turned,
            Vector3::new(step.offset[0], step.offset[1], step.offset[2]),
        ))
    }

    fn assert_outward(solid: &Shape<SolidTag>, context: &str) {
        validate_solid_orientation(solid.model(), solid.key())
            .unwrap_or_else(|error| panic!("{context} returned an invalid solid: {error}"));
    }

    proptest! {
        #![proptest_config(ProptestConfig {
            // A longer run for a hunt: NGK_PROPTEST_CASES=500 cargo test ...
            cases: std::env::var("NGK_PROPTEST_CASES")
                .ok()
                .and_then(|cases| cases.parse().ok())
                .unwrap_or(16),
            max_shrink_iters: 64,
            ..ProptestConfig::default()
        })]

        #[test]
        fn a_random_sequence_of_builders_never_panics_and_only_returns_outward_solids(
            steps in proptest::collection::vec(step(), 1..4)
        ) {
            let mut current: Option<Shape<SolidTag>> = None;
            for step in &steps {
                let Some(next) = build(step) else { continue };
                assert_outward(&next, "a primitive");
                current = match current.take() {
                    None => Some(next),
                    Some(target) => {
                        let combined = match step.combine {
                            Combine::Fuse => solids::fuse(target, [&next]),
                            Combine::Cut => solids::cut(target, [&next]),
                            Combine::Intersect => solids::intersect(target, [&next]),
                        };
                        match combined {
                            Ok(result) => {
                                assert_outward(&result, "a Boolean");
                                Some(result)
                            }
                            Err(_) => Some(next),
                        }
                    }
                };
            }
        }
    }
}

mod open_edit_reads {
    //! A view read on a labelling that does not describe the map reports the
    //! fault through the transaction instead of panicking.

    use ngk::builders::faces::add_polygon_with_holes;
    use ngk::geometry::{Plane, Point3};
    use ngk::model::Model;
    use ngk::topology::ModelEditError;
    use ngk::topology::embedding::EmbeddingError;
    use ngk::topology::payload::StandardPayload;

    type G = Model<StandardPayload>;

    fn triangle() -> (G, ngk::topology::shape_keys::FaceKey) {
        let mut g = G::new();
        let face = add_polygon_with_holes(
            &mut g,
            Plane::xy(),
            &[
                Point3::origin(),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            &[],
        )
        .expect("triangle");
        (g, face)
    }

    /// A registered vertex and an unmarked closure whose existing scaffold
    /// conflicts with a vertex attribute incorrectly reanchored onto it.
    fn vertex_and_closure(
        g: &mut G,
    ) -> (ngk::topology::shape_keys::VertexKey, ngk::topology::Dart) {
        let vertex = g.iter_vertices().next().expect("triangle vertex").0;
        let disc = ngk::builders::faces::add_circle(g, Plane::xy(), 2.0).expect("disc");
        let closure = g.face_unchecked(disc).edges()[0].dart();
        (vertex, closure)
    }

    #[test]
    fn a_view_read_on_a_conflicting_labelling_rolls_the_transaction_back() {
        let (mut g, face) = triangle();
        let (vertex, closure) = vertex_and_closure(&mut g);
        let darts = g.dart_count();

        let result = g.transaction(|edit| {
            edit.vertex_attr_mut_unchecked(vertex).dart = closure;
            assert!(matches!(
                edit.model().try_embedding_index(),
                Err(EmbeddingError::ConflictingOwnership { .. })
            ));
            // The read cannot be answered, and the operation is not told to
            // stop: it gets an empty answer and carries on.
            assert!(edit.face_unchecked(face).loops().is_empty());
            Ok::<_, ModelEditError>(())
        });

        assert!(
            matches!(
                result,
                Err(ModelEditError::InvalidEmbedding(
                    EmbeddingError::ConflictingOwnership { .. }
                ))
            ),
            "{result:?}"
        );
        assert_eq!(g.dart_count(), darts);
        assert_eq!(
            g.face_unchecked(face).loops().len(),
            1,
            "the model is as it was"
        );
    }

    #[test]
    #[should_panic(expected = "a triangle has an outer loop")]
    fn a_panic_after_a_read_fault_propagates() {
        let (mut g, face) = triangle();
        let (vertex, closure) = vertex_and_closure(&mut g);

        let _ = g.transaction(|edit| -> Result<(), ModelEditError> {
            edit.vertex_attr_mut_unchecked(vertex).dart = closure;
            let loops = edit.face_unchecked(face).loops();
            // What a builder does with the loops it never got.
            let _outer = loops.first().expect("a triangle has an outer loop");
            Ok(())
        });
    }

    #[test]
    #[should_panic(expected = "a builder's own mistake")]
    fn a_panic_without_a_read_fault_propagates() {
        let (mut g, _) = triangle();
        let _ = g.transaction(|edit| -> Result<(), ModelEditError> {
            edit.deferred(|raw| Ok::<_, ModelEditError>(raw.add_dart()))?;
            panic!("a builder's own mistake");
        });
    }
}

mod commit_checks {
    //! The solid post-condition is what refuses an inward solid, and it can be
    //! turned off.

    use ngk::geometry::{Axis3, Point3};
    use ngk::model::CommitChecks;
    use ngk::modeling::{faces, revolve};
    use ngk::topology::validation::validate_solid_orientation;
    use radians::Rad64;

    #[test]
    fn opening_an_existing_solid_shell_is_refused_and_restores_the_model() {
        let mut solid = ngk::modeling::solids::block(1.0, 1.0, 1.0).expect("block");
        let model = solid.model_mut();
        model.set_commit_checks(CommitChecks {
            solid_orientation: true,
        });
        let before = serde_json::to_value(&*model).expect("model serializes");
        let dart = model.iter_edges().next().expect("block edge").1.dart;

        let result = model.transaction(|edit| edit.unsew_edge(dart));

        assert!(
            matches!(
                result,
                Err(ngk::topology::ModelEditError::InvalidSolid {
                    reason: ngk::topology::validation::ModelValidationError::SolidShellOpen { .. },
                    ..
                })
            ),
            "{result:?}"
        );
        assert_eq!(
            serde_json::to_value(&*model).expect("model serializes"),
            before
        );
    }

    fn triangle() -> ngk::topology::shape::Shape<ngk::topology::shape::FaceTag> {
        faces::polygon(&[
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 1.0),
        ])
        .expect("triangle")
    }

    #[test]
    fn an_inward_solid_does_not_commit_unless_the_check_is_off() {
        let refused = revolve::revolve_face(triangle(), Axis3::z(), Rad64::new(-1.0));
        assert!(refused.is_err(), "a negative turn is refused");

        let mut face = triangle();
        face.model_mut().set_commit_checks(CommitChecks {
            solid_orientation: false,
        });
        let committed = revolve::revolve_face(face, Axis3::z(), Rad64::new(-1.0))
            .expect("with the check off the solid commits");
        assert!(
            validate_solid_orientation(committed.model(), committed.key()).is_err(),
            "and it is the check that would have refused it"
        );
    }
}
