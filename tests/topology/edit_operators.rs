//! Each edit operator leaves the model valid: the labelling describes the map
//! after every call, views read between two calls answer, and an operator that
//! refuses changes nothing.

use ngk::builders::edges::add_line;
use ngk::builders::faces::{add_circle, add_polygon_with_holes};
use ngk::geometry::{Curve, Fraction, Plane, Point3};
use ngk::model::{Cell1, Model};
use ngk::topology::ModelEditError;
use ngk::topology::edge::Edge;
use ngk::topology::edit_operators::{EdgeCutPlan, OperatorError};
use ngk::topology::gmap::Dart;
use ngk::topology::payload::StandardPayload;
use ngk::topology::shape_keys::{EdgeKey, FaceKey};

type G = Model<StandardPayload>;

fn square(g: &mut G, origin: Point3) -> FaceKey {
    let corner = |x: f64, y: f64| Point3::new(origin.x + x, origin.y + y, origin.z);
    add_polygon_with_holes(
        g,
        Plane::xy(),
        &[
            corner(0.0, 0.0),
            corner(1.0, 0.0),
            corner(1.0, 1.0),
            corner(0.0, 1.0),
        ],
        &[],
    )
    .expect("a square should build")
}

/// The dart of `face`'s boundary running along the edge that passes through `at`.
fn dart_through(g: &G, face: FaceKey, at: Point3) -> Dart {
    g.face_unchecked(face)
        .edges()
        .into_iter()
        .find(|edge| edge.trimmed_curve().contains(at, 1e-9))
        .expect("the face has an edge through the point")
        .dart()
}

/// The dart and key of the single rim edge of a disc.
fn rim(g: &G, face: FaceKey) -> (Dart, EdgeKey) {
    let edges = g.face_unchecked(face).edges();
    (edges[0].dart(), edges[0].key())
}

fn edge_count(g: &G) -> usize {
    g.iter_edges().count()
}

fn readable(g: &G, faces: &[FaceKey]) {
    for &face in faces {
        assert!(
            !g.face_unchecked(face).loops().is_empty(),
            "a face is readable after an operator"
        );
    }
}

#[test]
fn marking_an_unmarked_edge_puts_a_vertex_on_its_closure() {
    let mut g = G::new();
    let face = add_circle(&mut g, Plane::xy(), 2.0).expect("disc");
    let rim = g.face_unchecked(face).edges()[0].key();
    assert!(matches!(g.edge_unchecked(rim), Edge::Unmarked(_)));

    let vertex = g
        .transaction(|edit| {
            let vertex = edit.mark_edge(rim, Point3::new(2.0, 0.0, 0.0))?;
            edit.check_consistent()?;
            readable(edit, &[face]);
            Ok::<_, ModelEditError>(vertex)
        })
        .expect("an unmarked edge can be marked");

    assert!(matches!(g.edge_unchecked(rim), Edge::Marked(_)));
    assert_eq!(
        g.vertex_attr_unchecked(vertex).point,
        Point3::new(2.0, 0.0, 0.0)
    );
    g.embedding_index();
    assert!(g.try_embedding_index().is_ok());
}

#[test]
fn marking_a_marked_edge_is_refused_and_changes_nothing() {
    let mut g = G::new();
    let face = add_circle(&mut g, Plane::xy(), 2.0).expect("disc");
    let rim = g.face_unchecked(face).edges()[0].key();
    g.transaction(|edit| edit.mark_edge(rim, Point3::new(2.0, 0.0, 0.0)))
        .expect("first mark");
    let (darts, vertices) = (g.dart_count(), g.iter_vertices().count());

    let error = g
        .transaction(|edit| edit.mark_edge(rim, Point3::new(0.0, 2.0, 0.0)))
        .expect_err("a marked edge cannot be marked again");

    assert!(matches!(
        error,
        ModelEditError::Operator(OperatorError::EdgeNotUnmarked { .. })
    ));
    assert_eq!(
        (g.dart_count(), g.iter_vertices().count()),
        (darts, vertices)
    );
}

fn halves(from: Point3, to: Point3, at: Fraction) -> EdgeCutPlan {
    let point = from + (to - from) * at.value();
    EdgeCutPlan {
        point,
        first: Curve::line(from, point),
        second: Curve::line(point, to),
    }
}

#[test]
fn cutting_a_free_edge_makes_two_edges_meeting_at_a_vertex() {
    let mut g = G::new();
    let (from, to) = (Point3::origin(), Point3::new(4.0, 0.0, 0.0));
    let edge = add_line(&mut g, from, to).expect("line");

    let cut = g
        .transaction(|edit| {
            let cut = edit.cut_edge(edge, halves(from, to, Fraction::new(0.25)))?;
            edit.check_consistent()?;
            Ok::<_, ModelEditError>(cut)
        })
        .expect("a bounded edge can be cut");

    assert_eq!(edge_count(&g), 2);
    assert_eq!(g.iter_vertices().count(), 3);
    assert_eq!(cut.first, edge);
    assert_eq!(
        g.vertex_attr_unchecked(cut.vertex).point,
        Point3::new(1.0, 0.0, 0.0)
    );
}

#[test]
fn cutting_an_edge_preserves_alpha3_connections() {
    use ngk::topology::attributes::{EdgeAttr, VertexAttr};
    use ngk::topology::gmap::Dim;
    use ngk::topology::validation::validate_gmap;

    let mut model = G::new();
    let from = Point3::origin();
    let to = Point3::new(4.0, 0.0, 0.0);
    let edge = model
        .transaction(|edit| {
            edit.deferred(|raw| {
                let [a, b, c, d] = [(); 4].map(|_| raw.add_dart());
                raw.link(Dim::Zero, a, b)?;
                raw.link(Dim::Zero, c, d)?;
                raw.link(Dim::Three, a, c)?;
                raw.link(Dim::Three, b, d)?;
                raw.add_vertex(VertexAttr::new(a, from));
                raw.add_vertex(VertexAttr::new(b, to));
                Ok::<_, ModelEditError>(raw.add_edge(EdgeAttr::new(a, Curve::line(from, to))))
            })
        })
        .expect("an edge shared across alpha3 builds");

    let cut = model
        .transaction(|edit| {
            let cut = edit.cut_edge(edge, halves(from, to, Fraction::new(0.5)))?;
            validate_gmap(edit.topology()).expect("a cut preserves the map axioms");
            edit.check_consistent()?;
            Ok::<_, ModelEditError>(cut)
        })
        .expect("the shared edge can be cut");

    assert_eq!(cut.first, edge);
    assert_eq!(model.iter_edges().count(), 2);
    assert_eq!(model.iter_vertices().count(), 3);
    let new_cell = model.vertex_attr_unchecked(cut.vertex).dart;
    assert_ne!(model.alpha(Dim::Three, new_cell), new_cell);
}

#[test]
fn cutting_an_edge_two_faces_share_cuts_both_boundaries() {
    let mut g = G::new();
    let left = square(&mut g, Point3::origin());
    let right = square(&mut g, Point3::new(1.0, 0.0, 0.0));
    let at = Point3::new(1.0, 0.5, 0.0);
    let (shared_left, shared_right) = (dart_through(&g, left, at), dart_through(&g, right, at));
    g.transaction(|edit| edit.sew_edges(shared_left, shared_right))
        .expect("the sides match");
    let shared = g.cell_key_unchecked::<Cell1>(shared_left);

    let edges_before = edge_count(&g);
    g.transaction(|edit| {
        let (from, to) = (Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0));
        edit.cut_edge(shared, halves(from, to, Fraction::new(0.5)))?;
        edit.check_consistent()?;
        readable(edit, &[left, right]);
        Ok::<_, ModelEditError>(())
    })
    .expect("a shared edge can be cut");

    assert_eq!(edge_count(&g), edges_before + 1);
    assert_eq!(g.face_unchecked(left).edges().len(), 5);
    assert_eq!(g.face_unchecked(right).edges().len(), 5);
}

#[test]
fn cutting_an_unmarked_edge_is_refused() {
    let mut g = G::new();
    let face = add_circle(&mut g, Plane::xy(), 2.0).expect("disc");
    let rim = g.face_unchecked(face).edges()[0].key();
    let plan = halves(
        Point3::origin(),
        Point3::new(1.0, 0.0, 0.0),
        Fraction::new(0.5),
    );

    let error = g
        .transaction(|edit| edit.cut_edge(rim, plan))
        .expect_err("an unmarked edge has no corner to separate from");

    assert!(matches!(
        error,
        ModelEditError::Operator(OperatorError::EdgeHasNoCorner { .. })
    ));
}

#[test]
fn sewing_fuses_edges_and_vertices_and_unsewing_restores_them() {
    let mut g = G::new();
    let left = square(&mut g, Point3::origin());
    let right = square(&mut g, Point3::new(1.0, 0.0, 0.0));
    let at = Point3::new(1.0, 0.5, 0.0);
    let (survivor, removed) = (dart_through(&g, left, at), dart_through(&g, right, at));
    let (edges, vertices) = (edge_count(&g), g.iter_vertices().count());

    let sewing = g
        .transaction(|edit| {
            let sewing = edit.sew_edges(survivor, removed)?;
            edit.check_consistent()?;
            readable(edit, &[left, right]);
            Ok::<_, ModelEditError>(sewing)
        })
        .expect("matching sides sew");

    assert_eq!(edge_count(&g), edges - 1);
    assert_eq!(g.iter_vertices().count(), vertices - 2);
    assert_eq!(sewing.vertices.len(), 2);
    assert_eq!(g.alpha(ngk::topology::gmap::Dim::Two, survivor), removed);

    g.transaction(|edit| {
        let unsewing = edit.unsew_edge(survivor)?;
        edit.check_consistent()?;
        readable(edit, &[left, right]);
        assert_eq!(unsewing.vertices.len(), 2);
        Ok::<_, ModelEditError>(())
    })
    .expect("a sewn edge unsews");

    assert_eq!(edge_count(&g), edges);
    assert_eq!(g.iter_vertices().count(), vertices);
    assert_eq!(g.alpha(ngk::topology::gmap::Dim::Two, survivor), survivor);
}

#[test]
fn sewing_a_side_to_a_circle_or_to_a_side_already_sewn_is_refused_and_changes_nothing() {
    let mut g = G::new();
    let left = square(&mut g, Point3::origin());
    let middle = square(&mut g, Point3::new(1.0, 0.0, 0.0));
    let right = square(&mut g, Point3::new(2.0, 0.0, 0.0));
    let disc = add_circle(&mut g, Plane::xy(), 2.0).expect("disc");
    let at = Point3::new(1.0, 0.5, 0.0);
    let (left_side, middle_side) = (dart_through(&g, left, at), dart_through(&g, middle, at));
    g.transaction(|edit| edit.sew_edges(left_side, middle_side))
        .expect("the sides match");
    let rim = g.face_unchecked(disc).edges()[0].dart();
    let right_side = dart_through(&g, right, Point3::new(2.0, 0.5, 0.0));
    let darts = g.dart_count();

    for (first, second) in [(right_side, rim), (left_side, right_side)] {
        let error = g
            .transaction(|edit| edit.sew_edges(first, second))
            .expect_err("these cannot be sewn");
        assert!(
            matches!(
                error,
                ModelEditError::Operator(OperatorError::NotSewable { .. })
            ),
            "{error:?}"
        );
        assert_eq!(g.dart_count(), darts);
    }
}

#[test]
fn sewing_an_unmarked_rim_to_a_marked_one_gives_a_marked_edge() {
    let mut g = G::new();
    let marked = add_circle(&mut g, Plane::xy(), 2.0).expect("disc");
    let unmarked = add_circle(&mut g, Plane::xy(), 2.0).expect("disc");
    let (marked_dart, marked_edge) = rim(&g, marked);
    let (unmarked_dart, _) = rim(&g, unmarked);
    g.transaction(|edit| edit.mark_edge(marked_edge, Point3::new(2.0, 0.0, 0.0)))
        .expect("mark one rim");

    let sewing = g
        .transaction(|edit| {
            let sewing = edit.sew_edges(unmarked_dart, marked_dart)?;
            edit.check_consistent()?;
            readable(edit, &[marked, unmarked]);
            Ok::<_, ModelEditError>(sewing)
        })
        .expect("the rims match");

    assert!(g.try_embedding_index().is_ok());
    assert!(matches!(g.edge_unchecked(sewing.edge), Edge::Marked(_)));
}

#[test]
fn sewing_two_unmarked_rims_keeps_the_closure_as_scaffold() {
    let mut g = G::new();
    let first = add_circle(&mut g, Plane::xy(), 2.0).expect("disc");
    let second = add_circle(&mut g, Plane::xy(), 2.0).expect("disc");
    let ((first_dart, _), (second_dart, _)) = (rim(&g, first), rim(&g, second));

    let sewing = g
        .transaction(|edit| {
            let sewing = edit.sew_edges(first_dart, second_dart)?;
            edit.check_consistent()?;
            readable(edit, &[first, second]);
            Ok::<_, ModelEditError>(sewing)
        })
        .expect("the rims match");

    assert!(matches!(g.edge_unchecked(sewing.edge), Edge::Unmarked(_)));
    assert!(g.try_embedding_index().is_ok());
}

#[test]
fn a_refused_operator_inside_a_failing_transaction_restores_the_model() {
    let mut g = G::new();
    let (from, to) = (Point3::origin(), Point3::new(4.0, 0.0, 0.0));
    let edge = add_line(&mut g, from, to).expect("line");
    let (darts, edges) = (g.dart_count(), edge_count(&g));

    let result: Result<(), ModelEditError> = g.transaction(|edit| {
        edit.cut_edge(edge, halves(from, to, Fraction::new(0.5)))?;
        edit.cut_edge(EdgeKey::default(), halves(from, to, Fraction::new(0.5)))?;
        Ok(())
    });

    assert!(result.is_err());
    assert_eq!((g.dart_count(), edge_count(&g)), (darts, edges));
}

mod deferred_blocks {
    //! Raw surgery runs in a block that is checked when it closes.

    use ngk::geometry::{Curve, Point3};
    use ngk::model::Model;
    use ngk::topology::ModelEditError;
    use ngk::topology::attributes::{EdgeAttr, VertexAttr};
    use ngk::topology::embedding::EntityOwner;
    use ngk::topology::gmap::Dim;
    use ngk::topology::payload::StandardPayload;

    type G = Model<StandardPayload>;

    #[test]
    fn a_block_that_ends_valid_is_kept() {
        let mut g = G::new();
        let darts = g
            .transaction(|edit| {
                edit.deferred(|raw| {
                    let first = raw.add_dart();
                    let second = raw.add_dart();
                    raw.link(Dim::Zero, first, second)?;
                    Ok::<_, ModelEditError>((first, second))
                })
            })
            .expect("a block that ends valid commits");

        assert_eq!(g.alpha(Dim::Zero, darts.0), darts.1);
    }

    #[test]
    fn a_block_that_ends_with_a_conflicting_labelling_is_an_error_and_rolls_back() {
        let mut g = G::new();
        let (vertex_dart, edge) = g
            .transaction(|edit| {
                edit.deferred(|raw| {
                    let first = raw.add_dart();
                    let second = raw.add_dart();
                    raw.link(Dim::Zero, first, second)?;
                    raw.add_vertex(VertexAttr::new(first, Point3::origin()));
                    let edge = raw.add_edge(EdgeAttr::new(
                        first,
                        Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
                    ));
                    Ok::<_, ModelEditError>((first, edge))
                })
            })
            .expect("a vertex and an edge");
        let darts = g.dart_count();

        let result = g.transaction(|edit| {
            edit.deferred(|raw| {
                raw.add_dart();
                // A vertex holds this 0-cell, and a stored label gives it to
                // an edge as scaffold as well.
                raw.own_cell(Dim::Zero, vertex_dart, EntityOwner::Edge(edge));
                Ok::<_, ModelEditError>(())
            })
        });

        assert!(
            matches!(result, Err(ModelEditError::InvalidEmbedding(_))),
            "{result:?}"
        );
        assert_eq!(g.dart_count(), darts);
    }

    #[test]
    fn a_block_that_breaks_the_maps_axioms_is_an_error() {
        let mut g = G::new();
        let result = g.transaction(|edit| {
            edit.deferred(|raw| {
                let [a, b, c, d] = [(); 4].map(|()| raw.add_dart());
                raw.link(Dim::Zero, a, b)?;
                raw.link(Dim::Zero, c, d)?;
                // alpha0 o alpha2 is not an involution when only one side of
                // the pairs is linked.
                raw.link(Dim::Two, a, c)?;
                Ok::<_, ModelEditError>(())
            })
        });

        assert!(
            matches!(result, Err(ModelEditError::InvalidTopology(_))),
            "{result:?}"
        );
        assert_eq!(g.dart_count(), 0);
    }

    #[test]
    fn an_error_from_the_block_is_the_transactions_error() {
        let mut g = G::new();
        let result = g.transaction(|edit| {
            edit.deferred(|raw| {
                let dart = raw.add_dart();
                raw.unlink(Dim::Zero, dart)?;
                Ok::<_, ModelEditError>(())
            })
        });

        assert!(matches!(
            result,
            Err(ModelEditError::DartAlreadyFree { .. })
        ));
        assert_eq!(g.dart_count(), 0);
    }
}

mod loose_faces {
    //! The loose-face operator builds one ring of `2n` darts for an ordered
    //! boundary and registers what each side names, leaving the model valid
    //! when it returns.

    use ngk::builders::edges::add_line;
    use ngk::geometry::{Curve, Plane, Point2, Point3, Surface, TrimmedCurve2};
    use ngk::model::Model;
    use ngk::topology::edit_operators::{
        LooseFaceBoundary, LooseFaceEdge, LooseFaceSide, OperatorError,
    };
    use ngk::topology::gmap::Dim;
    use ngk::topology::payload::StandardPayload;
    use ngk::topology::shape_keys::{EdgeKey, FaceKey, ProfileKey, SheetKey, SolidKey, VertexKey};
    use ngk::topology::{EditKey, EditPolicy, ModelEditError, Origin};

    type G = Model<StandardPayload>;

    /// Records every creation the commit's policy sees.
    #[derive(Default)]
    struct Recorder {
        created: Vec<(EditKey, Origin)>,
    }

    impl EditPolicy<StandardPayload> for Recorder {
        type Error = ModelEditError;

        fn vertex_created(
            &mut self,
            key: VertexKey,
            origin: Origin,
            _before: &G,
        ) -> Result<(), ModelEditError> {
            self.created.push((EditKey::Vertex(key), origin));
            Ok(())
        }

        fn edge_created(
            &mut self,
            key: EdgeKey,
            origin: Origin,
            _before: &G,
        ) -> Result<(), ModelEditError> {
            self.created.push((EditKey::Edge(key), origin));
            Ok(())
        }

        fn profile_created(
            &mut self,
            key: ProfileKey,
            origin: Origin,
            _before: &G,
        ) -> Result<(), ModelEditError> {
            self.created.push((EditKey::Profile(key), origin));
            Ok(())
        }

        fn face_created(
            &mut self,
            key: FaceKey,
            origin: Origin,
            _before: &G,
        ) -> Result<(), ModelEditError> {
            self.created.push((EditKey::Face(key), origin));
            Ok(())
        }

        fn sheet_created(
            &mut self,
            key: SheetKey,
            origin: Origin,
            _before: &G,
        ) -> Result<(), ModelEditError> {
            self.created.push((EditKey::Sheet(key), origin));
            Ok(())
        }

        fn solid_created(
            &mut self,
            key: SolidKey,
            origin: Origin,
            _before: &G,
        ) -> Result<(), ModelEditError> {
            self.created.push((EditKey::Solid(key), origin));
            Ok(())
        }
    }

    fn assert_seen_exactly(recorder: &Recorder, expected: &[(EditKey, Origin)]) {
        let mut remaining = recorder.created.clone();
        for event in expected {
            let index = remaining
                .iter()
                .position(|candidate| candidate == event)
                .unwrap_or_else(|| panic!("missing lineage event {event:?}"));
            remaining.swap_remove(index);
        }
        assert!(remaining.is_empty(), "unexpected events: {remaining:?}");
    }

    const CORNERS: [Point3; 4] = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    ];

    const UV: [Point2; 4] = [
        Point2::new(0.0, 0.0),
        Point2::new(1.0, 0.0),
        Point2::new(1.0, 1.0),
        Point2::new(0.0, 1.0),
    ];

    /// A quad boundary whose per-side origins differ from side to side, the
    /// way an extruded or revolved face names them.
    fn quad_boundary(source: EdgeKey, ends: [VertexKey; 2]) -> LooseFaceBoundary {
        let corner_origins = [
            vec![EditKey::Edge(source)],
            vec![EditKey::Vertex(ends[1])],
            vec![EditKey::Edge(source), EditKey::Vertex(ends[0])],
            vec![EditKey::Vertex(ends[0])],
        ];
        let edge_origins = [
            vec![EditKey::Edge(source)],
            vec![EditKey::Vertex(ends[1])],
            vec![EditKey::Edge(source)],
            vec![EditKey::Vertex(ends[0])],
        ];
        LooseFaceBoundary {
            surface: Surface::Plane(Plane::xy()),
            origins: vec![EditKey::Edge(source)],
            sides: (0..4)
                .map(|i| {
                    LooseFaceSide::registered(
                        CORNERS[i],
                        corner_origins[i].clone(),
                        Curve::line(CORNERS[i], CORNERS[(i + 1) % 4]),
                        edge_origins[i].clone(),
                        TrimmedCurve2::segment(UV[i], UV[(i + 1) % 4]),
                    )
                })
                .collect(),
        }
    }

    #[test]
    fn a_loose_quad_face_registers_four_edges_and_four_vertices_in_a_valid_embedding() {
        let mut g = G::new();
        let edge = add_line(&mut g, Point3::origin(), Point3::new(4.0, 0.0, 0.0)).expect("line");
        let ends: Vec<VertexKey> = g.iter_vertices().map(|(key, _)| key).collect();

        let built = g
            .transaction(|edit| {
                let built = edit.add_loose_face(quad_boundary(edge, [ends[0], ends[1]]))?;
                edit.check_consistent()?;
                Ok::<_, ModelEditError>(built)
            })
            .expect("the loose face builds");

        let view = g.face_unchecked(built.face);
        assert_eq!(view.edges().len(), 4);
        assert_eq!(view.vertices().len(), 4);
        assert!(!view.loops().is_empty());
        assert!(g.try_embedding_index().is_ok());
    }

    #[test]
    fn each_side_runs_forward_then_back_and_keys_its_pcurve_on_the_forward_dart() {
        let mut g = G::new();
        let edge = add_line(&mut g, Point3::origin(), Point3::new(4.0, 0.0, 0.0)).expect("line");
        let ends: Vec<VertexKey> = g.iter_vertices().map(|(key, _)| key).collect();

        let built = g
            .transaction(|edit| {
                let built = edit.add_loose_face(quad_boundary(edge, [ends[0], ends[1]]))?;
                edit.check_consistent()?;
                Ok::<_, ModelEditError>(built)
            })
            .expect("the loose face builds");

        let pcurves = &g.face_attr_unchecked(built.face).pcurves;
        for side in 0..4 {
            let forward = built.forward(side);
            let back = built.back(side);
            // The backward dart is the forward dart's alpha0 partner, and the
            // next side's forward dart follows it along alpha1.
            assert_eq!(g.alpha(Dim::Zero, forward), back);
            assert_eq!(g.alpha(Dim::Zero, back), forward);
            assert_eq!(g.alpha(Dim::One, back), built.forward((side + 1) % 4));
            assert_eq!(g.alpha(Dim::One, forward), built.back((side + 3) % 4));
            // The side's pcurve is keyed on its forward dart, and the side's
            // edge is anchored there.
            assert_eq!(
                pcurves.get(&forward),
                Some(&TrimmedCurve2::segment(UV[side], UV[(side + 1) % 4]))
            );
            let edge_key = built.edge(side).expect("every side registered an edge");
            assert_eq!(g.edge_unchecked(edge_key).dart(), forward);
        }
        // Each corner's vertex sits on the 0-cell the side's two darts share
        // and carries the side's corner point.
        let points: Vec<Point3> = g
            .face_unchecked(built.face)
            .vertices()
            .iter()
            .map(|vertex| *vertex.point())
            .collect();
        assert_eq!(points.len(), 4);
        for corner in CORNERS {
            assert!(points.contains(&corner), "missing corner {corner:?}");
        }
    }

    #[test]
    fn a_loose_face_with_three_sides_builds() {
        let mut g = G::new();
        let edge = add_line(&mut g, Point3::origin(), Point3::new(4.0, 0.0, 0.0)).expect("line");

        let built = g
            .transaction(|edit| {
                let built = edit.add_loose_face(LooseFaceBoundary {
                    surface: Surface::Plane(Plane::xy()),
                    origins: vec![EditKey::Edge(edge)],
                    sides: (0..3)
                        .map(|i| {
                            LooseFaceSide::registered(
                                CORNERS[i],
                                vec![EditKey::Edge(edge)],
                                Curve::line(CORNERS[i], CORNERS[(i + 1) % 3]),
                                vec![EditKey::Edge(edge)],
                                TrimmedCurve2::segment(UV[i], UV[(i + 1) % 3]),
                            )
                        })
                        .collect(),
                })?;
                edit.check_consistent()?;
                Ok::<_, ModelEditError>(built)
            })
            .expect("a triangle builds");

        assert_eq!(g.face_unchecked(built.face).edges().len(), 3);
        assert_eq!(g.face_unchecked(built.face).vertices().len(), 3);
    }

    #[test]
    fn a_loose_face_with_fewer_than_three_sides_is_refused_and_changes_nothing() {
        let mut g = G::new();
        let edge = add_line(&mut g, Point3::origin(), Point3::new(4.0, 0.0, 0.0)).expect("line");
        let (darts, edges, vertices) = (
            g.dart_count(),
            super::edge_count(&g),
            g.iter_vertices().count(),
        );

        let error = g
            .transaction(|edit| {
                edit.add_loose_face(LooseFaceBoundary {
                    surface: Surface::Plane(Plane::xy()),
                    origins: vec![EditKey::Edge(edge)],
                    sides: (0..2)
                        .map(|i| {
                            LooseFaceSide::sewn(TrimmedCurve2::segment(UV[i], UV[(i + 1) % 2]))
                        })
                        .collect(),
                })
            })
            .expect_err("two sides cannot close a boundary");

        assert!(
            matches!(
                error,
                ModelEditError::Operator(OperatorError::TooFewLooseFaceSides { sides: 2 })
            ),
            "{error:?}"
        );
        assert_eq!(
            (
                g.dart_count(),
                super::edge_count(&g),
                g.iter_vertices().count()
            ),
            (darts, edges, vertices)
        );
    }

    #[test]
    fn the_origins_land_on_the_entities_the_sides_named() {
        let mut g = G::new();
        let edge = add_line(&mut g, Point3::origin(), Point3::new(4.0, 0.0, 0.0)).expect("line");
        let ends: Vec<VertexKey> = g.iter_vertices().map(|(key, _)| key).collect();
        let boundary = quad_boundary(edge, [ends[0], ends[1]]);

        let mut recorder = Recorder::default();
        let built = g
            .transaction_with_policy(&mut recorder, |edit| {
                let built = edit.add_loose_face(boundary)?;
                edit.check_consistent()?;
                Ok::<_, ModelEditError>(built)
            })
            .expect("the loose face builds");

        let mut expected = vec![(
            EditKey::Profile(g.profile_key(built.forward(0)).expect("a profile")),
            Origin::Derived {
                sources: vec![EditKey::Edge(edge)],
            },
        )];
        expected.push((
            EditKey::Face(built.face),
            Origin::Derived {
                sources: vec![EditKey::Edge(edge)],
            },
        ));
        for (side, corner) in CORNERS.iter().enumerate() {
            let corner_origins = match side {
                0 => vec![EditKey::Edge(edge)],
                1 => vec![EditKey::Vertex(ends[1])],
                2 => vec![EditKey::Edge(edge), EditKey::Vertex(ends[0])],
                _ => vec![EditKey::Vertex(ends[0])],
            };
            let edge_origins = match side {
                0 | 2 => vec![EditKey::Edge(edge)],
                1 => vec![EditKey::Vertex(ends[1])],
                _ => vec![EditKey::Vertex(ends[0])],
            };
            let vertex = g
                .face_unchecked(built.face)
                .vertices()
                .into_iter()
                .find(|vertex| vertex.point() == corner)
                .expect("each corner has a vertex");
            expected.push((
                EditKey::Vertex(vertex.key()),
                Origin::Derived {
                    sources: corner_origins,
                },
            ));
            expected.push((
                EditKey::Edge(built.edge(side).expect("an edge")),
                Origin::Derived {
                    sources: edge_origins,
                },
            ));
        }
        assert_seen_exactly(&recorder, &expected);
    }

    #[test]
    fn a_side_naming_no_corner_and_no_edge_registers_nothing_for_it() {
        let mut g = G::new();
        let edge = add_line(&mut g, Point3::origin(), Point3::new(4.0, 0.0, 0.0)).expect("line");
        let (edges, vertices) = (super::edge_count(&g), g.iter_vertices().count());

        let built = g
            .transaction(|edit| {
                let built = edit.add_loose_face(LooseFaceBoundary {
                    surface: Surface::Plane(Plane::xy()),
                    origins: vec![EditKey::Edge(edge)],
                    sides: (0..4)
                        .map(|i| LooseFaceSide {
                            corner: None,
                            edge: (i == 1).then_some(LooseFaceEdge {
                                curve: Curve::line(CORNERS[i], CORNERS[(i + 1) % 4]),
                                origins: vec![EditKey::Edge(edge)],
                            }),
                            pcurve: TrimmedCurve2::segment(UV[i], UV[(i + 1) % 4]),
                        })
                        .collect(),
                })?;
                edit.check_consistent()?;
                Ok::<_, ModelEditError>(built)
            })
            .expect("the ring builds");

        // Only one edge came into being besides the face and its profile: the
        // other sides' corners and edges are left to the caller to sew.
        assert_eq!(super::edge_count(&g), edges + 1);
        assert_eq!(g.iter_vertices().count(), vertices);
        assert_eq!(built.edge(0), None);
        assert!(built.edge(1).is_some());
        assert_eq!(built.edge(2), None);
        assert_eq!(built.edge(3), None);
        let pcurves = &g.face_attr_unchecked(built.face).pcurves;
        for side in 0..4 {
            assert!(pcurves.contains_key(&built.forward(side)));
            assert_eq!(g.alpha(Dim::Zero, built.forward(side)), built.back(side));
        }
        assert!(g.try_embedding_index().is_ok());
    }
}
