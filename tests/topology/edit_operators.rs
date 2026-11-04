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
        &[corner(0.0, 0.0), corner(1.0, 0.0), corner(1.0, 1.0), corner(0.0, 1.0)],
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
    assert_eq!(g.vertex_attr_unchecked(vertex).point, Point3::new(2.0, 0.0, 0.0));
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
    assert_eq!((g.dart_count(), g.iter_vertices().count()), (darts, vertices));
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
    assert_eq!(g.vertex_attr_unchecked(cut.vertex).point, Point3::new(1.0, 0.0, 0.0));
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
    let plan = halves(Point3::origin(), Point3::new(1.0, 0.0, 0.0), Fraction::new(0.5));

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

    use ngk::model::Model;
    use ngk::topology::ModelEditError;
    use ngk::topology::embedding::EntityOwner;
    use ngk::topology::gmap::Dim;
    use ngk::topology::payload::StandardPayload;
    use ngk::topology::attributes::{EdgeAttr, VertexAttr};
    use ngk::geometry::{Curve, Point3};

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
                let first = edit.add_dart();
                let second = edit.add_dart();
                edit.link(Dim::Zero, first, second)?;
                edit.add_vertex(VertexAttr::new(first, Point3::origin()));
                let edge = edit.add_edge(EdgeAttr::new(
                    first,
                    Curve::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)),
                ));
                Ok::<_, ModelEditError>((first, edge))
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

        assert!(matches!(result, Err(ModelEditError::DartAlreadyFree { .. })));
        assert_eq!(g.dart_count(), 0);
    }
}
