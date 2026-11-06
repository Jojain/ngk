use nalgebra::{Vector2, Vector3};
use ngk::builders::faces::add_face;
use ngk::builders::solids::add_extruded_face;
use ngk::geometry::dim2::{
    curves::Circle2,
    trimmed::{Arc2, Segment2},
    utils::Point2,
};
use ngk::geometry::{Plane, Point3};
use ngk::model::Model;
use ngk::sketch::{SketchBuildError, SketchCurve, SketchProfiles, build_profiles};
use ngk::solver::{ArcId, CircleId, Constraint2d as C, PointId, SegmentId, System2d};
use ngk::topology::closed::Closeable;
use ngk::topology::edge::Edge;
use std::f64::consts::PI;

const BOTTOM: SegmentId = SegmentId::new(1);
const RIGHT: SegmentId = SegmentId::new(2);
const TOP: SegmentId = SegmentId::new(3);
const LEFT: SegmentId = SegmentId::new(4);
const CHAMFER: SegmentId = SegmentId::new(5);

fn segment(a: (f64, f64), b: (f64, f64)) -> Segment2 {
    Segment2::new(Point2::new(a.0, a.1), Point2::new(b.0, b.1))
}

fn constrain(system: &mut System2d, constraints: Vec<C>) {
    for constraint in constraints {
        system.add_constraint(constraint).unwrap();
    }
}

/// A 10 by 10 square whose sides are drawn loose and closed by coincidences.
fn square() -> System2d {
    let mut system = System2d::default();
    system
        .insert_segment(BOTTOM, segment((0., 0.), (9., 0.)))
        .unwrap();
    system
        .insert_segment(RIGHT, segment((9.2, 0.1), (9.4, 9.)))
        .unwrap();
    system
        .insert_segment(TOP, segment((9.1, 9.2), (0.2, 9.)))
        .unwrap();
    system
        .insert_segment(LEFT, segment((-0.2, 9.1), (0.1, 0.1)))
        .unwrap();
    constrain(
        &mut system,
        vec![
            C::coincident(BOTTOM.end(), RIGHT.start()),
            C::coincident(RIGHT.end(), TOP.start()),
            C::coincident(TOP.end(), LEFT.start()),
            C::coincident(LEFT.end(), BOTTOM.start()),
            C::fixed_point(BOTTOM.start()),
            C::horizontal(BOTTOM),
            C::horizontal(TOP),
            C::vertical(RIGHT),
            C::vertical(LEFT),
            C::length(BOTTOM, 10.),
            C::length(LEFT, 10.),
        ],
    );
    system.solve().unwrap();
    system
}

const SQUARE: [SketchCurve; 4] = [
    SketchCurve::Segment(BOTTOM),
    SketchCurve::Segment(RIGHT),
    SketchCurve::Segment(TOP),
    SketchCurve::Segment(LEFT),
];

fn build(system: &System2d, curves: &[SketchCurve]) -> (Model, SketchProfiles) {
    let mut model = Model::new();
    let built = build_profiles(&mut model, system, Plane::xy(), curves).unwrap();
    (model, built)
}

fn refused(system: &System2d, curves: &[SketchCurve]) -> SketchBuildError {
    let mut model = Model::<ngk::topology::payload::StandardPayload>::new();
    let error = build_profiles(&mut model, system, Plane::xy(), curves).unwrap_err();
    assert_eq!(model.iter_edges().count(), 0, "a refusal adds nothing");
    error
}

fn ends(model: &Model, built: &SketchProfiles, curve: impl Into<SketchCurve>) -> [Point3; 2] {
    let Some(Edge::Bounded(edge)) = built.edge(model, curve).unwrap() else {
        panic!("the curve has no bounded edge");
    };
    [*edge.start().point(), *edge.end().point()]
}

fn near(a: Point3, b: (f64, f64, f64)) -> bool {
    (a - Point3::new(b.0, b.1, b.2)).norm() < 1e-7
}

#[test]
fn a_constrained_square_is_one_closed_profile_with_four_vertices() {
    let (model, built) = build(&square(), &SQUARE);
    assert_eq!(built.profiles.len(), 1);
    assert_eq!(built.profiles[0].curves.len(), 4);
    assert_eq!(built.edges.len(), 4);
    let profile = model.profile_unchecked(built.profiles[0].profile);
    assert!(profile.is_closed());
    assert_eq!(profile.vertices().len(), 4);
    assert!((profile.length() - 40.).abs() < 1e-7);
}

#[test]
fn a_curve_id_leads_to_the_edge_it_drew() {
    let (model, built) = build(&square(), &SQUARE);
    let [start, end] = ends(&model, &built, RIGHT);
    assert!(near(start, (10., 0., 0.)));
    assert!(near(end, (10., 10., 0.)));
}

#[test]
fn the_profile_makes_a_face_and_a_solid() {
    let (mut model, built) = build(&square(), &SQUARE);
    let face = add_face(&mut model, built.profiles[0].profile).unwrap();
    let solid = add_extruded_face(&mut model, face, Vector3::new(0., 0., 5.)).unwrap();
    let volume = model.solid_unchecked(solid.solid).volume().unwrap();
    assert!((volume.abs() - 500.).abs() < 1e-6);
}

/// The square with its bottom-right corner cut: the sides keep their ids and a new
/// segment joins them.
fn chamfered_square() -> System2d {
    let mut system = System2d::default();
    system
        .insert_segment(BOTTOM, segment((0., 0.), (8., 0.)))
        .unwrap();
    system
        .insert_segment(CHAMFER, segment((8., 0.), (10., 2.)))
        .unwrap();
    system
        .insert_segment(RIGHT, segment((10., 2.), (10., 10.)))
        .unwrap();
    system
        .insert_segment(TOP, segment((10., 10.), (0., 10.)))
        .unwrap();
    system
        .insert_segment(LEFT, segment((0., 10.), (0., 0.)))
        .unwrap();
    constrain(
        &mut system,
        vec![
            C::coincident(BOTTOM.end(), CHAMFER.start()),
            C::coincident(CHAMFER.end(), RIGHT.start()),
            C::coincident(RIGHT.end(), TOP.start()),
            C::coincident(TOP.end(), LEFT.start()),
            C::coincident(LEFT.end(), BOTTOM.start()),
            C::fixed_point(BOTTOM.start()),
        ],
    );
    system.solve().unwrap();
    system
}

#[test]
fn a_new_curve_leaves_the_others_their_identity_though_not_their_shape() {
    let (before_model, before) = build(&square(), &SQUARE);
    let [_, before_right_end] = ends(&before_model, &before, RIGHT);

    let curves = [
        BOTTOM.into(),
        CHAMFER.into(),
        RIGHT.into(),
        TOP.into(),
        LEFT.into(),
    ];
    let (model, after) = build(&chamfered_square(), &curves);
    assert!(
        model
            .profile_unchecked(after.profiles[0].profile)
            .is_closed()
    );

    let [start, end] = ends(&model, &after, RIGHT);
    assert!(near(start, (10., 2., 0.)), "the right side got shorter");
    assert!(near(end, (10., 10., 0.)));
    assert!((end - before_right_end).norm() < 1e-7);
    let [start, end] = ends(&model, &after, CHAMFER);
    assert!(near(start, (8., 0., 0.)));
    assert!(near(end, (10., 2., 0.)));
}

#[test]
fn a_stored_system_builds_the_same_profile() {
    let json = serde_json::to_string(&square()).unwrap();
    let mut restored: System2d = serde_json::from_str(&json).unwrap();
    restored.solve().unwrap();
    let (model, built) = build(&restored, &SQUARE);
    let [start, end] = ends(&model, &built, RIGHT);
    assert!(near(start, (10., 0., 0.)));
    assert!(near(end, (10., 10., 0.)));
}

#[test]
fn an_arc_keeps_its_direction_and_sweep() {
    for sweep in [3. * PI / 2., -3. * PI / 2.] {
        let (arc, line) = (ArcId::new(1), SegmentId::new(1));
        let mut system = System2d::default();
        system
            .insert_arc(arc, Arc2::new(Point2::origin(), Vector2::x(), 5., sweep))
            .unwrap();
        let end = system.point(arc.end()).unwrap();
        system
            .insert_segment(line, Segment2::new(end, Point2::new(5., 0.)))
            .unwrap();
        constrain(
            &mut system,
            vec![
                C::coincident(arc.end(), line.start()),
                C::coincident(line.end(), arc.start()),
            ],
        );
        system.solve().unwrap();
        let (model, built) = build(&system, &[arc.into(), line.into()]);
        let profile = model.profile_unchecked(built.profiles[0].profile);
        assert!(profile.is_closed());
        let expected = 5. * 3. * PI / 2. + 50_f64.sqrt();
        assert!(
            (profile.length() - expected).abs() < 1e-6,
            "sweep {sweep}: length {}",
            profile.length()
        );
    }
}

#[test]
fn an_arc_leaving_a_segment_tangentially_is_a_joint_not_a_crossing() {
    let (line, arc) = (SegmentId::new(1), ArcId::new(1));
    let mut system = System2d::default();
    system
        .insert_segment(line, segment((0., 0.), (10., 0.)))
        .unwrap();
    system
        .insert_arc(
            arc,
            Arc2::new(Point2::new(10., 3.), -Vector2::y(), 3., PI / 2.),
        )
        .unwrap();
    constrain(
        &mut system,
        vec![
            C::coincident(line.end(), arc.start()),
            C::tangent_line_circle(line, arc),
        ],
    );
    system.solve().unwrap();
    let (model, built) = build(&system, &[line.into(), arc.into()]);
    assert_eq!(built.profiles.len(), 1);
    assert!(
        !model
            .profile_unchecked(built.profiles[0].profile)
            .is_closed()
    );
}

#[test]
fn a_circle_is_a_profile_of_its_own() {
    let circle = CircleId::new(1);
    let mut system = System2d::default();
    system
        .insert_circle(circle, Circle2::new(Point2::new(3., 4.), Vector2::x(), 2.))
        .unwrap();
    system.solve().unwrap();
    let (model, built) = build(&system, &[circle.into()]);
    let profile = model.profile_unchecked(built.profiles[0].profile);
    assert!(profile.is_closed());
    assert!((profile.length() - 2. * PI * 2.).abs() < 1e-7);
}

#[test]
fn separate_runs_are_separate_profiles_open_or_closed() {
    let (a, b, c) = (SegmentId::new(1), SegmentId::new(2), SegmentId::new(3));
    let mut system = System2d::default();
    system
        .insert_segment(a, segment((0., 0.), (1., 0.)))
        .unwrap();
    system
        .insert_segment(b, segment((1., 0.), (1., 1.)))
        .unwrap();
    system
        .insert_segment(c, segment((5., 5.), (6., 6.)))
        .unwrap();
    constrain(&mut system, vec![C::coincident(a.end(), b.start())]);
    system.solve().unwrap();
    let (model, built) = build(&system, &[a.into(), b.into(), c.into()]);
    assert_eq!(built.profiles.len(), 2);
    assert_eq!(built.profiles[0].curves, vec![a.into(), b.into()]);
    assert_eq!(built.profiles[1].curves, vec![c.into()]);
    assert!(
        !model
            .profile_unchecked(built.profiles[0].profile)
            .is_closed()
    );
    assert_eq!(
        model
            .profile_unchecked(built.profiles[0].profile)
            .vertices()
            .len(),
        3
    );
}

#[test]
fn a_chain_of_coincidences_through_what_is_not_drawn_still_joins() {
    let (a, b, joint) = (SegmentId::new(1), SegmentId::new(2), PointId::new(1));
    let construction = SegmentId::new(3);
    let mut system = System2d::default();
    system
        .insert_segment(a, segment((0., 0.), (1., 0.)))
        .unwrap();
    system
        .insert_segment(b, segment((1., 0.), (1., 1.)))
        .unwrap();
    system.insert_point(joint, Point2::new(1., 0.)).unwrap();
    system
        .insert_segment(construction, segment((1., 0.), (3., 3.)))
        .unwrap();
    constrain(
        &mut system,
        vec![
            C::coincident(a.end(), joint),
            C::coincident(joint, construction.start()),
            C::coincident(construction.start(), b.start()),
        ],
    );
    system.solve().unwrap();
    let (_, built) = build(&system, &[a.into(), b.into()]);
    assert_eq!(built.profiles.len(), 1);
    assert_eq!(built.edges.len(), 2);
}

#[test]
fn a_build_goes_into_the_model_it_is_given() {
    let mut model = Model::new();
    let first = build_profiles(&mut model, &square(), Plane::xy(), &SQUARE).unwrap();
    let raised = Plane::new(Point3::new(0., 0., 7.), Vector3::x(), Vector3::z());
    let second = build_profiles(&mut model, &square(), raised, &SQUARE).unwrap();
    assert_eq!(model.iter_edges().count(), 8);
    let [start, _] = ends(&model, &second, RIGHT);
    assert!(near(start, (10., 0., 7.)));
    assert_ne!(first.profiles[0].profile, second.profiles[0].profile);
}

#[test]
fn an_unsolved_or_edited_system_is_refused() {
    let mut system = System2d::default();
    system
        .insert_segment(BOTTOM, segment((0., 0.), (1., 0.)))
        .unwrap();
    assert!(matches!(
        refused(&system, &[BOTTOM.into()]),
        SketchBuildError::NotSolved
    ));
    system.solve().unwrap();
    system
        .update_segment(BOTTOM, segment((0., 0.), (2., 0.)))
        .unwrap();
    assert!(matches!(
        refused(&system, &[BOTTOM.into()]),
        SketchBuildError::NotSolved
    ));
}

#[test]
fn an_unknown_or_repeated_curve_is_refused() {
    let system = square();
    assert!(matches!(
        refused(&system, &[CHAMFER.into()]),
        SketchBuildError::UnknownCurve(curve) if curve == CHAMFER.into()
    ));
    assert!(matches!(
        refused(&system, &[BOTTOM.into(), BOTTOM.into()]),
        SketchBuildError::DuplicateCurve(curve) if curve == BOTTOM.into()
    ));
}

#[test]
fn three_joined_ends_are_refused_by_name() {
    let (a, b, c) = (SegmentId::new(1), SegmentId::new(2), SegmentId::new(3));
    let mut system = System2d::default();
    system
        .insert_segment(a, segment((0., 0.), (1., 0.)))
        .unwrap();
    system
        .insert_segment(b, segment((1., 0.), (2., 1.)))
        .unwrap();
    system
        .insert_segment(c, segment((1., 0.), (2., -1.)))
        .unwrap();
    constrain(
        &mut system,
        vec![
            C::coincident(a.end(), b.start()),
            C::coincident(a.end(), c.start()),
        ],
    );
    system.solve().unwrap();
    match refused(&system, &[a.into(), b.into(), c.into()]) {
        SketchBuildError::Branching { curves } => assert_eq!(curves.len(), 3),
        other => panic!("expected a branching refusal, got {other:?}"),
    }
}

#[test]
fn curves_crossing_are_refused_by_name() {
    let (a, b) = (SegmentId::new(1), SegmentId::new(2));
    let mut system = System2d::default();
    system
        .insert_segment(a, segment((0., 0.), (2., 2.)))
        .unwrap();
    system
        .insert_segment(b, segment((0., 2.), (2., 0.)))
        .unwrap();
    system.solve().unwrap();
    assert!(matches!(
        refused(&system, &[a.into(), b.into()]),
        SketchBuildError::Crossing { first, second, .. }
            if first == a.into() && second == b.into()
    ));
}

#[test]
fn an_end_on_another_curve_is_refused() {
    let (a, b) = (SegmentId::new(1), SegmentId::new(2));
    let mut system = System2d::default();
    system
        .insert_segment(a, segment((0., 0.), (2., 0.)))
        .unwrap();
    system
        .insert_segment(b, segment((1., 0.), (1., 2.)))
        .unwrap();
    system.solve().unwrap();
    assert!(matches!(
        refused(&system, &[a.into(), b.into()]),
        SketchBuildError::Crossing { .. }
    ));
}

#[test]
fn ends_touching_without_a_coincidence_are_refused() {
    let (a, b) = (SegmentId::new(1), SegmentId::new(2));
    let mut system = System2d::default();
    system
        .insert_segment(a, segment((0., 0.), (1., 0.)))
        .unwrap();
    system
        .insert_segment(b, segment((1., 0.), (1., 1.)))
        .unwrap();
    system.solve().unwrap();
    assert!(matches!(
        refused(&system, &[a.into(), b.into()]),
        SketchBuildError::Crossing { .. }
    ));
}

#[test]
fn overlapping_curves_are_refused() {
    let (a, b) = (SegmentId::new(1), SegmentId::new(2));
    let mut system = System2d::default();
    system
        .insert_segment(a, segment((0., 0.), (2., 0.)))
        .unwrap();
    system
        .insert_segment(b, segment((1., 0.), (3., 0.)))
        .unwrap();
    system.solve().unwrap();
    assert!(matches!(
        refused(&system, &[a.into(), b.into()]),
        SketchBuildError::Overlapping { .. }
    ));
}
