use nalgebra::{Vector2, Vector3};
use ngk::builders::errors::SheetFromFacesError;
use ngk::builders::faces::{SketchError, SketchFromSystem, add_sketch_from_system};
use ngk::builders::sheets::add_sheet_from_faces;
use ngk::builders::solids::add_extruded_face;
use ngk::geometry::dim2::{
    curves::Circle2,
    trimmed::{Arc2, Segment2},
    utils::Point2,
};
use ngk::geometry::{Plane, Point3};
use ngk::healing::{HealingOptions, remove_redundant_cells};
use ngk::model::Model;
use ngk::solver::{ArcId, CircleId, Constraint2d as C, CurveId, SegmentId, System2d};
use ngk::topology::payload::StandardPayload;
use ngk::topology::shape_keys::FaceKey;
use std::f64::consts::{FRAC_PI_2, PI};

type M = Model<StandardPayload>;

fn segment(a: (f64, f64), b: (f64, f64)) -> Segment2 {
    Segment2::new(Point2::new(a.0, a.1), Point2::new(b.0, b.1))
}

/// Draws a closed polygon whose corners are joined by coincidences.
fn add_polygon(system: &mut System2d, corners: &[(f64, f64)]) -> Vec<CurveId> {
    let ids: Vec<SegmentId> = (0..corners.len())
        .map(|i| {
            system
                .add_segment(segment(corners[i], corners[(i + 1) % corners.len()]))
                .unwrap()
        })
        .collect();
    for i in 0..ids.len() {
        system
            .add_constraint(C::coincident(
                ids[i].end(),
                ids[(i + 1) % ids.len()].start(),
            ))
            .unwrap();
    }
    ids.into_iter().map(CurveId::from).collect()
}

fn add_rectangle(system: &mut System2d, low: (f64, f64), high: (f64, f64)) -> Vec<CurveId> {
    add_polygon(system, &[low, (high.0, low.1), high, (low.0, high.1)])
}

fn add_line(system: &mut System2d, a: (f64, f64), b: (f64, f64)) -> CurveId {
    system.add_segment(segment(a, b)).unwrap().into()
}

fn add_circle(system: &mut System2d, center: (f64, f64), radius: f64) -> CurveId {
    let circle: CircleId = system
        .add_circle(Circle2::new(
            Point2::new(center.0, center.1),
            Vector2::x(),
            radius,
        ))
        .unwrap();
    circle.into()
}

fn build(system: &mut System2d, curves: &[CurveId]) -> (M, SketchFromSystem) {
    system.solve().unwrap();
    let mut model = M::new();
    let sketch = add_sketch_from_system(&mut model, system, Plane::xy(), curves).unwrap();
    (model, sketch)
}

fn refused(system: &mut System2d, curves: &[CurveId]) -> SketchError {
    system.solve().unwrap();
    let mut model = M::new();
    let error = add_sketch_from_system(&mut model, system, Plane::xy(), curves).unwrap_err();
    assert_eq!(model.iter_edges().count(), 0, "a refusal adds nothing");
    assert_eq!(model.iter_faces().count(), 0, "a refusal adds nothing");
    error
}

fn faces(sketch: &SketchFromSystem) -> Vec<FaceKey> {
    sketch.regions.iter().map(|region| region.face).collect()
}

fn area(model: &M, face: FaceKey) -> f64 {
    model.face_unchecked(face).area().unwrap()
}

/// The regions' areas, smallest first.
fn areas(model: &M, sketch: &SketchFromSystem) -> Vec<f64> {
    let mut areas: Vec<f64> = faces(sketch)
        .iter()
        .map(|face| area(model, *face))
        .collect();
    areas.sort_by(f64::total_cmp);
    areas
}

fn assert_areas(model: &M, sketch: &SketchFromSystem, expected: &[f64]) {
    let actual = areas(model, sketch);
    assert_eq!(actual.len(), expected.len(), "regions: {actual:?}");
    for (actual, expected) in actual.iter().zip(expected) {
        assert!(
            (actual - expected).abs() < 1e-6,
            "areas {:?}, expected {expected:?}",
            areas(model, sketch)
        );
    }
}

fn counts(model: &M) -> (usize, usize, usize) {
    (
        model.iter_vertices().count(),
        model.iter_edges().count(),
        model.iter_faces().count(),
    )
}

fn swiss_cross() -> (M, SketchFromSystem) {
    let mut system = System2d::default();
    let mut curves = add_rectangle(&mut system, (4., 0.), (6., 10.));
    curves.extend(add_rectangle(&mut system, (0., 4.), (10., 6.)));
    build(&mut system, &curves)
}

#[test]
fn a_rectangle_is_one_region() {
    let mut system = System2d::default();
    let curves = add_rectangle(&mut system, (0., 0.), (10., 5.));
    let (model, sketch) = build(&mut system, &curves);
    assert_areas(&model, &sketch, &[50.]);
    assert_eq!(counts(&model), (4, 4, 1));
}

#[test]
fn a_circle_inside_a_rectangle_is_a_ring_and_a_disc() {
    let mut system = System2d::default();
    let mut curves = add_rectangle(&mut system, (0., 0.), (10., 10.));
    curves.push(add_circle(&mut system, (5., 5.), 2.));
    let (model, sketch) = build(&mut system, &curves);
    assert_areas(&model, &sketch, &[4. * PI, 100. - 4. * PI]);
    let holes: Vec<_> = sketch
        .regions
        .iter()
        .map(|region| model.face_unchecked(region.face).inner_loops().len())
        .collect();
    assert_eq!(holes.iter().sum::<usize>(), 1);
}

#[test]
fn the_ring_and_the_disc_extrude_on_their_own() {
    let mut system = System2d::default();
    let mut curves = add_rectangle(&mut system, (0., 0.), (10., 10.));
    curves.push(add_circle(&mut system, (5., 5.), 2.));
    let (mut model, sketch) = build(&mut system, &curves);
    let mut volumes = vec![];
    for face in faces(&sketch) {
        let solid = add_extruded_face(&mut model, face, Vector3::new(0., 0., 2.)).unwrap();
        volumes.push(model.solid_unchecked(solid.solid).volume().unwrap().abs());
    }
    volumes.sort_by(f64::total_cmp);
    assert!((volumes[0] - 8. * PI).abs() < 1e-6, "{volumes:?}");
    assert!((volumes[1] - (200. - 8. * PI)).abs() < 1e-6, "{volumes:?}");
}

#[test]
fn the_ring_and_the_disc_sew_and_heal_into_one_face() {
    let mut system = System2d::default();
    let mut curves = add_rectangle(&mut system, (0., 0.), (10., 10.));
    curves.push(add_circle(&mut system, (5., 5.), 2.));
    let (mut model, sketch) = build(&mut system, &curves);
    let sheet = add_sheet_from_faces(&mut model, &faces(&sketch)).unwrap();
    assert!((model.sheet_unchecked(sheet).area().unwrap() - 100.).abs() < 1e-6);
    remove_redundant_cells(&mut model, HealingOptions::default()).unwrap();
    assert_eq!(counts(&model), (4, 4, 1));
}

#[test]
fn nested_circles_give_nested_regions() {
    let mut system = System2d::default();
    let mut curves = add_rectangle(&mut system, (0., 0.), (20., 20.));
    curves.push(add_circle(&mut system, (10., 10.), 5.));
    curves.push(add_circle(&mut system, (10., 10.), 2.));
    let (model, sketch) = build(&mut system, &curves);
    assert_areas(&model, &sketch, &[4. * PI, 21. * PI, 400. - 25. * PI]);
}

#[test]
fn disjoint_rectangles_are_loose_regions_that_do_not_form_one_sheet() {
    let mut system = System2d::default();
    let mut curves = add_rectangle(&mut system, (0., 0.), (1., 1.));
    curves.extend(add_rectangle(&mut system, (5., 5.), (7., 7.)));
    let (mut model, sketch) = build(&mut system, &curves);
    assert_areas(&model, &sketch, &[1., 4.]);
    assert!(matches!(
        add_sheet_from_faces(&mut model, &faces(&sketch)),
        Err(SheetFromFacesError::Disconnected { components: 2 })
    ));
}

#[test]
fn an_open_curve_alone_draws_nothing() {
    let mut system = System2d::default();
    let line = add_line(&mut system, (0., 0.), (1., 1.));
    let (model, sketch) = build(&mut system, &[line]);
    assert!(sketch.regions.is_empty());
    assert_eq!(counts(&model), (0, 0, 0));
}

#[test]
fn a_swiss_cross_is_five_loose_regions() {
    let (model, sketch) = swiss_cross();
    assert_areas(&model, &sketch, &[4., 8., 8., 8., 8.]);
    assert_eq!(counts(&model), (20, 20, 5));
    assert_eq!(model.iter_sheets().count(), 0);
}

#[test]
fn each_region_of_the_cross_extrudes_on_its_own() {
    let (mut model, sketch) = swiss_cross();
    let mut total = 0.;
    for face in faces(&sketch) {
        let solid = add_extruded_face(&mut model, face, Vector3::new(0., 0., 1.)).unwrap();
        total += model.solid_unchecked(solid.solid).volume().unwrap().abs();
    }
    assert!((total - 36.).abs() < 1e-6);
}

#[test]
fn the_cross_sews_into_one_sheet_and_heals_into_one_face() {
    let (mut model, sketch) = swiss_cross();
    let sheet = add_sheet_from_faces(&mut model, &faces(&sketch)).unwrap();
    assert_eq!(counts(&model), (12, 16, 5));
    assert!((model.sheet_unchecked(sheet).area().unwrap() - 36.).abs() < 1e-6);
    remove_redundant_cells(&mut model, HealingOptions::default()).unwrap();
    assert_eq!(counts(&model), (12, 12, 1));
}

#[test]
fn a_chosen_subset_of_the_cross_sews_when_connected() {
    let (mut model, sketch) = swiss_cross();
    let centre = sketch
        .regions
        .iter()
        .find(|region| (area(&model, region.face) - 4.).abs() < 1e-6)
        .unwrap()
        .face;
    let arm = faces(&sketch)
        .into_iter()
        .find(|face| *face != centre)
        .unwrap();
    add_sheet_from_faces(&mut model, &[centre, arm]).unwrap();
}

#[test]
fn a_region_says_which_curves_bound_it() {
    let mut system = System2d::default();
    let rectangle = add_rectangle(&mut system, (0., 0.), (10., 10.));
    let cut = add_line(&mut system, (5., -3.), (5., 13.));
    let mut curves = rectangle.clone();
    curves.push(cut);
    let (model, sketch) = build(&mut system, &curves);
    assert_eq!(sketch.regions.len(), 2);
    for region in &sketch.regions {
        assert_eq!(region.boundary.len(), 4);
        assert_eq!(
            region
                .boundary
                .iter()
                .filter(|(curve, _)| *curve == cut)
                .count(),
            1,
            "each half is bounded by the cut once"
        );
        let edges = model.face_unchecked(region.face).edges();
        for (_, edge) in &region.boundary {
            assert!(edges.iter().any(|candidate| candidate.key() == *edge));
        }
    }
}

#[test]
fn a_line_overshooting_a_rectangle_leaves_just_two_regions() {
    let mut system = System2d::default();
    let mut curves = add_rectangle(&mut system, (0., 0.), (10., 10.));
    curves.push(add_line(&mut system, (3., -4.), (3., 14.)));
    let (model, sketch) = build(&mut system, &curves);
    assert_areas(&model, &sketch, &[30., 70.]);
    assert_eq!(counts(&model), (8, 8, 2));
}

#[test]
fn a_line_stopping_short_inside_leaves_the_rectangle_whole() {
    let mut system = System2d::default();
    let mut curves = add_rectangle(&mut system, (0., 0.), (10., 10.));
    curves.push(add_line(&mut system, (3., -4.), (3., 5.)));
    let (model, sketch) = build(&mut system, &curves);
    assert_areas(&model, &sketch, &[100.]);
    assert_eq!(
        counts(&model),
        (4, 4, 1),
        "the dropped line leaves no vertex"
    );
}

#[test]
fn a_line_ending_on_a_side_from_outside_leaves_the_rectangle_whole() {
    let mut system = System2d::default();
    let mut curves = add_rectangle(&mut system, (0., 0.), (10., 10.));
    curves.push(add_line(&mut system, (3., -4.), (3., 0.)));
    let (model, sketch) = build(&mut system, &curves);
    assert_areas(&model, &sketch, &[100.]);
    assert_eq!(counts(&model), (4, 4, 1));
}

#[test]
fn a_segment_joining_a_rectangle_to_a_circle_bounds_nothing() {
    let mut system = System2d::default();
    let mut curves = add_rectangle(&mut system, (0., 0.), (10., 10.));
    curves.push(add_circle(&mut system, (5., 5.), 2.));
    curves.push(add_line(&mut system, (0., 5.), (3., 5.)));
    let (model, sketch) = build(&mut system, &curves);
    assert_areas(&model, &sketch, &[4. * PI, 100. - 4. * PI]);
    // Four sides and each region's own copy of the circle, which has no corner left.
    assert_eq!(counts(&model), (4, 6, 2));
}

#[test]
fn lines_dropped_around_a_circle_leave_it_whole() {
    let mut system = System2d::default();
    let circle = add_circle(&mut system, (0., 0.), 2.);
    let across = add_line(&mut system, (-5., 3.), (5., 3.));
    let through = add_line(&mut system, (1., 1.), (9., 1.));
    let (model, sketch) = build(&mut system, &[circle, across, through]);
    assert_areas(&model, &sketch, &[4. * PI]);
    assert_eq!(counts(&model), (0, 1, 1));
}

#[test]
fn segments_touching_without_coincidences_still_close_a_region() {
    let mut system = System2d::default();
    let curves = [
        add_line(&mut system, (0., 0.), (4., 0.)),
        add_line(&mut system, (4., 0.), (0., 3.)),
        add_line(&mut system, (0., 3.), (0., 0.)),
    ];
    let (model, sketch) = build(&mut system, &curves);
    assert_areas(&model, &sketch, &[6.]);
    assert_eq!(counts(&model), (3, 3, 1));
}

#[test]
fn a_t_junction_splits_a_rectangle() {
    let mut system = System2d::default();
    let mut curves = add_rectangle(&mut system, (0., 0.), (10., 10.));
    curves.push(add_line(&mut system, (4., 0.), (4., 10.)));
    let (model, sketch) = build(&mut system, &curves);
    assert_areas(&model, &sketch, &[40., 60.]);
}

#[test]
fn both_diagonals_meet_at_one_node() {
    let mut system = System2d::default();
    let mut curves = add_rectangle(&mut system, (0., 0.), (10., 10.));
    curves.push(add_line(&mut system, (0., 0.), (10., 10.)));
    curves.push(add_line(&mut system, (10., 0.), (0., 10.)));
    let (model, sketch) = build(&mut system, &curves);
    assert_areas(&model, &sketch, &[25., 25., 25., 25.]);
}

#[test]
fn three_lines_through_one_point_are_one_node() {
    let mut system = System2d::default();
    let mut curves = add_rectangle(&mut system, (0., 0.), (10., 10.));
    curves.push(add_line(&mut system, (5., -1.), (5., 11.)));
    curves.push(add_line(&mut system, (-1., 5.), (11., 5.)));
    curves.push(add_line(&mut system, (0., 0.), (10., 10.)));
    let (model, sketch) = build(&mut system, &curves);
    assert_areas(&model, &sketch, &[12.5, 12.5, 12.5, 12.5, 25., 25.]);
}

#[test]
fn a_line_across_a_circle_gives_two_halves_that_sew() {
    let mut system = System2d::default();
    let circle = add_circle(&mut system, (0., 0.), 2.);
    let line = add_line(&mut system, (-5., 0.), (5., 0.));
    let (mut model, sketch) = build(&mut system, &[circle, line]);
    assert_areas(&model, &sketch, &[2. * PI, 2. * PI]);
    let sheet = add_sheet_from_faces(&mut model, &faces(&sketch)).unwrap();
    assert!((model.sheet_unchecked(sheet).area().unwrap() - 4. * PI).abs() < 1e-6);
}

#[test]
fn two_crossing_circles_give_three_regions() {
    let mut system = System2d::default();
    let first = add_circle(&mut system, (0., 0.), 2.);
    let second = add_circle(&mut system, (2., 0.), 2.);
    let (model, sketch) = build(&mut system, &[first, second]);
    let lens = 2. * 4. * (PI / 3.) - 2. * 3_f64.sqrt();
    let crescent = 4. * PI - lens;
    assert_areas(&model, &sketch, &[lens, crescent, crescent]);
}

#[test]
fn a_circle_touching_a_rectangle_from_outside_keeps_one_corner() {
    let mut system = System2d::default();
    let mut curves = add_rectangle(&mut system, (0., 0.), (10., 10.));
    curves.push(add_circle(&mut system, (5., 12.), 2.));
    let (mut model, sketch) = build(&mut system, &curves);
    assert_areas(&model, &sketch, &[4. * PI, 100.]);
    let disc = faces(&sketch)
        .into_iter()
        .find(|face| (area(&model, *face) - 4. * PI).abs() < 1e-6)
        .unwrap();
    assert_eq!(model.face_unchecked(disc).vertices().len(), 1);
    let solid = add_extruded_face(&mut model, disc, Vector3::new(0., 0., 1.)).unwrap();
    let volume = model.solid_unchecked(solid.solid).volume().unwrap().abs();
    assert!((volume - 4. * PI).abs() < 1e-6);
}

#[test]
fn an_arc_leaving_a_segment_tangentially_bounds_the_area_between_them() {
    let (arc, line, closing) = (ArcId::new(1), SegmentId::new(1), SegmentId::new(2));
    let mut system = System2d::default();
    system
        .insert_arc(
            arc,
            Arc2::new(Point2::new(0., 5.), -Vector2::y(), 5., FRAC_PI_2),
        )
        .unwrap();
    system
        .insert_segment(line, segment((0., 0.), (10., 0.)))
        .unwrap();
    system
        .insert_segment(closing, segment((10., 0.), (5., 5.)))
        .unwrap();
    for constraint in [
        C::coincident(line.start(), arc.start()),
        C::coincident(line.end(), closing.start()),
        C::coincident(closing.end(), arc.end()),
    ] {
        system.add_constraint(constraint).unwrap();
    }
    let curves: [CurveId; 3] = [arc.into(), line.into(), closing.into()];
    let (model, sketch) = build(&mut system, &curves);
    assert_areas(&model, &sketch, &[25. - 12.5 * (FRAC_PI_2 - 1.)]);
}

#[test]
fn every_region_faces_along_the_sketch_plane() {
    for plane in [
        Plane::xy(),
        Plane::new(Point3::origin(), Vector3::x(), -Vector3::z()),
        Plane::new(
            Point3::new(1., 2., 3.),
            Vector3::new(0., 1., 0.),
            Vector3::new(1., 0., 0.),
        ),
    ] {
        let mut system = System2d::default();
        let mut curves = add_rectangle(&mut system, (0., 0.), (10., 10.));
        curves.push(add_circle(&mut system, (5., 5.), 2.));
        system.solve().unwrap();
        let mut model = M::new();
        let sketch = add_sketch_from_system(&mut model, &system, plane.clone(), &curves).unwrap();
        assert_eq!(sketch.regions.len(), 2);
        for face in faces(&sketch) {
            let view = model.face_unchecked(face);
            let normal = view.normal_at(1., 1.);
            assert!(
                (normal.into_inner() - plane.normal().into_inner()).norm() < 1e-9,
                "normal {normal:?}, plane {:?}",
                plane.normal()
            );
        }
    }
}

#[test]
fn a_hole_touching_the_outer_boundary_at_one_point_is_refused_by_name() {
    let mut system = System2d::default();
    let mut curves = add_rectangle(&mut system, (0., 0.), (10., 10.));
    curves.extend(add_polygon(
        &mut system,
        &[(5., 0.), (7., 2.), (5., 4.), (3., 2.)],
    ));
    assert!(matches!(
        refused(&mut system, &curves),
        SketchError::PinchedRegion { point } if (point - Point2::new(5., 0.)).norm() < 1e-6
    ));
}

#[test]
fn overlapping_curves_are_refused_by_name() {
    let mut system = System2d::default();
    let a = add_line(&mut system, (0., 0.), (2., 0.));
    let b = add_line(&mut system, (1., 0.), (3., 0.));
    assert!(matches!(
        refused(&mut system, &[a, b]),
        SketchError::Overlapping { first, second } if first == a && second == b
    ));
}

#[test]
fn an_unsolved_or_badly_listed_system_is_refused() {
    let mut system = System2d::default();
    let line = add_line(&mut system, (0., 0.), (1., 0.));
    let mut model = M::new();
    assert!(matches!(
        add_sketch_from_system(&mut model, &system, Plane::xy(), &[line]),
        Err(SketchError::NotSolved)
    ));
    assert!(matches!(
        refused(&mut system, &[line, line]),
        SketchError::DuplicateCurve(curve) if curve == line
    ));
    let unknown: CurveId = SegmentId::new(99).into();
    assert!(matches!(
        refused(&mut system, &[unknown]),
        SketchError::UnknownCurve(curve) if curve == unknown
    ));
}

#[test]
fn the_same_system_gives_the_same_regions_in_the_same_order() {
    let (first_model, first) = swiss_cross();
    let (second_model, second) = swiss_cross();
    let read = |model: &M, sketch: &SketchFromSystem| -> Vec<f64> {
        faces(sketch)
            .iter()
            .map(|face| area(model, *face))
            .collect()
    };
    assert_eq!(read(&first_model, &first), read(&second_model, &second));
}
