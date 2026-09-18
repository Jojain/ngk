use ngk::builders::faces::{add_circle, add_polygon};
use ngk::builders::profiles::add_polyline;
use ngk::geometry::{Fraction, LINEAR_TOLERANCE, Plane, Point3};
use ngk::model::Model;
use ngk::topology::StandardPayload;
use ngk::topology::face::Face;
use ngk::topology::profile_curve::{
    ProfileCurve, ProfileCurveError, agree_directions, align_seams,
};
use ngk::topology::shape_keys::ProfileKey;
use std::f64::consts::TAU;

/// An L-shaped open polyline of legs 3 and 1, total length 4.
fn bent_polyline(model: &mut Model<StandardPayload>) -> ProfileKey {
    add_polyline(
        model,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(3.0, 1.0, 0.0),
        ],
    )
    .unwrap()
}

/// The unit square in the xy plane, walked counter-clockwise.
fn unit_square(model: &mut Model<StandardPayload>) -> ProfileKey {
    add_polygon(
        model,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ],
    )
}

fn assert_near(actual: Point3, expected: Point3, tolerance: f64) {
    let error = (actual - expected).norm();
    assert!(
        error <= tolerance,
        "expected {expected:?}, got {actual:?}, error {error}"
    );
}

#[test]
fn an_open_polyline_allocates_its_span_by_arc_length() {
    let mut model = Model::<StandardPayload>::new();
    let key = bent_polyline(&mut model);
    let profile = model.profile_unchecked(key);
    let section = ProfileCurve::from_profile(&profile).unwrap();

    assert!(!section.is_closed());
    assert!((section.length() - 4.0).abs() <= LINEAR_TOLERANCE);
    // The long leg is three quarters of the length, so it claims three
    // quarters of the parameter — not the half an equal slice per edge would
    // hand it.
    let breaks = section
        .breakpoints()
        .iter()
        .map(|fraction| fraction.value())
        .collect::<Vec<_>>();
    assert_eq!(breaks.len(), 3);
    assert!((breaks[1] - 0.75).abs() <= 1e-12, "breaks = {breaks:?}");

    assert_near(
        section.point_at(Fraction::new(0.5)),
        Point3::new(2.0, 0.0, 0.0),
        1e-12,
    );
    assert_near(
        section.point_at(Fraction::new(0.875)),
        Point3::new(3.0, 0.5, 0.0),
        1e-12,
    );
}

#[test]
fn locate_converts_a_traversal_fraction_to_a_span_fraction() {
    let mut model = Model::<StandardPayload>::new();
    let key = bent_polyline(&mut model);
    let profile = model.profile_unchecked(key);
    let section = ProfileCurve::from_profile(&profile).unwrap();

    let (span, local) = section.locate(Fraction::new(0.375));
    assert_eq!(span, 0);
    assert!((local.value() - 0.5).abs() <= 1e-12);

    // A fraction on a boundary belongs to the span it starts.
    let (span, local) = section.locate(Fraction::new(0.75));
    assert_eq!(span, 1);
    assert!(local.value().abs() <= 1e-12);
}

#[test]
fn a_rectangle_walks_all_four_corners_in_order() {
    let mut model = Model::<StandardPayload>::new();
    let key = unit_square(&mut model);
    let profile = model.profile_unchecked(key);
    let section = ProfileCurve::from_profile(&profile).unwrap();

    assert!(section.is_closed());
    assert!((section.length() - 4.0).abs() <= LINEAR_TOLERANCE);
    let breaks = section
        .breakpoints()
        .iter()
        .map(|fraction| fraction.value())
        .collect::<Vec<_>>();
    // Four corners, and `1` is not listed because it is `0` again.
    assert_eq!(breaks.len(), 4);
    for (index, break_at) in breaks.iter().enumerate() {
        assert!((break_at - index as f64 * 0.25).abs() <= 1e-12);
    }
}

#[test]
fn an_unmarked_circle_is_one_span_with_no_corner_on_it() {
    let mut model = Model::<StandardPayload>::new();
    let face = add_circle(&mut model, Plane::xy(), 2.0).unwrap();
    let disk = Face::new(&model, face);
    let boundary = disk.outer_loop().unwrap();
    let section = ProfileCurve::from_loop(&boundary).unwrap();

    assert!(section.is_closed());
    assert!((section.length() - TAU * 2.0).abs() <= 1e-6);
    assert_eq!(section.spans().len(), 1);
    // Nothing meets where a whole circle closes, so it has no breakpoint —
    // and the traversal says so rather than inventing one at `0`.
    assert!(section.breakpoints().is_empty());
    assert_near(
        section.point_at(Fraction::new(0.25)),
        Point3::new(0.0, 2.0, 0.0),
        1e-9,
    );
}

#[test]
fn subdividing_at_the_breakpoints_gives_one_piece_per_edge() {
    let mut model = Model::<StandardPayload>::new();
    let key = unit_square(&mut model);
    let profile = model.profile_unchecked(key);
    let section = ProfileCurve::from_profile(&profile).unwrap();

    let mut boundaries = section.breakpoints();
    boundaries.push(Fraction::END);
    let pieces = section.subdivided(&boundaries).unwrap();

    assert_eq!(pieces.len(), 4);
    for (index, piece) in pieces.iter().enumerate() {
        assert!((piece.length() - 1.0).abs() <= 1e-12);
        assert_near(
            piece.start(),
            section.point_at(boundaries[index]),
            LINEAR_TOLERANCE,
        );
    }
}

#[test]
fn subdividing_finer_than_the_edges_cuts_inside_them() {
    let mut model = Model::<StandardPayload>::new();
    let key = unit_square(&mut model);
    let profile = model.profile_unchecked(key);
    let section = ProfileCurve::from_profile(&profile).unwrap();

    let boundaries = (0..=8)
        .map(|step| Fraction::new(step as f64 / 8.0))
        .collect::<Vec<_>>();
    let pieces = section.subdivided(&boundaries).unwrap();

    assert_eq!(pieces.len(), 8);
    for piece in &pieces {
        assert!((piece.length() - 0.5).abs() <= 1e-12);
    }
}

#[test]
fn subdividing_across_a_corner_is_refused() {
    let mut model = Model::<StandardPayload>::new();
    let key = unit_square(&mut model);
    let profile = model.profile_unchecked(key);
    let section = ProfileCurve::from_profile(&profile).unwrap();

    assert!(matches!(
        section.subdivided(&[Fraction::START, Fraction::new(0.4), Fraction::END]),
        Err(ProfileCurveError::PieceCrossesCorner { .. })
    ));
    assert!(matches!(
        section.subdivided(&[Fraction::START]),
        Err(ProfileCurveError::TooFewBoundaries { got: 1 })
    ));
}

#[test]
fn reversing_a_traversal_runs_the_same_geometry_backwards() {
    let mut model = Model::<StandardPayload>::new();
    let key = bent_polyline(&mut model);
    let profile = model.profile_unchecked(key);
    let section = ProfileCurve::from_profile(&profile).unwrap();
    let reversed = section.reversed();

    for step in 0..=16 {
        let t = step as f64 / 16.0;
        assert_near(
            reversed.point_at(Fraction::new(t)),
            section.point_at(Fraction::new(1.0 - t)),
            1e-12,
        );
    }
}

#[test]
fn rotating_moves_the_start_and_re_derives_the_breakpoints() {
    let mut model = Model::<StandardPayload>::new();
    let key = unit_square(&mut model);
    let profile = model.profile_unchecked(key);
    let section = ProfileCurve::from_profile(&profile).unwrap();

    let rotated = section.rotated_to(Fraction::new(0.25)).unwrap();
    assert_near(
        rotated.point_at(Fraction::START),
        section.point_at(Fraction::new(0.25)),
        1e-12,
    );
    assert_eq!(rotated.breakpoints().len(), 4);

    // Rotating onto the middle of an edge leaves the new start on no corner,
    // and the vertex that was at `0` is still one.
    let off_corner = section.rotated_to(Fraction::new(0.125)).unwrap();
    let breaks = off_corner
        .breakpoints()
        .iter()
        .map(|fraction| fraction.value())
        .collect::<Vec<_>>();
    assert_eq!(breaks.len(), 4);
    assert!(breaks[0] > 1e-9, "breaks = {breaks:?}");
    assert!((breaks[0] - 0.125).abs() <= 1e-12, "breaks = {breaks:?}");
    for step in 0..=16 {
        let t = step as f64 / 16.0;
        assert_near(
            off_corner.point_at(Fraction::new(t)),
            section.point_at(Fraction::new(t + 0.125)),
            1e-12,
        );
    }
}

#[test]
fn an_open_traversal_has_no_seam_to_rotate() {
    let mut model = Model::<StandardPayload>::new();
    let key = bent_polyline(&mut model);
    let profile = model.profile_unchecked(key);
    let section = ProfileCurve::from_profile(&profile).unwrap();

    assert_eq!(
        section.rotated_to(Fraction::new(0.5)).err(),
        Some(ProfileCurveError::OpenTraversalRotation)
    );
}

#[test]
fn direction_agreement_turns_a_reversed_section_back_round() {
    let mut model = Model::<StandardPayload>::new();
    let lower = unit_square(&mut model);
    let upper = add_polygon(
        &mut model,
        &[
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(1.0, 0.0, 1.0),
            Point3::new(1.0, 1.0, 1.0),
            Point3::new(0.0, 1.0, 1.0),
        ],
    );

    let lower_profile = model.profile_unchecked(lower);
    let upper_profile = model.profile_unchecked(upper);
    let mut sections = vec![
        ProfileCurve::from_profile(&lower_profile).unwrap(),
        // Handed to the loft backwards, which is what would bowtie it.
        ProfileCurve::from_profile(&upper_profile)
            .unwrap()
            .reversed(),
    ];
    agree_directions(&mut sections);

    // Corresponding points now sit directly above one another rather than
    // crossing over.
    for step in 0..16 {
        let t = Fraction::new(step as f64 / 16.0);
        let below = sections[0].point_at(t);
        let above = sections[1].point_at(t);
        assert!(
            (above.x - below.x).abs() <= 1e-9 && (above.y - below.y).abs() <= 1e-9,
            "at {t}: {below:?} does not line up with {above:?}"
        );
    }
}

#[test]
fn seam_alignment_undoes_a_rotated_start() {
    let mut model = Model::<StandardPayload>::new();
    let lower = unit_square(&mut model);
    let upper = add_polygon(
        &mut model,
        &[
            // The same square, authored from a different corner.
            Point3::new(1.0, 1.0, 1.0),
            Point3::new(0.0, 1.0, 1.0),
            Point3::new(0.0, 0.0, 1.0),
            Point3::new(1.0, 0.0, 1.0),
        ],
    );

    let lower_profile = model.profile_unchecked(lower);
    let upper_profile = model.profile_unchecked(upper);
    let mut sections = vec![
        ProfileCurve::from_profile(&lower_profile).unwrap(),
        ProfileCurve::from_profile(&upper_profile).unwrap(),
    ];
    agree_directions(&mut sections);
    align_seams(&mut sections).unwrap();

    for step in 0..16 {
        let t = Fraction::new(step as f64 / 16.0);
        let below = sections[0].point_at(t);
        let above = sections[1].point_at(t);
        assert!(
            (above.x - below.x).abs() <= 1e-9 && (above.y - below.y).abs() <= 1e-9,
            "at {t}: {below:?} does not line up with {above:?}"
        );
    }
}

#[test]
fn a_long_chain_does_not_accumulate_seam_drift() {
    let mut model = Model::<StandardPayload>::new();
    let keys = (0..6)
        .map(|level| {
            // Each level is authored from a different corner, so a pairwise
            // alignment against the previous one could spiral.
            let corners = [
                Point3::new(0.0, 0.0, level as f64),
                Point3::new(1.0, 0.0, level as f64),
                Point3::new(1.0, 1.0, level as f64),
                Point3::new(0.0, 1.0, level as f64),
            ];
            let rotated = (0..4)
                .map(|index| corners[(index + level) % 4])
                .collect::<Vec<_>>();
            add_polygon(&mut model, &rotated)
        })
        .collect::<Vec<_>>();

    let profiles = keys
        .iter()
        .map(|key| model.profile_unchecked(*key))
        .collect::<Vec<_>>();
    let mut sections = profiles
        .iter()
        .map(|profile| ProfileCurve::from_profile(profile).unwrap())
        .collect::<Vec<_>>();
    agree_directions(&mut sections);
    align_seams(&mut sections).unwrap();

    for step in 0..16 {
        let t = Fraction::new(step as f64 / 16.0);
        let reference = sections[0].point_at(t);
        for (level, section) in sections.iter().enumerate().skip(1) {
            let point = section.point_at(t);
            assert!(
                (point.x - reference.x).abs() <= 1e-9 && (point.y - reference.y).abs() <= 1e-9,
                "level {level} at {t}: {point:?} drifted from {reference:?}"
            );
        }
    }
}
