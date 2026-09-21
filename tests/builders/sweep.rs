use nalgebra::Vector3;

use ngk::builders::edges::{add_edge, add_line};
use ngk::builders::faces::add_rectangle;
use ngk::builders::profiles::add_polyline;
use ngk::builders::sweep::{SweepError, SweepFrame, SweepOptions, SweepTransition, add_swept_face};
use ngk::geometry::{Plane, Point3, TrimmedCurve};
use ngk::model::Model;
use ngk::tessellate::{SurfaceOpts, TessellateOpts, tessellate_face};
use ngk::topology::payload::StandardPayload;
use ngk::topology::shape_keys::{FaceKey, ProfileKey, SolidKey};
use ngk::topology::validation::{validate_gmap, validate_solid_orientation};
use ngk::viz::debug_viewer::show;

fn polyline_spine(points: &[Point3]) -> (Model<StandardPayload>, ProfileKey) {
    let mut model = Model::new();
    let profile = add_polyline(&mut model, points).expect("spine profile should build");
    (model, profile)
}

fn elbow_spine() -> (Model<StandardPayload>, ProfileKey) {
    polyline_spine(&[
        Point3::origin(),
        Point3::new(0.0, 0.0, 4.0),
        Point3::new(4.0, 0.0, 4.0),
    ])
}

fn offset_section(model: &mut Model<StandardPayload>) -> FaceKey {
    add_rectangle(
        model,
        Plane::from_xy(Point3::new(1.0, 0.0, 0.0), Vector3::x(), Vector3::y()),
        1.0,
        1.0,
    )
    .expect("section should build")
}

fn outside_section(model: &mut Model<StandardPayload>) -> FaceKey {
    add_rectangle(
        model,
        Plane::from_xy(Point3::new(-2.0, 0.0, 0.0), Vector3::x(), Vector3::y()),
        1.0,
        1.0,
    )
    .expect("section should build")
}

/// Asserts that no two faces cross through one another.
///
/// The solid validators prove combinatorial closure and orientation. This
/// tessellated check adds the geometric condition needed by sweep regressions:
/// a proper triangle/triangle crossing away from a shared topological edge is
/// a self-intersection.
fn assert_not_self_intersecting(model: &Model<StandardPayload>, solid: SolidKey) {
    let options = TessellateOpts {
        surface: SurfaceOpts { nu: 12, nv: 12 },
        ..TessellateOpts::default()
    };
    let faces = model.solid_unchecked(solid).faces();
    let meshes = faces
        .iter()
        .map(|face| {
            tessellate_face(face, options)
                .unwrap_or_else(|error| panic!("face {:?} should tessellate: {error}", face.key()))
        })
        .collect::<Vec<_>>();

    for first in 0..faces.len() {
        for second in (first + 1)..faces.len() {
            for first_triangle in meshes[first].indices.chunks_exact(3) {
                let a = std::array::from_fn(|index| {
                    meshes[first].positions[first_triangle[index] as usize]
                });
                for second_triangle in meshes[second].indices.chunks_exact(3) {
                    let b = std::array::from_fn(|index| {
                        meshes[second].positions[second_triangle[index] as usize]
                    });
                    assert!(
                        !triangles_cross(a, b),
                        "faces {:?} and {:?} cross",
                        faces[first].key(),
                        faces[second].key()
                    );
                }
            }
        }
    }
}

fn triangles_cross(first: [Point3; 3], second: [Point3; 3]) -> bool {
    if !triangle_boxes_overlap(first, second) {
        return false;
    }
    triangle_edges(first).any(|(from, to)| segment_crosses_triangle(from, to, second))
        || triangle_edges(second).any(|(from, to)| segment_crosses_triangle(from, to, first))
}

fn triangle_boxes_overlap(first: [Point3; 3], second: [Point3; 3]) -> bool {
    (0..3).all(|axis| {
        let first_min = first
            .iter()
            .map(|point| point[axis])
            .fold(f64::INFINITY, f64::min);
        let first_max = first
            .iter()
            .map(|point| point[axis])
            .fold(f64::NEG_INFINITY, f64::max);
        let second_min = second
            .iter()
            .map(|point| point[axis])
            .fold(f64::INFINITY, f64::min);
        let second_max = second
            .iter()
            .map(|point| point[axis])
            .fold(f64::NEG_INFINITY, f64::max);
        first_max >= second_min && second_max >= first_min
    })
}

fn triangle_edges(triangle: [Point3; 3]) -> impl Iterator<Item = (Point3, Point3)> {
    (0..3).map(move |index| (triangle[index], triangle[(index + 1) % 3]))
}

fn segment_crosses_triangle(from: Point3, to: Point3, triangle: [Point3; 3]) -> bool {
    let direction = to - from;
    let edge_a = triangle[1] - triangle[0];
    let edge_b = triangle[2] - triangle[0];
    let determinant = edge_a.dot(&direction.cross(&edge_b));
    if determinant.abs() <= 1.0e-10 {
        return false;
    }
    let inverse = 1.0 / determinant;
    let offset = from - triangle[0];
    let u = offset.dot(&direction.cross(&edge_b)) * inverse;
    let cross = offset.cross(&edge_a);
    let v = direction.dot(&cross) * inverse;
    let along = edge_b.dot(&cross) * inverse;
    let margin = 1.0e-8;
    along > margin && along < 1.0 - margin && u > margin && v > margin && u + v < 1.0 - margin
}

#[test]
fn smooth_transition_requires_tangent_continuity() {
    let mut model = Model::<StandardPayload>::new();
    let face = offset_section(&mut model);
    let (spine_model, spine) = elbow_spine();
    let spine = spine_model.profile_unchecked(spine);

    let error = add_swept_face(&mut model, face, &spine, SweepOptions::default())
        .expect_err("a smooth transition cannot hide a sharp corner");

    assert!(matches!(
        error,
        SweepError::SpineTurnsACorner { junction: 1, .. }
    ));
}

#[test]
fn straight_transition_miters_a_c0_junction_into_a_valid_solid() {
    let mut model = Model::<StandardPayload>::new();
    let face = offset_section(&mut model);
    let options = SweepOptions {
        transition: SweepTransition::Straight,
        ..SweepOptions::default()
    };
    let (spine_model, spine) = elbow_spine();
    let spine = spine_model.profile_unchecked(spine);

    let sweep = add_swept_face(&mut model, face, &spine, options)
        .expect("a straight transition should miter the elbow");

    validate_gmap(model.topology()).expect("the sweep should keep a valid GMap");
    validate_solid_orientation(&model, sweep.solid)
        .expect("the mitered shell should face outwards");
    assert_eq!(model.solid_unchecked(sweep.solid).faces().len(), 10);

    let volume = model
        .solid_unchecked(sweep.solid)
        .volume()
        .expect("the mitered solid should measure");
    assert!((volume - 5.0).abs() <= 1.0e-6, "miter volume was {volume}");
}

#[test]
fn rounded_transition_revolves_a_corner_band_into_a_valid_solid() {
    let mut model = Model::<StandardPayload>::new();
    let face = outside_section(&mut model);
    let options = SweepOptions {
        transition: SweepTransition::Rounded,
        ..SweepOptions::default()
    };
    let (spine_model, spine) = elbow_spine();
    let _ = show(&spine_model);

    let spine = spine_model.profile_unchecked(spine);

    let sweep = add_swept_face(&mut model, face, &spine, options)
        .expect("a rounded transition should turn the section around the corner");
    let _ = show(&model);

    validate_gmap(model.topology()).expect("the sweep should keep a valid GMap");
    validate_solid_orientation(&model, sweep.solid)
        .expect("the rounded shell should face outwards");
    assert_not_self_intersecting(&model, sweep.solid);
    assert_eq!(model.solid_unchecked(sweep.solid).faces().len(), 14);
    let volume = model
        .solid_unchecked(sweep.solid)
        .volume()
        .expect("the rounded solid should measure");

    // The section lies outside the elbow, so the revolved corner adds one
    // quarter-annulus sector between the two straight tapes.
    let expected = 8.0 + 3.0 * std::f64::consts::PI / 4.0;
    assert!(
        (volume - expected).abs() <= 2.0e-2,
        "rounded volume was {volume}, expected {expected}"
    );
}

#[test]
fn rounded_transition_refuses_an_inside_section_that_would_self_intersect() {
    let mut model = Model::<StandardPayload>::new();
    let face = offset_section(&mut model);
    let options = SweepOptions {
        transition: SweepTransition::Rounded,
        ..SweepOptions::default()
    };
    let (spine_model, spine) = elbow_spine();
    let spine = spine_model.profile_unchecked(spine);

    let error = add_swept_face(&mut model, face, &spine, options)
        .expect_err("the inside corner would overlap both straight tapes");

    assert!(matches!(
        error,
        SweepError::RoundedTransitionWouldSelfIntersect { .. }
    ));
}

#[test]
fn smooth_transition_adds_no_corner_tape_at_a_c1_junction() {
    let mut model = Model::<StandardPayload>::new();
    let face = offset_section(&mut model);
    let (spine_model, spine) = polyline_spine(&[
        Point3::origin(),
        Point3::new(0.0, 0.0, 2.0),
        Point3::new(0.0, 0.0, 4.0),
    ]);
    let spine = spine_model.profile_unchecked(spine);

    let sweep = add_swept_face(&mut model, face, &spine, SweepOptions::default())
        .expect("a C1 junction needs no transition geometry");

    validate_solid_orientation(&model, sweep.solid)
        .expect("the smoothly joined shell should face outwards");
    assert_eq!(sweep.laterals.len(), 8, "only the two path tapes are built");
}

#[test]
fn rounded_transition_refuses_a_section_crossing_the_turn_axis() {
    let mut model = Model::<StandardPayload>::new();
    let face = add_rectangle(
        &mut model,
        Plane::from_xy(Point3::new(-0.5, -0.5, 0.0), Vector3::x(), Vector3::y()),
        1.0,
        1.0,
    )
    .expect("section should build");
    let options = SweepOptions {
        transition: SweepTransition::Rounded,
        ..SweepOptions::default()
    };
    let (spine_model, spine) = elbow_spine();
    let spine = spine_model.profile_unchecked(spine);

    let error = add_swept_face(&mut model, face, &spine, options)
        .expect_err("a zero-radius orbit pinches the rounded wall");

    assert!(matches!(
        error,
        SweepError::SectionMeetsTheTurningAxis { .. }
    ));
}

#[test]
fn curved_spine_segments_transport_the_section_with_both_frames() {
    let path = TrimmedCurve::arc(
        Plane::new(Point3::origin(), Vector3::x(), -Vector3::y()),
        4.0,
        std::f64::consts::FRAC_PI_2,
    );
    let mut spine_model = Model::<StandardPayload>::new();
    let spine = add_edge(
        &mut spine_model,
        path.start(),
        path.end(),
        path.curve().clone(),
    )
    .expect("curved spine edge should build");
    let spine = spine_model.edge_unchecked(spine);
    for frame in [SweepFrame::Parallel, SweepFrame::Frenet] {
        let mut model = Model::<StandardPayload>::new();
        let face = add_rectangle(
            &mut model,
            Plane::from_xy(Point3::new(4.5, 0.0, 0.0), Vector3::x(), Vector3::y()),
            0.5,
            1.0,
        )
        .expect("section should build");

        let sweep = add_swept_face(
            &mut model,
            face,
            &spine,
            SweepOptions {
                frame,
                ..SweepOptions::default()
            },
        )
        .expect("the curved segment should sweep");

        validate_solid_orientation(&model, sweep.solid)
            .expect("the curved sweep should face outwards");
        assert_eq!(sweep.laterals.len(), 4);
    }
}

#[test]
fn edge_and_profile_views_supply_sweep_spines() {
    let mut edge_model = Model::<StandardPayload>::new();
    let edge_key = add_line(
        &mut edge_model,
        Point3::origin(),
        Point3::new(0.0, 0.0, 4.0),
    )
    .expect("spine edge should build");
    let edge = edge_model.edge_unchecked(edge_key);
    let mut swept_edge_model = Model::<StandardPayload>::new();
    let edge_section = offset_section(&mut swept_edge_model);
    let edge_sweep = add_swept_face(
        &mut swept_edge_model,
        edge_section,
        &edge,
        SweepOptions::default(),
    )
    .expect("an edge view should supply a spine");
    assert_not_self_intersecting(&swept_edge_model, edge_sweep.solid);

    let mut profile_model = Model::<StandardPayload>::new();
    let profile_key = add_polyline(
        &mut profile_model,
        &[
            Point3::origin(),
            Point3::new(0.0, 0.0, 4.0),
            Point3::new(4.0, 0.0, 4.0),
        ],
    )
    .expect("spine profile should build");
    let profile = profile_model.profile_unchecked(profile_key);
    let mut swept_profile_model = Model::<StandardPayload>::new();
    let profile_section = offset_section(&mut swept_profile_model);
    let profile_sweep = add_swept_face(
        &mut swept_profile_model,
        profile_section,
        &profile,
        SweepOptions {
            transition: SweepTransition::Straight,
            ..SweepOptions::default()
        },
    )
    .expect("a profile view should supply its ordered edges as a spine");
    assert_not_self_intersecting(&swept_profile_model, profile_sweep.solid);
}
