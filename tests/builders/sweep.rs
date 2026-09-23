use nalgebra::Vector3;

use ngk::builders::edges::{add_edge, add_line};
use ngk::builders::faces::add_rectangle;
use ngk::builders::profiles::add_polyline;
use ngk::builders::sweep::{SweepError, SweepFrame, SweepOptions, SweepTransition, add_swept_face};
use ngk::geometry::{Plane, Point3, Surface, TrimmedCurve};
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
    // quarter-annulus sector between the two straight wall groups.
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
        .expect_err("the inside corner would overlap both straight wall groups");

    assert!(matches!(
        error,
        SweepError::RoundedTransitionWouldSelfIntersect { .. }
    ));
}

#[test]
fn smooth_transition_adds_no_corner_walls_at_a_c1_junction() {
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
    assert_eq!(
        sweep.laterals.len(),
        8,
        "only the two path wall groups are built"
    );
}

#[test]
fn rounded_transition_with_spine_on_left_edge_has_one_sharp_and_one_rounded_corner() {
    let mut model = Model::<StandardPayload>::new();
    let face = add_rectangle(
        &mut model,
        Plane::from_xy(Point3::new(0.0, -0.5, 0.0), Vector3::x(), Vector3::y()),
        1.0,
        1.0,
    )
    .expect("section should build");
    let options = SweepOptions {
        transition: SweepTransition::Rounded,
        ..SweepOptions::default()
    };
    let (spine_model, spine) = polyline_spine(&[
        Point3::origin(),
        Point3::new(0.0, 0.0, 4.0),
        Point3::new(-4.0, 0.0, 4.0),
    ]);
    let spine = spine_model.profile_unchecked(spine);

    let sweep = add_swept_face(&mut model, face, &spine, options).expect(
        "the section edge on the turn axis should stay sharp while the opposite edge rounds",
    );
    let _ = show(&model);

    validate_gmap(model.topology()).expect("the sweep should keep a valid GMap");
    validate_solid_orientation(&model, sweep.solid)
        .expect("the partly rounded shell should face outwards");
    assert_not_self_intersecting(&model, sweep.solid);

    let rounded_walls = sweep
        .laterals
        .iter()
        .filter(|face| {
            matches!(
                model.face_unchecked(**face).surface(),
                Surface::Revolution(_)
            )
        })
        .count();
    assert_eq!(
        rounded_walls, 3,
        "the edge on the axis makes no wall; the other three section edges revolve"
    );
    assert_eq!(sweep.laterals.len(), 11);
    assert_eq!(model.solid_unchecked(sweep.solid).faces().len(), 13);
    let volume = model
        .solid_unchecked(sweep.solid)
        .volume()
        .expect("the partly rounded solid should measure");
    let expected = 8.0 + std::f64::consts::PI / 4.0;
    assert!(
        (volume - expected).abs() <= 2.0e-2,
        "rounded volume was {volume}, expected {expected}"
    );
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

/// A helix of `turns` round the z axis, and the point it starts from.
fn helix_spine(
    radius: f64,
    pitch: f64,
    turns: f64,
) -> (
    Model<StandardPayload>,
    ngk::topology::shape_keys::EdgeKey,
    Point3,
) {
    use ngk::builders::edges::add_helix;
    use ngk::geometry::{Axis3, Helix, NativeParam};
    use radians::Rad64;

    let mut model = Model::<StandardPayload>::new();
    let axis = Axis3::z();
    let edge = add_helix(
        &mut model,
        axis,
        radius,
        pitch,
        Rad64::new(0.0),
        Rad64::new(std::f64::consts::TAU * turns),
    )
    .expect("helix spine should build");
    let start = Helix::from_axis(axis, radius, pitch).point_at(NativeParam::new(0.0));
    (model, edge, start)
}

#[test]
fn an_axial_sweep_keeps_a_section_in_a_plane_through_the_axis() {
    // A thread profile is drawn in a plane through the bolt's axis, and a
    // screw motion keeps it in one: after any number of turns the end cap is
    // still a section through the axis, not one leaning with the helix.
    let (radius, pitch, turns) = (2.0, 1.0, 2.5);
    let (spine_model, spine, start) = helix_spine(radius, pitch, turns);
    let spine = spine_model.edge_unchecked(spine);
    let outward = Vector3::new(start.x, start.y, 0.0).normalize();
    let (width, height) = (0.4, 0.5);

    let mut model = Model::<StandardPayload>::new();
    let face = add_rectangle(
        &mut model,
        Plane::from_xy(
            start - outward * (width / 2.0) - Vector3::z() * (height / 2.0),
            outward,
            Vector3::z(),
        ),
        width,
        height,
    )
    .expect("section should build");
    let sweep = add_swept_face(
        &mut model,
        face,
        &spine,
        SweepOptions {
            frame: SweepFrame::Axial(ngk::geometry::Axis3::z()),
            samples_per_segment: 64,
            ..SweepOptions::default()
        },
    )
    .expect("the section should sweep round the axis");

    validate_solid_orientation(&model, sweep.solid).expect("the sweep should face outwards");
    let end_cap = model.face_unchecked(sweep.end_cap);
    let normal = end_cap.normal_at(0.0, 0.0);
    assert!(
        normal.z.abs() <= 1e-9,
        "the end cap should contain the axis direction, its normal is {normal:?}"
    );
    let corner = end_cap.edges()[0].trimmed_curve().start();
    let radial = Vector3::new(corner.x, corner.y, 0.0);
    assert!(
        normal.dot(&radial).abs() <= 1e-9,
        "the end cap should lie in a plane through the axis, its normal is {normal:?}"
    );

    // A section turned about an axis sweeps its area times the distance its
    // centroid turns through; sliding along the axis, within the section's own
    // plane, sweeps nothing more.
    let expected = width * height * std::f64::consts::TAU * radius * turns;
    let volume = model
        .solid_unchecked(sweep.solid)
        .volume_properties()
        .expect("swept volume")
        .volume;
    assert!(
        (volume - expected).abs() <= 1e-2 * expected,
        "expected {expected}, got {volume}"
    );
}
