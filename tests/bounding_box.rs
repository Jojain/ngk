//! Exact bounding boxes of topology views, checked against closed forms.

use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, FRAC_PI_4, TAU};

use nalgebra::Vector3;
use ngk::builders::loft::LoftOptions;
use ngk::geometry::{Axis3, BBox, Frame, Plane, Point3};
use ngk::measure::MeasureError;
use ngk::modeling::solids::{block, block_at, cylinder_at, sphere_at, torus, torus_at};
use ngk::modeling::{edges, faces, loft, revolve};
use radians::Rad64;

const TOLERANCE: f64 = 1e-12;

/// Asserts the box's ranges along its own frame's axes.
fn assert_ranges(bbox: &BBox, min: Vector3<f64>, max: Vector3<f64>) {
    let frame = bbox.frame();
    let local_min = frame.coordinates_of(bbox.min());
    let local_max = frame.coordinates_of(bbox.max());
    assert!(
        (local_min - min).amax() <= TOLERANCE && (local_max - max).amax() <= TOLERANCE,
        "expected {min:?} to {max:?}, got {local_min:?} to {local_max:?}"
    );
}

/// The frame turned by `angle` about the world z axis, at the origin.
fn turned_about_z(angle: f64) -> Frame {
    Frame::from_xy(
        Point3::origin(),
        Vector3::new(angle.cos(), angle.sin(), 0.0),
        Vector3::new(-angle.sin(), angle.cos(), 0.0),
    )
}

#[test]
fn a_block_in_the_world_frame_gives_back_its_corners_exactly() {
    let shape = block(2.0, 3.0, 4.0).expect("block");

    let bbox = shape.solid().bounding_box().expect("box");

    assert_eq!(bbox.frame(), &Frame::xyz());
    assert_eq!(bbox.min(), Point3::new(0.0, 0.0, 0.0));
    assert_eq!(bbox.max(), Point3::new(2.0, 3.0, 4.0));
}

#[test]
fn a_turned_block_is_tight_in_its_own_frame_and_in_the_world() {
    let angle: f64 = 0.5;
    let placed = Frame::from_xy(
        Point3::new(1.0, 2.0, 3.0),
        Vector3::new(angle.cos(), angle.sin(), 0.0),
        Vector3::new(-angle.sin(), angle.cos(), 0.0),
    );
    let shape = block_at(placed.clone(), 2.0, 3.0, 4.0).expect("block");
    let solid = shape.solid();

    let own = solid
        .bounding_box_in(&placed)
        .expect("box in the block's frame");
    assert_ranges(&own, Vector3::zeros(), Vector3::new(2.0, 3.0, 4.0));

    let world = solid.bounding_box().expect("box in the world");
    let (c, s) = (angle.cos(), angle.sin());
    assert_ranges(
        &world,
        Vector3::new(1.0 - 3.0 * s, 2.0, 3.0),
        Vector3::new(1.0 + 2.0 * c, 2.0 + 2.0 * s + 3.0 * c, 7.0),
    );
}

/// A cylinder of axis `a` spans `h a` between its cap centres, and each cap
/// reaches `r √(1 - (a·e)²)` past its centre along a unit direction `e`.
#[test]
fn a_tilted_cylinder_matches_its_closed_form() {
    let (radius, height) = (2.0, 5.0);
    let axis = Vector3::new(1.0, 0.0, 2.0).normalize();
    let base = Point3::new(1.0, -1.0, 0.5);
    let shape = cylinder_at(
        Frame::from_xz(base, Vector3::new(2.0, 0.0, -1.0), axis),
        radius,
        height,
    )
    .expect("cylinder");

    let bbox = shape.solid().bounding_box().expect("box");

    let top = base + axis * height;
    let reach = Vector3::from_fn(|i, _| radius * (1.0 - axis[i] * axis[i]).sqrt());
    assert_ranges(
        &bbox,
        base.coords.inf(&top.coords) - reach,
        base.coords.sup(&top.coords) + reach,
    );
}

#[test]
fn a_sphere_spans_its_diameter_in_any_frame() {
    let centre = Point3::new(1.0, 2.0, -3.0);
    let shape = sphere_at(Frame::from_xy(centre, Vector3::x(), Vector3::y()), 5.0).expect("sphere");
    let frame = Frame::from_xz(
        Point3::new(-7.0, 0.0, 4.0),
        Vector3::new(1.0, 1.0, 0.0),
        Vector3::new(-1.0, 1.0, 1.0),
    );

    let bbox = shape.solid().bounding_box_in(&frame).expect("box");

    let local = frame.coordinates_of(centre);
    assert_ranges(
        &bbox,
        local - Vector3::repeat(5.0),
        local + Vector3::repeat(5.0),
    );
}

#[test]
fn a_torus_seen_along_its_axis_spans_its_outer_circle() {
    let shape = torus(10.0, 2.0).expect("torus");

    let bbox = shape.solid().bounding_box().expect("box");

    assert_ranges(
        &bbox,
        Vector3::new(-12.0, -12.0, -2.0),
        Vector3::new(12.0, 12.0, 2.0),
    );
}

/// A torus of axis `a` reaches `R √(1 - (a·e)²) + r` past its centre along `e`.
#[test]
fn a_tilted_torus_matches_its_closed_form() {
    let (major, minor) = (10.0, 2.0);
    let axis = Vector3::new(0.0, 1.0, 1.0).normalize();
    let centre = Point3::new(3.0, 0.0, -1.0);
    let shape = torus_at(Frame::from_xz(centre, Vector3::x(), axis), major, minor).expect("torus");

    let bbox = shape.solid().bounding_box().expect("box");

    let reach = Vector3::from_fn(|i, _| major * (1.0 - axis[i] * axis[i]).sqrt() + minor);
    assert_ranges(&bbox, centre.coords - reach, centre.coords + reach);
}

#[test]
fn an_arc_peaks_inside_its_span() {
    let shape = edges::arc(
        Plane::xy(),
        2.0,
        Rad64::new(FRAC_PI_4),
        Rad64::new(3.0 * FRAC_PI_4),
    )
    .expect("arc");

    let bbox = shape.edge().bounding_box().expect("box");

    let corner = 2.0 * FRAC_1_SQRT_2;
    assert_ranges(
        &bbox,
        Vector3::new(-corner, corner, 0.0),
        Vector3::new(corner, 2.0, 0.0),
    );
}

/// About the z axis a helix starts on `-x` and turns towards `-y`.
#[test]
fn a_quarter_helix_turn_climbs_a_quarter_pitch() {
    let shape =
        edges::helix(Axis3::z(), 1.0, 2.0, Rad64::ZERO, Rad64::new(FRAC_PI_2)).expect("helix");

    let bbox = shape.edge().bounding_box().expect("box");

    assert_ranges(
        &bbox,
        Vector3::new(-1.0, -1.0, 0.0),
        Vector3::new(0.0, 0.0, 0.5),
    );
}

#[test]
fn a_full_helix_turn_reaches_its_radius_on_both_sides() {
    let shape = edges::helix(Axis3::z(), 1.0, 2.0, Rad64::ZERO, Rad64::new(TAU)).expect("helix");

    let bbox = shape
        .edge()
        .bounding_box_in(&turned_about_z(0.3))
        .expect("box");

    assert_ranges(
        &bbox,
        Vector3::new(-1.0, -1.0, 0.0),
        Vector3::new(1.0, 1.0, 2.0),
    );
}

/// The outer half of a circle of radius 1 centred 5 from the z axis, swept a
/// quarter turn. Seen along the diagonal, the face peaks at radius 6 half way
/// through the sweep, where no edge runs.
#[test]
fn a_revolved_arc_peaks_where_no_edge_reaches() {
    let profile = edges::arc(
        Plane::new(Point3::new(5.0, 0.0, 0.0), Vector3::x(), -Vector3::y()),
        1.0,
        Rad64::new(-FRAC_PI_2),
        Rad64::new(FRAC_PI_2),
    )
    .expect("arc");
    let shape = revolve::revolve_edge(profile, Axis3::z(), Rad64::new(FRAC_PI_2)).expect("revolve");
    let face = shape.face();
    assert!(matches!(
        face.surface(),
        ngk::geometry::Surface::Revolution(_)
    ));

    let bbox = face
        .bounding_box_in(&turned_about_z(FRAC_PI_4))
        .expect("box");

    assert_ranges(
        &bbox,
        Vector3::new(5.0 * FRAC_1_SQRT_2, -6.0 * FRAC_1_SQRT_2, -1.0),
        Vector3::new(6.0, 6.0 * FRAC_1_SQRT_2, 1.0),
    );
}

#[test]
fn a_box_contains_every_vertex_of_its_solid() {
    let shape = block(2.0, 3.0, 4.0).expect("block");
    let solid = shape.solid();
    let frame = turned_about_z(1.0);

    let bbox = solid.bounding_box_in(&frame).expect("box");

    for vertex in solid.vertices() {
        assert!(bbox.contains_point(*vertex.point(), TOLERANCE));
    }
}

#[test]
fn a_nurbs_loft_is_refused_rather_than_bounded_loosely() {
    let square = faces::rectangle(
        Plane::new(Point3::new(-1.0, -1.0, 0.0), Vector3::x(), Vector3::z()),
        2.0,
        2.0,
    )
    .expect("square");
    let circle = faces::circle(
        Plane::new(Point3::new(0.0, 0.0, 3.0), Vector3::x(), Vector3::z()),
        1.0,
    )
    .expect("circle");
    let shape = loft::loft(&[&square, &circle], LoftOptions::default()).expect("loft");

    let error = shape
        .solid()
        .bounding_box()
        .expect_err("a NURBS loft has no bounding box yet");

    assert!(
        matches!(
            error,
            MeasureError::EdgeBoundingBoxNotImplemented { .. }
                | MeasureError::FaceBoundingBoxNotImplemented { .. }
        ),
        "{error}"
    );
}

#[test]
fn a_revolved_skew_line_is_refused() {
    let line = edges::line(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0)).expect("line");
    let shape = revolve::revolve_edge(line, Axis3::z(), Rad64::new(FRAC_PI_2)).expect("revolve");

    let error = shape
        .face()
        .bounding_box()
        .expect_err("a hyperboloid has no bounding box yet");

    assert!(
        matches!(error, MeasureError::FaceBoundingBoxNotImplemented { .. }),
        "{error}"
    );
}
