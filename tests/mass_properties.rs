use nalgebra::Vector3;
use ngk::builders::transform::rigid;
use ngk::geometry::Rigid;
use ngk::geometry::{Axis3, Frame, LINEAR_TOLERANCE, Point3, PointCoincidence};
use ngk::modeling::solids::{
    block, block_at, cut, cylinder, cylinder_at, intersect, sphere, torus,
};

fn assert_close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {expected}, got {actual} (error {})",
        (actual - expected).abs()
    );
}

#[test]
fn block_mass_properties_are_exact_for_linear_faces() {
    let shape = block(2.0, 3.0, 4.0).expect("block should build");
    let solid = shape.solid();
    let properties = solid
        .volume_properties()
        .expect("block volume properties should be available");

    assert_close(properties.volume, 24.0, 1e-12);
    assert!(
        properties
            .centroid
            .coincides(Point3::new(1.0, 1.5, 2.0), 1e-12)
    );
    assert_close(properties.inertia.moment_about(Axis3::x()), 50.0, 1e-12);
    assert_close(properties.inertia.moment_about(Axis3::y()), 40.0, 1e-12);
    assert_close(properties.inertia.moment_about(Axis3::z()), 26.0, 1e-12);

    assert_close(
        solid.surface_properties().expect("block area").area,
        52.0,
        1e-12,
    );
}

/// Relative tolerance every curved primitive is held to: the closed forms below
/// are exact, and so is what the measure is asked to reach.
const RELATIVE: f64 = 1e-10;

fn assert_relative(actual: f64, expected: f64) {
    assert_close(actual, expected, RELATIVE * expected.abs());
}

/// Asserts volume properties against closed forms: the centroid to `RELATIVE`
/// of `size`, and the inertia's diagonal relatively with its off-diagonal
/// terms to `RELATIVE` of the largest moment.
fn assert_volume_properties(
    properties: ngk::measure::VolumeProperties,
    volume: f64,
    centroid: Point3,
    moments: [f64; 3],
    size: f64,
) {
    assert_relative(properties.volume, volume);
    assert!(
        properties.centroid.coincides(centroid, RELATIVE * size),
        "expected centroid {centroid:?}, got {:?}",
        properties.centroid
    );
    let tensor = properties.inertia.tensor();
    let largest = moments.iter().copied().fold(0.0, f64::max);
    for row in 0..3 {
        assert_relative(tensor[(row, row)], moments[row]);
        for column in 0..3 {
            if row != column {
                assert_close(tensor[(row, column)], 0.0, RELATIVE * largest);
            }
        }
    }
}

#[test]
fn a_cylinder_measures_its_closed_forms() {
    use std::f64::consts::PI;
    let (radius, height) = (1.0_f64, 2.0_f64);
    let shape = cylinder(radius, height).expect("cylinder should build");
    let volume = PI * radius * radius * height;
    let properties = shape
        .solid()
        .volume_properties()
        .expect("cylinder volume properties");
    let across = volume * (3.0 * radius * radius + height * height) / 12.0;
    let along = volume * radius * radius / 2.0;
    assert_volume_properties(
        properties,
        volume,
        Point3::new(0.0, 0.0, height / 2.0),
        [across, across, along],
        height,
    );
    let area = shape
        .solid()
        .surface_properties()
        .expect("cylinder surface properties")
        .area;
    assert_relative(area, 2.0 * PI * radius * (radius + height));
}

#[test]
fn a_sphere_measures_its_closed_forms() {
    use std::f64::consts::PI;
    let radius = 1.0_f64;
    let shape = sphere(radius).expect("sphere should build");
    let volume = 4.0 * PI * radius.powi(3) / 3.0;
    let properties = shape
        .solid()
        .volume_properties()
        .expect("sphere volume properties");
    let moment = 2.0 * volume * radius * radius / 5.0;
    assert_volume_properties(properties, volume, Point3::origin(), [moment; 3], radius);

    // A thin spherical shell holds two thirds of its area times r² about
    // every axis through its centre.
    let surface = shape
        .solid()
        .surface_properties()
        .expect("sphere surface properties");
    let area = 4.0 * PI * radius * radius;
    assert_relative(surface.area, area);
    assert!(
        surface
            .centroid
            .coincides(Point3::origin(), RELATIVE * radius)
    );
    for axis in [Axis3::x(), Axis3::y(), Axis3::z()] {
        assert_relative(
            surface.inertia.moment_about(axis),
            2.0 * area * radius * radius / 3.0,
        );
    }
}

#[test]
fn a_torus_measures_its_closed_forms() {
    use std::f64::consts::PI;
    let (major, minor) = (3.0_f64, 1.0_f64);
    let shape = torus(major, minor).expect("torus should build");
    let volume = 2.0 * PI * PI * major * minor * minor;
    let properties = shape
        .solid()
        .volume_properties()
        .expect("torus volume properties");
    let across = volume * (4.0 * major * major + 5.0 * minor * minor) / 8.0;
    let along = volume * (major * major + 0.75 * minor * minor);
    assert_volume_properties(
        properties,
        volume,
        Point3::origin(),
        [across, across, along],
        major + minor,
    );

    // About its axis a toroidal shell holds A·(R² + 3r²/2).
    let surface = shape
        .solid()
        .surface_properties()
        .expect("torus surface properties");
    let area = 4.0 * PI * PI * major * minor;
    assert_relative(surface.area, area);
    assert_relative(
        surface.inertia.moment_about(Axis3::z()),
        area * (major * major + 1.5 * minor * minor),
    );
}

#[test]
fn a_cylinder_rim_measures_as_a_ring() {
    use std::f64::consts::PI;
    let (radius, height) = (1.5_f64, 2.0_f64);
    let shape = cylinder(radius, height).expect("cylinder should build");
    let rims = shape.solid().edges();
    assert_eq!(rims.len(), 2, "a cylinder is bounded by its two rims");
    for rim in rims {
        let properties = rim.linear_properties().expect("rim linear properties");
        let length = 2.0 * PI * radius;
        assert_relative(properties.length, length);
        let z = properties.centroid.z;
        assert!(
            properties
                .centroid
                .coincides(Point3::new(0.0, 0.0, z), RELATIVE * radius),
            "a rim's centroid is its centre, got {:?}",
            properties.centroid
        );
        assert!(z.abs() <= RELATIVE || (z - height).abs() <= RELATIVE * height);
        // A ring holds m·r² about its axis and half that about a diameter.
        assert_relative(
            properties.inertia.moment_about(Axis3::z()),
            length * radius * radius,
        );
        assert_relative(
            properties.inertia.moment_about(Axis3::x()),
            length * radius * radius / 2.0,
        );
    }
}

#[test]
fn edge_and_profile_linear_properties_are_available() {
    let shape = block(2.0, 3.0, 4.0).expect("block should build");
    let edge = shape.solid().edges()[0];
    let properties = edge
        .linear_properties()
        .expect("edge linear properties should be available");

    assert_close(properties.length, edge.length(), LINEAR_TOLERANCE);
    assert!(
        properties
            .centroid
            .coords
            .iter()
            .all(|value| value.is_finite())
    );
}

#[test]
fn a_block_shaped_cavity_subtracts_volume_and_adds_inner_surface() {
    let outer = block(4.0, 4.0, 4.0).expect("outer block");
    let inner = block_at(
        Frame::from_xy(Point3::new(1.0, 1.0, 1.0), Vector3::x(), Vector3::y()),
        2.0,
        2.0,
        2.0,
    )
    .expect("inner block");
    let cavity = cut(outer, [&inner]).expect("cut should build a cavity");
    let properties = cavity
        .solid()
        .volume_properties()
        .expect("cavity volume properties");

    assert_close(properties.volume, 56.0, 1e-10);
    assert!(
        properties
            .centroid
            .coincides(Point3::new(2.0, 2.0, 2.0), 1e-10)
    );
    assert_close(
        cavity
            .solid()
            .surface_properties()
            .expect("cavity surface properties")
            .area,
        120.0,
        1e-10,
    );
}

#[test]
fn rigid_translation_preserves_measures_and_moves_the_centroid() {
    let mut shape = block(2.0, 3.0, 4.0).expect("block should build");
    let before = shape
        .solid()
        .volume_properties()
        .expect("volume properties before translation");
    let offset = Vector3::new(7.0, -2.0, 5.0);

    rigid(shape.model_mut(), &Rigid::translation(offset));

    let after = shape
        .solid()
        .volume_properties()
        .expect("volume properties after translation");
    assert_close(after.volume, before.volume, 1e-12);
    assert!(after.centroid.coincides(before.centroid + offset, 1e-12));
    assert_close(
        after.inertia.moment_about(Axis3::x()),
        before.inertia.moment_about(Axis3::x()),
        1e-12,
    );
}

#[test]
fn a_rigid_motion_preserves_measures_and_carries_the_centroid() {
    use radians::Rad64;
    let before_shape = cylinder(1.0, 2.0).expect("cylinder should build");
    let before = before_shape
        .solid()
        .volume_properties()
        .expect("volume properties before the motion");
    let before_area = before_shape.solid().area().expect("area before the motion");

    let motion = Rigid::rotation(
        Axis3::new(Point3::new(1.0, -2.0, 0.5), Vector3::new(1.0, 2.0, 3.0)),
        Rad64::new(0.7),
    );
    let mut shape = cylinder(1.0, 2.0).expect("cylinder should build");
    rigid(shape.model_mut(), &motion);
    let after = shape
        .solid()
        .volume_properties()
        .expect("volume properties after the motion");

    assert_relative(after.volume, before.volume);
    assert_relative(
        shape.solid().area().expect("area after the motion"),
        before_area,
    );
    assert!(
        after
            .centroid
            .coincides(motion.apply(before.centroid), RELATIVE * 10.0),
        "the centroid should move with the solid"
    );
    let principal = |properties: &ngk::measure::VolumeProperties| {
        let mut moments = properties.inertia.principal().moments();
        moments.as_mut_slice().sort_by(f64::total_cmp);
        moments
    };
    let (moved, still) = (principal(&after), principal(&before));
    for index in 0..3 {
        assert_relative(moved[index], still[index]);
    }
}

#[test]
fn the_two_pieces_a_cut_makes_add_up_to_the_whole() {
    // A cylinder standing on the block's corner takes a quarter-round bite
    // out of it: what the cut leaves and what the intersection keeps share
    // the curved wall between them, and together they are the block.
    let tool = || {
        cylinder_at(
            Frame::from_xy(Point3::new(0.0, 0.0, -1.0), Vector3::x(), Vector3::y()),
            1.25,
            6.0,
        )
        .expect("tool should build")
    };
    let rest = cut(block(2.0, 3.0, 4.0).expect("block"), [&tool()]).expect("cut should build");
    let bite =
        intersect(block(2.0, 3.0, 4.0).expect("block"), [&tool()]).expect("intersect should build");
    let rest = rest.solid().volume_properties().expect("rest volume");
    let bite = bite.solid().volume_properties().expect("bite volume");

    let quarter = std::f64::consts::PI * 1.25 * 1.25 / 4.0 * 4.0;
    assert_relative(bite.volume, quarter);
    assert_relative(rest.volume + bite.volume, 24.0);
    let first = rest.centroid.coords * rest.volume + bite.centroid.coords * bite.volume;
    assert!(
        Point3::from(first / 24.0).coincides(Point3::new(1.0, 1.5, 2.0), RELATIVE * 4.0),
        "the pieces' first moments should add up to the block's"
    );
}

#[test]
fn a_section_swept_round_a_helix_encloses_its_area_times_the_helix_length() {
    // A section centred on its spine sweeps exactly its own area along every
    // unit of the spine's length. Eight turns of a slender helix wind each
    // wall many times round its parameter domain, which every turn of the
    // boundary has to be integrated along. The walls are fitted through
    // sixteen frames a turn, and the volume is exact only to that fit.
    use ngk::builders::sweep::{SweepFrame, SweepOptions};
    use ngk::geometry::{Helix, NativeParam, Plane};
    use ngk::modeling::sweep::sweep_face;
    use ngk::modeling::{edges, faces};
    use radians::Rad64;

    let (radius, pitch, turns, width, height) = (2.5, 1.25, 8.0, 0.4, 0.3);
    let axis = Axis3::z();
    let spine = edges::helix(
        axis,
        radius,
        pitch,
        Rad64::new(0.0),
        Rad64::new(std::f64::consts::TAU * turns),
    )
    .expect("helix should build");
    let helix = Helix::from_axis(axis, radius, pitch);
    let start = helix.point_at(NativeParam::new(0.0));
    let tangent = helix.derivative_at(NativeParam::new(0.0), 1).normalize();
    let outward = (start - axis.origin).normalize();
    let across = tangent.cross(&outward);
    let section = faces::rectangle(
        Plane::from_xy(
            start - outward * (width / 2.0) - across * (height / 2.0),
            outward,
            across,
        ),
        width,
        height,
    )
    .expect("section should build");
    let solid = sweep_face(
        section,
        &spine.edge(),
        SweepOptions {
            frame: SweepFrame::Frenet,
            samples_per_segment: 16 * turns as usize,
            ..SweepOptions::default()
        },
    )
    .expect("section should sweep");

    let length = turns * (std::f64::consts::TAU * radius).hypot(pitch);
    let expected = width * height * length;
    let volume = solid
        .solid()
        .volume_properties()
        .expect("swept volume")
        .volume;
    assert_close(volume, expected, 1e-4 * expected);
}
