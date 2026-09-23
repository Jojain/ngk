use nalgebra::Vector3;
use ngk::builders::transform::rigid;
use ngk::geometry::Rigid;
use ngk::geometry::{Axis3, Frame, LINEAR_TOLERANCE, Point3, PointCoincidence};
use ngk::modeling::solids::{block, block_at, cut, cylinder, sphere, torus};

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

#[test]
fn primitive_mass_properties_match_closed_forms_with_mesh_tolerance() {
    let cylinder = cylinder(1.0, 2.0).expect("cylinder should build");
    let cylinder_properties = cylinder
        .solid()
        .volume_properties()
        .expect("cylinder volume properties");
    assert_close(cylinder_properties.volume, 2.0 * std::f64::consts::PI, 0.12);
    assert!(
        cylinder_properties
            .centroid
            .coincides(Point3::new(0.0, 0.0, 1.0), 0.08)
    );
    assert_close(
        cylinder
            .solid()
            .surface_properties()
            .expect("cylinder surface properties")
            .area,
        6.0 * std::f64::consts::PI,
        0.25,
    );

    let sphere = sphere(1.0).expect("sphere should build");
    let sphere_properties = sphere
        .solid()
        .volume_properties()
        .expect("sphere volume properties");
    assert_close(
        sphere_properties.volume,
        4.0 * std::f64::consts::PI / 3.0,
        0.12,
    );
    assert!(sphere_properties.centroid.coincides(Point3::origin(), 0.08));
    assert_close(
        sphere
            .solid()
            .surface_properties()
            .expect("sphere surface properties")
            .area,
        4.0 * std::f64::consts::PI,
        0.25,
    );

    let torus = torus(3.0, 1.0).expect("torus should build");
    let torus_properties = torus
        .solid()
        .volume_properties()
        .expect("torus volume properties");
    assert_close(
        torus_properties.volume,
        2.0 * std::f64::consts::PI.powi(2) * 3.0,
        1.5,
    );
    assert!(torus_properties.centroid.coincides(Point3::origin(), 0.08));
    assert_close(
        torus
            .solid()
            .surface_properties()
            .expect("torus surface properties")
            .area,
        4.0 * std::f64::consts::PI.powi(2) * 3.0,
        3.0,
    );
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
    let cavity = cut(outer, inner).expect("cut should build a cavity");
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
fn a_section_swept_round_a_helix_encloses_its_area_times_the_helix_length() {
    // A section centred on its spine sweeps exactly its own area along every
    // unit of the spine's length. Eight turns of a slender helix is long
    // enough that a mesh sampling a face only at its corners would skip
    // whole turns of it.
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
    assert_close(volume, expected, 1e-2 * expected);
}
