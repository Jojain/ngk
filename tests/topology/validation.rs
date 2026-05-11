use radians::Rad64;

use ngk::builders::profiles::{add_face, add_polygon};
use ngk::builders::revolve::add_revolved_face;
use ngk::geometry::Point3;
use ngk::geometry::axis::Axis3;
use ngk::modeling::primitives::block;
use ngk::topology::gmap::GMap;
use ngk::topology::payload::StandardPayload;
use ngk::topology::validation::{
    validate_all_solid_manifolds, validate_solid_manifold, validate_solid_orientation,
};

#[test]
fn block_solid_orientation_validation_requires_outward_face_normals() {
    let block = block(1.0, 2.0, 3.0).expect("block should build");

    validate_all_solid_manifolds(block.map()).expect("block shell should be closed");
    validate_solid_orientation(block.map(), block.key())
        .expect("block face normals should point outside the solid");
}

#[test]
fn revolved_triangle_validates_as_closed_manifold_shell() {
    let mut g = GMap::<StandardPayload>::new();
    let loop_dart = add_polygon(
        &mut g,
        &[
            Point3::new(0.75, 0.0, -0.85),
            Point3::new(1.85, 0.0, -0.05),
            Point3::new(0.85, 0.0, 0.9),
        ],
    );
    let face_key = add_face(&mut g, loop_dart).expect("triangle face should build");
    let solid = add_revolved_face(
        &mut g,
        face_key,
        Axis3::new(Point3::origin(), nalgebra::Vector3::z()),
        Rad64::QUARTER_TURN,
    )
    .expect("revolved face should build");

    validate_solid_manifold(&g, solid).expect("revolved triangle shell should be closed");
}
