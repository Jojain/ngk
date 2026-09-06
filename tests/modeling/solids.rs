use nalgebra::Vector3;
use ngk::geometry::LINEAR_TOLERANCE;
use ngk::modeling::solids::{PrimitiveError, block, sphere};
use ngk::tessellate::{TessellateOpts, face::tessellate_face_key};
use ngk::topology::closed::Closed;
use ngk::topology::gmap::Dim;
use ngk::topology::sheet::Sheet;
use ngk::topology::validation::validate_solid_manifold;
use ngk::viz::debug_viewer::show;

#[test]
fn block_builds_closed_box_with_expected_cell_counts() {
    let shape = block(1.0, 2.0, 3.0).expect("block primitive should build");
    let g = shape.map();
    let solid = shape.solid();
    let shell = solid.outer_shell();

    assert!(
        Closed::new(
            Sheet::from_dart(g, shell.dart).expect("solid shell should have a registered sheet"),
        )
        .is_some(),
        "block outer shell should be closed"
    );
    assert_eq!(
        g.iter_faces().count(),
        6,
        "block should store six face attrs"
    );
    assert_eq!(
        g.iter_edges().count(),
        12,
        "block should store twelve edge attrs"
    );
    assert_eq!(
        g.iter_vertices().count(),
        8,
        "block should store eight vertex attrs"
    );
    assert_eq!(
        g.cells(Dim::Two).count(),
        6,
        "block should have six 2-cells"
    );
    assert_eq!(
        g.cells(Dim::One).count(),
        12,
        "block should have twelve 1-cells"
    );
    assert_eq!(
        g.cells(Dim::Zero).count(),
        8,
        "block should have eight 0-cells"
    );

    for (key, _) in g.iter_faces() {
        let mesh = tessellate_face_key(g, key, TessellateOpts::default())
            .expect("each block face should tessellate");
        assert!(
            !mesh.positions.is_empty(),
            "face {key:?} should emit vertices"
        );
        assert!(
            !mesh.indices.is_empty(),
            "face {key:?} should emit triangles"
        );
    }
}

#[test]
fn block_face_normals_point_outward_from_solid_center() {
    let shape = block(1.0, 2.0, 3.0).expect("block primitive should build");
    let solid_center = Vector3::new(0.5, 1.0, 1.5);

    for face in shape.solid().faces() {
        let vertices = face.vertices();
        let face_center = vertices
            .iter()
            .map(|vertex| {
                vertex
                    .point()
                    .expect("block face vertices should have geometry")
                    .coords
            })
            .sum::<Vector3<f64>>()
            / vertices.len() as f64;
        let outward = face_center - solid_center;
        let normal = face.normal_at(0.0, 0.0);

        assert!(
            normal.dot(&outward) > LINEAR_TOLERANCE,
            "face {:?} normal should point outward",
            face.key()
        );
    }
}

#[test]
fn solid_and_shell_expose_boundary_subtypes() {
    let shape = block(1.0, 2.0, 3.0).expect("block primitive should build");
    let solid = shape.solid();
    let shell = solid.outer_shell();

    assert_eq!(solid.key(), shape.key());
    assert_eq!(solid.shells().len(), 1);
    assert_eq!(solid.faces().len(), 6);
    assert_eq!(solid.edges().len(), 12);
    assert_eq!(solid.vertices().len(), 8);

    assert_eq!(shell.faces().len(), 6);
    assert_eq!(shell.edges().len(), 12);
    assert_eq!(shell.vertices().len(), 8);
}

#[test]
fn block_rejects_non_positive_or_non_finite_sizes() {
    assert_eq!(
        block(-1.0, 2.0, 3.0).err().expect("negative x should fail"),
        PrimitiveError::InvalidSize {
            axis: "x",
            value: -1.0,
        }
    );
    assert_eq!(
        block(1.0, 0.0, 3.0).err().expect("zero y should fail"),
        PrimitiveError::InvalidSize {
            axis: "y",
            value: 0.0,
        }
    );

    match block(1.0, 2.0, f64::NAN)
        .err()
        .expect("non-finite z should fail")
    {
        PrimitiveError::InvalidSize { axis, value } => {
            assert_eq!(axis, "z");
            assert!(value.is_nan());
        }
        other => panic!("expected invalid z size, got {other:?}"),
    }
}

#[test]
fn block_error_message_names_the_invalid_axis_and_value() {
    let error = block(1.0, -2.0, 3.0).err().expect("negative y should fail");

    assert_eq!(
        error.to_string(),
        "block y size must be greater than 0, got -2"
    );
}

#[test]
fn sphere_builds_a_well_formed_solid() {
    let shape = sphere(2.0).expect("sphere primitive should build");
    validate_solid_manifold(shape.map(), shape.key()).expect("sphere should be well formed");
}
