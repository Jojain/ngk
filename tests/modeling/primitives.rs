use ngk::modeling::primitives::{PrimitiveError, block};
use ngk::tessellate::{TessellateOpts, face::tessellate_face};
use ngk::topology::closed::Closed;
use ngk::topology::gmap::Dim;
use ngk::topology::sheet::Sheet;

#[test]
fn block_builds_closed_box_with_expected_cell_counts() {
    let shape = block(1.0, 2.0, 3.0).expect("block primitive should build");
    let g = shape.map();
    let solid = shape.solid();
    let shell = solid.outer_shell();

    assert!(
        Closed::new(Sheet::new(g, shell.dart)).is_some(),
        "block outer shell should be closed"
    );
    assert_eq!(
        g.iter_faces().count(),
        6,
        "block should store six face attrs"
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
        let mesh = tessellate_face(g, key, TessellateOpts::default())
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
