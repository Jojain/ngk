use nalgebra::Vector3;
use ngk::geometry::{LINEAR_TOLERANCE, PointCoincidence, Surface};
use ngk::modeling::solids::{
    PrimitiveError, block, block_at, cut, cylinder, fuse, intersect, sphere,
};
use ngk::tessellate::{TessellateOpts, face::tessellate_face_key};
use ngk::topology::closed::Closed;
use ngk::topology::gmap::Dim;
use ngk::topology::sheet::Sheet;
use ngk::topology::validation::{validate_solid_manifold, validate_solid_orientation};
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
    let faces = shape.solid().faces();
    assert_eq!(faces.len(), 1);
    assert!(
        matches!(faces[0].surface(), Surface::Sphere(surface) if surface.radius() == 2.0),
        "sphere primitive should preserve its analytical support"
    );

    let face = &faces[0];
    for edge in face
        .outer_loop()
        .expect("face should have an outer loop")
        .edges()
    {
        let curve = edge
            .curve()
            .expect("sphere seam should carry its meridian curve");
        let pcurve = face
            .pcurve(edge.dart())
            .expect("sphere seam should carry a longitude/latitude pcurve");
        for parameter in [0.0, 0.37, 1.0] {
            let uv = pcurve.point_at(parameter);
            let lifted = face.surface().point_at(uv.x, uv.y);
            assert!(
                lifted.coincides(curve.project(lifted), LINEAR_TOLERANCE),
                "sphere pcurve at dart {:?}, t={parameter}, uv={uv:?} lifted off its seam curve",
                edge.dart()
            );
        }
        assert!(
            face.surface()
                .point_at(pcurve.point_at(0.0).x, pcurve.point_at(0.0).y)
                .coincides(
                    *edge
                        .bounded_unchecked()
                        .start()
                        .point()
                        .expect("sphere pole should have geometry"),
                    LINEAR_TOLERANCE,
                )
        );
        assert!(
            face.surface()
                .point_at(pcurve.point_at(1.0).x, pcurve.point_at(1.0).y)
                .coincides(
                    *edge
                        .bounded_unchecked()
                        .end()
                        .point()
                        .expect("sphere pole should have geometry"),
                    LINEAR_TOLERANCE,
                )
        );
    }

    let mesh = tessellate_face_key(shape.map(), face.key(), TessellateOpts::default())
        .expect("sphere face should tessellate");
    assert!(mesh.indices.chunks_exact(3).all(|triangle| {
        let [a, b, c] = [
            triangle[0] as usize,
            triangle[1] as usize,
            triangle[2] as usize,
        ];
        (mesh.positions[b] - mesh.positions[a])
            .cross(&(mesh.positions[c] - mesh.positions[a]))
            .norm()
            > LINEAR_TOLERANCE * LINEAR_TOLERANCE
    }));
}

#[test]
fn solid_boolean_modeling_operations_accept_owned_shapes_and_return_closed_shapes() {
    let operations = [("fuse", 12), ("intersect", 6), ("cut", 9)];

    for (operation, expected_faces) in operations {
        let first = block(2.0, 2.0, 2.0).expect("first block");
        let second = block_at(
            ngk::geometry::Frame::from_xy(
                ngk::geometry::Point3::new(1.0, 1.0, 1.0),
                Vector3::x(),
                Vector3::y(),
            ),
            2.0,
            2.0,
            2.0,
        )
        .expect("second block");

        let result = match operation {
            "fuse" => fuse(first, second),
            "intersect" => intersect(first, second),
            "cut" => cut(first, second),
            _ => unreachable!(),
        }
        .expect("Boolean should succeed");

        validate_solid_manifold(result.map(), result.key())
            .expect("modeling Boolean result should be manifold");
        assert_eq!(result.solid().faces().len(), expected_faces);
    }
}

#[test]
fn a_curved_shell_encloses_positive_signed_volume() {
    // A sphere's only face is bounded by two straight pcurves along its seam.
    // Straight in parameter space is not straight in space, so sampling those
    // pcurves once per edge -- which is right on a plane -- leaves the loop
    // with too few points to span a triangle fan, and the shell measures as
    // enclosing nothing at all.
    let shape = sphere(2.0).expect("sphere primitive should build");
    validate_solid_orientation(shape.map(), shape.key())
        .expect("a sphere shell should be outward oriented");
}

#[test]
fn primitives_carry_their_canonical_analytic_supports() {
    // Dispatching on a surface variant only works if the builders emit the
    // variant the geometry actually is. Extruding a circle along its own
    // normal is a cylinder, not a ruled surface that happens to be one.
    let cylinder = cylinder(1.0, 2.0).expect("cylinder primitive should build");
    let mut variants = cylinder
        .solid()
        .faces()
        .iter()
        .map(|face| match face.surface() {
            Surface::Plane(_) => "plane",
            Surface::Cylinder(_) => "cylinder",
            other => panic!("unexpected cylinder support {other:?}"),
        })
        .collect::<Vec<_>>();
    variants.sort_unstable();
    assert_eq!(variants, ["cylinder", "plane", "plane"]);

    let sphere = sphere(1.0).expect("sphere primitive should build");
    assert!(matches!(
        sphere.solid().faces()[0].surface(),
        Surface::Sphere(_)
    ));

    let block = block(1.0, 1.0, 1.0).expect("block primitive should build");
    assert!(
        block
            .solid()
            .faces()
            .iter()
            .all(|face| matches!(face.surface(), Surface::Plane(_)))
    );
}

/// A ring face's mesh closes over the chart's cut.
///
/// The wall spans its whole period in `u`, so the column at the cut and the
/// column a period later are the same points on the surface. Emitting both would
/// leave a crack that no assertion about triangle quality would notice, which is
/// why this reads the mesh's own topology instead: a tube's only boundary is its
/// two rims, and nothing in between may be open or duplicated.
#[test]
fn a_cylinder_wall_tessellates_into_a_closed_tube() {
    let height = 2.0;
    let shape = cylinder(1.0, height).expect("cylinder primitive should build");
    let wall = shape
        .solid()
        .faces()
        .into_iter()
        .find(|face| matches!(face.surface(), Surface::Cylinder(_)))
        .expect("a cylinder has a cylindrical wall");
    assert!(
        wall.outer_loop().is_none(),
        "the wall is a ring, bounded by two wrapping loops and no outer loop"
    );

    let mesh = tessellate_face_key(shape.map(), wall.key(), TessellateOpts::default())
        .expect("the wall should tessellate");

    for (index, point) in mesh.positions.iter().enumerate() {
        for (other, duplicate) in mesh.positions.iter().enumerate().skip(index + 1) {
            assert!(
                !point.coincides(*duplicate, LINEAR_TOLERANCE),
                "vertices {index} and {other} share the point {point:?}, \
                 so the cut was meshed as two columns rather than one"
            );
        }
    }

    let mut uses = std::collections::HashMap::<(u32, u32), usize>::new();
    for triangle in mesh.indices.chunks_exact(3) {
        for (from, to) in [(0, 1), (1, 2), (2, 0)] {
            let (a, b) = (triangle[from], triangle[to]);
            *uses.entry((a.min(b), a.max(b))).or_default() += 1;
        }
    }
    for (&(a, b), &count) in &uses {
        assert!(count <= 2, "edge {a}-{b} is shared by {count} triangles");
        if count == 2 {
            continue;
        }
        for end in [a, b] {
            let z = mesh.positions[end as usize].z;
            assert!(
                z.abs() <= LINEAR_TOLERANCE || (z - height).abs() <= LINEAR_TOLERANCE,
                "edge {a}-{b} is open at z = {z}, away from either rim"
            );
        }
    }
}
