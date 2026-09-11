use std::collections::HashMap;

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
            Sheet::from_dart(g, shell.dart_unchecked())
                .expect("solid shell should have a registered sheet"),
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

/// A sphere has no boundary anywhere, so it carries no topology at all.
///
/// There is nothing for an edge or a vertex to be: the face covers its whole
/// support, and the poles are parametric singularities of `Sphere` rather than
/// cells. That leaves no dart for the shell to be rooted at either, which is
/// what `ShellRoot::Face` is for.
#[test]
fn sphere_builds_a_well_formed_boundaryless_solid() {
    let shape = sphere(2.0).expect("sphere primitive should build");
    let g = shape.map();
    validate_solid_manifold(g, shape.key()).expect("sphere should be well formed");

    assert_eq!(
        (
            g.dart_count(),
            g.iter_vertices().count(),
            g.iter_edges().count(),
            g.iter_faces().count()
        ),
        (0, 0, 0, 1),
        "a sphere is one face and nothing else"
    );
    let faces = shape.solid().faces();
    assert_eq!(faces.len(), 1);
    let face = &faces[0];
    assert!(
        matches!(face.surface(), Surface::Sphere(surface) if surface.radius() == 2.0),
        "sphere primitive should preserve its analytical support"
    );
    assert!(
        face.loops().is_empty(),
        "a sphere face has no boundary loop"
    );
    assert!(face.dart().is_none(), "a boundaryless face has no dart");
    assert_eq!(
        g.solid_attr_unchecked(shape.key()).outer_shell.face(),
        Some(face.key()),
        "the shell is rooted at the face, there being no dart to root at"
    );
}

/// Meshing a sphere leaves no crack at the pole and none at the cut.
///
/// Both used to be paid for in topology — a seam edge and two pole vertices —
/// and are now the mesher's job: a collapsed row becomes a triangle fan, and a
/// direction covering a whole period indexes its closing column back onto its
/// opening one.
#[test]
fn a_sphere_tessellates_into_a_closed_ball() {
    let shape = sphere(2.0).expect("sphere primitive should build");
    let face = shape.solid().faces()[0].key();
    let mesh = tessellate_face_key(shape.map(), face, TessellateOpts::default())
        .expect("sphere face should tessellate");

    assert!(
        mesh.indices.chunks_exact(3).all(|triangle| {
            let [a, b, c] = [
                triangle[0] as usize,
                triangle[1] as usize,
                triangle[2] as usize,
            ];
            (mesh.positions[b] - mesh.positions[a])
                .cross(&(mesh.positions[c] - mesh.positions[a]))
                .norm()
                > LINEAR_TOLERANCE * LINEAR_TOLERANCE
        }),
        "a pole must be fanned, not filled with collapsed quads"
    );

    let mut uses = HashMap::new();
    for triangle in mesh.indices.chunks_exact(3) {
        for pair in [
            (triangle[0], triangle[1]),
            (triangle[1], triangle[2]),
            (triangle[2], triangle[0]),
        ] {
            let edge = (pair.0.min(pair.1), pair.0.max(pair.1));
            *uses.entry(edge).or_insert(0) += 1;
        }
    }
    assert!(
        uses.values().all(|count| *count == 2),
        "every mesh edge of a closed ball is shared by exactly two triangles"
    );
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
    // A sphere has no boundary loop to fan a volume from, so its shell is
    // measured over the surface's own domain instead. Getting the sign right
    // there is the whole outwardness check for a boundaryless face: there is no
    // neighbour across an edge to agree with.
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

/// A ring face's mesh closes over the unwrapped domain's cut.
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
