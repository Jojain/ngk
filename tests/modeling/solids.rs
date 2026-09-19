use std::collections::HashMap;

use nalgebra::Vector3;
use ngk::StandardPayload;
use ngk::geometry::{
    Curve, Frame, LINEAR_TOLERANCE, Point3, PointCoincidence, Surface, SurfacePeriodicity,
};
use ngk::model::Cell2;
use ngk::modeling::solids::{
    PrimitiveError, block, block_at, cut, cylinder, cylinder_at, fuse, intersect, sphere, torus,
};
use ngk::tessellate::{CurveOpts, SurfaceOpts, TessellateOpts, face::tessellate_face_key};
use ngk::topology::closed::Closed;
use ngk::topology::gmap::Dim;
use ngk::topology::shape::{Shape, SolidTag};
use ngk::topology::shape_keys::FaceKey;
use ngk::topology::sheet::Sheet;
use ngk::topology::validation::{validate_solid_manifold, validate_solid_orientation};
use ngk::viz::debug_viewer::show;

#[test]
fn block_builds_closed_box_with_expected_cell_counts() {
    let shape = block(1.0, 2.0, 3.0).expect("block primitive should build");
    let g = shape.model();
    let solid = shape.solid();
    let shell = solid.outer_shell();

    assert!(
        Closed::new(
            Sheet::from_dart(g, shell.dart()).expect("solid shell should have a registered sheet"),
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
            .map(|vertex| vertex.point().coords)
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

/// A sphere has no boundary anywhere, so it carries no edge and no vertex.
///
/// There is nothing for either to be: the face covers its whole support, and
/// the poles are parametric singularities of `Sphere` rather than cells. What
/// the face does have is the raw 2-cell it is required to occupy — the bigon
/// underneath it — and that is where its shell is rooted.
#[test]
fn sphere_builds_a_well_formed_boundaryless_solid() {
    let shape = sphere(2.0).expect("sphere primitive should build");
    let g = shape.model();
    validate_solid_manifold(g, shape.key()).expect("sphere should be well formed");

    assert_eq!(
        (
            g.dart_count(),
            g.iter_vertices().count(),
            g.iter_edges().count(),
            g.iter_faces().count()
        ),
        (4, 0, 0, 1),
        "a sphere is one face over the bigon its embedded edge and poles make up"
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
    assert_eq!(
        g.cell_key::<Cell2>(
            g.cell_representative(g.solid_attr_unchecked(shape.key()).outer_shell, Dim::Two)
        ),
        Some(face.key()),
        "the shell is rooted in the 2-cell the face occupies"
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
    let mesh = tessellate_face_key(shape.model(), face, TessellateOpts::default())
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

/// A torus has no boundary either, and closes in *both* parameter directions.
///
/// A sphere closes in one direction by periodicity and in the other by
/// collapsing at its poles; a torus is periodic twice over, which is the case
/// `Surface::is_closed` has to answer for without any degenerate row to read.
/// The primitive builds a native [`Surface::Torus`] directly: there is no swept
/// profile and nothing for the shape to reuse as a boundary.
#[test]
fn torus_builds_a_well_formed_boundaryless_solid() {
    let shape = torus(3.0, 1.0).expect("torus primitive should build");
    let g = shape.model();
    validate_solid_manifold(g, shape.key()).expect("torus should be well formed");

    assert_eq!(
        (
            g.dart_count(),
            g.iter_vertices().count(),
            g.iter_edges().count(),
            g.iter_faces().count()
        ),
        (8, 0, 0, 1),
        "a torus is one face over the square its embedded edges and vertex make up"
    );

    let faces = shape.solid().faces();
    let face = &faces[0];
    assert!(
        matches!(face.surface(), Surface::Torus(_)),
        "the primitive should instantiate a torus support directly"
    );
    assert!(face.loops().is_empty(), "a torus face has no boundary loop");
    assert_eq!(
        g.cell_key::<Cell2>(
            g.cell_representative(g.solid_attr_unchecked(shape.key()).outer_shell, Dim::Two)
        ),
        Some(face.key()),
        "the shell is rooted in the 2-cell the face occupies"
    );
    assert!(
        matches!(
            face.surface().periodicity(),
            SurfacePeriodicity::UVPeriodic(_, _)
        ),
        "a torus is periodic in both its longitude and its tube"
    );
    assert!(
        face.surface().is_closed(),
        "both directions close, so the shell this face alone makes is closed"
    );
}

/// Meshing a torus leaves no crack where either direction closes on itself.
///
/// The sphere covers one wrapped direction against a pole; here both wrap, and
/// neither is bounded by a loop or a degenerate row, so the mesh closes only if
/// the last column *and* the last row index back onto the first.
#[test]
fn a_torus_tessellates_into_a_closed_tube() {
    let shape = torus(3.0, 1.0).expect("torus primitive should build");
    let face = shape.solid().faces()[0].key();
    let mesh = tessellate_face_key(shape.model(), face, TessellateOpts::default())
        .expect("torus face should tessellate");

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
        "every mesh edge of a closed tube is shared by exactly two triangles"
    );
    assert!(
        mesh.positions.iter().all(|point| {
            let radial = (point.x * point.x + point.y * point.y).sqrt();
            let offset = (radial - 3.0).hypot(point.z);
            (offset - 1.0).abs() <= 1e-6
        }),
        "every mesh vertex sits on the tube"
    );
}

/// A tube as wide as its offset reaches the axis, so it is not a torus.
#[test]
fn a_torus_whose_tube_reaches_the_axis_is_refused() {
    assert!(
        matches!(torus(1.0, 1.0), Err(PrimitiveError::SolidCreationFailed)),
        "a tube as wide as its offset touches the axis"
    );
    assert!(
        matches!(torus(1.0, 2.0), Err(PrimitiveError::SolidCreationFailed)),
        "a tube wider than its offset sweeps through itself"
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

        validate_solid_manifold(result.model(), result.key())
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
    validate_solid_orientation(shape.model(), shape.key())
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

    let mesh = tessellate_face_key(shape.model(), wall.key(), TessellateOpts::default())
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

/// A frame at `(x, y, z)` with the world's own axes.
fn frame_at(x: f64, y: f64, z: f64) -> Frame {
    Frame::from_xy(Point3::new(x, y, z), Vector3::x(), Vector3::y())
}

/// The volume a solid's meshed boundary encloses, by the divergence theorem.
///
/// Sampled finely, because a meshed circle is a polygon inscribed in it and so
/// measures a little short: at these settings a rim is a 128-gon, which holds
/// 99.96% of its circle's area, and the assertion below leaves a percent of
/// room for that. This is a check on which side of each face the material was
/// kept, not a mass property.
fn meshed_volume(shape: &Shape<SolidTag, StandardPayload>) -> f64 {
    let opts = TessellateOpts {
        curve: CurveOpts { segments: 128 },
        surface: SurfaceOpts { nu: 128, nv: 16 },
    };
    let mut volume = 0.0;
    for face in shape.solid().faces() {
        let mesh = tessellate_face_key(shape.model(), face.key(), opts)
            .unwrap_or_else(|error| panic!("face {:?} should mesh: {error}", face.key()));
        for triangle in mesh.indices.chunks_exact(3) {
            let corner = |slot: usize| mesh.positions[triangle[slot] as usize].coords;
            volume += corner(0).dot(&corner(1).cross(&corner(2))) / 6.0;
        }
    }
    volume
}

/// Every planar face the solid carries whose support sits at height `z`.
fn planes_at_height(shape: &Shape<SolidTag, StandardPayload>, z: f64) -> Vec<FaceKey> {
    shape
        .solid()
        .faces()
        .into_iter()
        .filter(|face| {
            matches!(face.surface(), Surface::Plane(plane)
                if (plane.frame.origin.z - z).abs() <= LINEAR_TOLERANCE)
        })
        .map(|face| face.key())
        .collect()
}

/// A cup: a cylinder with a narrower, taller one cut out from above its floor.
///
/// The bore runs out through the top, so the cut leaves an annulus there rather
/// than closing the tool off. Each of that annulus's rims is a whole circle, and
/// the pair of arcs meeting at two invented corners that would describe one just
/// as well is what the network used to record instead.
#[test]
fn a_cylinder_bored_from_above_its_floor_is_a_cup() {
    let outer = cylinder_at(frame_at(0.0, 0.0, 0.0), 20.0, 30.0).expect("outer cylinder");
    let inner = cylinder_at(frame_at(0.0, 0.0, 4.0), 16.0, 28.0).expect("boring tool");

    let cup = cut(outer, inner).expect("boring a cylinder should succeed");

    validate_solid_manifold(cup.model(), cup.key()).expect("the cup should be manifold");
    validate_solid_orientation(cup.model(), cup.key())
        .expect("the cup should be outward oriented");

    let faces = cup.solid().faces();
    assert_eq!(
        faces.len(),
        5,
        "a cup is a floor, an outer wall, a rim annulus, a bore wall and a bore floor"
    );
    let mut radii = faces
        .iter()
        .filter_map(|face| match face.surface() {
            Surface::Cylinder(wall) => Some(wall.radius),
            _ => None,
        })
        .collect::<Vec<_>>();
    radii.sort_by(f64::total_cmp);
    assert_eq!(radii, vec![16.0, 20.0], "the two walls keep their own radii");
    let mut heights = faces
        .iter()
        .filter_map(|face| match face.surface() {
            Surface::Plane(plane) => Some(plane.frame.origin.z),
            _ => None,
        })
        .collect::<Vec<_>>();
    heights.sort_by(f64::total_cmp);
    assert_eq!(
        heights,
        vec![0.0, 4.0, 30.0],
        "the floor is at 0, the bore floor at 4, and the rim at 30"
    );

    let [rim] = planes_at_height(&cup, 30.0)[..] else {
        panic!("the rim is one face");
    };
    let rim = cup.model().face_unchecked(rim);
    assert_eq!(
        rim.inner_loops().len(),
        1,
        "the bore opens through the rim, which is therefore an annulus"
    );
    for boundary in rim.loops() {
        let [edge] = boundary.edges()[..] else {
            panic!("each rim circle is one unbounded edge, not a pair of arcs");
        };
        assert!(
            matches!(edge.curve(), Curve::Circle(_)),
            "a whole circle stays a circle rather than becoming a fitted curve"
        );
    }

    let expected = std::f64::consts::PI * (400.0 * 30.0 - 256.0 * 26.0);
    let volume = meshed_volume(&cup);
    assert!(
        (volume - expected).abs() <= 0.01 * expected,
        "a cup of these sizes holds {expected}, not {volume}, \
         so a face was kept on the wrong side"
    );
}

/// A bar whose top is flush with a cylinder's, coplanar over part of its disc.
///
/// The two coincident faces share an arc of the disc's rim, which each operand
/// reads on its own branch of the circle's angle -- `-2.99` on one side and
/// `3.29` on the other, one period apart. Read literally those are two points,
/// the arc is recorded as two sections with one incidence each, and the
/// coincident region is left with a boundary that never closes.
#[test]
fn a_bar_flush_with_a_cylinders_top_fuses_across_the_rims_period() {
    let cylinder = cylinder(20.0, 30.0).expect("cylinder primitive should build");
    let bar = block_at(frame_at(-30.0, -3.0, 24.0), 12.5, 6.0, 6.0).expect("bar primitive");

    let result = fuse(cylinder, bar).expect("a flush bar should fuse");

    validate_solid_manifold(result.model(), result.key()).expect("the fusion should be manifold");
    validate_solid_orientation(result.model(), result.key())
        .expect("the fusion should be outward oriented");
    assert_eq!(
        planes_at_height(&result, 30.0).len(),
        1,
        "the disc and the bar top are coplanar and adjacent, so they are one face"
    );
}

/// A cup with a handle: one Boolean that both bores and notches a cylinder wall.
///
/// The handle meets the wall twice. The lower bar imprints a closed loop, which
/// is a hole, and the upper bar runs out through the rim, which is a chord. Both
/// land on a ring face, whose loops are each one period long and sample to open
/// polylines rather than polygons -- so neither the hole's own winding nor the
/// half of the chorded face it belongs to can be read by counting crossings over
/// the face's corners. The upper bar's top is flush with the cup's rim besides,
/// so two coincident faces meet along an arc each operand reads a period apart.
#[test]
fn a_handle_that_bores_and_notches_one_wall_fuses_to_a_mug() {
    let cup = cylinder_at(frame_at(0.0, 0.0, 0.0), 20.0, 30.0).expect("cup body");
    let lower = block_at(frame_at(-30.0, -3.0, 8.0), 12.5, 6.0, 6.0).expect("lower bar");
    let upper = block_at(frame_at(-30.0, -3.0, 24.0), 12.5, 6.0, 6.0).expect("upper bar");
    let spine = block_at(frame_at(-30.0, -3.0, 8.0), 6.0, 6.0, 22.0).expect("handle spine");
    let handle = fuse(fuse(lower, spine).expect("handle base"), upper).expect("handle");

    let mug = fuse(cup, handle).expect("a handle should fuse onto a cup");

    validate_solid_manifold(mug.model(), mug.key()).expect("the mug should be manifold");
    validate_solid_orientation(mug.model(), mug.key())
        .expect("the mug should be outward oriented");

    assert_eq!(
        mug.solid().faces().len(),
        10,
        "a floor, a wall and a rim, plus the seven outward faces of the handle"
    );
    let walls = mug
        .solid()
        .faces()
        .into_iter()
        .filter(|face| matches!(face.surface(), Surface::Cylinder(_)))
        .map(|face| face.key())
        .collect::<Vec<_>>();
    let [wall] = walls[..] else {
        panic!("the mug keeps exactly one cylindrical wall");
    };
    
    assert_eq!(
        mug.model().face_unchecked(wall).inner_loops().len(),
        1,
        "the lower bar bores the wall once; the upper bar notches its rim instead"
    );
    assert_eq!(
        planes_at_height(&mug, 30.0).len(),
        1,
        "the rim and the upper bar top are coplanar and adjacent, so they are one face"
    );
}
