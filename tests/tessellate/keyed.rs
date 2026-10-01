//! The public, keyed tessellation: every face range, edge polyline and vertex
//! point carries the key of the cell it comes from, in the cell's own order.

use std::collections::HashMap;

use nalgebra::Vector3;
use ngk::geometry::{Frame, Plane, Point3, Surface};
use ngk::modeling::blend::filleted;
use ngk::modeling::{edges, faces, profiles, solids};
use ngk::tessellate::{Tessellation, tessellate};
use ngk::topology::shape::{Shape, SolidTag};
use ngk::topology::solid::Solid;
use ngk::topology::{ModelEditError, StandardPayload};

fn holed_block() -> Shape<SolidTag> {
    let frame = Frame::from_xz(Point3::new(10.0, 10.0, -1.0), Vector3::x(), Vector3::z());
    let tool = solids::cylinder_at(frame, 4.0, 12.0).expect("cylinder builds");
    let block = solids::block(30.0, 20.0, 10.0).expect("block builds");
    solids::cut(block, tool).expect("cut builds")
}

fn filleted_block() -> Shape<SolidTag> {
    let block = solids::block(2.0, 2.0, 2.0).expect("block builds");
    let edge = block.solid().edges()[0].key();
    filleted(block, vec![edge], 0.1).expect("fillet builds")
}

fn solids_under_test() -> Vec<(&'static str, Shape<SolidTag>)> {
    vec![
        ("block", solids::block(1.0, 2.0, 3.0).expect("block builds")),
        ("holed block", holed_block()),
        ("filleted block", filleted_block()),
    ]
}

/// Ranges are contiguous, start at zero and cover their buffer exactly.
fn assert_ranges_cover(name: &str, ranges: impl Iterator<Item = (usize, usize)>, total: usize) {
    let mut next = 0;
    for (start, count) in ranges {
        assert_eq!(start, next, "{name}: ranges must be contiguous");
        next += count;
    }
    assert_eq!(next, total, "{name}: ranges must cover the buffer");
}

fn assert_keyed_like(solid: &Solid<'_, StandardPayload>, mesh: &Tessellation, name: &str) {
    let faces: Vec<_> = solid.faces().iter().map(|face| face.key()).collect();
    let edges: Vec<_> = solid.edges().iter().map(|edge| edge.key()).collect();
    let vertices: Vec<_> = solid.vertices().iter().map(|vertex| vertex.key()).collect();
    assert_eq!(
        mesh.faces.iter().map(|r| r.key).collect::<Vec<_>>(),
        faces,
        "{name}"
    );
    assert_eq!(
        mesh.edges.iter().map(|r| r.key).collect::<Vec<_>>(),
        edges,
        "{name}"
    );
    assert_eq!(
        mesh.vertices.iter().map(|v| v.key).collect::<Vec<_>>(),
        vertices,
        "{name}"
    );

    assert_ranges_cover(
        name,
        mesh.faces.iter().map(|r| (r.start, r.count)),
        mesh.mesh.indices.len(),
    );
    assert_ranges_cover(
        name,
        mesh.edges.iter().map(|r| (r.start, r.count)),
        mesh.edge_points.len(),
    );
    assert!(
        mesh.faces.iter().all(|r| r.count > 0 && r.count % 3 == 0),
        "{name}"
    );
    assert!(mesh.edges.iter().all(|r| r.count >= 2), "{name}");
    assert_eq!(mesh.mesh.normals.len(), mesh.mesh.positions.len(), "{name}");
}

#[test]
fn a_solid_tessellates_into_ranges_keyed_like_its_cells() {
    for (name, shape) in solids_under_test() {
        let solid = shape.solid();
        let mesh = tessellate(&solid).expect("solid tessellates");
        assert_keyed_like(&solid, &mesh, name);
    }
}

/// The triangles of a planar face's range are that face: their area centroid is
/// the face's centroid. A range attributed to the wrong face is far off.
#[test]
fn each_planar_face_range_is_meshed_where_its_face_is() {
    for (name, shape) in solids_under_test() {
        let solid = shape.solid();
        let mesh = tessellate(&solid).expect("solid tessellates");
        let faces: HashMap<_, _> = solid.faces().into_iter().map(|f| (f.key(), f)).collect();
        for range in &mesh.faces {
            let face = &faces[&range.key];
            if !matches!(face.surface(), Surface::Plane(_)) {
                continue;
            }
            let expected = face.surface_properties().expect("face measures").centroid;
            let actual = area_centroid(&mesh, range.start, range.count);
            assert!(
                (actual - expected).norm() < 1.0e-2,
                "{name}: face {:?} is meshed around {actual:?}, not {expected:?}",
                range.key
            );
        }
    }
}

fn area_centroid(mesh: &Tessellation, start: usize, count: usize) -> Point3 {
    let (mut weighted, mut area) = (Vector3::zeros(), 0.0);
    for triangle in mesh.mesh.indices[start..start + count].chunks_exact(3) {
        let [a, b, c] = [0, 1, 2].map(|i| mesh.mesh.positions[triangle[i] as usize]);
        let triangle_area = (b - a).cross(&(c - a)).norm() / 2.0;
        weighted += (a.coords + b.coords + c.coords) / 3.0 * triangle_area;
        area += triangle_area;
    }
    Point3::from(weighted / area)
}

#[test]
fn vertex_points_are_the_stored_vertex_positions() {
    let shape = holed_block();
    let solid = shape.solid();
    let mesh = tessellate(&solid).expect("solid tessellates");
    for (vertex, view) in mesh.vertices.iter().zip(solid.vertices()) {
        assert_eq!(vertex.point, *view.point());
    }
}

#[test]
fn a_closed_edge_polyline_ends_on_its_first_point() {
    let shape = holed_block();
    let mesh = tessellate(&shape.solid()).expect("solid tessellates");
    let closed = mesh
        .edges
        .iter()
        .filter(|range| shape.model().edge(range.key).unwrap().vertices().len() < 2)
        .collect::<Vec<_>>();
    assert!(!closed.is_empty(), "the hole is bounded by closed edges");
    for range in closed {
        let polyline = &mesh.edge_points[range.start..range.start + range.count];
        let (first, last) = (polyline[0], polyline[polyline.len() - 1]);
        assert!(
            (first - last).norm() < 1.0e-9,
            "{:?} does not close",
            range.key
        );
    }
}

#[test]
fn a_cell_tessellates_with_only_its_own_cells() {
    let face = faces::rectangle(Plane::xy(), 2.0, 3.0).expect("face builds");
    let mesh = tessellate(&face.face()).expect("face tessellates");
    assert_eq!(mesh.faces.len(), 1);
    assert_eq!(mesh.edges.len(), 4);
    assert_eq!(mesh.vertices.len(), 4);

    let profile = profiles::rectangle(Plane::xy(), 2.0, 3.0).expect("profile builds");
    let mesh = tessellate(&profile.profile()).expect("profile tessellates");
    assert!(mesh.faces.is_empty() && mesh.mesh.indices.is_empty());
    assert_eq!(mesh.edges.len(), 4);
    assert_eq!(mesh.vertices.len(), 4);

    let edge = edges::line(Point3::origin(), Point3::new(1.0, 0.0, 0.0)).expect("line builds");
    let mesh = tessellate(&edge.edge()).expect("edge tessellates");
    assert!(mesh.faces.is_empty());
    assert_eq!(mesh.edges.len(), 1);
    assert_eq!(mesh.edge_points.len(), 2);
    assert_eq!(mesh.vertices.len(), 2);
}

#[test]
fn a_model_tessellates_every_cell_it_holds() {
    let shape = holed_block();
    let model = shape.model();
    let mesh = tessellate(model).expect("model tessellates");
    assert_eq!(mesh.faces.len(), model.iter_faces().count());
    assert_eq!(mesh.edges.len(), model.iter_edges().count());
    assert_eq!(mesh.vertices.len(), model.iter_vertices().count());
}

#[test]
fn tessellating_is_deterministic() {
    let first = holed_block();
    let second = holed_block();
    let once = tessellate(&first.solid()).expect("solid tessellates");
    assert_eq!(once, tessellate(&first.solid()).expect("solid tessellates"));
    assert_eq!(
        once,
        tessellate(&second.solid()).expect("solid tessellates")
    );
}

/// A plane face with no boundary would have to be meshed over the plane's
/// whole, unbounded domain: the error says which face could not be meshed.
#[test]
fn a_face_that_cannot_be_meshed_is_named_in_the_error() {
    // A sphere is one boundaryless face; moved onto a plane, its domain is
    // the whole unbounded plane.
    let mut sphere = solids::sphere(1.0).expect("sphere builds");
    let face = sphere.solid().faces()[0].key();
    sphere
        .model_mut()
        .transaction(|edit| {
            edit.face_attr_mut(face).expect("the sphere's face").surface =
                Surface::Plane(Plane::xy());
            Ok::<_, ModelEditError>(())
        })
        .expect("the support swap commits");
    let error = tessellate(&sphere.solid()).expect_err("an unbounded face cannot be meshed");
    assert_eq!(error.face, face);
    assert!(error.to_string().contains(&format!("{face:?}")), "{error}");
}
