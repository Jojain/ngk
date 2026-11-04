use std::f64::consts::PI;

use nalgebra::Vector3;
use ngk::builders::edges::add_line;
use ngk::builders::faces::add_rectangle as add_rectangle_face;
use ngk::builders::offset::{
    OffsetBuildError, offset_edge, offset_face, offset_profile, offset_solid,
};
use ngk::builders::profiles::add_rectangle as add_rectangle_profile;
use ngk::builders::solids::{add_block, add_cylinder, add_sphere};
use ngk::geometry::{Frame, Plane, Point3};
use ngk::model::Model;
use ngk::topology::payload::StandardPayload;
use ngk::topology::validation::validate_gmap;

fn sorted_points(mut points: Vec<Point3>) -> Vec<Point3> {
    points.sort_by(|a, b| {
        (a.x, a.y, a.z)
            .partial_cmp(&(b.x, b.y, b.z))
            .expect("finite coordinates")
    });
    points
}

fn assert_points(actual: Vec<Point3>, expected: Vec<Point3>) {
    let (actual, expected) = (sorted_points(actual), sorted_points(expected));
    assert_eq!(actual.len(), expected.len(), "{actual:?} vs {expected:?}");
    for (a, e) in actual.iter().zip(&expected) {
        assert!((a - e).norm() <= 1e-9, "{actual:?} vs {expected:?}");
    }
}

#[test]
fn an_edge_offset_is_a_moved_copy_beside_its_source() {
    let mut model = Model::<StandardPayload>::new();
    let edge = add_line(&mut model, Point3::origin(), Point3::new(2.0, 0.0, 0.0)).unwrap();
    let offset = offset_edge(&mut model, edge, Vector3::z_axis(), 1.0).unwrap();
    assert_ne!(offset.edge, edge);
    let moved = offset.view(&model).unwrap();
    assert_points(
        moved
            .vertices()
            .iter()
            .map(|vertex| *vertex.point())
            .collect(),
        vec![Point3::new(0.0, -1.0, 0.0), Point3::new(2.0, -1.0, 0.0)],
    );
    let source = model.edge(edge).unwrap();
    assert_points(
        source
            .vertices()
            .iter()
            .map(|vertex| *vertex.point())
            .collect(),
        vec![Point3::origin(), Point3::new(2.0, 0.0, 0.0)],
    );
}

#[test]
fn a_rectangle_profile_grows_and_shrinks_with_mitred_corners() {
    let mut model = Model::<StandardPayload>::new();
    let profile = add_rectangle_profile(&mut model, Plane::xy(), 4.0, 2.0).unwrap();
    let grown = offset_profile(&mut model, profile, Vector3::z_axis(), 1.0).unwrap();
    let corners = |model: &Model<StandardPayload>, key| {
        model
            .profile(key)
            .unwrap()
            .vertices()
            .iter()
            .map(|vertex| *vertex.point())
            .collect::<Vec<_>>()
    };
    assert_points(
        corners(&model, grown.profile),
        vec![
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(5.0, -1.0, 0.0),
            Point3::new(5.0, 3.0, 0.0),
            Point3::new(-1.0, 3.0, 0.0),
        ],
    );
    let shrunk = offset_profile(&mut model, profile, Vector3::z_axis(), -0.5).unwrap();
    assert_points(
        corners(&model, shrunk.profile),
        vec![
            Point3::new(0.5, 0.5, 0.0),
            Point3::new(3.5, 0.5, 0.0),
            Point3::new(3.5, 1.5, 0.0),
            Point3::new(0.5, 1.5, 0.0),
        ],
    );
    assert!(matches!(
        offset_profile(&mut model, profile, Vector3::z_axis(), -1.5),
        Err(OffsetBuildError::EdgeVanishes { .. })
    ));
    validate_gmap(model.topology()).unwrap();
}

#[test]
fn a_face_moves_along_its_normal() {
    let mut model = Model::<StandardPayload>::new();
    let face = add_rectangle_face(&mut model, Plane::xy(), 2.0, 3.0).unwrap();
    let offset = offset_face(&mut model, face, 0.5).unwrap();
    let moved = offset.view(&model).unwrap();
    let normal = model.face(face).unwrap().normal_at(0.5, 0.5);
    let shift = normal.into_inner() * 0.5;
    assert_points(
        moved
            .vertices()
            .iter()
            .map(|vertex| *vertex.point())
            .collect(),
        [(0.0, 0.0), (2.0, 0.0), (2.0, 3.0), (0.0, 3.0)]
            .iter()
            .map(|&(x, y)| Point3::new(x, y, 0.0) + shift)
            .collect(),
    );
    assert!((moved.area().unwrap() - 6.0).abs() <= 1e-9);
    validate_gmap(model.topology()).unwrap();
}

#[test]
fn a_block_grows_and_shrinks_by_its_faces() {
    let mut model = Model::<StandardPayload>::new();
    let block = add_block(&mut model, Frame::xyz(), 2.0, 3.0, 4.0)
        .unwrap()
        .solid;
    let grown = offset_solid(&mut model, block, 0.5).unwrap();
    let volume = grown.view(&model).unwrap().volume().unwrap();
    assert!((volume - 3.0 * 4.0 * 5.0).abs() <= 1e-9, "volume {volume}");
    let shrunk = offset_solid(&mut model, block, -0.5).unwrap();
    let volume = shrunk.view(&model).unwrap().volume().unwrap();
    assert!((volume - 1.0 * 2.0 * 3.0).abs() <= 1e-9, "volume {volume}");
    assert!((model.solid(block).unwrap().volume().unwrap() - 24.0).abs() <= 1e-9);
    assert!(matches!(
        offset_solid(&mut model, block, -1.2),
        Err(OffsetBuildError::EdgeVanishes { .. })
    ));
    validate_gmap(model.topology()).unwrap();
}

#[test]
fn a_cylinder_grows_its_radius_and_its_height() {
    let mut model = Model::<StandardPayload>::new();
    let cylinder = add_cylinder(&mut model, Frame::xyz(), 1.0, 2.0)
        .unwrap()
        .solid;
    let grown = offset_solid(&mut model, cylinder, 0.5).unwrap();
    let volume = grown.view(&model).unwrap().volume().unwrap();
    let expected = PI * 1.5 * 1.5 * 3.0;
    assert!(
        (volume - expected).abs() <= 1e-6 * expected,
        "volume {volume}"
    );
    let shrunk = offset_solid(&mut model, cylinder, -0.25).unwrap();
    let volume = shrunk.view(&model).unwrap().volume().unwrap();
    let expected = PI * 0.75 * 0.75 * 1.5;
    assert!(
        (volume - expected).abs() <= 1e-6 * expected,
        "volume {volume}"
    );
    validate_gmap(model.topology()).unwrap();
}

#[test]
fn a_sphere_changes_radius() {
    let mut model = Model::<StandardPayload>::new();
    let sphere = add_sphere(&mut model, Frame::xyz(), 2.0).unwrap().solid;
    let grown = offset_solid(&mut model, sphere, 1.0).unwrap();
    let volume = grown.view(&model).unwrap().volume().unwrap();
    let expected = 4.0 / 3.0 * PI * 27.0;
    assert!(
        (volume - expected).abs() <= 1e-6 * expected,
        "volume {volume}"
    );
}

#[test]
fn a_creased_sheet_extends_its_free_boundaries_to_the_moved_crease() {
    use ngk::builders::offset::offset_sheet;
    use ngk::builders::profiles::add_polyline;
    use ngk::builders::sheets::add_extruded_profile;

    let mut model = Model::<StandardPayload>::new();
    let profile = add_polyline(
        &mut model,
        &[
            Point3::new(2.0, 0.0, 0.0),
            Point3::origin(),
            Point3::new(0.0, 2.0, 0.0),
        ],
    )
    .unwrap();
    let sheet = add_extruded_profile(&mut model, profile, Vector3::new(0.0, 0.0, 1.0))
        .unwrap()
        .sheet;
    let offset = offset_sheet(&mut model, sheet, 0.25).unwrap();
    let moved = offset.view(&model).unwrap();
    // The crease's corners sit a quarter off both moved walls; the free ends
    // a quarter off their own wall.
    let mut crease = 0;
    for vertex in moved.vertices() {
        let point = vertex.point();
        assert!(point.z.abs() <= 1e-9 || (point.z - 1.0).abs() <= 1e-9);
        if point.x.abs() < 1.0 && point.y.abs() < 1.0 {
            assert!((point.x.abs() - 0.25).abs() <= 1e-9, "{point:?}");
            assert!((point.y.abs() - 0.25).abs() <= 1e-9, "{point:?}");
            crease += 1;
        }
    }
    assert_eq!(crease, 2);
    validate_gmap(model.topology()).unwrap();
}

#[test]
fn a_smooth_sheet_moves_along_its_normal() {
    use ngk::builders::offset::offset_sheet;
    use ngk::builders::profiles::add_polyline;
    use ngk::builders::sheets::add_extruded_profile;

    let mut model = Model::<StandardPayload>::new();
    let profile =
        add_polyline(&mut model, &[Point3::origin(), Point3::new(2.0, 0.0, 0.0)]).unwrap();
    let sheet = add_extruded_profile(&mut model, profile, Vector3::new(0.0, 0.0, 1.0))
        .unwrap()
        .sheet;
    let offset = offset_sheet(&mut model, sheet, 0.25).unwrap();
    let moved = offset.view(&model).unwrap();
    let ys = moved
        .vertices()
        .iter()
        .map(|vertex| vertex.point().y)
        .collect::<Vec<_>>();
    assert_eq!(ys.len(), 4);
    assert!(ys.iter().all(|y| (y.abs() - 0.25).abs() <= 1e-9), "{ys:?}");
    validate_gmap(model.topology()).unwrap();
}

/// The face of `solid` all of whose edges lie at height `z`.
fn face_at_height(
    model: &Model<StandardPayload>,
    solid: ngk::topology::shape_keys::SolidKey,
    z: f64,
) -> ngk::topology::shape_keys::FaceKey {
    use ngk::geometry::Fraction;
    model
        .solid(solid)
        .unwrap()
        .faces()
        .into_iter()
        .find(|face| {
            face.edges()
                .iter()
                .all(|edge| (edge.trimmed_curve().point_at(Fraction::new(0.5)).z - z).abs() <= 1e-9)
        })
        .expect("a face at that height")
        .key()
}

#[test]
fn a_block_shelled_open_at_the_top_keeps_a_wall_of_the_thickness() {
    use ngk::builders::offset::shell_solid;

    let mut model = Model::<StandardPayload>::new();
    let block = add_block(&mut model, Frame::xyz(), 4.0, 4.0, 4.0)
        .unwrap()
        .solid;
    let top = face_at_height(&model, block, 4.0);
    let shelled = shell_solid(&mut model, block, &[top], 0.5).unwrap();
    let volume = shelled.view(&model).unwrap().volume().unwrap();
    assert!(
        (volume - (64.0 - 3.0 * 3.0 * 3.5)).abs() <= 1e-9,
        "volume {volume}"
    );
    validate_gmap(model.topology()).unwrap();
}

#[test]
fn a_block_shelled_with_no_open_face_holds_a_cavity() {
    use ngk::builders::offset::shell_solid;

    let mut model = Model::<StandardPayload>::new();
    let block = add_block(&mut model, Frame::xyz(), 4.0, 4.0, 4.0)
        .unwrap()
        .solid;
    let shelled = shell_solid(&mut model, block, &[], 0.5).unwrap();
    let volume = shelled.view(&model).unwrap().volume().unwrap();
    assert!((volume - (64.0 - 27.0)).abs() <= 1e-9, "volume {volume}");
}

#[test]
fn a_cylinder_shelled_open_at_the_top_is_a_cup() {
    use ngk::builders::offset::shell_solid;

    let mut model = Model::<StandardPayload>::new();
    let cylinder = add_cylinder(&mut model, Frame::xyz(), 1.0, 2.0)
        .unwrap()
        .solid;
    let top = face_at_height(&model, cylinder, 2.0);
    let shelled = shell_solid(&mut model, cylinder, &[top], 0.2).unwrap();
    let volume = shelled.view(&model).unwrap().volume().unwrap();
    let expected = PI * (2.0 - 0.8 * 0.8 * 1.8);
    assert!(
        (volume - expected).abs() <= 1e-6 * expected,
        "volume {volume}"
    );
}

#[test]
fn a_failed_shell_leaves_the_model_as_it_was() {
    use ngk::builders::offset::{ShellError, shell_solid};

    let mut model = Model::<StandardPayload>::new();
    let block = add_block(&mut model, Frame::xyz(), 4.0, 4.0, 4.0)
        .unwrap()
        .solid;
    let revision = model.revision();
    assert!(matches!(
        shell_solid(&mut model, block, &[], 2.5),
        Err(ShellError::InnerOffset(
            OffsetBuildError::EdgeVanishes { .. }
        ))
    ));
    assert_eq!(model.revision(), revision);
    assert_eq!(model.iter_faces().count(), 6);
}
