use nalgebra::Vector3;
use ngk::builders::edges::add_line;
use ngk::builders::faces::add_rectangle;
use ngk::builders::faces::{add_face, add_polygon};
use ngk::builders::solids::add_extruded_face;
use ngk::builders::split::{
    SplitError, split_edge_by_face, split_edge_by_surface, split_face_by_face,
    split_face_by_surface, split_solid_by_face, split_solid_by_surface,
};
use ngk::geometry::{LINEAR_TOLERANCE, Plane, Point3, PointCoincidence, Surface};
use ngk::modeling::solids::block;
use ngk::topology::gmap::{Dim, GMap};
use ngk::topology::payload::{Payload, StandardPayload};
use ngk::topology::validation::{
    validate_gmap, validate_solid_manifold, validate_solid_orientation,
};
use ngk::viz::debug_viewer::show_gmap;

#[test]
fn edge_crossing_plane_produces_classified_edges_and_section_vertex() {
    let mut g = GMap::<StandardPayload>::new();
    let (_, edge) = add_line(
        &mut g,
        Point3::new(0.0, 0.0, -1.0),
        Point3::new(0.0, 0.0, 1.0),
    )
    .expect("edge should build");

    let split = split_edge_by_surface(&mut g, edge, &Surface::Plane(Plane::xy()))
        .expect("crossing edge should split");

    assert_eq!(split.edges.negative.len(), 1);
    assert_eq!(split.edges.positive.len(), 1);
    assert_eq!(split.section_vertices.len(), 1);
    assert!(
        g.vertex_attr(split.section_vertices[0])
            .expect("section vertex should exist")
            .point
            .coincides(Point3::origin(), LINEAR_TOLERANCE)
    );
}

#[test]
fn edge_crossing_trimmed_face_uses_the_cutter_domain() {
    let mut g = GMap::<StandardPayload>::new();
    let (_, edge) = add_line(
        &mut g,
        Point3::new(1.0, 1.0, -1.0),
        Point3::new(1.0, 1.0, 1.0),
    )
    .expect("edge should build");
    let mut cutter = GMap::<StandardPayload>::new();
    let cutter_face =
        add_rectangle(&mut cutter, Plane::xy(), 2.0, 2.0).expect("cutter should build");

    let split = split_edge_by_face(&mut g, edge, &cutter, cutter_face)
        .expect("trimmed face should split the edge");

    assert_eq!(split.edges.negative.len(), 1);
    assert_eq!(split.edges.positive.len(), 1);
}

#[test]
fn rectangle_split_by_plane_produces_two_faces_with_complete_pcurves() {
    let mut g = GMap::<StandardPayload>::new();
    let face = add_rectangle(&mut g, Plane::xy(), 2.0, 2.0).expect("face should build");

    let cutter = Plane::from_xy(Point3::new(1.0, 0.0, 0.0), Vector3::y(), Vector3::z());
    let split = split_face_by_surface(&mut g, face, &Surface::Plane(cutter))
        .expect("rectangle should be partitioned");

    assert_eq!(split.faces.negative.len(), 1);
    assert_eq!(split.faces.positive.len(), 1);
    assert_eq!(split.section_edges.len(), 1);
    for face in split
        .faces
        .negative
        .iter()
        .chain(split.faces.positive.iter())
    {
        let face = g.face(*face).expect("split face should exist");
        assert!(
            face.edges()
                .iter()
                .all(|edge| face.pcurve(edge.dart).is_some())
        );
    }
}

#[test]
fn trimmed_face_intersection_outside_cutter_does_not_mutate_target() {
    let mut target = GMap::<StandardPayload>::new();
    let face = add_rectangle(&mut target, Plane::xy(), 2.0, 2.0).expect("target face should build");
    let before = target.clone();
    let mut cutter = GMap::<StandardPayload>::new();
    let cutter_plane = Plane::from_xy(Point3::new(3.0, 0.0, -1.0), Vector3::y(), Vector3::z());
    let cutter_face =
        add_rectangle(&mut cutter, cutter_plane, 1.0, 2.0).expect("cutter face should build");

    assert!(matches!(
        split_face_by_face(&mut target, face, &cutter, cutter_face),
        Err(SplitError::NoIntersection)
    ));
    assert_eq!(target.dart_count(), before.dart_count());
    assert_eq!(target.iter_faces().count(), before.iter_faces().count());
    assert_eq!(target.iter_edges().count(), before.iter_edges().count());
}

#[test]
fn finite_cutter_that_does_not_partition_rolls_back() {
    let mut target = GMap::<StandardPayload>::new();
    let face = add_rectangle(&mut target, Plane::xy(), 2.0, 2.0).expect("target face should build");
    let before = target.clone();
    let mut cutter = GMap::<StandardPayload>::new();
    let cutter_plane = Plane::from_xy(Point3::new(0.75, 0.75, -0.5), Vector3::x(), Vector3::z());
    let cutter_face =
        add_rectangle(&mut cutter, cutter_plane, 0.5, 1.0).expect("cutter face should build");

    assert!(matches!(
        split_face_by_face(&mut target, face, &cutter, cutter_face),
        Err(SplitError::NonSeparatingCutter)
    ));
    assert_eq!(target.dart_count(), before.dart_count());
    assert_eq!(target.iter_faces().count(), before.iter_faces().count());
    assert_eq!(target.iter_edges().count(), before.iter_edges().count());
}

#[test]
fn finite_face_cutter_partitions_rectangle_when_trim_spans_the_section() {
    let mut target = GMap::<StandardPayload>::new();
    let face = add_rectangle(&mut target, Plane::xy(), 2.0, 2.0).expect("target face should build");
    let mut cutter = GMap::<StandardPayload>::new();
    let cutter_plane = Plane::from_xy(Point3::new(1.0, 0.0, -1.0), Vector3::y(), Vector3::z());
    let cutter_face =
        add_rectangle(&mut cutter, cutter_plane, 2.0, 2.0).expect("cutter face should build");

    let split = split_face_by_face(&mut target, face, &cutter, cutter_face)
        .expect("finite cutter should partition the rectangle");

    assert_eq!(split.faces.negative.len(), 1);
    assert_eq!(split.faces.positive.len(), 1);
}

#[test]
fn block_split_by_plane_preserves_original_solid_and_builds_independent_caps() {
    let shape = block(2.0, 2.0, 2.0).expect("block should build");
    let (mut g, solid) = shape.into_map();
    let cutter = Surface::Plane(Plane::from_xy(
        Point3::new(0.0, 0.0, 1.0),
        Vector3::x(),
        Vector3::y(),
    ));

    let split =
        split_solid_by_surface(&mut g, solid, &cutter).expect("block should be partitioned");
    show_gmap(&g);
    assert_eq!(g.iter_solids().count(), 2);
    assert!(split.solids.negative.contains(&solid) || split.solids.positive.contains(&solid));
    assert_eq!(split.section_faces.negative.len(), 1);
    assert_eq!(split.section_faces.positive.len(), 1);
    for cap in split
        .section_faces
        .negative
        .iter()
        .chain(split.section_faces.positive.iter())
    {
        let cap = g.face(*cap).expect("cap should exist");
        assert_eq!(cap.edges().len(), 4);
        assert!(
            cap.outer_loop()
                .darts()
                .all(|dart| g.is_free(dart, Dim::Three))
        );
    }

    validate_gmap(&g).expect("split map should be valid");
    for solid in split
        .solids
        .negative
        .iter()
        .chain(split.solids.positive.iter())
    {
        assert_eq!(
            g.solid(*solid).expect("solid should exist").faces().len(),
            6
        );
        validate_solid_manifold(&g, *solid).expect("solid should be closed");
        validate_solid_orientation(&g, *solid).expect("solid should face outward");
    }
}

#[test]
fn block_split_by_trimmed_face_clones_solid_payload_for_new_component() {
    let mut g = GMap::<PayloadWithSolidData>::new();
    let loop_dart = add_polygon(
        &mut g,
        &[
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(2.0, 2.0, 0.0),
            Point3::new(0.0, 2.0, 0.0),
        ],
    );
    let face = add_face(&mut g, loop_dart).expect("base face should build");
    let solid =
        add_extruded_face(&mut g, face, Vector3::new(0.0, 0.0, 2.0)).expect("solid should build");

    let mut cutter = GMap::<StandardPayload>::new();
    let cutter_plane = Plane::from_xy(Point3::new(0.0, 0.0, 1.0), Vector3::x(), Vector3::y());
    let cutter_face =
        add_rectangle(&mut cutter, cutter_plane, 2.0, 2.0).expect("cutter face should build");

    let split = split_solid_by_face(&mut g, solid, &cutter, cutter_face)
        .expect("finite cutter should partition the solid");

    assert_eq!(g.iter_solids().count(), 2);
    assert!(split.solids.negative.contains(&solid) || split.solids.positive.contains(&solid));
    assert!(g.iter_solids().all(|(_, attr)| attr.data == SolidData(17)));
}

#[derive(Clone)]
struct PayloadWithSolidData;

impl Payload for PayloadWithSolidData {
    type V = ();
    type E = ();
    type F = ();
    type S = SolidData;
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct SolidData(u32);

impl Default for SolidData {
    fn default() -> Self {
        Self(17)
    }
}
