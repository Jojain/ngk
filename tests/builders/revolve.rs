use nalgebra::Vector3;
use radians::Rad64;

use ngk::builders::edges::add_edge;
use ngk::builders::faces::{add_face, add_polygon};
use ngk::builders::revolve::{add_revolved_edge, add_revolved_face};
use ngk::geometry::axis::Axis3;
use ngk::geometry::{Curve, LINEAR_TOLERANCE, Point3, PointCoincidence, Surface};
use ngk::tessellate::{TessellateOpts, tessellate_face_key};
use ngk::topology::attributes::{EdgeAttr, VertexAttr};
use ngk::topology::gmap::{Cell0, Dim, GMap, TopologyEditError};
use ngk::topology::payload::StandardPayload;

#[test]
fn partial_revolved_edge_creates_surface_face_with_rotated_boundary_edge() {
    let mut g = GMap::<StandardPayload>::new();
    let edge_key = add_edge(
        &mut g,
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)),
    )
    .expect("edge should build");

    let face_key = add_revolved_edge(
        &mut g,
        edge_key,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::QUARTER_TURN,
    )
    .unwrap();
    let face = g.face_unchecked(face_key);
    let edges = face.outer_loop().edges();
    let rotated = edges
        .iter()
        .find(|edge| {
            edge.start()
                .point()
                .unwrap()
                .coincides(Point3::new(0.0, 2.0, 0.0), LINEAR_TOLERANCE)
                && edge
                    .end()
                    .point()
                    .unwrap()
                    .coincides(Point3::new(0.0, 1.0, 0.0), LINEAR_TOLERANCE)
        })
        .expect("revolved face should contain the rotated source edge");

    assert_eq!(g.iter_faces().count(), 1);
    assert!(matches!(
        g.face_attr_unchecked(face_key).surface,
        Surface::Revolution(_)
    ));
    assert_eq!(g.face_attr_unchecked(face_key).pcurves.len(), edges.len());
    let mesh = tessellate_face_key(&g, face_key, TessellateOpts::default())
        .expect("partial revolved edge face should tessellate");
    assert!(!mesh.is_empty());
    assert!(
        rotated
            .end()
            .point()
            .unwrap()
            .coincides(Point3::new(0.0, 1.0, 0.0), LINEAR_TOLERANCE)
    );
    assert!(
        rotated
            .start()
            .point()
            .unwrap()
            .coincides(Point3::new(0.0, 2.0, 0.0), LINEAR_TOLERANCE)
    );
}

#[test]
fn partial_revolved_edge_circle_side_uses_short_positive_sweep() {
    let mut g = GMap::<StandardPayload>::new();
    let edge_key = add_edge(
        &mut g,
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)),
    )
    .expect("edge should build");

    add_revolved_edge(
        &mut g,
        edge_key,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::QUARTER_TURN,
    )
    .unwrap();

    let side_arc = g
        .iter_edges()
        .map(|(_, attr)| attr)
        .find(|attr| {
            let start = g.attribute::<Cell0>(attr.dart).unwrap().point;
            let end = g
                .attribute::<Cell0>(g.alpha(Dim::Zero, attr.dart))
                .unwrap()
                .point;
            matches!(attr.curve, Curve::Circle(_))
                && start.coincides(Point3::new(1.0, 0.0, 0.0), LINEAR_TOLERANCE)
                && end.coincides(Point3::new(0.0, 1.0, 0.0), LINEAR_TOLERANCE)
        })
        .expect("revolve should create a circular side arc");
    let t0 = side_arc.curve.param_at(Point3::new(1.0, 0.0, 0.0));
    let t1 = side_arc.curve.param_at(Point3::new(0.0, 1.0, 0.0));
    let midpoint = side_arc.curve.point_at(0.5 * (t0 + t1));

    assert!(
        midpoint.coincides(
            Point3::new(
                std::f64::consts::FRAC_1_SQRT_2,
                std::f64::consts::FRAC_1_SQRT_2,
                0.0
            ),
            LINEAR_TOLERANCE
        ),
        "side arc midpoint should stay on the same quarter-turn as the revolved face, got {midpoint:?}"
    );
}

#[test]
fn full_revolved_edge_creates_surface_face_between_endpoint_circles() {
    let mut g = GMap::<StandardPayload>::new();
    let edge_key = add_edge(
        &mut g,
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Curve::line(Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)),
    )
    .expect("edge should build");

    let face_key = add_revolved_edge(
        &mut g,
        edge_key,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::FULL_TURN,
    )
    .unwrap();
    let face = g.face_attr_unchecked(face_key);

    assert_eq!(g.iter_faces().count(), 1);
    assert_eq!(face.inner_loops.len(), 1);
    assert_eq!(face.pcurves.len(), 2);
    assert!(matches!(face.surface, Surface::Revolution(_)));
}

#[test]
fn full_revolved_closed_edge_creates_periodic_seam_face() {
    let mut g = GMap::<StandardPayload>::new();
    let edge_key = g
        .transaction(|edit| {
            let first = edit.add_dart();
            let second = edit.add_dart();
            edit.sew(Dim::Zero, first, second)?;
            edit.sew(Dim::One, first, second)?;
            edit.add_vertex(VertexAttr::new(first, Point3::new(1.0, 0.0, 0.0), ()));
            let edge_key = edit.add_edge(EdgeAttr::new(
                first,
                Curve::Circle(ngk::geometry::Circle::from_axis(
                    Axis3::new(Point3::origin(), Vector3::z()),
                    1.0,
                )),
                (),
            ));
            Ok::<_, TopologyEditError>(edge_key)
        })
        .unwrap();

    let face_key = add_revolved_edge(
        &mut g,
        edge_key,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::FULL_TURN,
    )
    .unwrap();
    let face = g.face_attr_unchecked(face_key);

    assert_eq!(g.iter_faces().count(), 1);
    assert!(face.inner_loops.is_empty());
    assert_eq!(face.pcurves.len(), 1);
    assert!(matches!(face.surface, Surface::Revolution(_)));
}

#[test]
fn revolved_face_adds_surface_of_revolution_faces() {
    let mut g = GMap::<StandardPayload>::new();
    let profile_key = add_polygon(
        &mut g,
        &[
            Point3::new(0.75, 0.0, -0.85),
            Point3::new(1.85, 0.0, -0.05),
            Point3::new(0.85, 0.0, 0.9),
        ],
    );
    let source_face = add_face(&mut g, profile_key).unwrap();

    add_revolved_face(
        &mut g,
        source_face,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::QUARTER_TURN,
    )
    .unwrap();

    let revolved_faces = g
        .iter_faces()
        .filter(|(_, attr)| matches!(attr.surface, Surface::Revolution(_)))
        .collect::<Vec<_>>();

    assert_eq!(revolved_faces.len(), 3);
    for (face_key, attr) in revolved_faces {
        assert_eq!(attr.pcurves.len(), 4);
        let mesh = tessellate_face_key(&g, face_key, TessellateOpts::default())
            .expect("revolved face should tessellate from its pcurves");
        assert!(!mesh.is_empty());
    }
    assert_eq!(g.iter_solids().count(), 1);
}
