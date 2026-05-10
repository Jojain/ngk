use nalgebra::Vector3;
use radians::Rad64;

use ngk::builders::profiles::{add_edge, add_face, add_polygon};
use ngk::builders::revolve::{add_revolved_edge, add_revolved_face};
use ngk::geometry::axis::Axis3;
use ngk::geometry::{Curve, LINEAR_TOLERANCE, Line, Point3, PointCoincidence, Surface};
use ngk::tessellate::{TessellateOpts, tessellate_face};
use ngk::topology::attributes::{EdgeAttr, VertexAttr};
use ngk::topology::edge::Edge;
use ngk::topology::gmap::{Cell0, Dim, GMap};
use ngk::topology::payload::StandardPayload;

#[test]
fn partial_revolved_edge_creates_rotated_endpoint_geometry() {
    let mut g = GMap::<StandardPayload>::new();
    let (edge_dart, _) = add_edge(
        &mut g,
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Curve::Line(Line::new(
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        )),
    );

    let rotated = add_revolved_edge(
        &mut g,
        edge_dart,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::QUARTER_TURN,
    )
    .unwrap();
    let edge = Edge::new(&g, rotated);

    assert_eq!(g.dart_count(), 8);
    assert!(
        edge.start()
            .point()
            .unwrap()
            .coincides(Point3::new(0.0, 1.0, 0.0), LINEAR_TOLERANCE)
    );
    assert!(
        edge.end()
            .point()
            .unwrap()
            .coincides(Point3::new(0.0, 2.0, 0.0), LINEAR_TOLERANCE)
    );
}

#[test]
fn partial_revolved_edge_circle_side_uses_short_positive_sweep() {
    let mut g = GMap::<StandardPayload>::new();
    let (edge_dart, _) = add_edge(
        &mut g,
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Curve::Line(Line::new(
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        )),
    );

    add_revolved_edge(
        &mut g,
        edge_dart,
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
fn full_revolved_edge_uses_closed_circle_special_case() {
    let mut g = GMap::<StandardPayload>::new();
    let (edge_dart, _) = add_edge(
        &mut g,
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Curve::Line(Line::new(
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        )),
    );

    let circle = add_revolved_edge(
        &mut g,
        edge_dart,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::FULL_TURN,
    )
    .unwrap();

    assert_eq!(g.dart_count(), 6);
    assert!(matches!(
        Edge::new(&g, circle).curve(),
        Some(Curve::Circle(_))
    ));
}

#[test]
fn full_revolved_closed_edge_creates_one_vertex_circle() {
    let mut g = GMap::<StandardPayload>::new();
    let first = g.add_dart();
    let second = g.add_dart();
    g.sew(Dim::Zero, first, second).unwrap();
    g.sew(Dim::One, first, second).unwrap();
    g.add_vertex(VertexAttr::new(first, Point3::new(1.0, 0.0, 0.0), ()));
    g.add_edge(EdgeAttr::new(
        first,
        Curve::Circle(ngk::geometry::Circle::from_axis(
            Axis3::new(Point3::origin(), Vector3::z()),
            1.0,
        )),
        (),
    ));

    let circle = add_revolved_edge(
        &mut g,
        first,
        Axis3::new(Point3::origin(), Vector3::z()),
        Rad64::FULL_TURN,
    )
    .unwrap();

    assert_eq!(g.dart_count(), 4);
    assert!(matches!(
        Edge::new(&g, circle).curve(),
        Some(Curve::Circle(_))
    ));
}

#[test]
fn revolved_face_adds_surface_of_revolution_faces() {
    let mut g = GMap::<StandardPayload>::new();
    let loop_dart = add_polygon(
        &mut g,
        &[
            Point3::new(0.75, 0.0, -0.85),
            Point3::new(1.85, 0.0, -0.05),
            Point3::new(0.85, 0.0, 0.9),
        ],
    );
    let source_face = add_face(&mut g, loop_dart).unwrap();

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
        let mesh = tessellate_face(&g, face_key, TessellateOpts::default())
            .expect("revolved face should tessellate from its pcurves");
        assert!(!mesh.is_empty());
    }
    assert_eq!(g.iter_solids().count(), 1);
}
