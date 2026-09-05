use nalgebra::Vector3;
use ngk::builders::boolean::{BooleanOperand, BooleanOptions, compute_boolean_intersections};
use ngk::builders::faces::add_rectangle;
use ngk::geometry::{Plane, Point3};
use ngk::modeling::faces;
use ngk::topology::TopologyEditError;
use ngk::topology::attributes::VertexAttr;
use ngk::topology::gmap::GMap;

fn add_isolated_vertex(map: &mut GMap<ngk::StandardPayload>, point: Point3) -> BooleanOperand {
    map.transaction(|edit| {
        let dart = edit.add_dart();
        Ok::<_, TopologyEditError>(BooleanOperand::Vertex(edit.add_vertex(VertexAttr::new(
            dart,
            point,
            (),
        ))))
    })
    .expect("isolated vertex should build")
}

#[test]
fn curved_outer_trim_admits_points_inside_the_exact_boundary() {
    let shape = faces::circle(Plane::xy(), 2.0).expect("circle face should build");
    let (mut map, face) = shape.into_map();
    let angle = std::f64::consts::PI / 16.0;
    let vertex = add_isolated_vertex(
        &mut map,
        Point3::new(1.99 * angle.cos(), 1.99 * angle.sin(), 0.0),
    );

    let plan = compute_boolean_intersections(
        &map,
        BooleanOperand::Face(face),
        vertex,
        BooleanOptions::default(),
    )
    .expect("trim query should succeed");

    assert_eq!(plan.network.events().len(), 1);
}

#[test]
fn curved_inner_trim_rejects_points_inside_the_exact_hole() {
    let shape = faces::annulus(Plane::xy(), 2.0, 1.0).expect("annular face should build");
    let (mut map, face) = shape.into_map();
    let angle = std::f64::consts::PI / 16.0;
    let vertex = add_isolated_vertex(
        &mut map,
        Point3::new(0.99 * angle.cos(), 0.99 * angle.sin(), 0.0),
    );

    let plan = compute_boolean_intersections(
        &map,
        BooleanOperand::Face(face),
        vertex,
        BooleanOptions::default(),
    )
    .expect("trim query should succeed");

    assert!(plan.network.events().is_empty());
}

#[test]
fn planar_section_crosses_a_curved_trim_at_exact_pcurve_points() {
    let shape = faces::circle(Plane::xy(), 2.0).expect("circle face should build");
    let (mut map, circle) = shape.into_map();
    let y = 2.0 * (std::f64::consts::PI / 16.0).sin();
    let section = add_rectangle(
        &mut map,
        Plane::from_xy(Point3::new(-3.0, y, -1.0), Vector3::x(), Vector3::z()),
        6.0,
        2.0,
    )
    .expect("section face should build");

    let plan = compute_boolean_intersections(
        &map,
        BooleanOperand::Face(circle),
        BooleanOperand::Face(section),
        BooleanOptions::default(),
    )
    .expect("planar section should intersect the circle");

    assert_eq!(plan.network.spans().len(), 1);
    let span = &plan.network.spans()[0];
    for point in [span.curve.point_at(0.0), span.curve.point_at(1.0)] {
        assert!((point.coords.norm() - 2.0).abs() <= 1.0e-10, "{point:?}");
    }
}
