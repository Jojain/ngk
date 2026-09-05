use std::f64::consts::FRAC_PI_2;

use nalgebra::Vector3;
use ngk::builders::edges::add_edge;
use ngk::geometry::{Curve, Interval, LINEAR_TOLERANCE, Plane, Point3};
use ngk::topology::StandardPayload;
use ngk::topology::gmap::GMap;
use ngk::viz::{VizHints, scene_from_gmap};

/// Builds a lone edge over a quarter circle of radius 1 centred on the origin,
/// carrying either the analytic arc or its NURBS form.
fn quarter_arc_edge(as_nurbs: bool) -> GMap<StandardPayload> {
    let plane = Plane::new(Point3::origin(), Vector3::x(), Vector3::z());
    let arc = Curve::arc(plane, 1.0, Interval::new(0.0, FRAC_PI_2));
    let start = arc.point_at(0.0);
    let end = arc.point_at(1.0);
    let curve = if as_nurbs {
        Curve::Nurbs(arc.to_nurbs().expect("arc as nurbs"))
    } else {
        arc
    };
    let mut g = GMap::<StandardPayload>::new();
    add_edge(&mut g, start, end, curve).expect("arc edge");
    g
}

/// A dart's shaft is a sample of the edge's own curve, so a NURBS arc must be
/// drawn curved rather than collapsed onto its chord.
#[test]
fn dart_shafts_of_a_nurbs_edge_follow_the_curve() {
    for as_nurbs in [false, true] {
        let g = quarter_arc_edge(as_nurbs);
        let scene = scene_from_gmap(&g, &VizHints::new());
        assert_eq!(scene.darts.len(), 2);
        for dart in &scene.darts {
            assert!(
                dart.shaft.len() > 2,
                "a curved dart shaft needs several samples, got {} (nurbs: {as_nurbs})",
                dart.shaft.len()
            );
            for point in &dart.shaft {
                let radius = Vector3::new(point[0], point[1], point[2]).norm();
                assert!(
                    (radius - 1.0).abs() <= LINEAR_TOLERANCE,
                    "shaft sample {point:?} left the arc (radius {radius}, nurbs: {as_nurbs})"
                );
            }
        }
    }
}

/// The two darts of an edge point away from opposite ends of it, whatever
/// direction the underlying curve happens to be parameterised in.
#[test]
fn each_dart_shaft_starts_at_its_own_vertex() {
    for as_nurbs in [false, true] {
        let g = quarter_arc_edge(as_nurbs);
        let scene = scene_from_gmap(&g, &VizHints::new());
        let starts: Vec<[f64; 3]> = scene.darts.iter().map(|d| d.shaft[0]).collect();
        let on_x = starts
            .iter()
            .any(|p| (p[0] - 1.0).abs() <= LINEAR_TOLERANCE && p[1].abs() <= LINEAR_TOLERANCE);
        let on_y = starts
            .iter()
            .any(|p| p[0].abs() <= LINEAR_TOLERANCE && (p[1] - 1.0).abs() <= LINEAR_TOLERANCE);
        assert!(
            on_x && on_y,
            "the two darts must start at opposite ends (nurbs: {as_nurbs}): {starts:?}"
        );
    }
}

/// A difference leaves the bore's two rims carrying different curve kinds — an
/// analytic arc at one end, its NURBS form at the other. The overlay must draw
/// both rims on the bore, not cut one of them straight across the hole.
#[test]
fn dart_shafts_stay_on_a_drilled_bore() {
    use ngk::builders::boolean::{BooleanOperation, BooleanOptions, boolean};
    use ngk::builders::solids::add_extruded_face;
    use ngk::modeling::faces;
    use ngk::topology::TopologyEditError;

    let plane = Plane::from_xy(Point3::origin(), Vector3::x(), Vector3::y());
    let (mut map, block) = {
        let base = faces::rectangle(plane, 2.0, 2.0).expect("block base");
        let (mut map, face) = base.into_map();
        let solid = add_extruded_face(&mut map, face, Vector3::new(0.0, 0.0, 2.0)).expect("block");
        (map, solid)
    };
    let (tool, tool_solid) = {
        let disc = faces::circle(
            Plane::from_xy(Point3::new(1.0, 1.0, -1.0), Vector3::x(), Vector3::y()),
            0.5,
        )
        .expect("bore base");
        let (mut map, face) = disc.into_map();
        let solid =
            add_extruded_face(&mut map, face, Vector3::new(0.0, 0.0, 4.0)).expect("bore body");
        (map, solid)
    };
    let cylinder = map
        .transaction(|edit| {
            let dart = edit.merge(tool.solid_unchecked(tool_solid));
            Ok::<_, TopologyEditError>(edit.solid_key(dart).unwrap())
        })
        .expect("import bore");
    boolean(
        &mut map,
        block,
        cylinder,
        BooleanOperation::Difference,
        BooleanOptions::default(),
    )
    .expect("through hole");

    let scene = scene_from_gmap(&map, &VizHints::new());
    for dart in &scene.darts {
        for point in &dart.shaft {
            let radius = ((point[0] - 1.0).powi(2) + (point[1] - 1.0).powi(2)).sqrt();
            let on_bore = (radius - 0.5).abs() <= 1e-6;
            let on_block = point[0]
                .min(2.0 - point[0])
                .min(point[1])
                .min(2.0 - point[1])
                .abs()
                <= LINEAR_TOLERANCE;
            assert!(
                on_bore || on_block,
                "dart {} sample {point:?} is neither on the bore nor on the block",
                dart.dart_id
            );
        }
    }
}
