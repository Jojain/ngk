//! Lightweight scene representations for standalone geometry debug values.

use std::f64::consts::TAU;

use nalgebra::Vector3;

use crate::geometry::{Curve, Plane, Point3, Surface};

use super::{VizEdge, VizFace, VizScene, VizVertex};

const DEBUG_EXTENT: f64 = 1.0;
const CURVE_SEGMENTS: usize = 64;
const SURFACE_U_SEGMENTS: usize = 32;
const SURFACE_V_SEGMENTS: usize = 20;

/// Renders a point as a single, clearly visible scene vertex.
pub fn scene_from_point(point: &Point3) -> VizScene {
    let mut scene = VizScene::new();
    scene.vertices.push(VizVertex {
        vertex_id: 0,
        position: point_array(point),
        color: Some("#ff9f1c".to_owned()),
        size: Some(0.07),
        label: Some("point".to_owned()),
    });
    scene
}

/// Renders a vector from the world origin to its component endpoint.
pub fn scene_from_vector(vector: &Vector3<f64>) -> VizScene {
    let endpoint = [vector.x, vector.y, vector.z];
    let mut scene = VizScene::new();
    scene.edges.push(VizEdge {
        edge_id: 0,
        polyline: vec![[0.0, 0.0, 0.0], endpoint],
        color: Some("#2ec4b6".to_owned()),
        width: Some(5.0),
        arrow_head: Some(true),
        label: Some("vector".to_owned()),
    });
    scene
}

/// Renders an unbounded plane as a finite patch plus its origin and normal.
pub fn scene_from_plane(plane: &Plane) -> VizScene {
    let mut scene = tessellate_surface(
        &Surface::Plane(plane.clone()),
        (-DEBUG_EXTENT, DEBUG_EXTENT),
        (-DEBUG_EXTENT, DEBUG_EXTENT),
        "#73a9d8",
        0.42,
    );
    let origin = plane.origin();
    let normal_end = origin + *plane.normal() * (DEBUG_EXTENT * 0.55);
    scene.vertices.push(VizVertex {
        vertex_id: 0,
        position: point_array(&origin),
        color: Some("#f4d35e".to_owned()),
        size: Some(0.055),
        label: Some("origin".to_owned()),
    });
    scene.edges.push(VizEdge {
        edge_id: 0,
        polyline: vec![point_array(&origin), point_array(&normal_end)],
        color: Some("#f4d35e".to_owned()),
        width: Some(3.0),
        arrow_head: None,
        label: Some("normal".to_owned()),
    });
    scene
}

/// Samples a curve over its meaningful finite debug interval.
pub fn scene_from_curve(curve: &Curve) -> VizScene {
    let (start, end) = curve_interval(curve);
    let polyline = (0..=CURVE_SEGMENTS)
        .map(|index| {
            let fraction = index as f64 / CURVE_SEGMENTS as f64;
            point_array(&curve.point_at(start + (end - start) * fraction))
        })
        .collect();

    let mut scene = VizScene::new();
    scene.edges.push(VizEdge {
        edge_id: 0,
        polyline,
        color: Some("#56cfe1".to_owned()),
        width: Some(5.0),
        arrow_head: None,
        label: Some("curve".to_owned()),
    });
    scene
}

/// Tessellates a surface over a finite debug window or its native domain.
pub fn scene_from_surface(surface: &Surface) -> VizScene {
    if let Surface::Plane(plane) = surface {
        return scene_from_plane(plane);
    }

    let (u, v) = surface_intervals(surface);
    tessellate_surface(surface, u, v, "#80b918", 0.58)
}

fn curve_interval(curve: &Curve) -> (f64, f64) {
    match curve {
        Curve::Line(_) => (-DEBUG_EXTENT, DEBUG_EXTENT),
        Curve::Circle(_) => (0.0, TAU),
        Curve::Nurbs(curve) => {
            let domain = curve.domain();
            (domain.start, domain.end)
        }
        Curve::Bounded(_) => (0.0, 1.0),
    }
}

fn surface_intervals(surface: &Surface) -> ((f64, f64), (f64, f64)) {
    match surface {
        Surface::Plane(_) => ((-DEBUG_EXTENT, DEBUG_EXTENT), (-DEBUG_EXTENT, DEBUG_EXTENT)),
        Surface::Cylinder(_) => ((0.0, TAU), (-DEBUG_EXTENT, DEBUG_EXTENT)),
        Surface::Ruled(surface) => (curve_interval(surface.curve()), (0.0, 1.0)),
        Surface::Revolution(surface) => (curve_interval(surface.curve()), (0.0, TAU)),
        Surface::Nurbs(surface) => {
            let u = surface.domain_u();
            let v = surface.domain_v();
            ((u.start, u.end), (v.start, v.end))
        }
    }
}

fn tessellate_surface(
    surface: &Surface,
    u: (f64, f64),
    v: (f64, f64),
    color: &str,
    opacity: f32,
) -> VizScene {
    let stride = SURFACE_U_SEGMENTS + 1;
    let mut positions = Vec::with_capacity(stride * (SURFACE_V_SEGMENTS + 1));
    let mut normals = Vec::with_capacity(positions.capacity());

    for row in 0..=SURFACE_V_SEGMENTS {
        let v_fraction = row as f64 / SURFACE_V_SEGMENTS as f64;
        let parameter_v = v.0 + (v.1 - v.0) * v_fraction;
        for column in 0..=SURFACE_U_SEGMENTS {
            let u_fraction = column as f64 / SURFACE_U_SEGMENTS as f64;
            let parameter_u = u.0 + (u.1 - u.0) * u_fraction;
            positions.push(point_array(&surface.point_at(parameter_u, parameter_v)));
            normals.push(vector_array(&surface.normal_at(parameter_u, parameter_v)));
        }
    }

    let mut indices = Vec::with_capacity(SURFACE_U_SEGMENTS * SURFACE_V_SEGMENTS * 6);
    for row in 0..SURFACE_V_SEGMENTS {
        for column in 0..SURFACE_U_SEGMENTS {
            let a = (row * stride + column) as u32;
            let b = a + 1;
            let c = a + stride as u32;
            let d = c + 1;
            indices.extend_from_slice(&[a, b, c, b, d, c]);
        }
    }

    let mut scene = VizScene::new();
    scene.faces.push(VizFace {
        face_id: 0,
        positions,
        normals,
        indices,
        color: Some(color.to_owned()),
        opacity: Some(opacity),
        double_sided: Some(true),
        label: Some("surface".to_owned()),
    });
    scene
}

fn point_array(point: &Point3) -> [f64; 3] {
    [point.x, point.y, point.z]
}

fn vector_array(vector: &nalgebra::UnitVector3<f64>) -> [f64; 3] {
    [vector.x, vector.y, vector.z]
}
