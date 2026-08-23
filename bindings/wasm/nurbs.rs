use js_sys::Float64Array;
use serde::Serialize;
use wasm_bindgen::prelude::*;

use crate::geometry::{
    ControlNet, ControlPolygon, Curve, CurveCurveIntersection, CurveSurfaceIntersection, Degree,
    KnotVector, NurbsCurve, NurbsSurface, Point3, Surface, SurfaceSurfaceIntersection,
    sample_curve_uniform, tessellate_curve_adaptive, tessellate_surface_grid,
};

fn js_err(e: impl ToString) -> JsValue {
    JsValue::from_str(&e.to_string())
}

fn points_from_flat(xyz: &[f64]) -> Result<Vec<Point3>, JsValue> {
    if xyz.len() % 3 != 0 {
        return Err(JsValue::from_str(
            "xyz array length must be a multiple of 3",
        ));
    }
    Ok(xyz
        .chunks_exact(3)
        .map(|c| Point3::new(c[0], c[1], c[2]))
        .collect())
}

fn flat_from_points(points: &[Point3]) -> Float64Array {
    let mut flat = Vec::with_capacity(points.len() * 3);
    for p in points {
        flat.push(p.x);
        flat.push(p.y);
        flat.push(p.z);
    }
    let out = Float64Array::new_with_length(flat.len() as u32);
    out.copy_from(&flat);
    out
}

fn flat_from_f64(values: &[f64]) -> Float64Array {
    let out = Float64Array::new_with_length(values.len() as u32);
    out.copy_from(values);
    out
}

#[wasm_bindgen(js_name = NurbsCurve)]
pub struct WasmNurbsCurve {
    inner: NurbsCurve,
}

#[wasm_bindgen]
impl WasmNurbsCurve {
    /// Build a NURBS curve from a flat `[x,y,z, x,y,z, ...]` array,
    /// per-point `weights`, and the `knots` vector.
    #[wasm_bindgen(constructor)]
    pub fn new(
        degree: usize,
        control_points_xyz: &[f64],
        weights: &[f64],
        knots: &[f64],
    ) -> Result<WasmNurbsCurve, JsValue> {
        let degree = Degree::new(degree).map_err(js_err)?;
        let points = points_from_flat(control_points_xyz)?;
        let cp = ControlPolygon::from_cartesian(points, weights).map_err(js_err)?;
        let kv = KnotVector::new(knots.to_vec()).map_err(js_err)?;
        let inner = NurbsCurve::new(degree, cp, kv).map_err(js_err)?;
        Ok(Self { inner })
    }

    /// Build with a default clamped-uniform knot vector on `[0, 1]`.
    #[wasm_bindgen(js_name = uniform)]
    pub fn uniform(
        degree: usize,
        control_points_xyz: &[f64],
        weights: &[f64],
    ) -> Result<WasmNurbsCurve, JsValue> {
        let degree = Degree::new(degree).map_err(js_err)?;
        let points = points_from_flat(control_points_xyz)?;
        let cp = ControlPolygon::from_cartesian(points, weights).map_err(js_err)?;
        let inner = NurbsCurve::with_uniform_knots(degree, cp).map_err(js_err)?;
        Ok(Self { inner })
    }

    #[wasm_bindgen(js_name = pointAt)]
    pub fn point_at(&self, u: f64) -> Float64Array {
        let p = self.inner.point_at(u);
        flat_from_points(&[p])
    }

    /// Uniformly sample `n + 1` points.
    #[wasm_bindgen]
    pub fn sample(&self, n: usize) -> Float64Array {
        flat_from_points(&sample_curve_uniform(&self.inner, n))
    }

    /// Adaptive tessellation controlled by chord-midpoint `tolerance`.
    #[wasm_bindgen(js_name = tessellateAdaptive)]
    pub fn tessellate_adaptive(&self, tolerance: f64, max_depth: usize) -> Float64Array {
        flat_from_points(&tessellate_curve_adaptive(
            &self.inner,
            tolerance,
            max_depth,
        ))
    }

    #[wasm_bindgen(js_name = insertKnot)]
    pub fn insert_knot(&mut self, u: f64) {
        self.inner.insert_knot(u);
    }

    #[wasm_bindgen(js_name = knots)]
    pub fn knots(&self) -> Float64Array {
        flat_from_f64(self.inner.knots().as_slice())
    }

    #[wasm_bindgen(js_name = degree)]
    pub fn degree(&self) -> usize {
        self.inner.degree().get()
    }

    #[wasm_bindgen(js_name = controlPointsXyz)]
    pub fn control_points_xyz(&self) -> Float64Array {
        let pts: Vec<Point3> = self
            .inner
            .control_points()
            .iter()
            .map(|hp| hp.to_cartesian())
            .collect();
        flat_from_points(&pts)
    }

    #[wasm_bindgen(js_name = weights)]
    pub fn weights(&self) -> Float64Array {
        let ws: Vec<f64> = self
            .inner
            .control_points()
            .iter()
            .map(|hp| hp.weight())
            .collect();
        flat_from_f64(&ws)
    }

    #[wasm_bindgen(js_name = domain)]
    pub fn domain(&self) -> Float64Array {
        let domain = self.inner.domain();
        flat_from_f64(&[domain.start, domain.end])
    }

    #[wasm_bindgen(js_name = intersectCurve)]
    pub fn intersect_curve(&self, other: &WasmNurbsCurve) -> Result<JsValue, JsValue> {
        let a = Curve::Nurbs(self.inner.clone());
        let b = Curve::Nurbs(other.inner.clone());
        let intersections = a.intersect_curve(&b).map_err(js_err)?;
        let out: Vec<WasmCurveCurveIntersection> = intersections
            .into_iter()
            .map(WasmCurveCurveIntersection::from)
            .collect();
        serde_wasm_bindgen::to_value(&out).map_err(js_err)
    }

    #[wasm_bindgen(js_name = intersectSurface)]
    pub fn intersect_surface(&self, surface: &WasmNurbsSurface) -> Result<JsValue, JsValue> {
        let curve = Curve::Nurbs(self.inner.clone());
        let surface = Surface::Nurbs(surface.inner.clone());
        let intersections = curve.intersect_surface(&surface).map_err(js_err)?;
        let out: Vec<WasmCurveSurfaceIntersection> = intersections
            .into_iter()
            .map(WasmCurveSurfaceIntersection::from)
            .collect();
        serde_wasm_bindgen::to_value(&out).map_err(js_err)
    }
}

#[derive(Serialize)]
struct SurfaceMesh {
    positions: Vec<f64>,
    normals: Vec<f64>,
    indices: Vec<u32>,
}

#[derive(Serialize)]
#[serde(tag = "kind")]
enum WasmCurveCurveIntersection {
    #[serde(rename = "point")]
    Point { point: [f64; 3], u_a: f64, u_b: f64 },
    #[serde(rename = "overlap")]
    Overlap {
        interval_a: [f64; 2],
        interval_b: [f64; 2],
    },
}

impl From<CurveCurveIntersection> for WasmCurveCurveIntersection {
    fn from(value: CurveCurveIntersection) -> Self {
        match value {
            CurveCurveIntersection::Point { point, u_a, u_b } => Self::Point {
                point: [point.x, point.y, point.z],
                u_a,
                u_b,
            },
            CurveCurveIntersection::Overlap {
                interval_a,
                interval_b,
            } => Self::Overlap {
                interval_a: [interval_a.start, interval_a.end],
                interval_b: [interval_b.start, interval_b.end],
            },
        }
    }
}

#[derive(Serialize)]
#[serde(tag = "kind")]
enum WasmCurveSurfaceIntersection {
    #[serde(rename = "point")]
    Point {
        point: [f64; 3],
        curve_u: f64,
        surface_u: f64,
        surface_v: f64,
    },
    #[serde(rename = "overlap")]
    Overlap { curve_interval: [f64; 2] },
}

impl From<CurveSurfaceIntersection> for WasmCurveSurfaceIntersection {
    fn from(value: CurveSurfaceIntersection) -> Self {
        match value {
            CurveSurfaceIntersection::Point {
                point,
                curve_u,
                surface_u,
                surface_v,
            } => Self::Point {
                point: [point.x, point.y, point.z],
                curve_u,
                surface_u,
                surface_v,
            },
            CurveSurfaceIntersection::Overlap { curve_interval } => Self::Overlap {
                curve_interval: [curve_interval.start, curve_interval.end],
            },
        }
    }
}

#[derive(Serialize)]
#[serde(tag = "kind")]
enum WasmSurfaceSurfaceIntersection {
    #[serde(rename = "point")]
    Point {
        point: [f64; 3],
        surface_a_u: f64,
        surface_a_v: f64,
        surface_b_u: f64,
        surface_b_v: f64,
    },
    #[serde(rename = "curve")]
    Curve { points: Vec<[f64; 3]> },
    #[serde(rename = "region")]
    Region,
}

impl From<SurfaceSurfaceIntersection> for WasmSurfaceSurfaceIntersection {
    fn from(value: SurfaceSurfaceIntersection) -> Self {
        match value {
            SurfaceSurfaceIntersection::Point {
                point,
                surface_a_u,
                surface_a_v,
                surface_b_u,
                surface_b_v,
            } => Self::Point {
                point: [point.x, point.y, point.z],
                surface_a_u,
                surface_a_v,
                surface_b_u,
                surface_b_v,
            },
            SurfaceSurfaceIntersection::Curve { points } => Self::Curve {
                points: points
                    .into_iter()
                    .map(|point| [point.x, point.y, point.z])
                    .collect(),
            },
            SurfaceSurfaceIntersection::Region => Self::Region,
        }
    }
}

#[wasm_bindgen(js_name = NurbsSurface)]
pub struct WasmNurbsSurface {
    inner: NurbsSurface,
}

#[wasm_bindgen]
impl WasmNurbsSurface {
    #[wasm_bindgen(constructor)]
    pub fn new(
        degree_u: usize,
        degree_v: usize,
        nu: usize,
        nv: usize,
        control_points_xyz: &[f64],
        weights: &[f64],
        knots_u: &[f64],
        knots_v: &[f64],
    ) -> Result<WasmNurbsSurface, JsValue> {
        let du = Degree::new(degree_u).map_err(js_err)?;
        let dv = Degree::new(degree_v).map_err(js_err)?;
        let points = points_from_flat(control_points_xyz)?;
        let cn = ControlNet::from_cartesian(points, weights, nu, nv).map_err(js_err)?;
        let ku = KnotVector::new(knots_u.to_vec()).map_err(js_err)?;
        let kv = KnotVector::new(knots_v.to_vec()).map_err(js_err)?;
        let inner = NurbsSurface::new(du, dv, cn, ku, kv).map_err(js_err)?;
        Ok(Self { inner })
    }

    #[wasm_bindgen(js_name = uniform)]
    pub fn uniform(
        degree_u: usize,
        degree_v: usize,
        nu: usize,
        nv: usize,
        control_points_xyz: &[f64],
        weights: &[f64],
    ) -> Result<WasmNurbsSurface, JsValue> {
        let du = Degree::new(degree_u).map_err(js_err)?;
        let dv = Degree::new(degree_v).map_err(js_err)?;
        let points = points_from_flat(control_points_xyz)?;
        let cn = ControlNet::from_cartesian(points, weights, nu, nv).map_err(js_err)?;
        let inner = NurbsSurface::with_uniform_knots(du, dv, cn).map_err(js_err)?;
        Ok(Self { inner })
    }

    #[wasm_bindgen(js_name = pointAt)]
    pub fn point_at(&self, u: f64, v: f64) -> Float64Array {
        let p = self.inner.point_at(u, v);
        flat_from_points(&[p])
    }

    /// Tessellate into a regular `nu × nv` grid. Returns an object
    /// `{ positions: Float64Array, normals: Float64Array, indices: Uint32Array }`.
    #[wasm_bindgen(js_name = sampleGrid)]
    pub fn sample_grid(&self, nu: usize, nv: usize) -> Result<JsValue, JsValue> {
        let mesh = tessellate_surface_grid(&self.inner, nu, nv);
        let mut positions = Vec::with_capacity(mesh.positions.len() * 3);
        for p in &mesh.positions {
            positions.push(p.x);
            positions.push(p.y);
            positions.push(p.z);
        }
        let mut normals = Vec::with_capacity(mesh.normals.len() * 3);
        for n in &mesh.normals {
            normals.push(n.x);
            normals.push(n.y);
            normals.push(n.z);
        }
        let out = SurfaceMesh {
            positions,
            normals,
            indices: mesh.indices,
        };
        serde_wasm_bindgen::to_value(&out).map_err(js_err)
    }

    #[wasm_bindgen(js_name = intersectSurface)]
    pub fn intersect_surface(&self, other: &WasmNurbsSurface) -> Result<JsValue, JsValue> {
        let a = Surface::Nurbs(self.inner.clone());
        let b = Surface::Nurbs(other.inner.clone());
        let intersections = a.intersect_surface(&b).map_err(js_err)?;
        let out: Vec<WasmSurfaceSurfaceIntersection> = intersections
            .into_iter()
            .map(WasmSurfaceSurfaceIntersection::from)
            .collect();
        serde_wasm_bindgen::to_value(&out).map_err(js_err)
    }
}
