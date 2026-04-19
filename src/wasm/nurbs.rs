use js_sys::Float64Array;
use serde::Serialize;
use wasm_bindgen::prelude::*;

use crate::geometry::nurbs::{
    ControlNet, ControlPolygon, Degree, KnotVector, NurbsCurve, NurbsSurface,
};
use crate::geometry::nurbs::tessellate::{
    sample_curve_uniform, tessellate_curve_adaptive, tessellate_surface_grid,
};
use crate::geometry::utils::Point3;

fn js_err(e: impl ToString) -> JsValue {
    JsValue::from_str(&e.to_string())
}

fn points_from_flat(xyz: &[f64]) -> Result<Vec<Point3>, JsValue> {
    if xyz.len() % 3 != 0 {
        return Err(JsValue::from_str("xyz array length must be a multiple of 3"));
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
        flat_from_points(&tessellate_curve_adaptive(&self.inner, tolerance, max_depth))
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
        let (a, b) = self.inner.domain();
        flat_from_f64(&[a, b])
    }
}

#[derive(Serialize)]
struct SurfaceMesh {
    positions: Vec<f64>,
    normals: Vec<f64>,
    indices: Vec<u32>,
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
        serde_wasm_bindgen::to_value(&out).map_err(|e| js_err(e))
    }
}
