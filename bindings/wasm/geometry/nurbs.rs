use js_sys::{Array, Float64Array};
use serde::Serialize;
use wasm_bindgen::prelude::*;

use crate::geometry::{
    ControlNet, ControlPolygon, Curve, Curve2, CurveCurveIntersection, CurveSurfaceIntersection,
    Degree, IntersectionCoverage, KnotVector, NurbsCurve, NurbsSurface, Point3, Surface,
    SurfaceSurfaceIntersection, sample_curve_uniform, tessellate_curve_adaptive,
    tessellate_surface_grid,
};

use super::values::{WasmPoint3, WasmVector3, point, unit_vector};

fn js_err(e: impl ToString) -> JsValue {
    JsValue::from_str(&e.to_string())
}

fn points_from_flat(xyz: &[f64]) -> Result<Vec<Point3>, JsValue> {
    if !xyz.len().is_multiple_of(3) {
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

impl WasmNurbsCurve {
    pub(crate) fn from_inner(inner: NurbsCurve) -> Self {
        Self { inner }
    }
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

    /// Evaluates the curve at the supplied parameter.
    #[wasm_bindgen(js_name = pointAt)]
    pub fn point_at(&self, u: f64) -> WasmPoint3 {
        point(self.inner.point_at(u))
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

    /// Inserts a knot while preserving the represented curve.
    #[wasm_bindgen(js_name = insertKnot)]
    pub fn insert_knot(&mut self, u: f64) {
        self.inner.insert_knot(u);
    }

    /// Returns the knot vector.
    #[wasm_bindgen(getter)]
    pub fn knots(&self) -> Float64Array {
        flat_from_f64(self.inner.knots().as_slice())
    }

    /// Returns the curve degree.
    #[wasm_bindgen(getter)]
    pub fn degree(&self) -> usize {
        self.inner.degree().get()
    }

    /// Returns the Cartesian control points as a flat xyz array.
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

    /// Returns `[Point3, weight]` pairs for the control polygon.
    #[wasm_bindgen(getter, js_name = controlPoints)]
    pub fn control_points(&self) -> Array {
        let values = Array::new();
        for control_point in self.inner.control_points().iter() {
            let pair = Array::new();
            pair.push(&point(control_point.to_cartesian()).into());
            pair.push(&JsValue::from_f64(control_point.weight()));
            values.push(&pair);
        }
        values
    }

    /// Returns the control-point weights.
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

    /// Returns the parameter domain as `[start, end]`.
    #[wasm_bindgen(getter)]
    pub fn domain(&self) -> Float64Array {
        let domain = self.inner.domain();
        flat_from_f64(&[domain.start, domain.end])
    }

    /// Intersects this curve with another NURBS curve.
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

    /// Intersects this curve with a NURBS surface.
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
        surface_a_uv: [f64; 2],
        surface_b_uv: [f64; 2],
        contact_kind: String,
        residual: f64,
    },
    #[serde(rename = "branch")]
    Branch {
        points: Vec<[f64; 3]>,
        surface_a_parameters: Vec<[f64; 2]>,
        surface_b_parameters: Vec<[f64; 2]>,
        curve_representation: &'static str,
        surface_a_curve_representation: &'static str,
        surface_b_curve_representation: &'static str,
        closed: bool,
        contact_kind: String,
        max_residual: f64,
        max_fit_error: f64,
        certified: bool,
    },
    #[serde(rename = "overlapCandidate")]
    OverlapCandidate {
        surface_a_u_domain: [f64; 2],
        surface_a_v_domain: [f64; 2],
        surface_b_u_domain: [f64; 2],
        surface_b_v_domain: [f64; 2],
    },
}

impl From<SurfaceSurfaceIntersection> for WasmSurfaceSurfaceIntersection {
    fn from(value: SurfaceSurfaceIntersection) -> Self {
        match value {
            SurfaceSurfaceIntersection::Point(point) => Self::Point {
                point: [point.point.x, point.point.y, point.point.z],
                surface_a_uv: [point.uv_a.x, point.uv_a.y],
                surface_b_uv: [point.uv_b.x, point.uv_b.y],
                contact_kind: format!("{:?}", point.kind),
                residual: point.residual,
            },
            SurfaceSurfaceIntersection::Branch(branch) => {
                let curve_representation = curve_representation(&branch.curve_3d);
                let surface_a_curve_representation = pcurve_representation(&branch.pcurve_a);
                let surface_b_curve_representation = pcurve_representation(&branch.pcurve_b);
                Self::Branch {
                    points: branch
                        .samples
                        .iter()
                        .map(|sample| [sample.point.x, sample.point.y, sample.point.z])
                        .collect(),
                    surface_a_parameters: branch
                        .samples
                        .iter()
                        .map(|sample| [sample.uv_a.x, sample.uv_a.y])
                        .collect(),
                    surface_b_parameters: branch
                        .samples
                        .iter()
                        .map(|sample| [sample.uv_b.x, sample.uv_b.y])
                        .collect(),
                    curve_representation,
                    surface_a_curve_representation,
                    surface_b_curve_representation,
                    closed: branch.closed,
                    contact_kind: format!("{:?}", branch.kind),
                    max_residual: branch.quality.max_residual,
                    max_fit_error: branch.quality.max_fit_error,
                    certified: branch.quality.certified,
                }
            }
            SurfaceSurfaceIntersection::OverlapCandidate(candidate) => Self::OverlapCandidate {
                surface_a_u_domain: [candidate.domain_a_u.start, candidate.domain_a_u.end],
                surface_a_v_domain: [candidate.domain_a_v.start, candidate.domain_a_v.end],
                surface_b_u_domain: [candidate.domain_b_u.start, candidate.domain_b_u.end],
                surface_b_v_domain: [candidate.domain_b_v.start, candidate.domain_b_v.end],
            },
        }
    }
}

fn curve_representation(curve: &Curve) -> &'static str {
    match curve {
        Curve::Line(_) => "line",
        Curve::Circle(_) => "circle",
        Curve::Nurbs(_) => "nurbs",
        Curve::Bounded(curve) => curve_representation(curve.inner()),
    }
}

fn pcurve_representation(curve: &Curve2) -> &'static str {
    match curve {
        Curve2::Line(_) => "line",
        Curve2::Circle(_) => "circle",
        Curve2::Nurbs(_) => "nurbs",
    }
}

#[derive(Serialize)]
struct WasmSurfaceSurfaceIntersections {
    intersections: Vec<WasmSurfaceSurfaceIntersection>,
    coverage: &'static str,
    incomplete_reasons: Vec<String>,
}

#[wasm_bindgen(js_name = NurbsSurface)]
pub struct WasmNurbsSurface {
    inner: NurbsSurface,
}

impl WasmNurbsSurface {
    pub(crate) fn from_inner(inner: NurbsSurface) -> Self {
        Self { inner }
    }
}

#[wasm_bindgen]
impl WasmNurbsSurface {
    /// Builds a NURBS surface from flat Cartesian control points and knot vectors.
    #[allow(clippy::too_many_arguments)]
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

    /// Builds a NURBS surface with clamped-uniform knot vectors on `[0, 1]`.
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

    /// Evaluates the surface at the supplied parameters.
    #[wasm_bindgen(js_name = pointAt)]
    pub fn point_at(&self, u: f64, v: f64) -> WasmPoint3 {
        point(self.inner.point_at(u, v))
    }

    /// Returns the degree in the u direction.
    #[wasm_bindgen(getter, js_name = degreeU)]
    pub fn degree_u(&self) -> usize {
        self.inner.degree_u().get()
    }

    /// Returns the degree in the v direction.
    #[wasm_bindgen(getter, js_name = degreeV)]
    pub fn degree_v(&self) -> usize {
        self.inner.degree_v().get()
    }

    /// Returns the u parameter domain as `[start, end]`.
    #[wasm_bindgen(getter, js_name = domainU)]
    pub fn domain_u(&self) -> Float64Array {
        let domain = self.inner.domain_u();
        flat_from_f64(&[domain.start, domain.end])
    }

    /// Returns the v parameter domain as `[start, end]`.
    #[wasm_bindgen(getter, js_name = domainV)]
    pub fn domain_v(&self) -> Float64Array {
        let domain = self.inner.domain_v();
        flat_from_f64(&[domain.start, domain.end])
    }

    /// Returns the u knot vector.
    #[wasm_bindgen(getter, js_name = knotsU)]
    pub fn knots_u(&self) -> Float64Array {
        flat_from_f64(self.inner.knots_u().as_slice())
    }

    /// Returns the v knot vector.
    #[wasm_bindgen(getter, js_name = knotsV)]
    pub fn knots_v(&self) -> Float64Array {
        flat_from_f64(self.inner.knots_v().as_slice())
    }

    /// Returns the weighted control net as rows of `[Point3, weight]` pairs.
    #[wasm_bindgen(getter, js_name = controlPoints)]
    pub fn control_points(&self) -> Array {
        let points = self.inner.control_points();
        let rows = Array::new();
        for v in 0..points.nv() {
            let row = Array::new();
            for u in 0..points.nu() {
                let control_point = points.get(u, v);
                let pair = Array::new();
                pair.push(&point(control_point.to_cartesian()).into());
                pair.push(&JsValue::from_f64(control_point.weight()));
                row.push(&pair);
            }
            rows.push(&row);
        }
        rows
    }

    /// Evaluates the surface normal.
    #[wasm_bindgen(js_name = normalAt)]
    pub fn normal_at(&self, u: f64, v: f64) -> WasmVector3 {
        unit_vector(self.inner.normal_at(u, v))
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

    /// Intersects this surface with another NURBS surface.
    #[wasm_bindgen(js_name = intersectSurface)]
    pub fn intersect_surface(&self, other: &WasmNurbsSurface) -> Result<JsValue, JsValue> {
        let a = Surface::Nurbs(self.inner.clone());
        let b = Surface::Nurbs(other.inner.clone());
        let intersections = a.intersect_surface(&b).map_err(js_err)?;
        let out = intersections
            .intersections()
            .iter()
            .cloned()
            .map(WasmSurfaceSurfaceIntersection::from)
            .collect();
        let (coverage, incomplete_reasons) = match intersections.coverage() {
            IntersectionCoverage::Complete => ("complete", Vec::new()),
            IntersectionCoverage::Incomplete(reasons) => (
                "incomplete",
                reasons.iter().map(|reason| format!("{reason:?}")).collect(),
            ),
        };
        let out = WasmSurfaceSurfaceIntersections {
            intersections: out,
            coverage,
            incomplete_reasons,
        };
        serde_wasm_bindgen::to_value(&out).map_err(js_err)
    }
}
