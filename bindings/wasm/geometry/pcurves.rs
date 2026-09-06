use js_sys::{Array, Float64Array};
use wasm_bindgen::prelude::*;

use crate::geometry::Point2;
use crate::geometry::dim2::curves::{Circle2, Curve2, Ellipse2, Line2};
use crate::geometry::dim2::nurbs::NurbsCurve2;

fn point_array(point: Point2) -> Float64Array {
    let out = Float64Array::new_with_length(2);
    out.copy_from(&[point.x, point.y]);
    out
}

fn points_array(points: &[Point2]) -> Float64Array {
    let mut flat = Vec::with_capacity(points.len() * 2);
    for point in points {
        flat.push(point.x);
        flat.push(point.y);
    }
    let out = Float64Array::new_with_length(flat.len() as u32);
    out.copy_from(&flat);
    out
}

fn f64_array(values: &[f64]) -> Float64Array {
    let out = Float64Array::new_with_length(values.len() as u32);
    out.copy_from(values);
    out
}

/// Read-only 2D line segment used to display face pcurves.
#[wasm_bindgen(js_name = Line2)]
pub struct WasmLine2 {
    pub(crate) inner: Line2,
}

impl WasmLine2 {
    pub(crate) fn from_inner(inner: Line2) -> Self {
        Self { inner }
    }
}

#[wasm_bindgen]
impl WasmLine2 {
    /// Returns the curve kind.
    #[wasm_bindgen(getter)]
    pub fn kind(&self) -> String {
        "line".to_owned()
    }

    /// Returns the start point.
    #[wasm_bindgen(getter)]
    pub fn start(&self) -> Float64Array {
        point_array(self.inner.start)
    }

    /// Returns the end point.
    #[wasm_bindgen(getter)]
    pub fn end(&self) -> Float64Array {
        point_array(self.inner.end)
    }

    /// Evaluates the segment.
    #[wasm_bindgen(js_name = pointAt)]
    pub fn point_at(&self, parameter: f64) -> Float64Array {
        point_array(self.inner.point_at(parameter))
    }

    /// Returns `segments + 1` uniform samples as a flattened `[u, v, ...]` array.
    pub fn sample(&self, segments: usize) -> Float64Array {
        points_array(&Curve2::Line(self.inner.clone()).sample(segments))
    }

    /// Returns the same segment in opposite direction.
    pub fn reversed(&self) -> WasmLine2 {
        Self::from_inner(self.inner.reversed())
    }
}

/// Read-only 2D circular arc used to display face pcurves.
#[wasm_bindgen(js_name = Circle2)]
pub struct WasmCircle2 {
    pub(crate) inner: Circle2,
}

impl WasmCircle2 {
    pub(crate) fn from_inner(inner: Circle2) -> Self {
        Self { inner }
    }
}

/// Read-only 2D elliptical arc used to display face pcurves.
#[wasm_bindgen(js_name = Ellipse2)]
pub struct WasmEllipse2 {
    pub(crate) inner: Ellipse2,
}

impl WasmEllipse2 {
    pub(crate) fn from_inner(inner: Ellipse2) -> Self {
        Self { inner }
    }
}

#[wasm_bindgen]
impl WasmEllipse2 {
    #[wasm_bindgen(getter)]
    pub fn kind(&self) -> String {
        "ellipse".to_owned()
    }

    #[wasm_bindgen(getter)]
    pub fn center(&self) -> Float64Array {
        point_array(self.inner.center())
    }

    #[wasm_bindgen(getter, js_name = majorRadius)]
    pub fn major_radius(&self) -> f64 {
        self.inner.major_radius()
    }

    #[wasm_bindgen(getter, js_name = minorRadius)]
    pub fn minor_radius(&self) -> f64 {
        self.inner.minor_radius()
    }

    #[wasm_bindgen(getter)]
    pub fn sweep(&self) -> f64 {
        self.inner.sweep()
    }

    #[wasm_bindgen(js_name = pointAt)]
    pub fn point_at(&self, parameter: f64) -> Float64Array {
        point_array(self.inner.point_at(parameter))
    }

    pub fn sample(&self, segments: usize) -> Float64Array {
        points_array(&Curve2::Ellipse(self.inner.clone()).sample(segments))
    }

    pub fn reversed(&self) -> WasmEllipse2 {
        Self::from_inner(self.inner.reversed())
    }
}

#[wasm_bindgen]
impl WasmCircle2 {
    /// Returns the curve kind.
    #[wasm_bindgen(getter)]
    pub fn kind(&self) -> String {
        "circle".to_owned()
    }

    /// Returns the arc center.
    #[wasm_bindgen(getter)]
    pub fn center(&self) -> Float64Array {
        point_array(self.inner.center())
    }

    /// Returns the arc radius.
    #[wasm_bindgen(getter)]
    pub fn radius(&self) -> f64 {
        self.inner.radius()
    }

    /// Returns the arc sweep angle in radians.
    #[wasm_bindgen(getter)]
    pub fn sweep(&self) -> f64 {
        self.inner.sweep()
    }

    /// Evaluates the arc.
    #[wasm_bindgen(js_name = pointAt)]
    pub fn point_at(&self, parameter: f64) -> Float64Array {
        point_array(self.inner.point_at(parameter))
    }

    /// Returns `segments + 1` uniform samples as a flattened `[u, v, ...]` array.
    pub fn sample(&self, segments: usize) -> Float64Array {
        points_array(&Curve2::Circle(self.inner.clone()).sample(segments))
    }

    /// Returns the same arc in opposite direction.
    pub fn reversed(&self) -> WasmCircle2 {
        Self::from_inner(self.inner.reversed())
    }
}

/// Read-only 2D NURBS curve used to display face pcurves.
#[wasm_bindgen(js_name = NurbsCurve2)]
pub struct WasmNurbsCurve2 {
    pub(crate) inner: NurbsCurve2,
}

impl WasmNurbsCurve2 {
    pub(crate) fn from_inner(inner: NurbsCurve2) -> Self {
        Self { inner }
    }
}

#[wasm_bindgen]
impl WasmNurbsCurve2 {
    /// Returns the curve kind.
    #[wasm_bindgen(getter)]
    pub fn kind(&self) -> String {
        "nurbs".to_owned()
    }

    /// Returns the curve degree.
    #[wasm_bindgen(getter)]
    pub fn degree(&self) -> usize {
        self.inner.degree().get()
    }

    /// Returns the native NURBS domain.
    #[wasm_bindgen(getter)]
    pub fn domain(&self) -> Float64Array {
        let domain = self.inner.domain();
        f64_array(&[domain.start, domain.end])
    }

    /// Returns the control-point weights.
    #[wasm_bindgen(getter)]
    pub fn weights(&self) -> Float64Array {
        let weights = self
            .inner
            .control_points()
            .as_slice()
            .iter()
            .map(|point| point.weight())
            .collect::<Vec<_>>();
        f64_array(&weights)
    }

    /// Returns the control polygon as `[Point2, weight]` pairs.
    #[wasm_bindgen(getter, js_name = controlPoints)]
    pub fn control_points(&self) -> Array {
        let values = Array::new();
        for control_point in self.inner.control_points().as_slice() {
            let pair = Array::new();
            pair.push(&point_array(control_point.to_cartesian()).into());
            pair.push(&JsValue::from_f64(control_point.weight()));
            values.push(&pair);
        }
        values
    }

    /// Evaluates the curve.
    #[wasm_bindgen(js_name = pointAt)]
    pub fn point_at(&self, parameter: f64) -> Float64Array {
        point_array(Curve2::Nurbs(self.inner.clone()).point_at(parameter))
    }

    /// Returns `segments + 1` uniform samples as a flattened `[u, v, ...]` array.
    pub fn sample(&self, segments: usize) -> Float64Array {
        let samples = Curve2::Nurbs(self.inner.clone()).sample(segments);
        points_array(&samples)
    }

    /// Returns the same curve with reversed direction.
    pub fn reversed(&self) -> WasmNurbsCurve2 {
        Self::from_inner(self.inner.reversed())
    }
}
