//! Read-only JavaScript views of face pcurves.
//!
//! A pcurve is a [`TrimmedCurve2`]: an unbounded analytic support plus the span
//! of it that is meant. Each class below exposes the support's own properties —
//! a circle's centre and radius — alongside the span, and every evaluation is
//! in the span's normalized traversal fraction, so `pointAt(0)` is the pcurve's
//! start whichever way its span runs.

use js_sys::{Array, Float64Array};
use wasm_bindgen::prelude::*;

use crate::geometry::Point2;
use crate::geometry::dim2::curves::{Circle2, Curve2, Ellipse2, Line2};
use crate::geometry::dim2::nurbs::NurbsCurve2;
use crate::geometry::dim2::trimmed::TrimmedCurve2;
use crate::geometry::parameter::Fraction;

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

/// Returns the span's native parameter interval as `[start, end]`.
fn interval_array(span: &TrimmedCurve2) -> Float64Array {
    let interval = span.interval();
    f64_array(&[interval.start.value(), interval.end.value()])
}

/// Read-only straight 2D pcurve: an infinite line plus the span meant.
#[wasm_bindgen(js_name = Line2)]
pub struct WasmLine2 {
    pub(crate) inner: TrimmedCurve2,
}

impl WasmLine2 {
    pub(crate) fn from_inner(inner: TrimmedCurve2) -> Self {
        Self { inner }
    }

    fn support(&self) -> &Line2 {
        match self.inner.curve() {
            Curve2::Line(line) => line,
            _ => unreachable!("WasmLine2 is only built from a Line2 support"),
        }
    }
}

#[wasm_bindgen]
impl WasmLine2 {
    /// Returns the curve kind.
    #[wasm_bindgen(getter)]
    pub fn kind(&self) -> String {
        "line".to_owned()
    }

    /// Returns the first point of the span.
    #[wasm_bindgen(getter)]
    pub fn start(&self) -> Float64Array {
        point_array(self.inner.start())
    }

    /// Returns the last point of the span.
    #[wasm_bindgen(getter)]
    pub fn end(&self) -> Float64Array {
        point_array(self.inner.end())
    }

    /// Returns a point the support passes through, span or no span.
    #[wasm_bindgen(getter)]
    pub fn origin(&self) -> Float64Array {
        point_array(self.support().origin())
    }

    /// Returns the span in the support's native parameters as `[start, end]`.
    #[wasm_bindgen(getter)]
    pub fn interval(&self) -> Float64Array {
        interval_array(&self.inner)
    }

    /// Evaluates at a normalized traversal fraction of the span.
    #[wasm_bindgen(js_name = pointAt)]
    pub fn point_at(&self, fraction: f64) -> Float64Array {
        point_array(self.inner.point_at(Fraction::new(fraction)))
    }

    /// Returns `segments + 1` uniform samples as a flattened `[u, v, ...]` array.
    pub fn sample(&self, segments: usize) -> Float64Array {
        points_array(&self.inner.sample(segments))
    }

    /// Returns the same geometry traversed in the opposite direction.
    pub fn reversed(&self) -> WasmLine2 {
        Self::from_inner(self.inner.reversed())
    }
}

/// Read-only circular 2D pcurve: a full circle plus the arc meant.
#[wasm_bindgen(js_name = Circle2)]
pub struct WasmCircle2 {
    pub(crate) inner: TrimmedCurve2,
}

impl WasmCircle2 {
    pub(crate) fn from_inner(inner: TrimmedCurve2) -> Self {
        Self { inner }
    }

    fn support(&self) -> &Circle2 {
        match self.inner.curve() {
            Curve2::Circle(circle) => circle,
            _ => unreachable!("WasmCircle2 is only built from a Circle2 support"),
        }
    }
}

#[wasm_bindgen]
impl WasmCircle2 {
    /// Returns the curve kind.
    #[wasm_bindgen(getter)]
    pub fn kind(&self) -> String {
        "circle".to_owned()
    }

    /// Returns the circle's center.
    #[wasm_bindgen(getter)]
    pub fn center(&self) -> Float64Array {
        point_array(self.support().center())
    }

    /// Returns the circle's radius.
    #[wasm_bindgen(getter)]
    pub fn radius(&self) -> f64 {
        self.support().radius()
    }

    /// Returns the signed angle the span sweeps, in radians.
    #[wasm_bindgen(getter)]
    pub fn sweep(&self) -> f64 {
        self.inner.interval().delta()
    }

    /// Returns the span in the support's native angles as `[start, end]`.
    #[wasm_bindgen(getter)]
    pub fn interval(&self) -> Float64Array {
        interval_array(&self.inner)
    }

    /// Evaluates at a normalized traversal fraction of the span.
    #[wasm_bindgen(js_name = pointAt)]
    pub fn point_at(&self, fraction: f64) -> Float64Array {
        point_array(self.inner.point_at(Fraction::new(fraction)))
    }

    /// Returns `segments + 1` uniform samples as a flattened `[u, v, ...]` array.
    pub fn sample(&self, segments: usize) -> Float64Array {
        points_array(&self.inner.sample(segments))
    }

    /// Returns the same arc traversed in the opposite direction.
    pub fn reversed(&self) -> WasmCircle2 {
        Self::from_inner(self.inner.reversed())
    }
}

/// Read-only elliptical 2D pcurve: a full ellipse plus the arc meant.
#[wasm_bindgen(js_name = Ellipse2)]
pub struct WasmEllipse2 {
    pub(crate) inner: TrimmedCurve2,
}

impl WasmEllipse2 {
    pub(crate) fn from_inner(inner: TrimmedCurve2) -> Self {
        Self { inner }
    }

    fn support(&self) -> &Ellipse2 {
        match self.inner.curve() {
            Curve2::Ellipse(ellipse) => ellipse,
            _ => unreachable!("WasmEllipse2 is only built from an Ellipse2 support"),
        }
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
        point_array(self.support().center())
    }

    #[wasm_bindgen(getter, js_name = majorRadius)]
    pub fn major_radius(&self) -> f64 {
        self.support().major_radius()
    }

    #[wasm_bindgen(getter, js_name = minorRadius)]
    pub fn minor_radius(&self) -> f64 {
        self.support().minor_radius()
    }

    /// Returns the signed angle the span sweeps, in radians.
    #[wasm_bindgen(getter)]
    pub fn sweep(&self) -> f64 {
        self.inner.interval().delta()
    }

    /// Returns the span in the support's native angles as `[start, end]`.
    #[wasm_bindgen(getter)]
    pub fn interval(&self) -> Float64Array {
        interval_array(&self.inner)
    }

    #[wasm_bindgen(js_name = pointAt)]
    pub fn point_at(&self, fraction: f64) -> Float64Array {
        point_array(self.inner.point_at(Fraction::new(fraction)))
    }

    pub fn sample(&self, segments: usize) -> Float64Array {
        points_array(&self.inner.sample(segments))
    }

    pub fn reversed(&self) -> WasmEllipse2 {
        Self::from_inner(self.inner.reversed())
    }
}

/// Read-only 2D NURBS pcurve: a NURBS support plus the span meant.
#[wasm_bindgen(js_name = NurbsCurve2)]
pub struct WasmNurbsCurve2 {
    pub(crate) inner: TrimmedCurve2,
}

impl WasmNurbsCurve2 {
    pub(crate) fn from_inner(inner: TrimmedCurve2) -> Self {
        Self { inner }
    }

    fn support(&self) -> &NurbsCurve2 {
        match self.inner.curve() {
            Curve2::Nurbs(curve) => curve,
            _ => unreachable!("WasmNurbsCurve2 is only built from a NurbsCurve2 support"),
        }
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
        self.support().degree().get()
    }

    /// Returns the native NURBS domain.
    #[wasm_bindgen(getter)]
    pub fn domain(&self) -> Float64Array {
        let domain = self.support().domain();
        f64_array(&[domain.start.value(), domain.end.value()])
    }

    /// Returns the span in the support's native parameters as `[start, end]`.
    #[wasm_bindgen(getter)]
    pub fn interval(&self) -> Float64Array {
        interval_array(&self.inner)
    }

    /// Returns the control-point weights.
    #[wasm_bindgen(getter)]
    pub fn weights(&self) -> Float64Array {
        let weights = self
            .support()
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
        for control_point in self.support().control_points().as_slice() {
            let pair = Array::new();
            pair.push(&point_array(control_point.to_cartesian()).into());
            pair.push(&JsValue::from_f64(control_point.weight()));
            values.push(&pair);
        }
        values
    }

    /// Evaluates at a normalized traversal fraction of the span.
    #[wasm_bindgen(js_name = pointAt)]
    pub fn point_at(&self, fraction: f64) -> Float64Array {
        point_array(self.inner.point_at(Fraction::new(fraction)))
    }

    /// Returns `segments + 1` uniform samples as a flattened `[u, v, ...]` array.
    pub fn sample(&self, segments: usize) -> Float64Array {
        points_array(&self.inner.sample(segments))
    }

    /// Returns the same curve traversed in the opposite direction.
    pub fn reversed(&self) -> WasmNurbsCurve2 {
        Self::from_inner(self.inner.reversed())
    }
}
