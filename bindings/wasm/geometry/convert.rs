use wasm_bindgen::prelude::*;

use crate::geometry::{Curve, Surface};

use super::curves::{WasmCircle, WasmLine};
use super::nurbs::{WasmNurbsCurve, WasmNurbsSurface};
use super::surfaces::{WasmCylinder, WasmPlane, WasmRuledSurface, WasmSurfaceOfRevolution};

fn js_err(error: impl ToString) -> JsValue {
    JsValue::from_str(&error.to_string())
}

/// Converts a polymorphic kernel curve to a concrete JavaScript class.
pub(crate) fn curve_to_js(curve: Curve) -> Result<JsValue, JsValue> {
    match curve {
        Curve::Line(line) => Ok(WasmLine { inner: line }.into()),
        Curve::Circle(circle) => Ok(WasmCircle { inner: circle }.into()),
        Curve::Nurbs(curve) => Ok(WasmNurbsCurve::from_inner(curve).into()),
        Curve::Bounded(curve) => {
            Ok(WasmNurbsCurve::from_inner(curve.to_nurbs().map_err(js_err)?).into())
        }
    }
}

/// Converts a polymorphic kernel surface to a concrete JavaScript class.
pub(crate) fn surface_to_js(surface: Surface) -> JsValue {
    match surface {
        Surface::Plane(inner) => WasmPlane { inner }.into(),
        Surface::Cylinder(inner) => WasmCylinder { inner }.into(),
        Surface::Ruled(inner) => WasmRuledSurface { inner }.into(),
        Surface::Revolution(inner) => WasmSurfaceOfRevolution { inner }.into(),
        Surface::Nurbs(inner) => WasmNurbsSurface::from_inner(inner).into(),
    }
}
