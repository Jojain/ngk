use wasm_bindgen::prelude::*;

use crate::geometry::{Curve, Curve2, Surface};

use super::curves::{WasmCircle, WasmEllipse, WasmLine};
use super::nurbs::{WasmNurbsCurve, WasmNurbsSurface};
use super::pcurves::{WasmCircle2, WasmEllipse2, WasmLine2, WasmNurbsCurve2};
use super::surfaces::{
    WasmCone, WasmCylinder, WasmPlane, WasmRuledSurface, WasmSphere, WasmSurfaceOfRevolution,
};

fn js_err(error: impl ToString) -> JsValue {
    JsValue::from_str(&error.to_string())
}

/// Converts a polymorphic kernel curve to a concrete JavaScript class.
pub(crate) fn curve_to_js(curve: Curve) -> Result<JsValue, JsValue> {
    match curve {
        Curve::Line(line) => Ok(WasmLine { inner: line }.into()),
        Curve::Circle(circle) => Ok(WasmCircle { inner: circle }.into()),
        Curve::Ellipse(ellipse) => Ok(WasmEllipse { inner: ellipse }.into()),
        Curve::Nurbs(curve) => Ok(WasmNurbsCurve::from_inner(curve).into()),
        Curve::Bounded(curve) => {
            Ok(WasmNurbsCurve::from_inner(curve.to_nurbs().map_err(js_err)?).into())
        }
    }
}

/// Converts a polymorphic 2D curve to its concrete JavaScript class.
pub(crate) fn curve2_to_js(curve: Curve2) -> JsValue {
    match curve {
        Curve2::Line(line) => WasmLine2::from_inner(line).into(),
        Curve2::Circle(circle) => WasmCircle2::from_inner(circle).into(),
        Curve2::Ellipse(ellipse) => WasmEllipse2::from_inner(ellipse).into(),
        Curve2::Nurbs(curve) => WasmNurbsCurve2::from_inner(curve).into(),
    }
}

/// Converts a polymorphic kernel surface to a concrete JavaScript class.
pub(crate) fn surface_to_js(surface: Surface) -> JsValue {
    match surface {
        Surface::Plane(inner) => WasmPlane { inner }.into(),
        Surface::Cylinder(inner) => WasmCylinder { inner }.into(),
        Surface::Sphere(inner) => WasmSphere { inner }.into(),
        Surface::Cone(inner) => WasmCone { inner }.into(),
        Surface::Ruled(inner) => WasmRuledSurface { inner }.into(),
        Surface::Revolution(inner) => WasmSurfaceOfRevolution { inner }.into(),
        Surface::Nurbs(inner) => WasmNurbsSurface::from_inner(inner).into(),
    }
}
