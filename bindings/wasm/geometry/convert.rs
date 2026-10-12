use wasm_bindgen::prelude::*;

use crate::geometry::{Curve, Curve2, Surface, TrimmedCurve2};

use super::curves::{WasmCircle, WasmEllipse, WasmHelix, WasmLine};
use super::nurbs::{WasmNurbsCurve, WasmNurbsSurface};
use super::pcurves::{WasmCircle2, WasmEllipse2, WasmLine2, WasmNurbsCurve2};
use super::surfaces::{
    WasmCone, WasmCylinder, WasmPlane, WasmRuledSurface, WasmSphere, WasmSurfaceOfRevolution,
    WasmTorus,
};

// The unions name exactly the classes the converters below can return.
#[wasm_bindgen(typescript_custom_section)]
const GEOMETRY_UNIONS: &'static str = r#"
export type Curve = Line | Circle | Ellipse | Helix | NurbsCurve;
export type Curve2 = Line2 | Circle2 | Ellipse2 | NurbsCurve2;
export type Surface = Plane | Cylinder | Sphere | Cone | Torus | RuledSurface | SurfaceOfRevolution | NurbsSurface;
"#;

/// Converts a polymorphic kernel curve to a concrete JavaScript class.
pub(crate) fn curve_to_js(curve: Curve) -> Result<JsValue, JsValue> {
    match curve {
        Curve::Line(line) => Ok(WasmLine { inner: line }.into()),
        Curve::Circle(circle) => Ok(WasmCircle { inner: circle }.into()),
        Curve::Ellipse(ellipse) => Ok(WasmEllipse { inner: ellipse }.into()),
        Curve::Helix(helix) => Ok(WasmHelix { inner: helix }.into()),
        Curve::Nurbs(curve) => Ok(WasmNurbsCurve::from_inner(curve).into()),
    }
}

/// Converts a 2D span to the JavaScript class of the support it rests on.
pub(crate) fn curve2_to_js(span: TrimmedCurve2) -> JsValue {
    match span.curve() {
        Curve2::Line(_) => WasmLine2::from_inner(span).into(),
        Curve2::Circle(_) => WasmCircle2::from_inner(span).into(),
        Curve2::Ellipse(_) => WasmEllipse2::from_inner(span).into(),
        Curve2::Nurbs(_) => WasmNurbsCurve2::from_inner(span).into(),
    }
}

/// Converts a polymorphic kernel surface to a concrete JavaScript class.
pub(crate) fn surface_to_js(surface: Surface) -> JsValue {
    match surface {
        Surface::Plane(inner) => WasmPlane { inner }.into(),
        Surface::Cylinder(inner) => WasmCylinder { inner }.into(),
        Surface::Sphere(inner) => WasmSphere { inner }.into(),
        Surface::Cone(inner) => WasmCone { inner }.into(),
        Surface::Torus(inner) => WasmTorus { inner }.into(),
        Surface::Ruled(inner) => WasmRuledSurface { inner }.into(),
        Surface::Revolution(inner) => WasmSurfaceOfRevolution { inner }.into(),
        Surface::Nurbs(inner) => WasmNurbsSurface::from_inner(inner).into(),
    }
}
