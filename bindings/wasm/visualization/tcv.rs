use js_sys::{Object, Reflect};
use nalgebra::Vector3;
use wasm_bindgen::prelude::*;

use crate::geometry::{Curve, Plane, Point3, Surface};
use crate::modeling::solids;
use crate::viz::{
    ScriptResult, Style, VizHints, VizScene, scene_from_curve, scene_from_plane, scene_from_point,
    scene_from_surface, scene_from_vector,
};

use super::super::geometry::{WasmPlane, curve_to_js, point, surface_to_js, vector};
use super::super::topology::WasmGMap;

fn js_err(error: impl ToString) -> JsValue {
    JsValue::from_str(&error.to_string())
}

/// Tessellates a deserialized browser-owned GMap for the debug viewer.
#[wasm_bindgen(js_name = sceneFromGMap)]
pub fn scene_from_gmap(gmap: &WasmGMap) -> Result<JsValue, JsValue> {
    let scene = crate::viz::scene_from_gmap(gmap.inner.map(), &VizHints::new());
    serde_wasm_bindgen::to_value(&scene).map_err(js_err)
}

/// Restores a serialized kernel geometry value and its finite debug scene.
#[wasm_bindgen(js_name = hydrateDebugGeometry)]
pub fn hydrate_debug_geometry(kind: &str, serialized: &str) -> Result<JsValue, JsValue> {
    let (value, scene): (JsValue, VizScene) = match kind {
        "point" => {
            let value: Point3 = serde_json::from_str(serialized).map_err(js_err)?;
            let scene = scene_from_point(&value);
            (point(value).into(), scene)
        }
        "vector" => {
            let value: Vector3<f64> = serde_json::from_str(serialized).map_err(js_err)?;
            let scene = scene_from_vector(&value);
            (vector(value).into(), scene)
        }
        "plane" => {
            let value: Plane = serde_json::from_str(serialized).map_err(js_err)?;
            let scene = scene_from_plane(&value);
            (WasmPlane { inner: value }.into(), scene)
        }
        "curve" => {
            let value: Curve = serde_json::from_str(serialized).map_err(js_err)?;
            let scene = scene_from_curve(&value);
            (curve_to_js(value)?, scene)
        }
        "surface" => {
            let value: Surface = serde_json::from_str(serialized).map_err(js_err)?;
            let scene = scene_from_surface(&value);
            (surface_to_js(value), scene)
        }
        _ => return Err(js_err(format!("unsupported debug geometry kind: {kind}"))),
    };

    hydrated_geometry(value, scene)
}

fn hydrated_geometry(value: JsValue, scene: VizScene) -> Result<JsValue, JsValue> {
    let result = Object::new();
    Reflect::set(result.as_ref(), &JsValue::from_str("value"), &value)?;
    Reflect::set(
        result.as_ref(),
        &JsValue::from_str("scene"),
        &serde_wasm_bindgen::to_value(&scene).map_err(js_err)?,
    )?;
    Ok(result.into())
}

/// Builds the visualization-only block scene used by the frontend experiment.
#[wasm_bindgen(js_name = blockScene)]
pub fn block_scene(x_size: f64, y_size: f64, z_size: f64) -> Result<JsValue, JsValue> {
    let shape = solids::block(x_size, y_size, z_size).map_err(js_err)?;

    let mut hints = VizHints::new();
    for (key, _) in shape.map().iter_faces() {
        hints.face(
            key,
            Style::default()
                .color("#5aa9e6")
                .opacity(0.78)
                .double_sided(true),
        );
    }

    let result = ScriptResult::from_gmap_with_hints(shape.map(), &hints);
    serde_wasm_bindgen::to_value(&result).map_err(js_err)
}
