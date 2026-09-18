use wasm_bindgen::prelude::*;

use crate::builders::loft::LoftOptions;
use crate::geometry::Degree;
use crate::modeling;

use super::super::topology::{WasmFace, WasmProfile, WasmSheet, WasmSolid};
use super::common::{js_err, wasm_sheet, wasm_solid};

/// Skins an ordered sequence of profiles into one sheet.
///
/// `vDegree` is the degree across the sections; `0` leaves it to the loft,
/// which takes the highest the section count supports up to a cubic. `1`
/// gives a ruled loft, straight between consecutive sections.
#[wasm_bindgen(js_name = loftProfiles)]
pub fn loft_profiles(sections: Vec<WasmProfile>, v_degree: usize) -> Result<WasmSheet, JsValue> {
    let shapes = sections
        .iter()
        .map(|section| section.isolated_shape())
        .collect::<Result<Vec<_>, _>>()?;
    let references = shapes.iter().collect::<Vec<_>>();
    modeling::loft::loft(&references, loft_options(v_degree)?)
        .map_err(js_err)
        .and_then(wasm_sheet)
}

/// Skins an ordered sequence of faces into one solid, capped at both ends.
#[wasm_bindgen(js_name = loftFaces)]
pub fn loft_faces(sections: Vec<WasmFace>, v_degree: usize) -> Result<WasmSolid, JsValue> {
    let shapes = sections
        .iter()
        .map(|section| section.isolated_shape())
        .collect::<Result<Vec<_>, _>>()?;
    let references = shapes.iter().collect::<Vec<_>>();
    modeling::loft::loft(&references, loft_options(v_degree)?)
        .map_err(js_err)
        .and_then(wasm_solid)
}

/// Reads the degree across the sections, `0` leaving it to the loft.
fn loft_options(v_degree: usize) -> Result<LoftOptions, JsValue> {
    let v_degree = match v_degree {
        0 => None,
        degree => Some(Degree::new(degree).map_err(js_err)?),
    };
    Ok(LoftOptions { v_degree })
}
