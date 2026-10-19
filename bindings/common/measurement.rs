//! Language-neutral measurement values for topology handles.

use crate::measure::{Inertia, LinearProperties, SurfaceProperties, VolumeProperties};

use super::explore::{SharedEdge, SharedFace, SharedProfile, SharedSheet, SharedSolid};

/// The centroidal inertia tensor, row by row.
pub(crate) fn tensor<D>(inertia: Inertia<D>) -> [f64; 9] {
    let matrix = inertia.tensor();
    [
        matrix[(0, 0)],
        matrix[(0, 1)],
        matrix[(0, 2)],
        matrix[(1, 0)],
        matrix[(1, 1)],
        matrix[(1, 2)],
        matrix[(2, 0)],
        matrix[(2, 1)],
        matrix[(2, 2)],
    ]
}

pub(crate) fn edge(value: &SharedEdge) -> Result<LinearProperties, String> {
    value
        .model()
        .model()
        .edge(value.key())
        .ok_or_else(|| "missing edge".to_string())?
        .linear_properties()
        .map_err(|error| error.to_string())
}

pub(crate) fn profile(value: &SharedProfile) -> Result<LinearProperties, String> {
    value
        .model()
        .model()
        .profile(value.key())
        .ok_or_else(|| "missing profile".to_string())?
        .linear_properties()
        .map_err(|error| error.to_string())
}

pub(crate) fn profile_length(value: &SharedProfile) -> Result<f64, String> {
    Ok(value
        .model()
        .model()
        .profile(value.key())
        .ok_or_else(|| "missing profile".to_string())?
        .length())
}

pub(crate) fn face(value: &SharedFace) -> Result<SurfaceProperties, String> {
    value
        .model()
        .model()
        .face(value.key())
        .ok_or_else(|| "missing face".to_string())?
        .surface_properties()
        .map_err(|error| error.to_string())
}

pub(crate) fn sheet(value: &SharedSheet) -> Result<SurfaceProperties, String> {
    value
        .model()
        .model()
        .sheet(value.key())
        .ok_or_else(|| "missing sheet".to_string())?
        .surface_properties()
        .map_err(|error| error.to_string())
}

pub(crate) fn solid_surface(value: &SharedSolid) -> Result<SurfaceProperties, String> {
    value
        .model()
        .model()
        .solid(value.key())
        .ok_or_else(|| "missing solid".to_string())?
        .surface_properties()
        .map_err(|error| error.to_string())
}

pub(crate) fn solid(value: &SharedSolid) -> Result<VolumeProperties, String> {
    value
        .model()
        .model()
        .solid(value.key())
        .ok_or_else(|| "missing solid".to_string())?
        .volume_properties()
        .map_err(|error| error.to_string())
}
