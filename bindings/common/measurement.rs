//! Language-neutral measurement values for topology handles.

use crate::geometry::{BBox, Frame};
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

/// Which bounding box a binding asks an entity for.
pub(crate) enum BoxRequest<'a> {
    /// Along the world axes when `None`, else along the frame's.
    Aligned(Option<&'a Frame>),
    Oriented,
}

pub(crate) fn edge_box(value: &SharedEdge, request: BoxRequest) -> Result<BBox, String> {
    let model = value.model();
    let edge = model.model().edge(value.key()).ok_or("missing edge")?;
    match request {
        BoxRequest::Aligned(None) => edge.bounding_box(),
        BoxRequest::Aligned(Some(frame)) => edge.bounding_box_in(frame),
        BoxRequest::Oriented => edge.oriented_bounding_box(),
    }
    .map_err(|error| error.to_string())
}

pub(crate) fn profile_box(value: &SharedProfile, request: BoxRequest) -> Result<BBox, String> {
    let model = value.model();
    let profile = model
        .model()
        .profile(value.key())
        .ok_or("missing profile")?;
    match request {
        BoxRequest::Aligned(None) => profile.bounding_box(),
        BoxRequest::Aligned(Some(frame)) => profile.bounding_box_in(frame),
        BoxRequest::Oriented => profile.oriented_bounding_box(),
    }
    .map_err(|error| error.to_string())
}

pub(crate) fn face_box(value: &SharedFace, request: BoxRequest) -> Result<BBox, String> {
    let model = value.model();
    let face = model.model().face(value.key()).ok_or("missing face")?;
    match request {
        BoxRequest::Aligned(None) => face.bounding_box(),
        BoxRequest::Aligned(Some(frame)) => face.bounding_box_in(frame),
        BoxRequest::Oriented => face.oriented_bounding_box(),
    }
    .map_err(|error| error.to_string())
}

pub(crate) fn sheet_box(value: &SharedSheet, request: BoxRequest) -> Result<BBox, String> {
    let model = value.model();
    let sheet = model.model().sheet(value.key()).ok_or("missing sheet")?;
    match request {
        BoxRequest::Aligned(None) => sheet.bounding_box(),
        BoxRequest::Aligned(Some(frame)) => sheet.bounding_box_in(frame),
        BoxRequest::Oriented => sheet.oriented_bounding_box(),
    }
    .map_err(|error| error.to_string())
}

pub(crate) fn solid_box(value: &SharedSolid, request: BoxRequest) -> Result<BBox, String> {
    let model = value.model();
    let solid = model.model().solid(value.key()).ok_or("missing solid")?;
    match request {
        BoxRequest::Aligned(None) => solid.bounding_box(),
        BoxRequest::Aligned(Some(frame)) => solid.bounding_box_in(frame),
        BoxRequest::Oriented => solid.oriented_bounding_box(),
    }
    .map_err(|error| error.to_string())
}
