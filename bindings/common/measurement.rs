//! Language-neutral measurement values for topology handles.

use crate::geometry::Point3;
use crate::measure::{Area, Inertia, Length, Volume};

use super::explore::{SharedEdge, SharedFace, SharedProfile, SharedSheet, SharedSolid};

pub(crate) struct MeasuredProperties {
    pub amount: f64,
    pub centroid: Point3,
    pub inertia: [f64; 9],
}

fn tensor<D>(inertia: Inertia<D>) -> [f64; 9] {
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

impl MeasuredProperties {
    fn linear(value: crate::measure::LinearProperties) -> Self {
        Self {
            amount: value.length,
            centroid: value.centroid,
            inertia: tensor::<Length>(value.inertia),
        }
    }

    fn surface(value: crate::measure::SurfaceProperties) -> Self {
        Self {
            amount: value.area,
            centroid: value.centroid,
            inertia: tensor::<Area>(value.inertia),
        }
    }

    fn volume(value: crate::measure::VolumeProperties) -> Self {
        Self {
            amount: value.volume,
            centroid: value.centroid,
            inertia: tensor::<Volume>(value.inertia),
        }
    }
}

pub(crate) fn edge(value: &SharedEdge) -> Result<MeasuredProperties, String> {
    value
        .model()
        .model()
        .edge(value.key())
        .ok_or_else(|| "missing edge".to_string())?
        .linear_properties()
        .map(MeasuredProperties::linear)
        .map_err(|error| error.to_string())
}

pub(crate) fn profile(value: &SharedProfile) -> Result<MeasuredProperties, String> {
    value
        .model()
        .model()
        .profile(value.key())
        .ok_or_else(|| "missing profile".to_string())?
        .linear_properties()
        .map(MeasuredProperties::linear)
        .map_err(|error| error.to_string())
}

pub(crate) fn face(value: &SharedFace) -> Result<MeasuredProperties, String> {
    value
        .model()
        .model()
        .face(value.key())
        .ok_or_else(|| "missing face".to_string())?
        .surface_properties()
        .map(MeasuredProperties::surface)
        .map_err(|error| error.to_string())
}

pub(crate) fn sheet(value: &SharedSheet) -> Result<MeasuredProperties, String> {
    value
        .model()
        .model()
        .sheet(value.key())
        .ok_or_else(|| "missing sheet".to_string())?
        .surface_properties()
        .map(MeasuredProperties::surface)
        .map_err(|error| error.to_string())
}

pub(crate) fn solid(value: &SharedSolid) -> Result<MeasuredProperties, String> {
    value
        .model()
        .model()
        .solid(value.key())
        .ok_or_else(|| "missing solid".to_string())?
        .volume_properties()
        .map(MeasuredProperties::volume)
        .map_err(|error| error.to_string())
}
