use std::error::Error;

use pyo3::create_exception;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::projection::ProjectionError as CoreError;

create_exception!(
    ngk.projection,
    ProjectionError,
    PyValueError,
    "A projection that has no single answer."
);
create_exception!(
    ngk.projection,
    MissError,
    ProjectionError,
    "Nothing lies where the projection looks."
);
create_exception!(
    ngk.projection,
    AmbiguousError,
    ProjectionError,
    "Several feet are equally near the point."
);
create_exception!(
    ngk.projection,
    GrazingRayError,
    ProjectionError,
    "The ray touches the surface without crossing it."
);
create_exception!(
    ngk.projection,
    RayInSurfaceError,
    ProjectionError,
    "The ray lies in the surface."
);
create_exception!(
    ngk.projection,
    UnsupportedSurfaceError,
    ProjectionError,
    "The projection has no closed form onto this kind of surface: `surface` names it."
);
create_exception!(
    ngk.projection,
    UnsupportedCurveError,
    ProjectionError,
    "The curve has no exact image the kernel can build: `curve` names its kind."
);

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    let py = module.py();
    module.add("ProjectionError", py.get_type_bound::<ProjectionError>())?;
    module.add("MissError", py.get_type_bound::<MissError>())?;
    module.add("AmbiguousError", py.get_type_bound::<AmbiguousError>())?;
    module.add("GrazingRayError", py.get_type_bound::<GrazingRayError>())?;
    module.add(
        "RayInSurfaceError",
        py.get_type_bound::<RayInSurfaceError>(),
    )?;
    module.add(
        "UnsupportedSurfaceError",
        py.get_type_bound::<UnsupportedSurfaceError>(),
    )?;
    module.add(
        "UnsupportedCurveError",
        py.get_type_bound::<UnsupportedCurveError>(),
    )?;
    Ok(())
}

/// An error's message followed by the messages of what caused it.
pub(crate) fn chain_message(error: &dyn Error) -> String {
    let mut message = error.to_string();
    let mut cause = error.source();
    while let Some(inner) = cause {
        message.push_str(": ");
        message.push_str(&inner.to_string());
        cause = inner.source();
    }
    message
}

/// The named exception for a refused projection.
pub(super) fn from_core(py: Python<'_>, error: CoreError) -> PyErr {
    let message = chain_message(&error);
    match error {
        CoreError::Miss => MissError::new_err(message),
        CoreError::Ambiguous => AmbiguousError::new_err(message),
        CoreError::GrazingRay => GrazingRayError::new_err(message),
        CoreError::RayInSurface => RayInSurfaceError::new_err(message),
        CoreError::UnsupportedSurface { surface } => with_attribute(
            py,
            UnsupportedSurfaceError::new_err(message),
            "surface",
            surface,
        ),
        CoreError::UnsupportedCurve { curve } => {
            with_attribute(py, UnsupportedCurveError::new_err(message), "curve", curve)
        }
        CoreError::Nurbs(_)
        | CoreError::Intersection(_)
        | CoreError::Trim(_)
        | CoreError::TrimCrossing(_)
        | CoreError::UnboundedSource
        | CoreError::DegenerateImage
        | CoreError::IncompleteCrossings => ProjectionError::new_err(message),
    }
}

/// An exception carrying the name of what was refused.
fn with_attribute(py: Python<'_>, error: PyErr, name: &str, value: &'static str) -> PyErr {
    match error.value_bound(py).setattr(name, value) {
        Ok(()) => error,
        Err(failure) => failure,
    }
}
