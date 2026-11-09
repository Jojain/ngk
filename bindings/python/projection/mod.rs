//! Projection: `ngk.projection`.

mod curve;
mod dispatch;
mod errors;
mod point;

use pyo3::exceptions::PyTypeError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use super::geometry::{PyPoint3, PyVector3};
use super::modeling::{EXTENSION_QUALNAME, new_submodule};
use dispatch::{Outcome, Source, options, run};

pub(crate) use curve::PyCurveProjection;
pub(crate) use dispatch::Target;
pub(crate) use errors::chain_message;

pub(super) fn register(parent: &Bound<'_, PyModule>) -> PyResult<()> {
    let py = parent.py();
    let projection = new_submodule(py, "projection", EXTENSION_QUALNAME)?;

    projection.add_function(wrap_pyfunction!(project, &projection)?)?;
    projection.add_function(wrap_pyfunction!(project_point, &projection)?)?;
    projection.add_class::<point::PyPointProjection>()?;
    projection.add_class::<curve::PyCurveProjection>()?;
    projection.add_class::<curve::PyProjectedChain>()?;
    projection.add_class::<curve::PyProjectedSegment>()?;
    projection.add_class::<curve::PySourceSpan>()?;
    projection.add_class::<curve::PyChainEnd>()?;
    projection.add_class::<curve::PyExactness>()?;
    projection.add_class::<curve::PyTrimmedCurve>()?;
    projection.add_class::<curve::PyTrimmedCurve2>()?;
    errors::register(&projection)?;

    parent.add_submodule(&projection)?;
    Ok(())
}

/// Projects `source` onto `target`.
///
/// A point projects onto a plane, a surface, a curve, an axis, an edge, a face or a
/// list of faces and gives a `PointProjection`. A curve, an edge or a profile projects onto
/// a plane, a face or a list of faces and gives a `CurveProjection`.
///
/// Without `along` the projection is the perpendicular; with it, the first hit of
/// the half-line from the source in that direction. A curve, an axis or an edge has only
/// the perpendicular, so `along` is refused for them. `tolerance` is the distance
/// within which two points coincide.
#[pyfunction]
#[pyo3(signature = (source, target, along=None, tolerance=None))]
fn project(
    py: Python<'_>,
    source: &Bound<'_, PyAny>,
    target: &Bound<'_, PyAny>,
    along: Option<PyVector3>,
    tolerance: Option<f64>,
) -> PyResult<PyObject> {
    project_onto(py, source, &Target::extract(target)?, &along, tolerance)
}

/// Projects `source` onto an extracted `target`: the path `ngk.projection.project`
/// and the `project` method of every target class share.
pub(crate) fn project_onto(
    py: Python<'_>,
    source: &Bound<'_, PyAny>,
    target: &Target,
    along: &Option<PyVector3>,
    tolerance: Option<f64>,
) -> PyResult<PyObject> {
    let source = Source::extract(source)?;
    let options = options(tolerance)?;
    match run(py, &source, target, along, &options)? {
        Outcome::Point(projection) => Ok(Py::new(py, projection)?.into_py(py)),
        Outcome::Curve(projection) => Ok(Py::new(py, projection)?.into_py(py)),
    }
}

/// Projects a point onto `target` and keeps only the projected point.
#[pyfunction]
#[pyo3(signature = (source, target, along=None, tolerance=None))]
fn project_point(
    py: Python<'_>,
    source: &Bound<'_, PyAny>,
    target: &Bound<'_, PyAny>,
    along: Option<PyVector3>,
    tolerance: Option<f64>,
) -> PyResult<PyPoint3> {
    let source = Source::extract(source)?;
    if !matches!(source, Source::Point(_)) {
        return Err(PyTypeError::new_err(
            "project_point takes a point; use project for a curve, an edge or a profile",
        ));
    }
    let target = Target::extract(target)?;
    let options = options(tolerance)?;
    match run(py, &source, &target, &along, &options)? {
        Outcome::Point(projection) => Ok(projection.projected_point()),
        Outcome::Curve(_) => unreachable!("a point projects to a point"),
    }
}
