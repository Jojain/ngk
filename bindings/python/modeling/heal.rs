use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::healing::{HealingOptions, HealingReport};
use crate::modeling;

use super::super::topology::PySolid;
use super::common::py_solid;

#[pyclass(name = "HealingOptions", module = "ngk.modeling.heal")]
#[derive(Clone)]
pub(crate) struct PyHealingOptions {
    inner: HealingOptions,
}

#[pymethods]
impl PyHealingOptions {
    #[new]
    #[pyo3(signature = (remove_redundant_vertices=true, remove_redundant_edges=true, remove_seams=true, remove_filled_inner_loops=true, linear_tolerance=None, angular_tolerance=None, max_iterations=16))]
    fn new(
        remove_redundant_vertices: bool,
        remove_redundant_edges: bool,
        remove_seams: bool,
        remove_filled_inner_loops: bool,
        linear_tolerance: Option<f64>,
        angular_tolerance: Option<f64>,
        max_iterations: usize,
    ) -> Self {
        let defaults = HealingOptions::default();
        let inner = HealingOptions {
            remove_redundant_vertices,
            remove_redundant_edges,
            remove_seams,
            remove_filled_inner_loops,
            linear_tolerance: linear_tolerance.unwrap_or(defaults.linear_tolerance),
            angular_tolerance: angular_tolerance.unwrap_or(defaults.angular_tolerance),
            max_iterations,
            ..defaults
        };
        Self { inner }
    }

    #[staticmethod]
    fn seams_only() -> Self {
        Self {
            inner: HealingOptions::seams_only(),
        }
    }
}

#[pyclass(name = "HealingReport", module = "ngk.modeling.heal")]
#[derive(Clone)]
pub struct PyHealingReport {
    inner: HealingReport,
}

#[pymethods]
impl PyHealingReport {
    #[getter]
    fn changes(&self) -> usize {
        self.inner.changes()
    }
    #[getter]
    fn iterations(&self) -> usize {
        self.inner.iterations
    }
    #[getter]
    fn skipped(&self) -> Vec<String> {
        self.inner
            .skipped
            .iter()
            .map(|skip| format!("{:?}: {:?}", skip.cell, skip.reason))
            .collect()
    }
}

#[pyclass(name = "HealingResult", module = "ngk.modeling.heal")]
pub struct PyHealingResult {
    solid: PySolid,
    report: PyHealingReport,
}

#[pymethods]
impl PyHealingResult {
    #[getter]
    fn solid(&self) -> PySolid {
        self.solid.clone()
    }
    #[getter]
    fn report(&self) -> PyHealingReport {
        self.report.clone()
    }
}

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<PyHealingOptions>()?;
    module.add_class::<PyHealingReport>()?;
    module.add_class::<PyHealingResult>()?;
    module.add_function(wrap_pyfunction!(solid, module)?)?;
    Ok(())
}

#[pyfunction]
#[pyo3(signature = (shape, options=None))]
fn solid(shape: &PySolid, options: Option<&PyHealingOptions>) -> PyResult<PyHealingResult> {
    let shape = shape
        .inner
        .isolated_shape()
        .map_err(|error| PyValueError::new_err(error.to_string()))?;
    let healed = modeling::heal::solid(
        shape,
        options.map_or_else(HealingOptions::default, |value| value.inner.clone()),
    )
    .map_err(|error| PyValueError::new_err(error.to_string()))?;
    Ok(PyHealingResult {
        solid: py_solid(healed.shape)?,
        report: PyHealingReport {
            inner: healed.report,
        },
    })
}
