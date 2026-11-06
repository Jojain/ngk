use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::geometry;

use super::super::geometry::{PyPoint2, PyVector2};

/// A directed segment.
#[pyclass(module = "ngk.solver", frozen)]
#[derive(Clone)]
pub(crate) struct Segment2(pub(crate) geometry::Segment2);

#[pymethods]
impl Segment2 {
    #[new]
    fn new(start: PyPoint2, end: PyPoint2) -> Self {
        Self(geometry::Segment2::new(start.point(), end.point()))
    }

    #[getter]
    fn start(&self) -> PyPoint2 {
        self.0.start().into()
    }

    #[getter]
    fn end(&self) -> PyPoint2 {
        self.0.end().into()
    }

    #[getter]
    fn length(&self) -> f64 {
        self.0.length()
    }

    fn __repr__(&self) -> String {
        let (start, end) = (self.0.start(), self.0.end());
        format!(
            "Segment2(Point2({}, {}), Point2({}, {}))",
            start.x, start.y, end.x, end.y
        )
    }
}

/// A directed circular arc: it starts along `x_dir` and `sweep` radians give its extent
/// and, by their sign, its direction.
#[pyclass(module = "ngk.solver", frozen)]
#[derive(Clone)]
pub(crate) struct Arc2(pub(crate) geometry::Arc2);

#[pymethods]
impl Arc2 {
    #[new]
    fn new(center: PyPoint2, x_dir: PyVector2, radius: f64, sweep: f64) -> Self {
        Self(geometry::Arc2::new(
            center.point(),
            x_dir.vector(),
            radius,
            sweep,
        ))
    }

    #[getter]
    fn center(&self) -> PyPoint2 {
        self.0.curve().center().into()
    }

    #[getter]
    fn radius(&self) -> f64 {
        self.0.curve().radius()
    }

    #[getter]
    fn sweep(&self) -> f64 {
        self.0.interval().delta()
    }

    #[getter]
    fn start(&self) -> PyPoint2 {
        self.0.start().into()
    }

    #[getter]
    fn end(&self) -> PyPoint2 {
        self.0.end().into()
    }

    #[getter]
    fn length(&self) -> f64 {
        self.0.length()
    }
}

/// A full circle.
#[pyclass(module = "ngk.solver", frozen)]
#[derive(Clone)]
pub(crate) struct Circle2(pub(crate) geometry::Circle2);

#[pymethods]
impl Circle2 {
    #[new]
    fn new(center: PyPoint2, radius: f64) -> Self {
        Self(geometry::Circle2::new(
            center.point(),
            nalgebra::Vector2::x(),
            radius,
        ))
    }

    #[getter]
    fn center(&self) -> PyPoint2 {
        self.0.center().into()
    }

    #[getter]
    fn radius(&self) -> f64 {
        self.0.radius()
    }
}

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<Segment2>()?;
    module.add_class::<Arc2>()?;
    module.add_class::<Circle2>()?;
    Ok(())
}
