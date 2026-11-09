use pyo3::prelude::*;

use crate::geometry::Point3;
use crate::projection::{PointProjection, TargetParameters};

use super::super::geometry::{PyPoint2, PyPoint3, point};

/// A point carried onto a target.
#[pyclass(name = "PointProjection", module = "ngk.projection", frozen)]
pub(crate) struct PyPointProjection {
    point: Point3,
    parameters: Option<TargetParameters>,
    distance: f64,
    hit: PyObject,
}

impl PyPointProjection {
    /// A projection with the hit the target reported, already as a Python object.
    pub(super) fn new<Hit>(result: PointProjection<Hit>, hit: PyObject) -> Self {
        Self {
            point: result.point,
            parameters: result.parameters,
            distance: result.distance,
            hit,
        }
    }

    pub(super) fn projected_point(&self) -> PyPoint3 {
        point(self.point)
    }
}

#[pymethods]
impl PyPointProjection {
    /// The projected point, on the target.
    #[getter]
    fn point(&self) -> PyPoint3 {
        point(self.point)
    }

    /// The parameter on a curve, or the `(u, v)` on a surface, when computed.
    #[getter]
    fn parameters(&self, py: Python<'_>) -> Option<PyObject> {
        self.parameters.map(|parameters| match parameters {
            TargetParameters::Curve(parameter) => parameter.value().into_py(py),
            TargetParameters::Surface(uv) => PyPoint2::from(uv).into_py(py),
        })
    }

    /// The distance from the source to the projected point.
    #[getter]
    fn distance(&self) -> f64 {
        self.distance
    }

    /// The edge or face of the target the point landed on; `None` for pure geometry.
    #[getter]
    fn hit(&self, py: Python<'_>) -> PyObject {
        self.hit.clone_ref(py)
    }
}
