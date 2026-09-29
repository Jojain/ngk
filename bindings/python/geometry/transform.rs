use pyo3::prelude::*;
use radians::Rad64;

use crate::geometry::Rigid;

use super::{PyAxis3, PyFrame, PyPoint3, PyVector3, point, vector};

/// A rigid motion that preserves distances and orientation.
#[pyclass(name = "Rigid", module = "ngk.geometry")]
#[derive(Clone)]
pub(crate) struct PyRigid {
    pub(crate) inner: Rigid,
}

#[pymethods]
impl PyRigid {
    #[staticmethod]
    fn identity() -> Self {
        Self {
            inner: Rigid::identity(),
        }
    }

    #[staticmethod]
    fn translation(offset: &PyVector3) -> Self {
        Self {
            inner: Rigid::translation(offset.vector),
        }
    }

    #[staticmethod]
    fn rotation(axis: &PyAxis3, angle: f64) -> Self {
        Self {
            inner: Rigid::rotation(axis.axis, Rad64::new(angle)),
        }
    }

    #[staticmethod]
    fn between_frames(from: &PyFrame, to: &PyFrame) -> Self {
        Self {
            inner: Rigid::between_frames(&from.frame, &to.frame),
        }
    }

    /// Applies this motion first and `other` second.
    fn compose(&self, other: &Self) -> Self {
        Self {
            inner: self.inner.compose(other.inner),
        }
    }

    fn inverse(&self) -> Self {
        Self {
            inner: self.inner.inverse(),
        }
    }

    fn apply(&self, value: &PyPoint3) -> PyPoint3 {
        point(self.inner.apply(value.point))
    }

    fn apply_vector(&self, value: &PyVector3) -> PyVector3 {
        vector(self.inner.apply_vector(value.vector))
    }
}
