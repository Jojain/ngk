use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use crate::binding_common::explore::{ExploreError, SharedGMap};
use crate::topology::StandardPayload;

pub(crate) type Map = SharedGMap<StandardPayload>;

pub(crate) fn py_err(error: ExploreError) -> PyErr {
    PyValueError::new_err(error.to_string())
}

pub(crate) fn hash_identity<T: Hash>(map: &Map, key: T) -> isize {
    let mut hasher = DefaultHasher::new();
    map.identity().hash(&mut hasher);
    key.hash(&mut hasher);
    hasher.finish() as isize
}

macro_rules! entity_methods {
    ($type:ident, $inner:ty, $name:literal, { $($extra:item)* }) => {
        impl $type {
            pub(crate) fn from_inner(inner: $inner) -> Self {
                Self { inner }
            }
        }

        #[pymethods]
        impl $type {
            #[getter]
            fn gmap(&self) -> PyGMap {
                PyGMap::from_inner(self.inner.gmap())
            }

            #[getter]
            fn key(&self) -> String {
                format!("{:?}", self.inner.key())
            }

            #[getter]
            fn dart_id(&self) -> usize {
                self.inner.dart_id()
            }

            fn __richcmp__(&self, other: PyRef<'_, $type>, op: CompareOp) -> PyResult<bool> {
                match op {
                    CompareOp::Eq => Ok(self.inner.same_entity(&other.inner)),
                    CompareOp::Ne => Ok(!self.inner.same_entity(&other.inner)),
                    _ => Err(PyValueError::new_err(concat!(
                        $name,
                        " ordering is not defined; use == or !="
                    ))),
                }
            }

            fn __hash__(&self) -> isize {
                hash_identity(&self.inner.gmap(), self.inner.key())
            }

            $($extra)*
        }
    };
}

pub(crate) use entity_methods;
