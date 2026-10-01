use pyo3::exceptions::{PyTypeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::tessellate::{KeyedRange, Tessellate, Tessellation, TessellationError, tessellate};

use super::topology::{PyEdge, PyFace, PyProfile, PySheet, PySolid};

/// The run of a buffer one face or edge owns, named by the cell's key.
#[pyclass(name = "Range", module = "ngk", frozen, get_all, eq)]
#[derive(Clone, PartialEq)]
pub struct PyRange {
    key: String,
    start: usize,
    count: usize,
}

impl<K: std::fmt::Debug> From<&KeyedRange<K>> for PyRange {
    fn from(range: &KeyedRange<K>) -> Self {
        Self {
            key: format!("{:?}", range.key),
            start: range.start,
            count: range.count,
        }
    }
}

#[pymethods]
impl PyRange {
    fn __repr__(&self) -> String {
        format!(
            "Range(key={:?}, start={}, count={})",
            self.key, self.start, self.count
        )
    }
}

/// A shape's mesh in three.js layout, every range and point keyed by its cell.
#[pyclass(name = "Tessellation", module = "ngk", frozen, get_all, eq)]
#[derive(PartialEq)]
pub struct PyTessellation {
    positions: Vec<f64>,
    normals: Vec<f64>,
    indices: Vec<u32>,
    faces: Vec<PyRange>,
    edge_points: Vec<f64>,
    edges: Vec<PyRange>,
    vertex_points: Vec<f64>,
    vertices: Vec<String>,
}

impl From<Tessellation> for PyTessellation {
    fn from(tessellation: Tessellation) -> Self {
        let Tessellation {
            mesh,
            faces,
            edge_points,
            edges,
            vertices,
        } = tessellation;
        Self {
            positions: mesh
                .positions
                .iter()
                .flat_map(|p| [p.x, p.y, p.z])
                .collect(),
            normals: mesh.normals.iter().flat_map(|n| [n.x, n.y, n.z]).collect(),
            indices: mesh.indices,
            faces: faces.iter().map(PyRange::from).collect(),
            edge_points: edge_points.iter().flat_map(|p| [p.x, p.y, p.z]).collect(),
            edges: edges.iter().map(PyRange::from).collect(),
            vertex_points: vertices
                .iter()
                .flat_map(|vertex| [vertex.point.x, vertex.point.y, vertex.point.z])
                .collect(),
            vertices: vertices
                .iter()
                .map(|vertex| format!("{:?}", vertex.key))
                .collect(),
        }
    }
}

pub(super) fn register(parent: &Bound<'_, PyModule>) -> PyResult<()> {
    let module = PyModule::new_bound(parent.py(), "tessellation")?;
    let qualified = "ngk.core.tessellation";
    parent
        .py()
        .import_bound("sys")?
        .getattr("modules")?
        .set_item(qualified, &module)?;
    module.add_class::<PyRange>()?;
    module.add_class::<PyTessellation>()?;
    module.add_function(wrap_pyfunction!(tessellate_shape, &module)?)?;
    parent.add_submodule(&module)?;
    Ok(())
}

#[pyfunction]
#[pyo3(name = "tessellate")]
fn tessellate_shape(shape: &Bound<'_, PyAny>) -> PyResult<PyTessellation> {
    fn mesh<T: Tessellate + ?Sized>(view: Option<&T>, kind: &str) -> PyResult<PyTessellation> {
        let view = view.ok_or_else(|| PyValueError::new_err(format!("missing {kind}")))?;
        tessellate(view)
            .map(PyTessellation::from)
            .map_err(|error: TessellationError| PyValueError::new_err(error.to_string()))
    }

    if let Ok(solid) = shape.extract::<PyRef<'_, PySolid>>() {
        let model = solid.inner.model();
        return mesh(model.model().solid(solid.inner.key()).as_ref(), "solid");
    }
    if let Ok(sheet) = shape.extract::<PyRef<'_, PySheet>>() {
        let model = sheet.inner.model();
        return mesh(model.model().sheet(sheet.inner.key()).as_ref(), "sheet");
    }
    if let Ok(face) = shape.extract::<PyRef<'_, PyFace>>() {
        let model = face.inner.model();
        return mesh(model.model().face(face.inner.key()).as_ref(), "face");
    }
    if let Ok(profile) = shape.extract::<PyRef<'_, PyProfile>>() {
        let model = profile.inner.model();
        return mesh(
            model.model().profile(profile.inner.key()).as_ref(),
            "profile",
        );
    }
    if let Ok(edge) = shape.extract::<PyRef<'_, PyEdge>>() {
        let model = edge.inner.model();
        return mesh(model.model().edge(edge.inner.key()).as_ref(), "edge");
    }
    Err(PyTypeError::new_err(
        "tessellate takes a Solid, Sheet, Face, Profile or Edge",
    ))
}
