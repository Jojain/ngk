#![allow(unsafe_op_in_unsafe_fn)]
#![allow(clippy::useless_conversion)]

use std::collections::HashSet;
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};
use std::sync::Arc;

use nalgebra::{UnitVector3, Vector3};
use pyo3::basic::CompareOp;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::StandardPayload;
use crate::geometry::{
    Circle, Curve, NurbsCurve, NurbsSurface, Plane, Point3, Surface, SurfaceOfRevolution,
};
use crate::geometry::{Cylinder, Line, RuledSurface};
use crate::modeling::primitives;
use crate::topology::edge::Edge;
use crate::topology::face::Face;
use crate::topology::facet::Facet;
use crate::topology::gmap::{Dart, Dim, GMap};
use crate::topology::profile::{Loop, Profile};
use crate::topology::shape_keys::{EdgeKey, FaceKey, SolidKey, VertexKey};
use crate::topology::sheet::ShellRef;
use crate::topology::solid::Solid;
use crate::topology::vertex::Vertex;

type SharedMap = Arc<GMap<StandardPayload>>;

#[pymodule]
pub fn _ngk(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(block, m)?)?;
    m.add_class::<PySolid>()?;
    m.add_class::<PyShell>()?;
    m.add_class::<PyFace>()?;
    m.add_class::<PyFacet>()?;
    m.add_class::<PyLoop>()?;
    m.add_class::<PyEdge>()?;
    m.add_class::<PyVertex>()?;
    m.add_class::<PyPoint3>()?;
    m.add_class::<PyVector3>()?;
    m.add_class::<PyLine>()?;
    m.add_class::<PyCircle>()?;
    m.add_class::<PyNurbsCurve>()?;
    m.add_class::<PyPlane>()?;
    m.add_class::<PyCylinder>()?;
    m.add_class::<PyRuledSurface>()?;
    m.add_class::<PySurfaceOfRevolution>()?;
    m.add_class::<PyNurbsSurface>()?;
    Ok(())
}

#[pyfunction]
fn block(x: f64, y: f64, z: f64) -> PyResult<PySolid> {
    let shape = primitives::block(x, y, z).map_err(|err| PyValueError::new_err(err.to_string()))?;
    let (map, key) = shape.into_map();
    Ok(PySolid::new(Arc::new(map), key))
}

fn missing_topology(message: impl Into<String>) -> PyErr {
    PyValueError::new_err(message.into())
}

fn point(point: Point3) -> PyPoint3 {
    PyPoint3 { point }
}

fn vector(vector: Vector3<f64>) -> PyVector3 {
    PyVector3 { vector }
}

fn unit_vector(vector: UnitVector3<f64>) -> PyVector3 {
    PyVector3 {
        vector: vector.into_inner(),
    }
}

fn point_tuple_repr(point: Option<&Point3>) -> String {
    match point {
        Some(point) => format!("({}, {}, {})", point.x, point.y, point.z),
        None => "(?, ?, ?)".to_string(),
    }
}

fn hash_topology_identity<T: Hash>(map: &SharedMap, value: T) -> isize {
    let mut hasher = DefaultHasher::new();
    Arc::as_ptr(map).hash(&mut hasher);
    value.hash(&mut hasher);
    hasher.finish() as isize
}

fn edge_key(map: &GMap<StandardPayload>, edge: &Edge<'_, StandardPayload>) -> Option<EdgeKey> {
    let repr = map.cell_representative(edge.dart, Dim::One);
    map.dart_to_edge
        .get(&repr)
        .or_else(|| map.dart_to_edge.get(&edge.dart))
        .copied()
}

fn vertex_key(
    map: &GMap<StandardPayload>,
    vertex: &Vertex<'_, StandardPayload>,
) -> Option<VertexKey> {
    let repr = map.cell_representative(vertex.dart, Dim::Zero);
    map.dart_to_vertex
        .get(&repr)
        .or_else(|| map.dart_to_vertex.get(&vertex.dart))
        .copied()
}

fn face_edges<'g>(face: &Face<'g, StandardPayload>) -> Vec<Edge<'g, StandardPayload>> {
    let mut edges = face.outer_loop().edges();
    for loop_ in face.inner_loops() {
        edges.extend(loop_.edges());
    }
    edges
}

fn face_vertices<'g>(face: &Face<'g, StandardPayload>) -> Vec<Vertex<'g, StandardPayload>> {
    let mut vertices = face.outer_loop().vertices();
    for loop_ in face.inner_loops() {
        vertices.extend(loop_.vertices());
    }
    vertices
}

fn face_loop_darts(face: &Face<'_, StandardPayload>) -> Vec<Dart> {
    let mut darts = vec![face.outer_loop().dart];
    darts.extend(face.inner_loops().into_iter().map(|loop_| loop_.dart));
    darts
}

fn curve_to_py(py: Python<'_>, curve: Curve) -> PyResult<PyObject> {
    match curve {
        Curve::Line(line) => Ok(Py::new(py, PyLine { line })?.into_py(py)),
        Curve::Circle(circle) => Ok(Py::new(py, PyCircle { circle })?.into_py(py)),
        Curve::Nurbs(curve) => Ok(Py::new(py, PyNurbsCurve { curve })?.into_py(py)),
    }
}

fn surface_to_py(py: Python<'_>, surface: Surface) -> PyResult<PyObject> {
    match surface {
        Surface::Plane(plane) => Ok(Py::new(py, PyPlane { plane })?.into_py(py)),
        Surface::Cylinder(cylinder) => Ok(Py::new(py, PyCylinder { cylinder })?.into_py(py)),
        Surface::Ruled(surface) => Ok(Py::new(py, PyRuledSurface { surface })?.into_py(py)),
        Surface::Revolution(surface) => {
            Ok(Py::new(py, PySurfaceOfRevolution { surface })?.into_py(py))
        }
        Surface::Nurbs(surface) => Ok(Py::new(py, PyNurbsSurface { surface })?.into_py(py)),
    }
}

#[pyclass(name = "Solid", module = "ngk")]
#[derive(Clone)]
pub struct PySolid {
    map: SharedMap,
    key: SolidKey,
}

impl PySolid {
    fn new(map: SharedMap, key: SolidKey) -> Self {
        Self { map, key }
    }
}

#[pymethods]
impl PySolid {
    #[getter]
    fn key(&self) -> String {
        format!("{:?}", self.key)
    }

    #[getter]
    fn outer_shell(&self) -> PyResult<PyShell> {
        let map = self.map.as_ref();
        let attr = map
            .solid(self.key)
            .ok_or_else(|| missing_topology(format!("missing solid {:?}", self.key)))?;
        let shell = Solid::new(map, attr).outer_shell();
        Ok(PyShell::new(Arc::clone(&self.map), shell))
    }

    #[getter]
    fn shells(&self) -> PyResult<Vec<PyShell>> {
        let map = self.map.as_ref();
        let attr = map
            .solid(self.key)
            .ok_or_else(|| missing_topology(format!("missing solid {:?}", self.key)))?;
        Ok(Solid::new(map, attr)
            .shells()
            .into_iter()
            .map(|shell| PyShell::new(Arc::clone(&self.map), shell))
            .collect())
    }

    fn faces(&self) -> PyResult<Vec<PyFace>> {
        self.map
            .solid(self.key)
            .ok_or_else(|| missing_topology(format!("missing solid {:?}", self.key)))?;
        Ok(self
            .map
            .iter_faces()
            .map(|(key, _)| PyFace::new(Arc::clone(&self.map), key))
            .collect())
    }

    #[getter]
    fn face_count(&self) -> usize {
        self.map.iter_faces().count()
    }

    #[getter]
    fn edge_count(&self) -> usize {
        self.map.cells(Dim::One).count()
    }

    #[getter]
    fn vertex_count(&self) -> usize {
        self.map.cells(Dim::Zero).count()
    }

    fn __richcmp__(&self, other: PyRef<'_, PySolid>, op: CompareOp) -> PyResult<bool> {
        match op {
            CompareOp::Eq => Ok(Arc::ptr_eq(&self.map, &other.map) && self.key == other.key),
            CompareOp::Ne => Ok(!Arc::ptr_eq(&self.map, &other.map) || self.key != other.key),
            _ => Err(PyValueError::new_err(
                "solid ordering is not defined; use == or !=",
            )),
        }
    }

    fn __hash__(&self) -> isize {
        hash_topology_identity(&self.map, self.key)
    }

    fn __repr__(&self) -> String {
        format!("Solid(key={:?})", self.key)
    }
}

#[pyclass(name = "Shell", module = "ngk")]
#[derive(Clone)]
pub struct PyShell {
    map: SharedMap,
    dart: Dart,
}

impl PyShell {
    fn new(map: SharedMap, shell: ShellRef<'_, StandardPayload>) -> Self {
        let dart = map.cell_representative(shell.dart, Dim::Three);
        Self { map, dart }
    }
}

#[pymethods]
impl PyShell {
    #[getter]
    fn dart_id(&self) -> usize {
        self.dart.id()
    }

    fn faces(&self) -> Vec<PyFace> {
        self.map
            .iter_faces()
            .map(|(key, _)| PyFace::new(Arc::clone(&self.map), key))
            .collect()
    }

    fn __richcmp__(&self, other: PyRef<'_, PyShell>, op: CompareOp) -> PyResult<bool> {
        match op {
            CompareOp::Eq => Ok(Arc::ptr_eq(&self.map, &other.map) && self.dart == other.dart),
            CompareOp::Ne => Ok(!Arc::ptr_eq(&self.map, &other.map) || self.dart != other.dart),
            _ => Err(PyValueError::new_err(
                "shell ordering is not defined; use == or !=",
            )),
        }
    }

    fn __hash__(&self) -> isize {
        hash_topology_identity(&self.map, self.dart)
    }

    fn __repr__(&self) -> String {
        format!("Shell(dart_id={})", self.dart.id())
    }
}

#[pyclass(name = "Face", module = "ngk")]
#[derive(Clone)]
pub struct PyFace {
    map: SharedMap,
    key: FaceKey,
}

impl PyFace {
    fn new(map: SharedMap, key: FaceKey) -> Self {
        Self { map, key }
    }
}

#[pymethods]
impl PyFace {
    #[getter]
    fn key(&self) -> String {
        format!("{:?}", self.key)
    }

    #[getter]
    fn surface(&self, py: Python<'_>) -> PyResult<PyObject> {
        let attr = self
            .map
            .face(self.key)
            .ok_or_else(|| missing_topology(format!("missing face {:?}", self.key)))?;
        surface_to_py(py, attr.surface.clone())
    }

    #[getter]
    fn outer_loop(&self) -> PyResult<PyLoop> {
        let map = self.map.as_ref();
        let attr = map
            .face(self.key)
            .ok_or_else(|| missing_topology(format!("missing face {:?}", self.key)))?;
        let face = Face::new(map, attr);
        Ok(PyLoop::new(Arc::clone(&self.map), face.outer_loop()))
    }

    #[getter]
    fn inner_loops(&self) -> PyResult<Vec<PyLoop>> {
        let map = self.map.as_ref();
        let attr = map
            .face(self.key)
            .ok_or_else(|| missing_topology(format!("missing face {:?}", self.key)))?;
        Ok(Face::new(map, attr)
            .inner_loops()
            .into_iter()
            .map(|loop_| PyLoop::new(Arc::clone(&self.map), loop_))
            .collect())
    }

    fn edges(&self) -> PyResult<Vec<PyEdge>> {
        let map = self.map.as_ref();
        let attr = map
            .face(self.key)
            .ok_or_else(|| missing_topology(format!("missing face {:?}", self.key)))?;
        let face = Face::new(map, attr);
        Ok(face_edges(&face)
            .into_iter()
            .map(|edge| PyEdge::new(Arc::clone(&self.map), edge))
            .collect())
    }

    fn vertices(&self) -> PyResult<Vec<PyVertex>> {
        let map = self.map.as_ref();
        let attr = map
            .face(self.key)
            .ok_or_else(|| missing_topology(format!("missing face {:?}", self.key)))?;
        let face = Face::new(map, attr);
        Ok(face_vertices(&face)
            .into_iter()
            .map(|vertex| PyVertex::new(Arc::clone(&self.map), vertex))
            .collect())
    }

    fn __richcmp__(&self, other: PyRef<'_, PyFace>, op: CompareOp) -> PyResult<bool> {
        match op {
            CompareOp::Eq => Ok(Arc::ptr_eq(&self.map, &other.map) && self.key == other.key),
            CompareOp::Ne => Ok(!Arc::ptr_eq(&self.map, &other.map) || self.key != other.key),
            _ => Err(PyValueError::new_err(
                "face ordering is not defined; use == or !=",
            )),
        }
    }

    fn __hash__(&self) -> isize {
        hash_topology_identity(&self.map, self.key)
    }

    fn __repr__(&self) -> String {
        format!("Face(key={:?})", self.key)
    }
}

#[pyclass(name = "Loop", module = "ngk")]
#[derive(Clone)]
pub struct PyLoop {
    map: SharedMap,
    dart: Dart,
}

impl PyLoop {
    fn new(map: SharedMap, loop_: Loop<'_, StandardPayload>) -> Self {
        let dart = loop_
            .darts()
            .min()
            .expect("loop topology should contain at least one dart");
        Self { map, dart }
    }
}

#[pymethods]
impl PyLoop {
    #[getter]
    fn dart_id(&self) -> usize {
        self.dart.id()
    }

    fn edges(&self) -> Vec<PyEdge> {
        Profile::new(self.map.as_ref(), self.dart)
            .edges()
            .into_iter()
            .map(|edge| PyEdge::new(Arc::clone(&self.map), edge))
            .collect()
    }

    fn vertices(&self) -> Vec<PyVertex> {
        Profile::new(self.map.as_ref(), self.dart)
            .vertices()
            .into_iter()
            .map(|vertex| PyVertex::new(Arc::clone(&self.map), vertex))
            .collect()
    }

    fn __richcmp__(&self, other: PyRef<'_, PyLoop>, op: CompareOp) -> PyResult<bool> {
        match op {
            CompareOp::Eq => Ok(Arc::ptr_eq(&self.map, &other.map) && self.dart == other.dart),
            CompareOp::Ne => Ok(!Arc::ptr_eq(&self.map, &other.map) || self.dart != other.dart),
            _ => Err(PyValueError::new_err(
                "loop ordering is not defined; use == or !=",
            )),
        }
    }

    fn __hash__(&self) -> isize {
        hash_topology_identity(&self.map, self.dart)
    }

    fn __repr__(&self) -> String {
        format!("Loop(dart_id={})", self.dart.id())
    }
}

#[pyclass(name = "Facet", module = "ngk")]
#[derive(Clone)]
pub struct PyFacet {
    map: SharedMap,
    dart: Dart,
}

impl PyFacet {
    fn new(map: SharedMap, facet: Facet<'_, StandardPayload>) -> Self {
        let dart = map.cell_representative(facet.dart, Dim::Two);
        Self { map, dart }
    }

    fn facet(&self) -> Facet<'_, StandardPayload> {
        Facet::new(self.map.as_ref(), self.dart)
    }
}

#[pymethods]
impl PyFacet {
    #[getter]
    fn dart_id(&self) -> usize {
        self.dart.id()
    }

    #[getter]
    fn face(&self) -> Option<PyFace> {
        self.facet().face().and_then(|face| {
            self.map
                .attribute::<crate::topology::gmap::Cell2>(face.outer_loop().dart)
                .copied()
                .map(|key| PyFace::new(Arc::clone(&self.map), key))
        })
    }

    fn edges(&self) -> Vec<PyEdge> {
        self.facet()
            .edges()
            .into_iter()
            .map(|edge| PyEdge::new(Arc::clone(&self.map), edge))
            .collect()
    }

    fn vertices(&self) -> Vec<PyVertex> {
        self.facet()
            .vertices()
            .into_iter()
            .map(|vertex| PyVertex::new(Arc::clone(&self.map), vertex))
            .collect()
    }

    fn __richcmp__(&self, other: PyRef<'_, PyFacet>, op: CompareOp) -> PyResult<bool> {
        match op {
            CompareOp::Eq => Ok(Arc::ptr_eq(&self.map, &other.map) && self.dart == other.dart),
            CompareOp::Ne => Ok(!Arc::ptr_eq(&self.map, &other.map) || self.dart != other.dart),
            _ => Err(PyValueError::new_err(
                "facet ordering is not defined; use == or !=",
            )),
        }
    }

    fn __hash__(&self) -> isize {
        hash_topology_identity(&self.map, self.dart)
    }

    fn __repr__(&self) -> String {
        format!("Facet(dart_id={})", self.dart.id())
    }
}

#[pyclass(name = "Edge", module = "ngk")]
#[derive(Clone)]
pub struct PyEdge {
    map: SharedMap,
    key: EdgeKey,
}

impl PyEdge {
    fn new(map: SharedMap, edge: Edge<'_, StandardPayload>) -> Self {
        let key = edge_key(map.as_ref(), &edge).expect("edge view must have an edge attribute key");
        Self { map, key }
    }

    fn edge(&self) -> PyResult<Edge<'_, StandardPayload>> {
        let attr = self
            .map
            .edge(self.key)
            .ok_or_else(|| missing_topology(format!("missing edge {:?}", self.key)))?;
        Ok(attr.edge(self.map.as_ref()))
    }
}

#[pymethods]
impl PyEdge {
    #[getter]
    fn key(&self) -> String {
        format!("{:?}", self.key)
    }

    #[getter]
    fn dart_id(&self) -> PyResult<usize> {
        Ok(self.edge()?.dart.id())
    }

    #[getter]
    fn start(&self) -> PyResult<PyVertex> {
        let edge = self.edge()?;
        Ok(PyVertex::new(Arc::clone(&self.map), edge.start()))
    }

    #[getter]
    fn end(&self) -> PyResult<PyVertex> {
        let edge = self.edge()?;
        Ok(PyVertex::new(Arc::clone(&self.map), edge.end()))
    }

    #[getter]
    fn length(&self) -> PyResult<Option<f64>> {
        let edge = self.edge()?;
        let Some(curve) = edge.curve() else {
            return Ok(None);
        };
        let Some(start) = edge.start().point().copied() else {
            return Ok(None);
        };
        let Some(end) = edge.end().point().copied() else {
            return Ok(None);
        };
        let t0 = curve.param_at(start);
        let t1 = curve.param_at(end);
        Ok(Some(curve.length(t0, t1)))
    }

    #[getter]
    fn curve(&self, py: Python<'_>) -> PyResult<Option<PyObject>> {
        let edge = self.edge()?;
        match edge.curve().cloned() {
            Some(curve) => Ok(Some(curve_to_py(py, curve)?)),
            None => Ok(None),
        }
    }

    fn vertices(&self) -> PyResult<Vec<PyVertex>> {
        Ok(self
            .edge()?
            .vertices()
            .into_iter()
            .map(|vertex| PyVertex::new(Arc::clone(&self.map), vertex))
            .collect())
    }

    fn faces(&self) -> PyResult<Vec<PyFace>> {
        let map = self.map.as_ref();
        let incident_facets = self
            .edge()?
            .facets()
            .into_iter()
            .map(|facet| map.cell_representative(facet.dart, Dim::Two))
            .collect::<HashSet<_>>();

        let mut faces = Vec::new();
        for (key, attr) in map.iter_faces() {
            let face = Face::new(map, attr);
            let matched_facet = face_loop_darts(&face)
                .into_iter()
                .map(|dart| map.cell_representative(dart, Dim::Two))
                .any(|dart| incident_facets.contains(&dart));
            let matched_edge = face_edges(&face)
                .iter()
                .any(|edge| edge_key(map, edge) == Some(self.key));

            if matched_facet || matched_edge {
                faces.push(PyFace::new(Arc::clone(&self.map), key));
            }
        }
        Ok(faces)
    }

    fn __richcmp__(&self, other: PyRef<'_, PyEdge>, op: CompareOp) -> PyResult<bool> {
        match op {
            CompareOp::Eq => Ok(Arc::ptr_eq(&self.map, &other.map) && self.key == other.key),
            CompareOp::Ne => Ok(!Arc::ptr_eq(&self.map, &other.map) || self.key != other.key),
            _ => Err(PyValueError::new_err(
                "edge ordering is not defined; use == or !=",
            )),
        }
    }

    fn __hash__(&self) -> isize {
        hash_topology_identity(&self.map, self.key)
    }

    fn __repr__(&self) -> PyResult<String> {
        let edge = self.edge()?;
        let start = edge.start();
        let end = edge.end();
        Ok(format!(
            "E{}-{}->{}",
            format!("{:?}", self.key),
            point_tuple_repr(start.point()),
            point_tuple_repr(end.point())
        ))
    }
}

#[pyclass(name = "Vertex", module = "ngk")]
#[derive(Clone)]
pub struct PyVertex {
    map: SharedMap,
    key: VertexKey,
}

impl PyVertex {
    fn new(map: SharedMap, vertex: Vertex<'_, StandardPayload>) -> Self {
        let key = vertex_key(map.as_ref(), &vertex)
            .expect("vertex view must have a vertex attribute key");
        Self { map, key }
    }

    fn vertex(&self) -> PyResult<Vertex<'_, StandardPayload>> {
        let attr = self
            .map
            .vertex(self.key)
            .ok_or_else(|| missing_topology(format!("missing vertex {:?}", self.key)))?;
        Ok(attr.vertex(self.map.as_ref()))
    }
}

#[pymethods]
impl PyVertex {
    #[getter]
    fn key(&self) -> String {
        format!("{:?}", self.key)
    }

    #[getter]
    fn dart_id(&self) -> PyResult<usize> {
        Ok(self.vertex()?.dart.id())
    }

    #[getter]
    fn point(&self) -> PyResult<Option<PyPoint3>> {
        Ok(self.vertex()?.point().copied().map(point))
    }

    fn edges(&self) -> PyResult<Vec<PyEdge>> {
        Ok(self
            .vertex()?
            .edges()
            .into_iter()
            .map(|edge| PyEdge::new(Arc::clone(&self.map), edge))
            .collect())
    }

    fn facets(&self) -> PyResult<Vec<PyFacet>> {
        Ok(self
            .vertex()?
            .facets()
            .into_iter()
            .map(|facet| PyFacet::new(Arc::clone(&self.map), facet))
            .collect())
    }

    fn faces(&self) -> PyResult<Vec<PyFace>> {
        let map = self.map.as_ref();
        let mut faces = Vec::new();
        for (key, attr) in map.iter_faces() {
            let face = Face::new(map, attr);
            if face_vertices(&face)
                .iter()
                .any(|vertex| vertex_key(map, vertex) == Some(self.key))
            {
                faces.push(PyFace::new(Arc::clone(&self.map), key));
            }
        }
        Ok(faces)
    }

    fn __richcmp__(&self, other: PyRef<'_, PyVertex>, op: CompareOp) -> PyResult<bool> {
        match op {
            CompareOp::Eq => Ok(Arc::ptr_eq(&self.map, &other.map) && self.key == other.key),
            CompareOp::Ne => Ok(!Arc::ptr_eq(&self.map, &other.map) || self.key != other.key),
            _ => Err(PyValueError::new_err(
                "vertex ordering is not defined; use == or !=",
            )),
        }
    }

    fn __hash__(&self) -> isize {
        hash_topology_identity(&self.map, self.key)
    }

    fn __repr__(&self) -> PyResult<String> {
        let vertex = self.vertex()?;
        Ok(format!(
            "V{}-{}",
            format!("{:?}", self.key),
            point_tuple_repr(vertex.point())
        ))
    }
}

#[pyclass(name = "Point3", module = "ngk")]
#[derive(Clone)]
pub struct PyPoint3 {
    point: Point3,
}

#[pymethods]
impl PyPoint3 {
    #[getter]
    fn x(&self) -> f64 {
        self.point.x
    }

    #[getter]
    fn y(&self) -> f64 {
        self.point.y
    }

    #[getter]
    fn z(&self) -> f64 {
        self.point.z
    }

    fn as_tuple(&self) -> (f64, f64, f64) {
        (self.point.x, self.point.y, self.point.z)
    }

    fn __repr__(&self) -> String {
        format!(
            "Point3({}, {}, {})",
            self.point.x, self.point.y, self.point.z
        )
    }
}

#[pyclass(name = "Vector3", module = "ngk")]
#[derive(Clone)]
pub struct PyVector3 {
    vector: Vector3<f64>,
}

#[pymethods]
impl PyVector3 {
    #[getter]
    fn x(&self) -> f64 {
        self.vector.x
    }

    #[getter]
    fn y(&self) -> f64 {
        self.vector.y
    }

    #[getter]
    fn z(&self) -> f64 {
        self.vector.z
    }

    fn as_tuple(&self) -> (f64, f64, f64) {
        (self.vector.x, self.vector.y, self.vector.z)
    }

    fn __repr__(&self) -> String {
        format!(
            "Vector3({}, {}, {})",
            self.vector.x, self.vector.y, self.vector.z
        )
    }
}

#[pyclass(name = "Line", module = "ngk")]
#[derive(Clone)]
pub struct PyLine {
    line: Line,
}

#[pymethods]
impl PyLine {
    #[getter]
    fn start(&self) -> PyPoint3 {
        point(self.line.start())
    }

    #[getter]
    fn end(&self) -> PyPoint3 {
        point(self.line.end())
    }

    #[getter]
    fn kind(&self) -> &'static str {
        "line"
    }

    fn point_at(&self, t: f64) -> PyPoint3 {
        point(self.line.point_at(t))
    }

    fn __repr__(&self) -> &'static str {
        "Line()"
    }
}

#[pyclass(name = "Circle", module = "ngk")]
#[derive(Clone)]
pub struct PyCircle {
    circle: Circle,
}

#[pymethods]
impl PyCircle {
    #[getter]
    fn plane(&self) -> PyPlane {
        PyPlane {
            plane: self.circle.plane().clone(),
        }
    }

    #[getter]
    fn radius(&self) -> f64 {
        self.circle.radius()
    }

    #[getter]
    fn kind(&self) -> &'static str {
        "circle"
    }

    fn point_at(&self, t: f64) -> PyPoint3 {
        point(self.circle.point_at(t))
    }

    fn __repr__(&self) -> String {
        format!("Circle(radius={})", self.circle.radius())
    }
}

#[pyclass(name = "NurbsCurve", module = "ngk")]
#[derive(Clone)]
pub struct PyNurbsCurve {
    curve: NurbsCurve,
}

#[pymethods]
impl PyNurbsCurve {
    #[getter]
    fn degree(&self) -> usize {
        self.curve.degree().get()
    }

    #[getter]
    fn domain(&self) -> (f64, f64) {
        self.curve.domain()
    }

    #[getter]
    fn knots(&self) -> Vec<f64> {
        self.curve.knots().as_slice().to_vec()
    }

    #[getter]
    fn control_points(&self) -> Vec<(PyPoint3, f64)> {
        self.curve
            .control_points()
            .iter()
            .map(|p| (point(p.to_cartesian()), p.weight()))
            .collect()
    }

    #[getter]
    fn kind(&self) -> &'static str {
        "nurbs_curve"
    }

    fn point_at(&self, u: f64) -> PyPoint3 {
        point(self.curve.point_at(u))
    }

    fn __repr__(&self) -> String {
        format!("NurbsCurve(degree={})", self.curve.degree().get())
    }
}

#[pyclass(name = "Plane", module = "ngk")]
#[derive(Clone)]
pub struct PyPlane {
    plane: Plane,
}

#[pymethods]
impl PyPlane {
    #[getter]
    fn origin(&self) -> PyPoint3 {
        point(self.plane.origin())
    }

    #[getter]
    fn x_dir(&self) -> PyVector3 {
        unit_vector(self.plane.x_dir())
    }

    #[getter]
    fn y_dir(&self) -> PyVector3 {
        unit_vector(self.plane.y_dir())
    }

    #[getter]
    fn normal(&self) -> PyVector3 {
        unit_vector(self.plane.normal())
    }

    #[getter]
    fn kind(&self) -> &'static str {
        "plane"
    }

    fn point_at(&self, u: f64, v: f64) -> PyPoint3 {
        point(self.plane.point_at(u, v))
    }

    fn normal_at(&self, _u: f64, _v: f64) -> PyVector3 {
        unit_vector(self.plane.normal())
    }

    fn __repr__(&self) -> &'static str {
        "Plane()"
    }
}

#[pyclass(name = "Cylinder", module = "ngk")]
#[derive(Clone)]
pub struct PyCylinder {
    cylinder: Cylinder,
}

#[pymethods]
impl PyCylinder {
    #[getter]
    fn origin(&self) -> PyPoint3 {
        point(self.cylinder.origin())
    }

    #[getter]
    fn x_dir(&self) -> PyVector3 {
        unit_vector(self.cylinder.x_dir())
    }

    #[getter]
    fn axis(&self) -> PyVector3 {
        unit_vector(self.cylinder.axis())
    }

    #[getter]
    fn radius(&self) -> f64 {
        self.cylinder.radius
    }

    #[getter]
    fn kind(&self) -> &'static str {
        "cylinder"
    }

    fn point_at(&self, u: f64, v: f64) -> PyPoint3 {
        point(self.cylinder.point_at(u, v))
    }

    fn normal_at(&self, u: f64, v: f64) -> PyVector3 {
        unit_vector(self.cylinder.normal_at(u, v))
    }

    fn __repr__(&self) -> String {
        format!("Cylinder(radius={})", self.cylinder.radius)
    }
}

#[pyclass(name = "RuledSurface", module = "ngk")]
#[derive(Clone)]
pub struct PyRuledSurface {
    surface: RuledSurface,
}

#[pymethods]
impl PyRuledSurface {
    #[getter]
    fn curve(&self, py: Python<'_>) -> PyResult<PyObject> {
        curve_to_py(py, self.surface.curve().clone())
    }

    #[getter]
    fn direction(&self) -> PyVector3 {
        vector(self.surface.direction())
    }

    #[getter]
    fn kind(&self) -> &'static str {
        "ruled"
    }

    fn point_at(&self, u: f64, v: f64) -> PyPoint3 {
        point(self.surface.point_at(u, v))
    }

    fn normal_at(&self, u: f64, v: f64) -> PyVector3 {
        unit_vector(self.surface.normal_at(u, v))
    }

    fn __repr__(&self) -> &'static str {
        "RuledSurface()"
    }
}

#[pyclass(name = "SurfaceOfRevolution", module = "ngk")]
#[derive(Clone)]
pub struct PySurfaceOfRevolution {
    surface: SurfaceOfRevolution,
}

#[pymethods]
impl PySurfaceOfRevolution {
    #[getter]
    fn curve(&self, py: Python<'_>) -> PyResult<PyObject> {
        curve_to_py(py, self.surface.curve().clone())
    }

    #[getter]
    fn origin(&self) -> PyPoint3 {
        point(self.surface.origin())
    }

    #[getter]
    fn axis(&self) -> PyVector3 {
        unit_vector(self.surface.axis())
    }

    #[getter]
    fn kind(&self) -> &'static str {
        "revolution"
    }

    fn point_at(&self, u: f64, v: f64) -> PyPoint3 {
        point(self.surface.point_at(u, v))
    }

    fn normal_at(&self, u: f64, v: f64) -> PyVector3 {
        unit_vector(self.surface.normal_at(u, v))
    }

    fn __repr__(&self) -> &'static str {
        "SurfaceOfRevolution()"
    }
}

#[pyclass(name = "NurbsSurface", module = "ngk")]
#[derive(Clone)]
pub struct PyNurbsSurface {
    surface: NurbsSurface,
}

#[pymethods]
impl PyNurbsSurface {
    #[getter]
    fn degree_u(&self) -> usize {
        self.surface.degree_u().get()
    }

    #[getter]
    fn degree_v(&self) -> usize {
        self.surface.degree_v().get()
    }

    #[getter]
    fn domain_u(&self) -> (f64, f64) {
        self.surface.domain_u()
    }

    #[getter]
    fn domain_v(&self) -> (f64, f64) {
        self.surface.domain_v()
    }

    #[getter]
    fn knots_u(&self) -> Vec<f64> {
        self.surface.knots_u().as_slice().to_vec()
    }

    #[getter]
    fn knots_v(&self) -> Vec<f64> {
        self.surface.knots_v().as_slice().to_vec()
    }

    #[getter]
    fn control_points(&self) -> Vec<Vec<(PyPoint3, f64)>> {
        let points = self.surface.control_points();
        (0..points.nv())
            .map(|v| {
                (0..points.nu())
                    .map(|u| {
                        let point_ = points.get(u, v);
                        (point(point_.to_cartesian()), point_.weight())
                    })
                    .collect()
            })
            .collect()
    }

    #[getter]
    fn kind(&self) -> &'static str {
        "nurbs_surface"
    }

    fn point_at(&self, u: f64, v: f64) -> PyPoint3 {
        point(self.surface.point_at(u, v))
    }

    fn normal_at(&self, u: f64, v: f64) -> PyVector3 {
        unit_vector(self.surface.normal_at(u, v))
    }

    fn __repr__(&self) -> String {
        format!(
            "NurbsSurface(degree_u={}, degree_v={})",
            self.surface.degree_u().get(),
            self.surface.degree_v().get()
        )
    }
}
