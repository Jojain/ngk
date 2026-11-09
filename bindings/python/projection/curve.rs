use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use crate::binding_common::explore::SharedModel;
use crate::geometry::parameter::Fraction;
use crate::geometry::{TrimmedCurve, TrimmedCurve2};
use crate::projection::{
    ChainEnd, CurveProjection, Exactness, ProjectedChain, ProjectedSegment, SegmentSource,
};
use crate::topology::StandardPayload;

use super::super::geometry::{PyPoint2, PyPoint3, curve_to_py, point};
use super::super::topology::{PyEdge, PyVertex};

type Model = SharedModel<StandardPayload>;

/// A span of a curve in 3D.
#[pyclass(name = "TrimmedCurve", module = "ngk.projection", frozen)]
#[derive(Clone)]
pub(crate) struct PyTrimmedCurve {
    inner: TrimmedCurve,
}

#[pymethods]
impl PyTrimmedCurve {
    /// The whole curve the span is cut from.
    #[getter]
    fn curve(&self, py: Python<'_>) -> PyResult<PyObject> {
        curve_to_py(py, self.inner.curve().clone())
    }

    /// The directed span of the curve's own parameters.
    #[getter]
    fn interval(&self) -> (f64, f64) {
        let interval = self.inner.interval();
        (interval.start.value(), interval.end.value())
    }

    #[getter]
    fn start(&self) -> PyPoint3 {
        point(self.inner.start())
    }

    #[getter]
    fn end(&self) -> PyPoint3 {
        point(self.inner.end())
    }

    #[getter]
    fn length(&self) -> f64 {
        self.inner.length()
    }

    /// The point at `fraction` of the way from start to end.
    fn point_at(&self, fraction: f64) -> PyPoint3 {
        point(self.inner.point_at(Fraction::new(fraction)))
    }
}

/// A span of a curve in a surface's parameter space.
#[pyclass(name = "TrimmedCurve2", module = "ngk.projection", frozen)]
#[derive(Clone)]
pub(crate) struct PyTrimmedCurve2 {
    inner: TrimmedCurve2,
}

#[pymethods]
impl PyTrimmedCurve2 {
    /// The directed span of the curve's own parameters.
    #[getter]
    fn interval(&self) -> (f64, f64) {
        let interval = self.inner.interval();
        (interval.start.value(), interval.end.value())
    }

    #[getter]
    fn start(&self) -> PyPoint2 {
        PyPoint2::from(self.inner.start())
    }

    #[getter]
    fn end(&self) -> PyPoint2 {
        PyPoint2::from(self.inner.end())
    }

    #[getter]
    fn length(&self) -> f64 {
        self.inner.length()
    }

    /// The point at `fraction` of the way from start to end.
    fn point_at(&self, fraction: f64) -> PyPoint2 {
        PyPoint2::from(self.inner.point_at(Fraction::new(fraction)))
    }
}

/// How closely a projected segment follows the image of its source.
#[pyclass(name = "Exactness", module = "ngk.projection", frozen)]
#[derive(Clone)]
pub(crate) struct PyExactness {
    inner: Exactness,
}

#[pymethods]
impl PyExactness {
    /// Whether the segment is the image itself rather than a fit to it.
    #[getter]
    fn exact(&self) -> bool {
        matches!(self.inner, Exactness::Exact)
    }

    /// The largest distance between a fit and the image; `None` when exact.
    #[getter]
    fn error(&self) -> Option<f64> {
        match self.inner {
            Exactness::Exact => None,
            Exactness::Approx { error } => Some(error),
        }
    }
}

/// Why a chain begins or stops where it does.
#[pyclass(name = "ChainEnd", module = "ngk.projection", frozen)]
#[derive(Clone)]
pub(crate) struct PyChainEnd {
    inner: ChainEnd,
    vertex: Option<PyVertex>,
}

impl PyChainEnd {
    fn new(inner: ChainEnd, source_model: Option<&Model>) -> Self {
        let vertex = match (inner, source_model) {
            (ChainEnd::SourceVertex(key), Some(model)) => {
                model.vertex_by_key(key).map(PyVertex::from_inner)
            }
            _ => None,
        };
        Self { inner, vertex }
    }
}

#[pymethods]
impl PyChainEnd {
    /// One of `"closed"`, `"source_end"`, `"source_vertex"`, `"miss"` and `"jump"`.
    #[getter]
    fn kind(&self) -> &'static str {
        match self.inner {
            ChainEnd::Closed => "closed",
            ChainEnd::SourceEnd => "source_end",
            ChainEnd::SourceVertex(_) => "source_vertex",
            ChainEnd::Miss => "miss",
            ChainEnd::Jump => "jump",
        }
    }

    /// The vertex of the source a `"source_vertex"` end is on.
    #[getter]
    fn vertex(&self) -> Option<PyVertex> {
        self.vertex.clone()
    }
}

/// What a projected segment is the image of: a span of a curve or edge.
#[pyclass(name = "SourceSpan", module = "ngk.projection", frozen)]
#[derive(Clone)]
pub(crate) struct PySourceSpan {
    interval: (f64, f64),
    edge: Option<PyEdge>,
}

#[pymethods]
impl PySourceSpan {
    /// The edge that was projected; `None` when the source is a curve.
    #[getter]
    fn edge(&self) -> Option<PyEdge> {
        self.edge.clone()
    }

    /// The directed span of the source's own parameters the segment covers.
    #[getter]
    fn interval(&self) -> (f64, f64) {
        self.interval
    }
}

/// One piece of a projection: the image of one span of the source on one part of the target.
#[pyclass(name = "ProjectedSegment", module = "ngk.projection", frozen)]
pub(crate) struct PyProjectedSegment {
    inner: ProjectedSegment<()>,
    edge: Option<PyEdge>,
    hit: PyObject,
}

impl PyProjectedSegment {
    fn trimmed_curve(&self) -> PyTrimmedCurve {
        PyTrimmedCurve {
            inner: self.inner.curve.clone(),
        }
    }

    fn trimmed_pcurve(&self) -> PyTrimmedCurve2 {
        PyTrimmedCurve2 {
            inner: self.inner.pcurve.clone(),
        }
    }
}

#[pymethods]
impl PyProjectedSegment {
    /// The piece in 3D, on the target.
    #[getter]
    fn curve(&self) -> PyTrimmedCurve {
        self.trimmed_curve()
    }

    /// The piece in the target's parameter space.
    #[getter]
    fn pcurve(&self) -> PyTrimmedCurve2 {
        self.trimmed_pcurve()
    }

    /// The face of the target the piece lies on; `None` for a plane.
    #[getter]
    fn hit(&self, py: Python<'_>) -> PyObject {
        self.hit.clone_ref(py)
    }

    /// The part of the source the piece is the image of.
    #[getter]
    fn source(&self) -> PySourceSpan {
        let interval = self.inner.source.interval;
        PySourceSpan {
            interval: (interval.start.value(), interval.end.value()),
            edge: self.edge.clone(),
        }
    }

    /// Whether the piece is the image itself or a fit to it.
    #[getter]
    fn fit(&self) -> PyExactness {
        PyExactness {
            inner: self.inner.fit,
        }
    }
}

/// A run of segments that each begin where the previous one ends.
#[pyclass(name = "ProjectedChain", module = "ngk.projection", frozen)]
pub(crate) struct PyProjectedChain {
    segments: Vec<Py<PyProjectedSegment>>,
    start: PyChainEnd,
    end: PyChainEnd,
}

impl PyProjectedChain {
    fn trimmed_curves(&self) -> impl Iterator<Item = PyTrimmedCurve> + '_ {
        self.segments
            .iter()
            .map(|segment| segment.get().trimmed_curve())
    }

    fn trimmed_pcurves(&self) -> impl Iterator<Item = PyTrimmedCurve2> + '_ {
        self.segments
            .iter()
            .map(|segment| segment.get().trimmed_pcurve())
    }
}

#[pymethods]
impl PyProjectedChain {
    /// The segments in order, never empty.
    #[getter]
    fn segments(&self, py: Python<'_>) -> Vec<Py<PyProjectedSegment>> {
        self.segments
            .iter()
            .map(|segment| segment.clone_ref(py))
            .collect()
    }

    /// Why the chain begins where it does.
    #[getter]
    fn start(&self) -> PyChainEnd {
        self.start.clone()
    }

    /// Why the chain stops where it does.
    #[getter]
    fn end(&self) -> PyChainEnd {
        self.end.clone()
    }

    /// Whether the chain runs round to where it began.
    #[getter]
    fn closed(&self) -> bool {
        matches!(
            (self.start.inner, self.end.inner),
            (ChainEnd::Closed, ChainEnd::Closed)
        )
    }

    /// The 3D curve of every segment.
    fn curves(&self) -> Vec<PyTrimmedCurve> {
        self.trimmed_curves().collect()
    }

    /// The pcurve of every segment.
    fn pcurves(&self) -> Vec<PyTrimmedCurve2> {
        self.trimmed_pcurves().collect()
    }
}

/// A curve carried onto a target, as the chains of segments the target gave it.
#[pyclass(name = "CurveProjection", module = "ngk.projection", frozen)]
pub(crate) struct PyCurveProjection {
    chains: Vec<Py<PyProjectedChain>>,
}

impl PyCurveProjection {
    /// The projection as Python objects, with each segment's hit converted by `hit`.
    ///
    /// `source_model` is the model of the edge or profile that was projected, which
    /// the source spans and chain ends name entities of.
    pub(super) fn build<Hit>(
        py: Python<'_>,
        projection: CurveProjection<Hit>,
        source_model: Option<&Model>,
        hit: impl Fn(Python<'_>, &Hit) -> PyResult<PyObject>,
    ) -> PyResult<Self> {
        let chains = projection
            .chains
            .into_iter()
            .map(|chain| {
                let segments = chain
                    .segments
                    .into_iter()
                    .map(|segment| {
                        let hit = hit(py, &segment.hit)?;
                        let edge = match segment.source.source {
                            SegmentSource::Curve => None,
                            SegmentSource::Edge(key) => Some(
                                source_model
                                    .and_then(|model| model.edge_by_key(key))
                                    .map(PyEdge::from_inner)
                                    .ok_or_else(|| {
                                        PyValueError::new_err(format!("missing edge {key:?}"))
                                    })?,
                            ),
                        };
                        Py::new(
                            py,
                            PyProjectedSegment {
                                inner: ProjectedSegment {
                                    curve: segment.curve,
                                    pcurve: segment.pcurve,
                                    hit: (),
                                    source: segment.source,
                                    fit: segment.fit,
                                },
                                edge,
                                hit,
                            },
                        )
                    })
                    .collect::<PyResult<Vec<_>>>()?;
                Py::new(
                    py,
                    PyProjectedChain {
                        segments,
                        start: PyChainEnd::new(chain.start, source_model),
                        end: PyChainEnd::new(chain.end, source_model),
                    },
                )
            })
            .collect::<PyResult<Vec<_>>>()?;
        Ok(Self { chains })
    }

    /// The projection without its hits, as the profile builder reads it.
    pub(crate) fn without_hits(&self) -> CurveProjection<()> {
        CurveProjection {
            chains: self
                .chains
                .iter()
                .map(|chain| {
                    let chain = chain.get();
                    ProjectedChain {
                        segments: chain
                            .segments
                            .iter()
                            .map(|segment| segment.get().inner.clone())
                            .collect(),
                        start: chain.start.inner,
                        end: chain.end.inner,
                    }
                })
                .collect(),
        }
    }
}

#[pymethods]
impl PyCurveProjection {
    /// The runs of continuous segments, in the order the source is walked.
    #[getter]
    fn chains(&self, py: Python<'_>) -> Vec<Py<PyProjectedChain>> {
        self.chains
            .iter()
            .map(|chain| chain.clone_ref(py))
            .collect()
    }

    /// Every segment of every chain.
    fn segments(&self, py: Python<'_>) -> Vec<Py<PyProjectedSegment>> {
        self.chains
            .iter()
            .flat_map(|chain| chain.get().segments(py))
            .collect()
    }

    /// The 3D curve of every segment.
    fn curves(&self) -> Vec<PyTrimmedCurve> {
        self.chains
            .iter()
            .flat_map(|chain| chain.get().curves())
            .collect()
    }

    /// The pcurve of every segment.
    fn pcurves(&self) -> Vec<PyTrimmedCurve2> {
        self.chains
            .iter()
            .flat_map(|chain| chain.get().pcurves())
            .collect()
    }
}
