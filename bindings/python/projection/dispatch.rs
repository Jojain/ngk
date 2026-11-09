use pyo3::exceptions::{PyTypeError, PyValueError};
use pyo3::prelude::*;

use crate::binding_common::explore::SharedModel;
use crate::geometry::{Axis3, Curve, Plane, Point3, Surface};
use crate::projection::{
    CurveProjection, FaceList, PointProjection, Project, ProjectOptions, Projection, project_with,
};
use crate::topology::StandardPayload;
use crate::topology::edge::Edge;
use crate::topology::profile::Profile;
use crate::topology::shape_keys::{EdgeKey, FaceKey};

use super::super::geometry::{PyAxis3, PyPoint3, PyVector3, curve_from_py, surface_from_py};
use super::super::topology::{PyEdge, PyFace, PyProfile};
use super::curve::PyCurveProjection;
use super::errors::from_core;
use super::point::PyPointProjection;

type Model = SharedModel<StandardPayload>;

/// What is projected.
pub(super) enum Source {
    Point(Point3),
    Curve(Curve),
    Edge(PyEdge),
    Profile(PyProfile),
}

impl Source {
    pub(super) fn extract(object: &Bound<'_, PyAny>) -> PyResult<Self> {
        if let Ok(point) = object.extract::<PyPoint3>() {
            Ok(Self::Point(point.point))
        } else if let Ok((x, y, z)) = object.extract::<(f64, f64, f64)>() {
            Ok(Self::Point(Point3::new(x, y, z)))
        } else if let Ok(edge) = object.extract::<PyEdge>() {
            Ok(Self::Edge(edge))
        } else if let Ok(profile) = object.extract::<PyProfile>() {
            Ok(Self::Profile(profile))
        } else if let Some(curve) = curve_from_py(object) {
            Ok(Self::Curve(curve))
        } else {
            Err(PyTypeError::new_err(format!(
                "cannot project a {}: expected a point, a curve, an edge or a profile",
                type_name(object)
            )))
        }
    }

    fn kind(&self) -> &'static str {
        match self {
            Self::Point(_) => "point",
            Self::Curve(_) => "curve",
            Self::Edge(_) => "edge",
            Self::Profile(_) => "profile",
        }
    }
}

/// What is projected onto.
pub(crate) enum Target {
    Axis(Axis3),
    Plane(Plane),
    Surface(Surface),
    Curve(Curve),
    Edge(PyEdge),
    Face(PyFace),
    Faces(Vec<PyFace>),
}

impl Target {
    pub(super) fn extract(object: &Bound<'_, PyAny>) -> PyResult<Self> {
        if let Ok(face) = object.extract::<PyFace>() {
            Ok(Self::Face(face))
        } else if let Ok(edge) = object.extract::<PyEdge>() {
            Ok(Self::Edge(edge))
        } else if let Ok(axis) = object.extract::<PyAxis3>() {
            Ok(Self::Axis(axis.axis))
        } else if let Some(surface) = surface_from_py(object) {
            Ok(match surface {
                Surface::Plane(plane) => Self::Plane(plane),
                surface => Self::Surface(surface),
            })
        } else if let Some(curve) = curve_from_py(object) {
            Ok(Self::Curve(curve))
        } else if let Ok(faces) = object.extract::<Vec<PyFace>>() {
            Ok(Self::Faces(faces))
        } else {
            Err(PyTypeError::new_err(format!(
                "cannot project onto a {}: expected a plane, a surface, a curve, an axis, an edge, a face or a list of faces",
                type_name(object)
            )))
        }
    }

    fn kind(&self) -> &'static str {
        match self {
            Self::Axis(_) => "axis",
            Self::Plane(_) => "plane",
            Self::Surface(_) => "surface",
            Self::Curve(_) => "curve",
            Self::Edge(_) => "edge",
            Self::Face(_) => "face",
            Self::Faces(_) => "list of faces",
        }
    }
}

/// What a projection returned.
pub(super) enum Outcome {
    Point(PyPointProjection),
    Curve(PyCurveProjection),
}

fn type_name(object: &Bound<'_, PyAny>) -> String {
    object
        .get_type()
        .name()
        .map_or_else(|_| "value".to_owned(), |name| name.to_string())
}

/// The options a caller asked for.
pub(super) fn options(tolerance: Option<f64>) -> PyResult<ProjectOptions> {
    let mut options = ProjectOptions::default();
    if let Some(tolerance) = tolerance {
        if !(tolerance.is_finite() && tolerance >= 0.0) {
            return Err(PyValueError::new_err(
                "tolerance must be a finite, non-negative distance",
            ));
        }
        options.tolerance = tolerance;
    }
    Ok(options)
}

/// How a two-dimensional target is approached: along `along`, or by the perpendicular.
fn surface_mode(along: &Option<PyVector3>) -> PyResult<Projection> {
    match along {
        None => Ok(Projection::Normal),
        Some(direction) => {
            let vector = direction.vector;
            if !vector.iter().all(|x| x.is_finite()) || vector.norm() == 0.0 {
                return Err(PyValueError::new_err(
                    "along must be a finite, non-zero direction",
                ));
            }
            Ok(Projection::along(vector))
        }
    }
}

/// A curve has only the perpendicular, so a direction for it is refused.
fn perpendicular_only(along: &Option<PyVector3>, target: &str) -> PyResult<()> {
    if along.is_some() {
        return Err(PyValueError::new_err(format!(
            "a directional projection (along) onto a {target} is not defined: a {target} has only the perpendicular"
        )));
    }
    Ok(())
}

fn unsupported(source: &Source, target: &Target) -> PyErr {
    PyTypeError::new_err(format!(
        "projecting a {} onto a {} is not supported",
        source.kind(),
        target.kind()
    ))
}

fn explore_error(error: impl ToString) -> PyErr {
    PyValueError::new_err(error.to_string())
}

fn point_outcome<Hit>(
    py: Python<'_>,
    result: Result<PointProjection<Hit>, crate::projection::ProjectionError>,
    hit: impl FnOnce(&Hit) -> PyResult<PyObject>,
) -> PyResult<Outcome> {
    let result = result.map_err(|error| from_core(py, error))?;
    let hit = hit(&result.hit)?;
    Ok(Outcome::Point(PyPointProjection::new(result, hit)))
}

fn edge_hit(py: Python<'_>, model: &Model, key: EdgeKey) -> PyResult<PyObject> {
    model
        .edge_by_key(key)
        .map(|edge| PyEdge::from_inner(edge).into_py(py))
        .ok_or_else(|| PyValueError::new_err(format!("missing edge {key:?}")))
}

fn face_hit(py: Python<'_>, model: &Model, key: FaceKey) -> PyResult<PyObject> {
    model
        .face_by_key(key)
        .map(|face| PyFace::from_inner(face).into_py(py))
        .ok_or_else(|| PyValueError::new_err(format!("missing face {key:?}")))
}

/// The model the faces share, and their keys in list order.
fn face_list(faces: &[PyFace]) -> PyResult<(Model, Vec<FaceKey>)> {
    let first = faces
        .first()
        .ok_or_else(|| PyValueError::new_err("the list of faces is empty"))?;
    let model = first.inner.model();
    if faces.iter().any(|face| !face.inner.model().ptr_eq(&model)) {
        return Err(PyValueError::new_err(
            "the faces of a list must belong to the same model",
        ));
    }
    Ok((model, faces.iter().map(|face| face.inner.key()).collect()))
}

/// Carries a curve, an edge or a profile onto `target`.
fn carry<T, Hit>(
    py: Python<'_>,
    source: &Source,
    target: &T,
    mode: Projection,
    options: &ProjectOptions,
    hit: impl Fn(Python<'_>, &Hit) -> PyResult<PyObject>,
) -> PyResult<Outcome>
where
    T: Project<Curve, Mode = Projection, Output = CurveProjection<Hit>>
        + for<'a> Project<Edge<'a, StandardPayload>, Mode = Projection, Output = CurveProjection<Hit>>
        + for<'a> Project<
            Profile<'a, StandardPayload>,
            Mode = Projection,
            Output = CurveProjection<Hit>,
        > + ?Sized,
{
    let (result, source_model) = match source {
        Source::Curve(curve) => (project_with(curve, target, mode, options), None),
        Source::Edge(edge) => {
            let view = edge.inner.view().map_err(explore_error)?;
            (
                project_with(&view, target, mode, options),
                Some(edge.inner.model()),
            )
        }
        Source::Profile(profile) => {
            let view = profile.inner.view().map_err(explore_error)?;
            (
                project_with(&view, target, mode, options),
                Some(profile.inner.model()),
            )
        }
        Source::Point(_) => unreachable!("a point is not a curve source"),
    };
    let projection = result.map_err(|error| from_core(py, error))?;
    PyCurveProjection::build(py, projection, source_model.as_ref(), hit).map(Outcome::Curve)
}

/// Projects `source` onto `target`.
pub(super) fn run(
    py: Python<'_>,
    source: &Source,
    target: &Target,
    along: &Option<PyVector3>,
    options: &ProjectOptions,
) -> PyResult<Outcome> {
    let no_hit = |py: Python<'_>, _: &()| -> PyResult<PyObject> { Ok(py.None()) };
    match (source, target) {
        (Source::Point(point), Target::Axis(axis)) => {
            perpendicular_only(along, "axis")?;
            point_outcome(
                py,
                project_with(point, axis, (), options),
                |_| Ok(py.None()),
            )
        }
        (Source::Point(point), Target::Plane(plane)) => {
            let mode = surface_mode(along)?;
            point_outcome(py, project_with(point, plane, mode, options), |_| {
                Ok(py.None())
            })
        }
        (Source::Point(point), Target::Surface(surface)) => {
            let mode = surface_mode(along)?;
            point_outcome(py, project_with(point, surface, mode, options), |_| {
                Ok(py.None())
            })
        }
        (Source::Point(point), Target::Curve(curve)) => {
            perpendicular_only(along, "curve")?;
            point_outcome(py, project_with(point, curve, (), options), |_| {
                Ok(py.None())
            })
        }
        (Source::Point(point), Target::Edge(edge)) => {
            perpendicular_only(along, "edge")?;
            let view = edge.inner.view().map_err(explore_error)?;
            let model = edge.inner.model();
            point_outcome(py, project_with(point, &view, (), options), |key| {
                edge_hit(py, &model, *key)
            })
        }
        (Source::Point(point), Target::Face(face)) => {
            let mode = surface_mode(along)?;
            let view = face.inner.view().map_err(explore_error)?;
            let model = face.inner.model();
            point_outcome(py, project_with(point, &view, mode, options), |key| {
                face_hit(py, &model, *key)
            })
        }
        (Source::Point(point), Target::Faces(faces)) => {
            let mode = surface_mode(along)?;
            let (model, keys) = face_list(faces)?;
            let list = FaceList::new(model.model(), &keys);
            point_outcome(py, project_with(point, &list, mode, options), |key| {
                face_hit(py, &model, *key)
            })
        }
        (_, Target::Plane(plane)) => {
            let mode = surface_mode(along)?;
            carry(py, source, plane, mode, options, no_hit)
        }
        (_, Target::Face(face)) => {
            let mode = surface_mode(along)?;
            let view = face.inner.view().map_err(explore_error)?;
            let model = face.inner.model();
            carry(py, source, &view, mode, options, |py, key| {
                face_hit(py, &model, *key)
            })
        }
        (_, Target::Faces(faces)) => {
            let mode = surface_mode(along)?;
            let (model, keys) = face_list(faces)?;
            let list = FaceList::new(model.model(), &keys);
            carry(py, source, &list, mode, options, |py, key| {
                face_hit(py, &model, *key)
            })
        }
        _ => Err(unsupported(source, target)),
    }
}
