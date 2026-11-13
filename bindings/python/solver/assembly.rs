//! The assembly solver: `ngk.solver.assembly`.

use std::collections::HashMap;
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use pyo3::create_exception;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyModule};

use crate::solver::assembly as solver;

use super::super::geometry::{PyFrame, PyPoint3, PyRigid, point};

macro_rules! id_class {
    ($name:ident) => {
        #[doc = concat!("A `", stringify!($name), "`: issued by an assembly, or given back from its value.")]
        #[pyclass(module = "ngk.solver.assembly", frozen)]
        #[derive(Clone, Copy)]
        pub(crate) struct $name(pub(crate) solver::$name);

        #[pymethods]
        impl $name {
            #[new]
            fn new(value: u64) -> Self {
                Self(solver::$name::new(value))
            }

            #[getter]
            fn value(&self) -> u64 {
                self.0.value()
            }

            fn __eq__(&self, other: &Self) -> bool {
                self.0 == other.0
            }

            fn __hash__(&self) -> u64 {
                let mut hasher = DefaultHasher::new();
                self.0.hash(&mut hasher);
                hasher.finish()
            }

            fn __repr__(&self) -> String {
                format!("{}({})", stringify!($name), self.0.value())
            }
        }
    };
}

id_class!(BodyId);
id_class!(JointId);

/// What a joint lets the second frame do relative to the first.
#[pyclass(module = "ngk.solver.assembly", eq, eq_int, frozen)]
#[derive(Clone, Copy, PartialEq)]
pub(crate) enum JointKind {
    Fixed,
    Revolute,
    Prismatic,
    Helical,
    Cylindrical,
    Universal,
    Spherical,
    Planar,
    CylinderOnPlane,
    SphereInCylinder,
    SphereOnPlane,
}

impl JointKind {
    fn to_rust(self, pitch: Option<f64>) -> PyResult<solver::JointKind> {
        use solver::JointKind as K;
        if pitch.is_some() != (self == Self::Helical) {
            return Err(PyValueError::new_err(
                "a helical joint, and only it, takes a pitch",
            ));
        }
        Ok(match self {
            Self::Fixed => K::Fixed,
            Self::Revolute => K::Revolute,
            Self::Prismatic => K::Prismatic,
            Self::Helical => K::Helical {
                pitch: pitch.unwrap_or_default(),
            },
            Self::Cylindrical => K::Cylindrical,
            Self::Universal => K::Universal,
            Self::Spherical => K::Spherical,
            Self::Planar => K::Planar,
            Self::CylinderOnPlane => K::CylinderOnPlane,
            Self::SphereInCylinder => K::SphereInCylinder,
            Self::SphereOnPlane => K::SphereOnPlane,
        })
    }

    fn from_rust(kind: solver::JointKind) -> (Self, Option<f64>) {
        use solver::JointKind as K;
        match kind {
            K::Fixed => (Self::Fixed, None),
            K::Revolute => (Self::Revolute, None),
            K::Prismatic => (Self::Prismatic, None),
            K::Helical { pitch } => (Self::Helical, Some(pitch)),
            K::Cylindrical => (Self::Cylindrical, None),
            K::Universal => (Self::Universal, None),
            K::Spherical => (Self::Spherical, None),
            K::Planar => (Self::Planar, None),
            K::CylinderOnPlane => (Self::CylinderOnPlane, None),
            K::SphereInCylinder => (Self::SphereInCylinder, None),
            K::SphereOnPlane => (Self::SphereOnPlane, None),
        }
    }
}

/// A scalar coordinate of a joint: the second frame relative to the first.
#[pyclass(module = "ngk.solver.assembly", eq, eq_int, frozen, hash)]
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) enum Coordinate {
    Angle,
    Slide,
    X,
    Y,
    Roll,
    Alpha,
    Beta,
}

impl From<Coordinate> for solver::Coordinate {
    fn from(coordinate: Coordinate) -> Self {
        match coordinate {
            Coordinate::Angle => Self::Angle,
            Coordinate::Slide => Self::Slide,
            Coordinate::X => Self::X,
            Coordinate::Y => Self::Y,
            Coordinate::Roll => Self::Roll,
            Coordinate::Alpha => Self::Alpha,
            Coordinate::Beta => Self::Beta,
        }
    }
}

impl From<solver::Coordinate> for Coordinate {
    fn from(coordinate: solver::Coordinate) -> Self {
        match coordinate {
            solver::Coordinate::Angle => Self::Angle,
            solver::Coordinate::Slide => Self::Slide,
            solver::Coordinate::X => Self::X,
            solver::Coordinate::Y => Self::Y,
            solver::Coordinate::Roll => Self::Roll,
            solver::Coordinate::Alpha => Self::Alpha,
            solver::Coordinate::Beta => Self::Beta,
        }
    }
}

/// Bounds a coordinate keeps within, either side optional.
#[pyclass(module = "ngk.solver.assembly", frozen, get_all)]
#[derive(Clone, Copy)]
pub(crate) struct Limit {
    min: Option<f64>,
    max: Option<f64>,
}

#[pymethods]
impl Limit {
    #[new]
    #[pyo3(signature = (min = None, max = None))]
    fn new(min: Option<f64>, max: Option<f64>) -> Self {
        Self { min, max }
    }
}

/// Where a joint holds a body: a frame in that body's own coordinates.
#[pyclass(module = "ngk.solver.assembly", frozen)]
#[derive(Clone)]
pub(crate) struct Attachment(solver::Attachment);

#[pymethods]
impl Attachment {
    #[new]
    fn new(body: BodyId, frame: PyFrame) -> Self {
        Self(solver::Attachment::new(body.0, frame.frame))
    }

    #[getter]
    fn body(&self) -> BodyId {
        BodyId(self.0.body)
    }

    #[getter]
    fn frame(&self) -> PyFrame {
        PyFrame {
            frame: self.0.frame.clone(),
        }
    }
}

/// A permanent joint between two bodies, each held by a frame; its coordinates express
/// the second frame in the first.
///
/// `pitch` is the slide per turn of a helical joint, and only of it. `values` imposes
/// coordinates, `limits` bounds them, and `cone` bounds a free rotation by the largest
/// angle between the two Z axes. Angles are radians; they are checked when the joint is
/// added.
#[pyclass(module = "ngk.solver.assembly", frozen)]
#[derive(Clone)]
pub(crate) struct Joint(solver::Joint3d);

#[pymethods]
impl Joint {
    #[new]
    #[pyo3(signature = (kind, first, second, *, pitch = None, values = None, limits = None, cone = None))]
    fn new(
        kind: JointKind,
        first: Attachment,
        second: Attachment,
        pitch: Option<f64>,
        values: Option<HashMap<Coordinate, f64>>,
        limits: Option<HashMap<Coordinate, Limit>>,
        cone: Option<f64>,
    ) -> PyResult<Self> {
        let mut joint = solver::Joint3d::new(kind.to_rust(pitch)?, first.0, second.0);
        for (coordinate, value) in values.unwrap_or_default() {
            joint = joint.with_value(coordinate.into(), value);
        }
        for (coordinate, limit) in limits.unwrap_or_default() {
            joint = joint.with_limit(
                coordinate.into(),
                solver::Limit {
                    min: limit.min,
                    max: limit.max,
                },
            );
        }
        joint.cone = cone;
        Ok(Self(joint))
    }

    #[getter]
    fn kind(&self) -> JointKind {
        JointKind::from_rust(self.0.kind).0
    }

    #[getter]
    fn pitch(&self) -> Option<f64> {
        JointKind::from_rust(self.0.kind).1
    }

    #[getter]
    fn first(&self) -> Attachment {
        Attachment(self.0.first.clone())
    }

    #[getter]
    fn second(&self) -> Attachment {
        Attachment(self.0.second.clone())
    }

    #[getter]
    fn values(&self) -> HashMap<Coordinate, f64> {
        self.0
            .values
            .iter()
            .map(|(coordinate, value)| ((*coordinate).into(), *value))
            .collect()
    }

    #[getter]
    fn limits(&self) -> HashMap<Coordinate, Limit> {
        self.0
            .limits
            .iter()
            .map(|(coordinate, limit)| {
                (
                    (*coordinate).into(),
                    Limit {
                        min: limit.min,
                        max: limit.max,
                    },
                )
            })
            .collect()
    }

    #[getter]
    fn cone(&self) -> Option<f64> {
        self.0.cone
    }
}

/// A temporary objective of one solve: bring `point`, in the body's own coordinates,
/// toward the world `position`.
#[pyclass(module = "ngk.solver.assembly", frozen)]
pub(crate) struct Target(solver::Target3d);

#[pymethods]
impl Target {
    #[new]
    #[pyo3(signature = (body, point, position, weight = 1.0))]
    fn new(body: BodyId, point: PyPoint3, position: PyPoint3, weight: f64) -> Self {
        Self(solver::Target3d::point(body.0, point.point, position.point).with_weight(weight))
    }
}

/// One side of a joint's limit: `side` is `"min"`, `"max"` or `"cone"`, and
/// `coordinate` is `None` for a cone.
#[pyclass(module = "ngk.solver.assembly", frozen, get_all)]
#[derive(Clone)]
pub(crate) struct ActiveLimit {
    joint: JointId,
    side: &'static str,
    coordinate: Option<Coordinate>,
}

impl ActiveLimit {
    fn new(joint: solver::JointId, bound: solver::Bound) -> Self {
        let (side, coordinate) = match bound {
            solver::Bound::Min(coordinate) => ("min", Some(coordinate.into())),
            solver::Bound::Max(coordinate) => ("max", Some(coordinate.into())),
            solver::Bound::Cone => ("cone", None),
        };
        Self {
            joint: JointId(joint),
            side,
            coordinate,
        }
    }
}

/// A joint's residuals in the caller's units and radians, and their largest ratio to the
/// tolerance.
#[pyclass(module = "ngk.solver.assembly", frozen, get_all)]
#[derive(Clone)]
pub(crate) struct JointResidual {
    joint: JointId,
    values: Vec<f64>,
    max_tolerance_ratio: f64,
}

impl From<&solver::JointResidual> for JointResidual {
    fn from(residual: &solver::JointResidual) -> Self {
        Self {
            joint: JointId(residual.joint),
            values: residual.values.clone(),
            max_tolerance_ratio: residual.max_tolerance_ratio,
        }
    }
}

#[pyclass(module = "ngk.solver.assembly", frozen, get_all)]
#[derive(Clone)]
pub(crate) struct TargetReport {
    index: usize,
    achieved: PyPoint3,
    distance: f64,
    reached: bool,
}

/// Diagnostics of the joints, linearized at the solution; targets and limits consume
/// no freedom.
#[pyclass(module = "ngk.solver.assembly", frozen)]
pub(crate) struct SolveReport(solver::SolveReport);

#[pymethods]
impl SolveReport {
    #[getter]
    fn mobility(&self) -> usize {
        self.0.mobility
    }

    /// The degree of hyperstatism: joint equations depending on others.
    #[getter]
    fn redundancy(&self) -> usize {
        self.0.redundancy
    }

    /// The freedom each body keeps. A motion shared by several counts in each.
    #[getter]
    fn freedoms<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        let freedoms = PyDict::new_bound(py);
        for freedom in &self.0.freedoms {
            freedoms.set_item(BodyId(freedom.body).into_py(py), freedom.degrees)?;
        }
        Ok(freedoms)
    }

    /// Per joint, how many of its equations depend on the joints before it.
    #[getter]
    fn redundancies<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        let redundancies = PyDict::new_bound(py);
        for redundancy in &self.0.redundancies {
            redundancies.set_item(JointId(redundancy.joint).into_py(py), redundancy.equations)?;
        }
        Ok(redundancies)
    }

    #[getter]
    fn active_limits(&self) -> Vec<ActiveLimit> {
        self.0
            .active_limits
            .iter()
            .map(|limit| ActiveLimit::new(limit.joint, limit.bound))
            .collect()
    }

    #[getter]
    fn residuals(&self) -> Vec<JointResidual> {
        self.0.residuals.iter().map(Into::into).collect()
    }

    #[getter]
    fn targets(&self) -> Vec<TargetReport> {
        self.0
            .targets
            .iter()
            .map(|target| TargetReport {
                index: target.index,
                achieved: point(target.achieved),
                distance: target.distance,
                reached: target.reached,
            })
            .collect()
    }

    #[getter]
    fn iterations(&self) -> usize {
        self.0.iterations
    }

    #[getter]
    fn target_iterations(&self) -> usize {
        self.0.target_iterations
    }

    #[getter]
    fn target_budget_exhausted(&self) -> bool {
        self.0.target_budget_exhausted
    }
}

/// An immutable result: its poses survive later edits.
#[pyclass(module = "ngk.solver.assembly", frozen)]
pub(crate) struct Solution3d(solver::Solution3d);

#[pymethods]
impl Solution3d {
    /// Where a body's own coordinates land in the world.
    fn pose(&self, body: BodyId) -> PyResult<PyRigid> {
        let inner = self.0.pose(body.0).map_err(edit_error)?;
        Ok(PyRigid { inner })
    }

    /// A joint's coordinate: an angle in radians, unwrapped, or a length.
    fn coordinate(&self, joint: JointId, coordinate: Coordinate) -> PyResult<f64> {
        self.0
            .coordinate(joint.0, coordinate.into())
            .map_err(edit_error)
    }

    /// The pose of a joint's second frame in its first.
    fn relative(&self, joint: JointId) -> PyResult<PyRigid> {
        let inner = self.0.relative(joint.0).map_err(edit_error)?;
        Ok(PyRigid { inner })
    }

    #[getter]
    fn report(&self) -> SolveReport {
        SolveReport(self.0.report().clone())
    }
}

/// Rigid bodies and the joints between them.
///
/// A body is a pose with no shape. `add_*` issues the id of what it adds; `insert_*`
/// takes an id issued earlier back, to rebuild an assembly from stored ids.
#[pyclass(module = "ngk.solver.assembly")]
pub(crate) struct Assembly3d(solver::Assembly3d);

#[pymethods]
impl Assembly3d {
    #[new]
    fn new() -> Self {
        Self(solver::Assembly3d::default())
    }

    fn add_body(&mut self, seed: PyRigid) -> PyResult<BodyId> {
        Ok(BodyId(self.0.add_body(seed.inner).map_err(edit_error)?))
    }

    fn insert_body(&mut self, id: BodyId, seed: PyRigid) -> PyResult<()> {
        self.0.insert_body(id.0, seed.inner).map_err(edit_error)
    }

    /// The pose a body starts the next solve from.
    fn body(&self, id: BodyId) -> PyResult<PyRigid> {
        let inner = self.0.body(id.0).map_err(edit_error)?;
        Ok(PyRigid { inner })
    }

    fn update_body(&mut self, id: BodyId, seed: PyRigid) -> PyResult<()> {
        self.0.update_body(id.0, seed.inner).map_err(edit_error)
    }

    /// Holds a body at its current seed; fixing it again recaptures the seed.
    fn fix(&mut self, id: BodyId) -> PyResult<()> {
        self.0.fix(id.0).map_err(edit_error)
    }

    fn unfix(&mut self, id: BodyId) -> PyResult<()> {
        self.0.unfix(id.0).map_err(edit_error)
    }

    /// The pose a fixed body is held at, or `None` for a free body.
    fn fixing(&self, id: BodyId) -> PyResult<Option<PyRigid>> {
        let fixing = self.0.fixing(id.0).map_err(edit_error)?;
        Ok(fixing.map(|inner| PyRigid { inner }))
    }

    /// Removes a body no joint holds.
    fn remove_body(&mut self, id: BodyId) -> PyResult<()> {
        self.0.remove_body(id.0).map_err(edit_error)
    }

    /// Removes a body and the joints holding it, and returns those.
    fn remove_cascade(&mut self, id: BodyId) -> PyResult<Vec<JointId>> {
        let removal = self.0.remove_cascade(id.0).map_err(edit_error)?;
        Ok(removal.joints.into_iter().map(JointId).collect())
    }

    fn add_joint(&mut self, joint: &Joint) -> PyResult<JointId> {
        Ok(JointId(
            self.0.add_joint(joint.0.clone()).map_err(edit_error)?,
        ))
    }

    fn insert_joint(&mut self, id: JointId, joint: &Joint) -> PyResult<()> {
        self.0
            .insert_joint(id.0, joint.0.clone())
            .map_err(edit_error)
    }

    fn joint(&self, id: JointId) -> PyResult<Joint> {
        Ok(Joint(self.0.joint(id.0).map_err(edit_error)?.clone()))
    }

    fn update_joint(&mut self, id: JointId, joint: &Joint) -> PyResult<()> {
        self.0
            .update_joint(id.0, joint.0.clone())
            .map_err(edit_error)
    }

    fn remove_joint(&mut self, id: JointId) -> PyResult<()> {
        self.0.remove_joint(id.0).map_err(edit_error)
    }

    /// Sets the unwrapped value an angle of a joint starts from.
    fn set_coordinate_seed(
        &mut self,
        id: JointId,
        coordinate: Coordinate,
        value: f64,
    ) -> PyResult<()> {
        self.0
            .set_coordinate_seed(id.0, coordinate.into(), value)
            .map_err(edit_error)
    }

    fn coordinate_seed(&self, id: JointId, coordinate: Coordinate) -> PyResult<Option<f64>> {
        self.0
            .coordinate_seed(id.0, coordinate.into())
            .map_err(edit_error)
    }

    #[getter]
    fn body_count(&self) -> usize {
        self.0.body_count()
    }

    #[getter]
    fn joint_count(&self) -> usize {
        self.0.joint_count()
    }

    /// Solves from the seeds. Failure leaves the assembly as it was.
    fn solve(&mut self, py: Python<'_>) -> PyResult<Solution3d> {
        let solution = self.0.solve().map_err(|error| solve_error(py, error))?;
        Ok(Solution3d(solution))
    }

    /// Moves toward temporary targets while keeping every joint and limit.
    fn solve_toward(
        &mut self,
        py: Python<'_>,
        targets: Vec<Bound<'_, Target>>,
    ) -> PyResult<Solution3d> {
        let targets: Vec<_> = targets
            .iter()
            .map(|target| target.get().0.clone())
            .collect();
        let solution = self
            .0
            .solve_toward(&targets)
            .map_err(|error| solve_error(py, error))?;
        Ok(Solution3d(solution))
    }

    /// The bodies, joints, fixings and seeds; not the solutions.
    fn to_json(&self) -> PyResult<String> {
        serde_json::to_string(&self.0).map_err(|error| PyValueError::new_err(error.to_string()))
    }

    /// Reads what `to_json` wrote, refusing what an edit could not have produced.
    #[staticmethod]
    fn from_json(json: &str) -> PyResult<Self> {
        serde_json::from_str(json)
            .map(Self)
            .map_err(|error| PyValueError::new_err(error.to_string()))
    }
}

create_exception!(
    ngk.solver.assembly,
    NonConvergenceError,
    PyValueError,
    "Local solving did not converge: `joints` names the connected group, not a minimal conflict."
);
create_exception!(
    ngk.solver.assembly,
    LimitError,
    PyValueError,
    "A joint cannot keep within its limit: `limit` names the joint and the side."
);

fn solve_error(py: Python<'_>, error: solver::SolveError) -> PyErr {
    let message = error.to_string();
    let with = |exception: PyErr, name: &str, value: PyObject| {
        exception
            .value_bound(py)
            .setattr(name, value)
            .map_or_else(|failure| failure, |()| exception)
    };
    match &error {
        solver::SolveError::NonConvergence { joints, residuals } => {
            let joints: Vec<JointId> = joints.iter().copied().map(JointId).collect();
            let residuals: Vec<JointResidual> = residuals.iter().map(Into::into).collect();
            let exception = with(
                NonConvergenceError::new_err(message),
                "joints",
                joints.into_py(py),
            );
            with(exception, "residuals", residuals.into_py(py))
        }
        solver::SolveError::Limit { joint, bound } => with(
            LimitError::new_err(message),
            "limit",
            ActiveLimit::new(*joint, *bound).into_py(py),
        ),
        _ => PyValueError::new_err(message),
    }
}

fn edit_error(error: solver::EditError) -> PyErr {
    PyValueError::new_err(error.to_string())
}

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    let py = module.py();
    module.add_class::<BodyId>()?;
    module.add_class::<JointId>()?;
    module.add_class::<JointKind>()?;
    module.add_class::<Coordinate>()?;
    module.add_class::<Limit>()?;
    module.add_class::<Attachment>()?;
    module.add_class::<Joint>()?;
    module.add_class::<Target>()?;
    module.add_class::<ActiveLimit>()?;
    module.add_class::<JointResidual>()?;
    module.add_class::<TargetReport>()?;
    module.add_class::<SolveReport>()?;
    module.add_class::<Solution3d>()?;
    module.add_class::<Assembly3d>()?;
    module.add(
        "NonConvergenceError",
        py.get_type_bound::<NonConvergenceError>(),
    )?;
    module.add("LimitError", py.get_type_bound::<LimitError>())?;
    Ok(())
}
