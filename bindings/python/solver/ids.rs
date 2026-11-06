use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::solver;

macro_rules! id_class {
    ($name:ident, { $($extra:tt)* }) => {
        #[doc = concat!("A `", stringify!($name), "`: issued by a system, or given back from its value.")]
        #[pyclass(module = "ngk.solver", frozen)]
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

            $($extra)*
        }
    };
}

id_class!(PointId, {});
id_class!(ConstraintId, {});
id_class!(SegmentId, {
    fn start(&self) -> PointRef {
        PointRef(self.0.start())
    }
    fn end(&self) -> PointRef {
        PointRef(self.0.end())
    }
});
id_class!(CircleId, {
    fn center(&self) -> PointRef {
        PointRef(self.0.center())
    }
});
id_class!(ArcId, {
    fn start(&self) -> PointRef {
        PointRef(self.0.start())
    }
    fn end(&self) -> PointRef {
        PointRef(self.0.end())
    }
    fn center(&self) -> PointRef {
        PointRef(self.0.center())
    }
});

/// A point of a primitive: read off its id, as `segment.end()`.
#[pyclass(module = "ngk.solver", frozen)]
#[derive(Clone, Copy)]
pub(crate) struct PointRef(pub(crate) solver::PointRef);

#[pymethods]
impl PointRef {
    fn __eq__(&self, other: &Self) -> bool {
        self.0 == other.0
    }

    fn __hash__(&self) -> u64 {
        let mut hasher = DefaultHasher::new();
        self.0.hash(&mut hasher);
        hasher.finish()
    }

    fn __repr__(&self) -> String {
        format!("{:?}", self.0)
    }
}

/// What can stand for a point: a `PointRef`, or the `PointId` of an independent point.
#[derive(Clone, Copy)]
pub(crate) struct PointArg(pub(crate) solver::PointRef);

impl<'py> FromPyObject<'py> for PointArg {
    fn extract_bound(object: &Bound<'py, PyAny>) -> PyResult<Self> {
        if let Ok(point) = object.extract::<PointRef>() {
            return Ok(Self(point.0));
        }
        Ok(Self(object.extract::<PointId>()?.0.into()))
    }
}

impl IntoPy<PyObject> for PointArg {
    fn into_py(self, py: Python<'_>) -> PyObject {
        PointRef(self.0).into_py(py)
    }
}

/// A circle or an arc, whichever a circular relation was given.
#[derive(Clone, Copy, FromPyObject)]
pub(crate) enum CircularArg {
    Circle(CircleId),
    Arc(ArcId),
}

impl From<CircularArg> for solver::CircularId {
    fn from(argument: CircularArg) -> Self {
        match argument {
            CircularArg::Circle(id) => id.0.into(),
            CircularArg::Arc(id) => id.0.into(),
        }
    }
}

impl From<solver::CircularId> for CircularArg {
    fn from(id: solver::CircularId) -> Self {
        match solver::AnyId::from(solver::GeometryId::from(id)) {
            solver::AnyId::Circle(id) => Self::Circle(CircleId(id)),
            solver::AnyId::Arc(id) => Self::Arc(ArcId(id)),
            solver::AnyId::Point(_) | solver::AnyId::Segment(_) => {
                unreachable!("a circular id is a circle or an arc")
            }
        }
    }
}

impl IntoPy<PyObject> for CircularArg {
    fn into_py(self, py: Python<'_>) -> PyObject {
        match self {
            Self::Circle(id) => id.into_py(py),
            Self::Arc(id) => id.into_py(py),
        }
    }
}

/// The id of any primitive.
#[derive(Clone, Copy, FromPyObject)]
pub(crate) enum GeometryArg {
    Point(PointId),
    Segment(SegmentId),
    Circle(CircleId),
    Arc(ArcId),
}

impl From<GeometryArg> for solver::GeometryId {
    fn from(argument: GeometryArg) -> Self {
        match argument {
            GeometryArg::Point(id) => id.0.into(),
            GeometryArg::Segment(id) => id.0.into(),
            GeometryArg::Circle(id) => id.0.into(),
            GeometryArg::Arc(id) => id.0.into(),
        }
    }
}

impl From<solver::GeometryId> for GeometryArg {
    fn from(id: solver::GeometryId) -> Self {
        match solver::AnyId::from(id) {
            solver::AnyId::Point(id) => Self::Point(PointId(id)),
            solver::AnyId::Segment(id) => Self::Segment(SegmentId(id)),
            solver::AnyId::Circle(id) => Self::Circle(CircleId(id)),
            solver::AnyId::Arc(id) => Self::Arc(ArcId(id)),
        }
    }
}

impl IntoPy<PyObject> for GeometryArg {
    fn into_py(self, py: Python<'_>) -> PyObject {
        match self {
            Self::Point(id) => id.into_py(py),
            Self::Segment(id) => id.into_py(py),
            Self::Circle(id) => id.into_py(py),
            Self::Arc(id) => id.into_py(py),
        }
    }
}

/// The id of a primitive that is a curve.
#[derive(Clone, Copy, FromPyObject)]
pub(crate) enum CurveArg {
    Segment(SegmentId),
    Circle(CircleId),
    Arc(ArcId),
}

impl From<CurveArg> for solver::CurveId {
    fn from(argument: CurveArg) -> Self {
        match argument {
            CurveArg::Segment(id) => id.0.into(),
            CurveArg::Circle(id) => id.0.into(),
            CurveArg::Arc(id) => id.0.into(),
        }
    }
}

impl From<solver::CurveId> for CurveArg {
    fn from(id: solver::CurveId) -> Self {
        match id {
            solver::CurveId::Segment(id) => Self::Segment(SegmentId(id)),
            solver::CurveId::Circle(id) => Self::Circle(CircleId(id)),
            solver::CurveId::Arc(id) => Self::Arc(ArcId(id)),
        }
    }
}

impl IntoPy<PyObject> for CurveArg {
    fn into_py(self, py: Python<'_>) -> PyObject {
        match self {
            Self::Segment(id) => id.into_py(py),
            Self::Circle(id) => id.into_py(py),
            Self::Arc(id) => id.into_py(py),
        }
    }
}

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<PointId>()?;
    module.add_class::<SegmentId>()?;
    module.add_class::<CircleId>()?;
    module.add_class::<ArcId>()?;
    module.add_class::<ConstraintId>()?;
    module.add_class::<PointRef>()?;
    Ok(())
}
