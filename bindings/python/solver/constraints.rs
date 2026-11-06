use pyo3::prelude::*;
use pyo3::types::{PyModule, PyTuple};
use radians::Rad64;

use crate::solver::{self, Constraint2d};

use super::ids::{CircularArg, GeometryArg, PointArg, SegmentId};

/// The contact branch of two circles.
#[pyclass(module = "ngk.solver.constraints", eq, eq_int, frozen)]
#[derive(Clone, Copy, PartialEq)]
pub(crate) enum TangencyMode {
    Automatic,
    External,
    Internal,
    FirstContainsSecond,
    SecondContainsFirst,
}

impl From<TangencyMode> for solver::TangencyMode {
    fn from(mode: TangencyMode) -> Self {
        match mode {
            TangencyMode::Automatic => Self::Automatic,
            TangencyMode::External => Self::External,
            TangencyMode::Internal => Self::Internal,
            TangencyMode::FirstContainsSecond => Self::FirstContainsSecond,
            TangencyMode::SecondContainsFirst => Self::SecondContainsFirst,
        }
    }
}

impl From<solver::TangencyMode> for TangencyMode {
    fn from(mode: solver::TangencyMode) -> Self {
        match mode {
            solver::TangencyMode::Automatic => Self::Automatic,
            solver::TangencyMode::External => Self::External,
            solver::TangencyMode::Internal => Self::Internal,
            solver::TangencyMode::FirstContainsSecond => Self::FirstContainsSecond,
            solver::TangencyMode::SecondContainsFirst => Self::SecondContainsFirst,
        }
    }
}

/// A permanent relation; each one is a class deriving from this.
#[pyclass(module = "ngk.solver.constraints", subclass, frozen)]
pub(crate) struct Constraint {
    pub(crate) definition: Constraint2d,
}

#[pymethods]
impl Constraint {
    fn __eq__(&self, other: &Self) -> bool {
        self.definition == other.definition
    }
}

/// Declares one relation class: its fields, in the order `match` reads them, and the
/// solver relation they make.
macro_rules! constraint {
    (
        $name:ident ( $($field:ident : $type:ty),* ) [ $($signature:tt)* ] => $definition:expr
    ) => {
        #[pyclass(module = "ngk.solver.constraints", extends = Constraint, frozen)]
        pub(crate) struct $name {
            $(#[pyo3(get)] $field: $type,)*
        }

        #[pymethods]
        impl $name {
            #[new]
            #[pyo3(signature = ($($signature)*))]
            pub(crate) fn new($($field: $type),*) -> PyClassInitializer<Self> {
                let definition = $definition;
                PyClassInitializer::from(Constraint { definition }).add_subclass(Self { $($field),* })
            }

            #[classattr]
            fn __match_args__(py: Python<'_>) -> Py<PyTuple> {
                let names: &[&str] = &[$(stringify!($field)),*];
                PyTuple::new_bound(py, names).unbind()
            }

            fn __repr__(&self, py: Python<'_>) -> PyResult<String> {
                let fields: Vec<String> = vec![$(
                    self.$field.clone().into_py(py).bind(py).repr()?.to_string()
                ),*];
                Ok(format!("{}({})", stringify!($name), fields.join(", ")))
            }
        }
    };
}

constraint!(Coincident(first: PointArg, second: PointArg) [first, second] =>
    Constraint2d::Coincident(first.0, second.0));
constraint!(PointOnLine(point: PointArg, line: SegmentId) [point, line] =>
    Constraint2d::PointOnLine(point.0, line.0));
constraint!(PointOnCircle(point: PointArg, circle: CircularArg) [point, circle] =>
    Constraint2d::PointOnCircle(point.0, circle.into()));
constraint!(Horizontal(line: SegmentId) [line] => Constraint2d::Horizontal(line.0));
constraint!(Vertical(line: SegmentId) [line] => Constraint2d::Vertical(line.0));
constraint!(Distance(first: PointArg, second: PointArg, value: f64) [first, second, value] =>
    Constraint2d::Distance(first.0, second.0, value));
constraint!(DistanceToLine(point: PointArg, line: SegmentId, value: f64) [point, line, value] =>
    Constraint2d::DistanceToLine(point.0, line.0, value));
constraint!(OffsetX(first: PointArg, second: PointArg, value: f64) [first, second, value] =>
    Constraint2d::OffsetX(first.0, second.0, value));
constraint!(OffsetY(first: PointArg, second: PointArg, value: f64) [first, second, value] =>
    Constraint2d::OffsetY(first.0, second.0, value));
constraint!(Length(line: SegmentId, value: f64) [line, value] =>
    Constraint2d::Length(line.0, value));
constraint!(Radius(circle: CircularArg, value: f64) [circle, value] =>
    Constraint2d::Radius(circle.into(), value));
constraint!(EqualLength(first: SegmentId, second: SegmentId) [first, second] =>
    Constraint2d::EqualLength(first.0, second.0));
constraint!(EqualRadius(first: CircularArg, second: CircularArg) [first, second] =>
    Constraint2d::EqualRadius(first.into(), second.into()));
constraint!(Parallel(first: SegmentId, second: SegmentId) [first, second] =>
    Constraint2d::Parallel(first.0, second.0));
constraint!(Perpendicular(first: SegmentId, second: SegmentId) [first, second] =>
    Constraint2d::Perpendicular(first.0, second.0));
constraint!(Angle(first: SegmentId, second: SegmentId, radians: f64, oriented: bool)
[first, second, radians, oriented = false] =>
Constraint2d::Angle {
    first: first.0,
    second: second.0,
    angle: Rad64::new(radians),
    oriented,
});
constraint!(Concentric(first: CircularArg, second: CircularArg) [first, second] =>
    Constraint2d::Concentric(first.into(), second.into()));
constraint!(Midpoint(point: PointArg, start: PointArg, end: PointArg) [point, start, end] =>
    Constraint2d::Midpoint { point: point.0, start: start.0, end: end.0 });
constraint!(SymmetricAboutLine(first: PointArg, second: PointArg, line: SegmentId)
    [first, second, line] =>
    Constraint2d::SymmetricAboutLine { first: first.0, second: second.0, line: line.0 });
constraint!(SymmetricAboutPoint(first: PointArg, second: PointArg, center: PointArg)
    [first, second, center] =>
    Constraint2d::SymmetricAboutPoint { first: first.0, second: second.0, center: center.0 });
constraint!(FixedPoint(point: PointArg) [point] => Constraint2d::FixedPoint(point.0));
constraint!(Fixed(primitive: GeometryArg) [primitive] => Constraint2d::Fixed(primitive.into()));
constraint!(TangentLineCircle(line: SegmentId, circle: CircularArg) [line, circle] =>
    Constraint2d::TangentLineCircle(line.0, circle.into()));
constraint!(TangentCircles(first: CircularArg, second: CircularArg, mode: TangencyMode)
[first, second, mode = TangencyMode::Automatic] =>
Constraint2d::TangentCircles {
    first: first.into(),
    second: second.into(),
    mode: mode.into(),
});

/// The class of a relation, as `System2d.constraint` reads it back.
pub(crate) fn constraint_to_python(
    py: Python<'_>,
    definition: &Constraint2d,
) -> PyResult<PyObject> {
    macro_rules! object {
        ($class:ident ( $($argument:expr),* )) => {
            Ok(Py::new(py, $class::new($($argument),*))?.into_py(py))
        };
    }
    let segment = |id: solver::SegmentId| SegmentId(id);
    match definition.clone() {
        Constraint2d::Coincident(a, b) => object!(Coincident(PointArg(a), PointArg(b))),
        Constraint2d::PointOnLine(p, l) => object!(PointOnLine(PointArg(p), segment(l))),
        Constraint2d::PointOnCircle(p, c) => object!(PointOnCircle(PointArg(p), c.into())),
        Constraint2d::Horizontal(l) => object!(Horizontal(segment(l))),
        Constraint2d::Vertical(l) => object!(Vertical(segment(l))),
        Constraint2d::Distance(a, b, v) => object!(Distance(PointArg(a), PointArg(b), v)),
        Constraint2d::DistanceToLine(p, l, v) => {
            object!(DistanceToLine(PointArg(p), segment(l), v))
        }
        Constraint2d::OffsetX(a, b, v) => object!(OffsetX(PointArg(a), PointArg(b), v)),
        Constraint2d::OffsetY(a, b, v) => object!(OffsetY(PointArg(a), PointArg(b), v)),
        Constraint2d::Length(l, v) => object!(Length(segment(l), v)),
        Constraint2d::Radius(c, v) => object!(Radius(c.into(), v)),
        Constraint2d::EqualLength(a, b) => object!(EqualLength(segment(a), segment(b))),
        Constraint2d::EqualRadius(a, b) => object!(EqualRadius(a.into(), b.into())),
        Constraint2d::Parallel(a, b) => object!(Parallel(segment(a), segment(b))),
        Constraint2d::Perpendicular(a, b) => object!(Perpendicular(segment(a), segment(b))),
        Constraint2d::Angle {
            first,
            second,
            angle,
            oriented,
        } => object!(Angle(
            segment(first),
            segment(second),
            angle.val(),
            oriented
        )),
        Constraint2d::Concentric(a, b) => object!(Concentric(a.into(), b.into())),
        Constraint2d::Midpoint { point, start, end } => {
            object!(Midpoint(PointArg(point), PointArg(start), PointArg(end)))
        }
        Constraint2d::SymmetricAboutLine {
            first,
            second,
            line,
        } => object!(SymmetricAboutLine(
            PointArg(first),
            PointArg(second),
            segment(line)
        )),
        Constraint2d::SymmetricAboutPoint {
            first,
            second,
            center,
        } => object!(SymmetricAboutPoint(
            PointArg(first),
            PointArg(second),
            PointArg(center)
        )),
        Constraint2d::FixedPoint(p) => object!(FixedPoint(PointArg(p))),
        Constraint2d::Fixed(g) => object!(Fixed(g.into())),
        Constraint2d::TangentLineCircle(l, c) => object!(TangentLineCircle(segment(l), c.into())),
        Constraint2d::TangentCircles {
            first,
            second,
            mode,
        } => object!(TangentCircles(first.into(), second.into(), mode.into())),
    }
}

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<TangencyMode>()?;
    module.add_class::<Constraint>()?;
    module.add_class::<Coincident>()?;
    module.add_class::<PointOnLine>()?;
    module.add_class::<PointOnCircle>()?;
    module.add_class::<Horizontal>()?;
    module.add_class::<Vertical>()?;
    module.add_class::<Distance>()?;
    module.add_class::<DistanceToLine>()?;
    module.add_class::<OffsetX>()?;
    module.add_class::<OffsetY>()?;
    module.add_class::<Length>()?;
    module.add_class::<Radius>()?;
    module.add_class::<EqualLength>()?;
    module.add_class::<EqualRadius>()?;
    module.add_class::<Parallel>()?;
    module.add_class::<Perpendicular>()?;
    module.add_class::<Angle>()?;
    module.add_class::<Concentric>()?;
    module.add_class::<Midpoint>()?;
    module.add_class::<SymmetricAboutLine>()?;
    module.add_class::<SymmetricAboutPoint>()?;
    module.add_class::<FixedPoint>()?;
    module.add_class::<Fixed>()?;
    module.add_class::<TangentLineCircle>()?;
    module.add_class::<TangentCircles>()?;
    Ok(())
}
