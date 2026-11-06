use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::geometry::parameter::{Fraction, NativeParam};
use crate::solver::Target2d;

use super::super::geometry::PyPoint2;
use super::ids::{ArcId, CircleId, PointArg, SegmentId};

/// A temporary positional objective of one solve; each kind is a class deriving from this.
#[pyclass(module = "ngk.solver", subclass, frozen)]
pub(crate) struct Target {
    pub(crate) target: Target2d,
}

macro_rules! target {
    ($name:ident ( $($field:ident : $type:ty),* ) => $target:expr) => {
        #[pyclass(module = "ngk.solver", extends = Target, frozen)]
        pub(crate) struct $name;

        #[pymethods]
        impl $name {
            #[new]
            #[pyo3(signature = ($($field,)* position, weight = 1.0))]
            fn new($($field: $type,)* position: PyPoint2, weight: f64) -> PyClassInitializer<Self> {
                let position = position.point();
                let target = ($target)(position).with_weight(weight);
                PyClassInitializer::from(Target { target }).add_subclass(Self)
            }
        }
    };
}

target!(PointTarget(point: PointArg) => |position| Target2d::point(point.0, position));
target!(SegmentTarget(segment: SegmentId, fraction: f64) => |position| {
    Target2d::on_segment(segment.0, Fraction::new(fraction), position)
});
target!(ArcTarget(arc: ArcId, fraction: f64) => |position| {
    Target2d::on_arc(arc.0, Fraction::new(fraction), position)
});
target!(CircleTarget(circle: CircleId, angle: f64) => |position| {
    Target2d::on_circle(circle.0, NativeParam::new(angle), position)
});

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<Target>()?;
    module.add_class::<PointTarget>()?;
    module.add_class::<SegmentTarget>()?;
    module.add_class::<ArcTarget>()?;
    module.add_class::<CircleTarget>()?;
    Ok(())
}
