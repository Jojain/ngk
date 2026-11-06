use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyModule;

use crate::solver;

use super::super::geometry::PyPoint2;
use super::constraints::{Constraint, constraint_to_python};
use super::errors::{edit_error, solve_error};
use super::ids::{ArcId, CircleId, ConstraintId, GeometryArg, PointArg, PointId, SegmentId};
use super::report::SolveReport;
use super::targets::Target;
use super::values::{Arc2, Circle2, Segment2};

/// An editable 2D constraint system.
///
/// `add_*` issues the id of what it adds. `insert_*` takes an id issued earlier back, to
/// rebuild a system from stored ids.
#[pyclass(module = "ngk.solver")]
pub(crate) struct System2d(pub(crate) solver::System2d);

#[pymethods]
impl System2d {
    #[new]
    fn new() -> Self {
        Self(solver::System2d::default())
    }

    fn add_point(&mut self, point: PyPoint2) -> PyResult<PointId> {
        let id = self.0.add_point(point.point()).map_err(edit_error)?;
        Ok(PointId(id))
    }

    fn add_segment(&mut self, segment: Segment2) -> PyResult<SegmentId> {
        let id = self.0.add_segment(segment.0).map_err(edit_error)?;
        Ok(SegmentId(id))
    }

    fn add_circle(&mut self, circle: Circle2) -> PyResult<CircleId> {
        let id = self.0.add_circle(circle.0).map_err(edit_error)?;
        Ok(CircleId(id))
    }

    fn add_arc(&mut self, arc: Arc2) -> PyResult<ArcId> {
        let id = self.0.add_arc(arc.0).map_err(edit_error)?;
        Ok(ArcId(id))
    }

    fn add_constraint(&mut self, constraint: &Bound<'_, Constraint>) -> PyResult<ConstraintId> {
        let definition = constraint.get().definition.clone();
        let id = self.0.add_constraint(definition).map_err(edit_error)?;
        Ok(ConstraintId(id))
    }

    fn insert_point(&mut self, id: PointId, point: PyPoint2) -> PyResult<()> {
        self.0.insert_point(id.0, point.point()).map_err(edit_error)
    }

    fn insert_segment(&mut self, id: SegmentId, segment: Segment2) -> PyResult<()> {
        self.0.insert_segment(id.0, segment.0).map_err(edit_error)
    }

    fn insert_circle(&mut self, id: CircleId, circle: Circle2) -> PyResult<()> {
        self.0.insert_circle(id.0, circle.0).map_err(edit_error)
    }

    fn insert_arc(&mut self, id: ArcId, arc: Arc2) -> PyResult<()> {
        self.0.insert_arc(id.0, arc.0).map_err(edit_error)
    }

    fn insert_constraint(
        &mut self,
        id: ConstraintId,
        constraint: &Bound<'_, Constraint>,
    ) -> PyResult<()> {
        let definition = constraint.get().definition.clone();
        self.0
            .insert_constraint(id.0, definition)
            .map_err(edit_error)
    }

    fn update_point(&mut self, id: PointId, point: PyPoint2) -> PyResult<()> {
        self.0.update_point(id.0, point.point()).map_err(edit_error)
    }

    fn update_segment(&mut self, id: SegmentId, segment: Segment2) -> PyResult<()> {
        self.0.update_segment(id.0, segment.0).map_err(edit_error)
    }

    fn update_circle(&mut self, id: CircleId, circle: Circle2) -> PyResult<()> {
        self.0.update_circle(id.0, circle.0).map_err(edit_error)
    }

    fn update_arc(&mut self, id: ArcId, arc: Arc2) -> PyResult<()> {
        self.0.update_arc(id.0, arc.0).map_err(edit_error)
    }

    fn update_constraint(
        &mut self,
        id: ConstraintId,
        constraint: &Bound<'_, Constraint>,
    ) -> PyResult<()> {
        let definition = constraint.get().definition.clone();
        self.0
            .update_constraint(id.0, definition)
            .map_err(edit_error)
    }

    fn remove_constraint(&mut self, id: ConstraintId) -> PyResult<()> {
        self.0.remove_constraint(id.0).map_err(edit_error)
    }

    /// Removes a primitive no relation references.
    fn remove_geometry(&mut self, id: GeometryArg) -> PyResult<()> {
        self.0
            .remove_geometry(solver::GeometryId::from(id))
            .map_err(edit_error)
    }

    /// Removes a primitive and the relations referencing it, and returns those.
    fn remove_cascade(&mut self, id: GeometryArg) -> PyResult<Vec<ConstraintId>> {
        let removal = self
            .0
            .remove_cascade(solver::GeometryId::from(id))
            .map_err(edit_error)?;
        Ok(removal.constraints.into_iter().map(ConstraintId).collect())
    }

    /// The relation as it was given, as its class.
    fn constraint(&self, py: Python<'_>, id: ConstraintId) -> PyResult<PyObject> {
        let definition = self.0.constraint(id.0).map_err(edit_error)?;
        constraint_to_python(py, definition)
    }

    /// The editable seed, which need not satisfy the relations.
    fn point(&self, point: PointArg) -> PyResult<PyPoint2> {
        Ok(self.0.point(point.0).map_err(edit_error)?.into())
    }

    fn segment(&self, id: SegmentId) -> PyResult<Segment2> {
        Ok(Segment2(self.0.segment(id.0).map_err(edit_error)?))
    }

    fn circle(&self, id: CircleId) -> PyResult<Circle2> {
        Ok(Circle2(self.0.circle(id.0).map_err(edit_error)?))
    }

    fn arc(&self, id: ArcId) -> PyResult<Arc2> {
        Ok(Arc2(self.0.arc(id.0).map_err(edit_error)?))
    }

    #[getter]
    fn geometry_count(&self) -> usize {
        self.0.geometry_count()
    }

    #[getter]
    fn constraint_count(&self) -> usize {
        self.0.constraint_count()
    }

    /// Solves from the seed. Failure leaves the system as it was.
    fn solve(&mut self, py: Python<'_>) -> PyResult<Solution2d> {
        let solution = self.0.solve().map_err(|error| solve_error(py, error))?;
        Ok(Solution2d(solution))
    }

    /// Moves toward temporary targets while keeping every relation.
    fn solve_toward(
        &mut self,
        py: Python<'_>,
        targets: Vec<Bound<'_, Target>>,
    ) -> PyResult<Solution2d> {
        let targets: Vec<_> = targets
            .iter()
            .map(|target| target.get().target.clone())
            .collect();
        let solution = self
            .0
            .solve_toward(&targets)
            .map_err(|error| solve_error(py, error))?;
        Ok(Solution2d(solution))
    }

    /// The problem, its seed and what it captured; not its solutions.
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

/// An immutable result: its geometry survives later edits.
#[pyclass(module = "ngk.solver", frozen)]
pub(crate) struct Solution2d(solver::Solution2d);

#[pymethods]
impl Solution2d {
    fn point(&self, point: PointArg) -> PyResult<PyPoint2> {
        Ok(self.0.point(point.0).map_err(edit_error)?.into())
    }

    fn segment(&self, id: SegmentId) -> PyResult<Segment2> {
        Ok(Segment2(self.0.segment(id.0).map_err(edit_error)?))
    }

    fn circle(&self, id: CircleId) -> PyResult<Circle2> {
        Ok(Circle2(self.0.circle(id.0).map_err(edit_error)?))
    }

    fn arc(&self, id: ArcId) -> PyResult<Arc2> {
        Ok(Arc2(self.0.arc(id.0).map_err(edit_error)?))
    }

    #[getter]
    fn report(&self) -> SolveReport {
        SolveReport(self.0.report().clone())
    }
}

pub(super) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<System2d>()?;
    module.add_class::<Solution2d>()?;
    Ok(())
}
