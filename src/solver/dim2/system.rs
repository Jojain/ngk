use super::{
    ArcId, CircleId, Constraint2d, ConstraintId, EditError, GeometryId, PointId, PointRef,
    SegmentId, Solution2d, SolveError, SolveOptions, TangencyMode, Target2d,
};
use super::{
    constraints::{StoredConstraint, TargetLocation},
    primitives::{Param, Primitive},
    solution::SolutionData,
    solving,
};
use crate::geometry::dim2::{
    curves::Circle2,
    trimmed::{Arc2, Segment2},
    utils::Point2,
};
use Constraint2d::*;
use std::collections::BTreeMap;
use std::sync::Arc;

/// Mutable 2D constraint system, independent of NGK topology and GUI interaction.
///
/// `add_*` issues the id of what it adds, and never issues one twice. `insert_*` takes an
/// id issued earlier back, so a system can be rebuilt from stored ids and still mean the
/// same thing.
#[derive(Debug)]
pub struct System2d {
    pub(crate) geometry: BTreeMap<GeometryId, Primitive>,
    pub(crate) constraints: BTreeMap<ConstraintId, StoredConstraint>,
    pub(crate) next_id: u64,
    options: SolveOptions,
    revision: u64,
    sequence: u64,
    last: Option<Solution2d>,
}

/// The explicit effects of a cascading primitive removal.
#[derive(Clone, Debug)]
pub struct Removal {
    pub geometry: GeometryId,
    pub constraints: Vec<ConstraintId>,
}

impl Default for System2d {
    fn default() -> Self {
        Self::new(SolveOptions::default())
    }
}
impl System2d {
    /// Creates an empty system; invalid numerical options are reported when solving.
    pub fn new(options: SolveOptions) -> Self {
        Self::restored(options, BTreeMap::new(), BTreeMap::new(), 1)
    }
    /// A system holding this problem, at the start of its own session.
    pub(super) fn restored(
        options: SolveOptions,
        geometry: BTreeMap<GeometryId, Primitive>,
        constraints: BTreeMap<ConstraintId, StoredConstraint>,
        next_id: u64,
    ) -> Self {
        Self {
            geometry,
            constraints,
            next_id,
            options,
            revision: 0,
            sequence: 0,
            last: None,
        }
    }
    pub fn revision(&self) -> u64 {
        self.revision
    }
    pub fn options(&self) -> &SolveOptions {
        &self.options
    }
    /// Changes tolerances and invalidates the current-result designation.
    pub fn set_options(&mut self, options: SolveOptions) -> Result<(), SolveError> {
        if !options.valid() {
            return Err(SolveError::InvalidOptions);
        }
        self.options = options;
        self.revision += 1;
        Ok(())
    }
    /// Returns a result only when it describes the current problem revision.
    pub fn solution(&self) -> Option<&Solution2d> {
        self.last.as_ref().filter(|s| s.revision() == self.revision)
    }
    /// Returns the last success, including historical geometry after edits or failures.
    pub fn last_solution(&self) -> Option<&Solution2d> {
        self.last.as_ref()
    }
    pub fn geometry_count(&self) -> usize {
        self.geometry.len()
    }
    pub fn constraint_count(&self) -> usize {
        self.constraints.len()
    }
    pub fn add_point(&mut self, point: Point2) -> Result<PointId, EditError> {
        let primitive = Primitive::point(point)?;
        let id = PointId::new(self.issue_id());
        self.store(id.0, primitive)?;
        Ok(id)
    }
    pub fn add_segment(&mut self, line: Segment2) -> Result<SegmentId, EditError> {
        let primitive = Primitive::segment(line)?;
        let id = SegmentId::new(self.issue_id());
        self.store(id.0, primitive)?;
        Ok(id)
    }
    pub fn add_circle(&mut self, circle: Circle2) -> Result<CircleId, EditError> {
        let primitive = Primitive::circle(circle)?;
        let id = CircleId::new(self.issue_id());
        self.store(id.0, primitive)?;
        Ok(id)
    }
    pub fn add_arc(&mut self, arc: Arc2) -> Result<ArcId, EditError> {
        let primitive = Primitive::arc(arc)?;
        let id = ArcId::new(self.issue_id());
        self.store(id.0, primitive)?;
        Ok(id)
    }
    /// Adds a point under an id issued earlier, to rebuild a system from its stored ids.
    pub fn insert_point(&mut self, id: PointId, point: Point2) -> Result<(), EditError> {
        self.reinstate(id.0, Primitive::point(point)?)
    }
    /// Adds a segment under an id issued earlier, to rebuild a system from its stored ids.
    pub fn insert_segment(&mut self, id: SegmentId, line: Segment2) -> Result<(), EditError> {
        self.reinstate(id.0, Primitive::segment(line)?)
    }
    /// Adds a circle under an id issued earlier, to rebuild a system from its stored ids.
    pub fn insert_circle(&mut self, id: CircleId, circle: Circle2) -> Result<(), EditError> {
        self.reinstate(id.0, Primitive::circle(circle)?)
    }
    /// Adds an arc under an id issued earlier, to rebuild a system from its stored ids.
    pub fn insert_arc(&mut self, id: ArcId, arc: Arc2) -> Result<(), EditError> {
        self.reinstate(id.0, Primitive::arc(arc)?)
    }
    /// A number this system never issued, whatever was removed since.
    fn issue_id(&mut self) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }
    /// What is inserted under a stored id must not collide with what is issued later.
    fn reserve_id(&mut self, value: u64) {
        self.next_id = self.next_id.max(value.saturating_add(1));
    }
    fn reinstate(&mut self, id: GeometryId, primitive: Primitive) -> Result<(), EditError> {
        self.store(id, primitive)?;
        self.reserve_id(id.value);
        Ok(())
    }
    fn store(&mut self, id: GeometryId, primitive: Primitive) -> Result<(), EditError> {
        if self.geometry.contains_key(&id) {
            return Err(EditError::DuplicateGeometry(id));
        }
        self.geometry.insert(id, primitive);
        self.revision += 1;
        Ok(())
    }
    /// Reads the editable seed, which is not necessarily a solved configuration.
    pub fn point(&self, point: impl Into<PointRef>) -> Result<Point2, EditError> {
        let point = point.into();
        Ok(self.get(point.geometry)?.point_at(point.role))
    }
    pub fn segment(&self, key: SegmentId) -> Result<Segment2, EditError> {
        Ok(self.get(key.0)?.as_segment())
    }
    pub fn circle(&self, key: CircleId) -> Result<Circle2, EditError> {
        Ok(self.get(key.0)?.as_circle())
    }
    pub fn arc(&self, key: ArcId) -> Result<Arc2, EditError> {
        Ok(self.get(key.0)?.as_arc())
    }
    pub fn update_point(&mut self, key: PointId, point: Point2) -> Result<(), EditError> {
        self.replace(key.0, Primitive::point(point)?)
    }
    pub fn update_segment(&mut self, key: SegmentId, curve: Segment2) -> Result<(), EditError> {
        self.replace(key.0, Primitive::segment(curve)?)
    }
    pub fn update_circle(&mut self, key: CircleId, curve: Circle2) -> Result<(), EditError> {
        self.replace(key.0, Primitive::circle(curve)?)
    }
    pub fn update_arc(&mut self, key: ArcId, curve: Arc2) -> Result<(), EditError> {
        self.replace(key.0, Primitive::arc(curve)?)
    }
    fn replace(&mut self, key: GeometryId, primitive: Primitive) -> Result<(), EditError> {
        let old = self.get(key)?;
        if old.values == primitive.values
            && old.sense == primitive.sense
            && old.circle_axis == primitive.circle_axis
        {
            return Ok(());
        }
        self.geometry.insert(key, primitive);
        self.revision += 1;
        Ok(())
    }
    pub(crate) fn get(&self, key: GeometryId) -> Result<&Primitive, EditError> {
        self.geometry
            .get(&key)
            .ok_or(EditError::InvalidGeometryReference(key))
    }
    fn constraint_record(&self, key: ConstraintId) -> Result<&StoredConstraint, EditError> {
        self.constraints
            .get(&key)
            .ok_or(EditError::InvalidConstraintReference(key))
    }
    pub fn constraint(&self, key: ConstraintId) -> Result<&Constraint2d, EditError> {
        Ok(&self.constraint_record(key)?.definition)
    }
    /// Validates references and captures fixed values or automatic contact choices.
    /// Compatibility with other constraints is evaluated during solving.
    pub fn add_constraint(&mut self, definition: Constraint2d) -> Result<ConstraintId, EditError> {
        let constraint = self.prepare_constraint(definition)?;
        let id = ConstraintId::new(self.issue_id());
        self.constraints.insert(id, constraint);
        self.revision += 1;
        Ok(id)
    }
    /// Adds a relation under an id issued earlier, to rebuild a system from its stored ids.
    pub fn insert_constraint(
        &mut self,
        id: ConstraintId,
        definition: Constraint2d,
    ) -> Result<(), EditError> {
        if self.constraints.contains_key(&id) {
            return Err(EditError::DuplicateConstraint(id));
        }
        let constraint = self.prepare_constraint(definition)?;
        self.constraints.insert(id, constraint);
        self.reserve_id(id.value());
        self.revision += 1;
        Ok(())
    }
    /// Replaces a relation without changing its id; captured values and automatic choices refresh.
    pub fn update_constraint(
        &mut self,
        key: ConstraintId,
        definition: Constraint2d,
    ) -> Result<(), EditError> {
        self.constraint_record(key)?;
        let constraint = self.prepare_constraint(definition)?;
        self.constraints.insert(key, constraint);
        self.revision += 1;
        Ok(())
    }
    pub fn remove_constraint(&mut self, key: ConstraintId) -> Result<(), EditError> {
        self.constraint_record(key)?;
        self.constraints.remove(&key);
        self.revision += 1;
        Ok(())
    }
    /// Removes an unreferenced primitive, otherwise identifies its dependants.
    pub fn remove_geometry(&mut self, key: impl Into<GeometryId>) -> Result<(), EditError> {
        let key = key.into();
        self.get(key)?;
        let constraints = self.dependants(key);
        if !constraints.is_empty() {
            return Err(EditError::ReferencedGeometry {
                geometry: key,
                constraints,
            });
        }
        self.geometry.remove(&key);
        self.revision += 1;
        Ok(())
    }
    /// Removes a primitive and all its dependent constraints in one edit.
    pub fn remove_cascade(&mut self, key: impl Into<GeometryId>) -> Result<Removal, EditError> {
        let key = key.into();
        self.get(key)?;
        let constraints = self.dependants(key);
        for constraint in &constraints {
            self.constraints.remove(constraint);
        }
        self.geometry.remove(&key);
        self.revision += 1;
        Ok(Removal {
            geometry: key,
            constraints,
        })
    }
    fn dependants(&self, key: GeometryId) -> Vec<ConstraintId> {
        self.constraints
            .iter()
            .filter(|(_, c)| c.definition.geometries().contains(&key))
            .map(|(id, _)| *id)
            .collect()
    }
    /// Applies a batch atomically. What an aborted batch added is not in the system.
    ///
    /// A successful batch counts as one revision. Resolve after the batch; a
    /// result computed inside it is not designated as the committed solution.
    pub fn edit<T>(
        &mut self,
        edit: impl FnOnce(&mut Self) -> Result<T, EditError>,
    ) -> Result<T, EditError> {
        let mut draft = Self {
            geometry: self.geometry.clone(),
            next_id: self.next_id,
            constraints: self.constraints.clone(),
            options: self.options.clone(),
            revision: self.revision,
            sequence: self.sequence,
            last: self.last.clone(),
        };
        let output = edit(&mut draft);
        // An id issued in an aborted batch is not issued again.
        self.next_id = draft.next_id;
        let output = output?;
        if draft.revision != self.revision {
            draft.revision = self.revision + 1;
            draft.last = self.last.clone();
        }
        *self = draft;
        Ok(output)
    }
    /// Solves from the editable seed, publishing only a completely validated result.
    /// Underconstraint and compatible redundancy succeed and are described in the report.
    pub fn solve(&mut self) -> Result<Solution2d, SolveError> {
        self.solve_toward(&[])
    }
    /// Moves toward temporary targets while validating every permanent constraint.
    /// Failure preserves the editable seed, accepted edits, and last successful result.
    pub fn solve_toward(&mut self, targets: &[Target2d]) -> Result<Solution2d, SolveError> {
        if !self.options.valid() {
            return Err(SolveError::InvalidOptions);
        }
        for target in targets {
            self.validate_target(target)?;
        }
        let mut constraints = self.constraints.clone();
        for constraint in constraints.values_mut() {
            constraint.side = self.prepare_constraint(constraint.definition.clone())?.side;
        }
        let (geometry, report) =
            solving::solve(&self.geometry, &constraints, targets, &self.options)?;
        self.sequence += 1;
        let solution = Solution2d(Arc::new(SolutionData {
            revision: self.revision,
            sequence: self.sequence,
            geometry: geometry.clone(),
            report,
        }));
        self.geometry = geometry;
        self.constraints = constraints;
        self.last = Some(solution.clone());
        Ok(solution)
    }
    fn validate_target(&self, target: &Target2d) -> Result<(), EditError> {
        self.get(target.geometry())?;
        if !target.position.coords.iter().all(|v| v.is_finite())
            || !target.weight.is_finite()
            || target.weight <= 0.
        {
            return Err(EditError::InvalidConstraint(
                "target coordinates and positive weight must be finite",
            ));
        }
        let valid = match target.location {
            TargetLocation::Point(_) => true,
            TargetLocation::Fraction(_, fraction) => (0. ..=1.).contains(&fraction.value()),
            TargetLocation::Native(_, parameter) => parameter.is_finite(),
        };
        if !valid {
            return Err(EditError::InvalidConstraint(
                "invalid target curve parameter",
            ));
        }
        Ok(())
    }
    /// Captures fixed values and discrete contact choices without changing live state.
    fn prepare_constraint(&self, definition: Constraint2d) -> Result<StoredConstraint, EditError> {
        for key in definition.geometries() {
            self.get(key)?;
        }
        if !definition.valid_values() {
            return Err(EditError::InvalidConstraint("invalid dimension or angle"));
        }
        let mut result = StoredConstraint {
            definition,
            captured: vec![],
            captured_frame: None,
            side: 1.,
            tangency: TangencyMode::External,
        };
        match &result.definition {
            Horizontal(line) | Vertical(line) => {
                let line = self.segment(*line)?;
                let direction = line.end() - line.start();
                let coordinate = if matches!(result.definition, Horizontal(_)) {
                    direction.x
                } else {
                    direction.y
                };
                result.side = if coordinate < 0. { -1. } else { 1. };
            }
            Fixed(g) => {
                let primitive = self.get(*g)?;
                result.captured = primitive.values.clone();
                result.captured_frame = Some((primitive.sense, primitive.circle_axis));
            }
            FixedPoint(p) => {
                let p = self.point(*p)?;
                result.captured = vec![p.x, p.y];
            }
            DistanceToLine(p, l, _) => result.side = self.line_side(*l, self.point(*p)?)?,
            TangentLineCircle(l, c) => {
                result.side =
                    self.line_side(*l, self.get(c.0)?.point_at(super::ids::PointRole::Center))?
            }
            Angle { first, second, .. } => {
                let a = self.segment(*first)?;
                let b = self.segment(*second)?;
                let cross = (a.end() - a.start()).perp(&(b.end() - b.start()));
                result.side = if cross < 0. { -1. } else { 1. };
            }
            Parallel(first, second) => {
                let a = self.segment(*first)?;
                let b = self.segment(*second)?;
                result.side = if (a.end() - a.start()).dot(&(b.end() - b.start())) < 0. {
                    -1.
                } else {
                    1.
                };
            }
            Perpendicular(first, second) => {
                let a = self.segment(*first)?;
                let b = self.segment(*second)?;
                result.side = if (a.end() - a.start()).perp(&(b.end() - b.start())) < 0. {
                    -1.
                } else {
                    1.
                };
            }
            TangentCircles {
                first,
                second,
                mode,
            } => {
                let a = self.get(first.0)?;
                let b = self.get(second.0)?;
                let d = (a.point_at(super::ids::PointRole::Center)
                    - b.point_at(super::ids::PointRole::Center))
                .norm();
                let (r1, r2) = (a.get(Param::Radius), b.get(Param::Radius));
                let mode = if *mode == TangencyMode::Automatic {
                    if (d - (r1 + r2)).abs() <= (d - (r1 - r2).abs()).abs() {
                        TangencyMode::External
                    } else {
                        TangencyMode::Internal
                    }
                } else {
                    *mode
                };
                result.tangency = if mode == TangencyMode::Internal {
                    if r1 >= r2 {
                        TangencyMode::FirstContainsSecond
                    } else {
                        TangencyMode::SecondContainsFirst
                    }
                } else {
                    mode
                };
            }
            _ => {}
        }
        Ok(result)
    }
    fn line_side(&self, line: SegmentId, point: Point2) -> Result<f64, EditError> {
        let line = self.segment(line)?;
        Ok(
            if (line.end() - line.start()).perp(&(point - line.start())) < 0. {
                -1.
            } else {
                1.
            },
        )
    }
}
