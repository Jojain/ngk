use super::{
    ArcKey, CircleKey, Constraint2d, ConstraintKey, EditError, GeometryKey, PointKey, PointRef,
    SegmentKey, Solution2d, SolveError, SolveOptions, TangencyMode, Target2d,
};
use super::{
    constraints::{StoredConstraint, TargetLocation},
    keys::{ConstraintSlot, GeometrySlot},
    numerical,
    primitives::{Kind, Param, Primitive},
    solution::SolutionData,
};
use crate::geometry::dim2::{curves::Circle2, trimmed::TrimmedCurve2, utils::Point2};
use Constraint2d::*;
use slotmap::SlotMap;
use std::sync::{
    Arc,
    atomic::{AtomicU64, Ordering},
};

static NEXT_IDENTITY: AtomicU64 = AtomicU64::new(1);

/// Issues identities independently of editable state, including aborted batches.
fn identity() -> u64 {
    NEXT_IDENTITY
        .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |v| v.checked_add(1))
        .expect("solver identity space exhausted")
}

/// Mutable 2D constraint system, independent of NGK topology and GUI interaction.
#[derive(Debug)]
pub struct System2d {
    pub(crate) owner: u64,
    pub(crate) geometry: SlotMap<GeometrySlot, Primitive>,
    pub(crate) constraints: SlotMap<ConstraintSlot, StoredConstraint>,
    options: SolveOptions,
    revision: u64,
    sequence: u64,
    last: Option<Solution2d>,
}

/// The explicit effects of a cascading primitive removal.
#[derive(Clone, Debug)]
pub struct Removal {
    pub geometry: GeometryKey,
    pub constraints: Vec<ConstraintKey>,
}

impl Default for System2d {
    fn default() -> Self {
        Self::new(SolveOptions::default())
    }
}
impl System2d {
    /// Creates an empty system; invalid numerical options are reported when solving.
    pub fn new(options: SolveOptions) -> Self {
        Self {
            owner: identity(),
            geometry: SlotMap::with_key(),
            constraints: SlotMap::with_key(),
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
    pub fn add_point(&mut self, point: Point2) -> Result<PointKey, EditError> {
        Ok(PointKey(self.insert(Primitive::point(point)?)))
    }
    pub fn add_segment(&mut self, line: TrimmedCurve2) -> Result<SegmentKey, EditError> {
        Ok(SegmentKey(self.insert(Primitive::segment(line)?)))
    }
    pub fn add_circle(&mut self, circle: Circle2) -> Result<CircleKey, EditError> {
        Ok(CircleKey(self.insert(Primitive::circle(circle)?)))
    }
    pub fn add_arc(&mut self, arc: TrimmedCurve2) -> Result<ArcKey, EditError> {
        Ok(ArcKey(self.insert(Primitive::arc(arc)?)))
    }
    fn insert(&mut self, mut primitive: Primitive) -> GeometryKey {
        primitive.serial = identity();
        let serial = primitive.serial;
        let slot = self.geometry.insert(primitive);
        self.revision += 1;
        GeometryKey {
            owner: self.owner,
            slot,
            serial,
        }
    }
    /// Reads the editable seed, which is not necessarily a solved configuration.
    pub fn point(&self, point: impl Into<PointRef>) -> Result<Point2, EditError> {
        let point = point.into();
        Ok(self.get(point.geometry)?.point_at(point.role))
    }
    pub fn segment(&self, key: SegmentKey) -> Result<TrimmedCurve2, EditError> {
        Ok(self.get(key.0)?.as_segment())
    }
    pub fn circle(&self, key: CircleKey) -> Result<Circle2, EditError> {
        Ok(self.get(key.0)?.as_circle())
    }
    pub fn arc(&self, key: ArcKey) -> Result<TrimmedCurve2, EditError> {
        Ok(self.get(key.0)?.as_arc())
    }
    pub fn update_point(&mut self, key: PointKey, point: Point2) -> Result<(), EditError> {
        self.replace(key.0, Primitive::point(point)?)
    }
    pub fn update_segment(
        &mut self,
        key: SegmentKey,
        curve: TrimmedCurve2,
    ) -> Result<(), EditError> {
        self.replace(key.0, Primitive::segment(curve)?)
    }
    pub fn update_circle(&mut self, key: CircleKey, curve: Circle2) -> Result<(), EditError> {
        self.replace(key.0, Primitive::circle(curve)?)
    }
    pub fn update_arc(&mut self, key: ArcKey, curve: TrimmedCurve2) -> Result<(), EditError> {
        self.replace(key.0, Primitive::arc(curve)?)
    }
    fn replace(&mut self, key: GeometryKey, mut primitive: Primitive) -> Result<(), EditError> {
        let old = self.get(key)?;
        if old.kind != primitive.kind {
            return Err(EditError::InvalidGeometryReference(key));
        }
        if old.values == primitive.values
            && old.sense == primitive.sense
            && old.circle_axis == primitive.circle_axis
        {
            return Ok(());
        }
        primitive.serial = key.serial;
        self.geometry[key.slot] = primitive;
        self.revision += 1;
        Ok(())
    }
    pub(crate) fn get(&self, key: GeometryKey) -> Result<&Primitive, EditError> {
        if key.owner != self.owner {
            return Err(EditError::InvalidGeometryReference(key));
        }
        self.geometry
            .get(key.slot)
            .filter(|p| p.serial == key.serial)
            .ok_or(EditError::InvalidGeometryReference(key))
    }
    fn constraint_record(&self, key: ConstraintKey) -> Result<&StoredConstraint, EditError> {
        if key.owner != self.owner {
            return Err(EditError::InvalidConstraintReference(key));
        }
        self.constraints
            .get(key.slot)
            .filter(|c| c.serial == key.serial)
            .ok_or(EditError::InvalidConstraintReference(key))
    }
    pub fn constraint(&self, key: ConstraintKey) -> Result<&Constraint2d, EditError> {
        Ok(&self.constraint_record(key)?.definition)
    }
    /// Validates references and captures fixed values or automatic contact choices.
    /// Compatibility with other constraints is evaluated during solving.
    pub fn add_constraint(&mut self, definition: Constraint2d) -> Result<ConstraintKey, EditError> {
        let mut constraint = self.prepare_constraint(definition)?;
        constraint.serial = identity();
        let serial = constraint.serial;
        let slot = self.constraints.insert(constraint);
        self.revision += 1;
        Ok(ConstraintKey {
            owner: self.owner,
            slot,
            serial,
        })
    }
    /// Replaces a relation without changing its key; captured values and automatic choices refresh.
    pub fn update_constraint(
        &mut self,
        key: ConstraintKey,
        definition: Constraint2d,
    ) -> Result<(), EditError> {
        self.constraint_record(key)?;
        let mut constraint = self.prepare_constraint(definition)?;
        constraint.serial = key.serial;
        self.constraints[key.slot] = constraint;
        self.revision += 1;
        Ok(())
    }
    pub fn remove_constraint(&mut self, key: ConstraintKey) -> Result<(), EditError> {
        self.constraint_record(key)?;
        self.constraints.remove(key.slot);
        self.revision += 1;
        Ok(())
    }
    /// Removes an unreferenced primitive, otherwise identifies its dependants.
    pub fn remove_geometry(&mut self, key: impl Into<GeometryKey>) -> Result<(), EditError> {
        let key = key.into();
        self.get(key)?;
        let constraints = self.dependants(key);
        if !constraints.is_empty() {
            return Err(EditError::ReferencedGeometry {
                geometry: key,
                constraints,
            });
        }
        self.geometry.remove(key.slot);
        self.revision += 1;
        Ok(())
    }
    /// Removes a primitive and all its dependent constraints in one edit.
    pub fn remove_cascade(&mut self, key: impl Into<GeometryKey>) -> Result<Removal, EditError> {
        let key = key.into();
        self.get(key)?;
        let constraints = self.dependants(key);
        for constraint in &constraints {
            self.constraints.remove(constraint.slot);
        }
        self.geometry.remove(key.slot);
        self.revision += 1;
        Ok(Removal {
            geometry: key,
            constraints,
        })
    }
    fn dependants(&self, key: GeometryKey) -> Vec<ConstraintKey> {
        self.constraints
            .iter()
            .filter(|(_, c)| c.definition.geometries().contains(&key))
            .map(|(slot, c)| ConstraintKey {
                owner: self.owner,
                slot,
                serial: c.serial,
            })
            .collect()
    }
    /// Applies a batch atomically. Keys issued by an aborted batch remain invalid.
    ///
    /// A successful batch counts as one revision. Resolve after the batch; a
    /// result computed inside it is not designated as the committed solution.
    pub fn edit<T>(
        &mut self,
        edit: impl FnOnce(&mut Self) -> Result<T, EditError>,
    ) -> Result<T, EditError> {
        let mut draft = Self {
            owner: self.owner,
            geometry: self.geometry.clone(),
            constraints: self.constraints.clone(),
            options: self.options.clone(),
            revision: self.revision,
            sequence: self.sequence,
            last: self.last.clone(),
        };
        let output = edit(&mut draft)?;
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
        for (_, constraint) in &mut constraints {
            constraint.side = self.prepare_constraint(constraint.definition.clone())?.side;
        }
        let (geometry, report) = numerical::solve(
            self.owner,
            &self.geometry,
            &constraints,
            targets,
            &self.options,
        )?;
        self.sequence += 1;
        let solution = Solution2d(Arc::new(SolutionData {
            owner: self.owner,
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
        let primitive = self.get(target.geometry())?;
        if !target.position.coords.iter().all(|v| v.is_finite())
            || !target.weight.is_finite()
            || target.weight <= 0.
        {
            return Err(EditError::InvalidConstraint(
                "target coordinates and positive weight must be finite",
            ));
        }
        if let TargetLocation::Sample(_, t) = target.location
            && (!t.is_finite() || (primitive.kind != Kind::Circle && !(0. ..=1.).contains(&t)))
        {
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
            serial: 0,
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
                    self.line_side(*l, self.get(c.0)?.point_at(super::keys::PointRole::Center))?
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
                let d = (a.point_at(super::keys::PointRole::Center)
                    - b.point_at(super::keys::PointRole::Center))
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
    fn line_side(&self, line: SegmentKey, point: Point2) -> Result<f64, EditError> {
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
