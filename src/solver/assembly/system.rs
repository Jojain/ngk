use super::solution::SolutionData;
use super::{
    BodyId, Coordinate, EditError, Joint3d, JointId, Solution3d, SolveError, SolveOptions,
    Target3d, solving,
};
use crate::geometry::dim3::utils::Point3;
use crate::geometry::transform::Rigid;
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::sync::Arc;

/// A body as the assembly holds it: where it starts from, and where it is held if fixed.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub(super) struct Body {
    pub seed: Rigid,
    /// The pose captured when the body was fixed.
    pub fixed: Option<Rigid>,
}

/// A joint as the assembly holds it: its definition, and the unwrapped values its
/// angles start from.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub(super) struct StoredJoint {
    pub definition: Joint3d,
    /// Per angular coordinate, the value the next solve unwraps nearest to.
    pub seeds: BTreeMap<Coordinate, f64>,
}

/// Mutable assembly of rigid bodies and the joints between them, independent of NGK
/// topology and GUI interaction.
///
/// A body is a pose with no shape: what it stands for is the caller's mapping. `add_*`
/// issues the id of what it adds, and never issues one twice; `insert_*` takes an id
/// issued earlier back, so an assembly can be rebuilt from stored ids.
#[derive(Debug)]
pub struct Assembly3d {
    pub(super) bodies: BTreeMap<BodyId, Body>,
    pub(super) joints: BTreeMap<JointId, StoredJoint>,
    pub(super) next_id: u64,
    options: SolveOptions,
    revision: u64,
    sequence: u64,
    last: Option<Solution3d>,
}

/// The explicit effects of a cascading body removal.
#[derive(Clone, Debug)]
pub struct Removal {
    pub body: BodyId,
    pub joints: Vec<JointId>,
}

impl Default for Assembly3d {
    fn default() -> Self {
        Self::new(SolveOptions::default())
    }
}

impl Assembly3d {
    /// Creates an empty assembly; invalid numerical options are reported when solving.
    pub fn new(options: SolveOptions) -> Self {
        Self::restored(options, BTreeMap::new(), BTreeMap::new(), 1)
    }
    /// An assembly holding this problem, at the start of its own session.
    pub(super) fn restored(
        options: SolveOptions,
        bodies: BTreeMap<BodyId, Body>,
        joints: BTreeMap<JointId, StoredJoint>,
        next_id: u64,
    ) -> Self {
        Self {
            bodies,
            joints,
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
    pub fn solution(&self) -> Option<&Solution3d> {
        self.last.as_ref().filter(|s| s.revision() == self.revision)
    }
    /// Returns the last success, including historical poses after edits or failures.
    pub fn last_solution(&self) -> Option<&Solution3d> {
        self.last.as_ref()
    }
    pub fn body_count(&self) -> usize {
        self.bodies.len()
    }
    pub fn joint_count(&self) -> usize {
        self.joints.len()
    }

    /// Adds a body starting from the pose `seed`.
    pub fn add_body(&mut self, seed: Rigid) -> Result<BodyId, EditError> {
        valid_pose(seed)?;
        let id = BodyId::new(self.issue_id());
        self.store_body(id, seed)?;
        Ok(id)
    }
    /// Adds a body under an id issued earlier, to rebuild an assembly from its stored ids.
    pub fn insert_body(&mut self, id: BodyId, seed: Rigid) -> Result<(), EditError> {
        valid_pose(seed)?;
        self.store_body(id, seed)?;
        self.reserve_id(id.value());
        Ok(())
    }
    /// The pose a body starts the next solve from.
    pub fn body(&self, id: BodyId) -> Result<Rigid, EditError> {
        Ok(self.get(id)?.seed)
    }
    /// Moves a body's seed. A fixed body stays where its fixing captured it.
    pub fn update_body(&mut self, id: BodyId, seed: Rigid) -> Result<(), EditError> {
        valid_pose(seed)?;
        self.get_mut(id)?.seed = seed;
        self.revision += 1;
        Ok(())
    }
    /// Holds a body at its current seed. Fixing it again recaptures the seed.
    pub fn fix(&mut self, id: BodyId) -> Result<(), EditError> {
        let body = self.get_mut(id)?;
        body.fixed = Some(body.seed);
        self.revision += 1;
        Ok(())
    }
    /// Releases a fixed body; releasing a free one changes nothing.
    pub fn unfix(&mut self, id: BodyId) -> Result<(), EditError> {
        let body = self.get_mut(id)?;
        if body.fixed.take().is_some() {
            self.revision += 1;
        }
        Ok(())
    }
    /// The pose a fixed body is held at, or `None` for a free body.
    pub fn fixing(&self, id: BodyId) -> Result<Option<Rigid>, EditError> {
        Ok(self.get(id)?.fixed)
    }
    /// Removes a body no joint holds.
    pub fn remove_body(&mut self, id: BodyId) -> Result<(), EditError> {
        self.get(id)?;
        let joints = self.joints_on(id);
        if !joints.is_empty() {
            return Err(EditError::ReferencedBody { body: id, joints });
        }
        self.bodies.remove(&id);
        self.revision += 1;
        Ok(())
    }
    /// Removes a body and every joint holding it, reporting those joints.
    pub fn remove_cascade(&mut self, id: BodyId) -> Result<Removal, EditError> {
        self.get(id)?;
        let joints = self.joints_on(id);
        for joint in &joints {
            self.joints.remove(joint);
        }
        self.bodies.remove(&id);
        self.revision += 1;
        Ok(Removal { body: id, joints })
    }

    pub fn joint(&self, id: JointId) -> Result<&Joint3d, EditError> {
        Ok(&self.stored(id)?.definition)
    }
    pub fn add_joint(&mut self, joint: Joint3d) -> Result<JointId, EditError> {
        self.validate_joint(&joint)?;
        let id = JointId::new(self.issue_id());
        self.store_joint(id, joint)?;
        Ok(id)
    }
    /// Adds a joint under an id issued earlier, to rebuild an assembly from its stored ids.
    pub fn insert_joint(&mut self, id: JointId, joint: Joint3d) -> Result<(), EditError> {
        self.validate_joint(&joint)?;
        self.store_joint(id, joint)?;
        self.reserve_id(id.value());
        Ok(())
    }
    /// Replaces a joint's definition, keeping its id.
    pub fn update_joint(&mut self, id: JointId, joint: Joint3d) -> Result<(), EditError> {
        self.joint(id)?;
        self.validate_joint(&joint)?;
        let stored = self.joints.get_mut(&id).expect("checked above");
        // Seeds of the angles the new kind still has stay.
        stored
            .seeds
            .retain(|coordinate, _| joint.kind.coordinates().contains(coordinate));
        stored.definition = joint;
        self.revision += 1;
        Ok(())
    }
    /// Sets the unwrapped value an angle of a joint starts from, such as one read from a
    /// stored solution: the next solve takes the representative of the angle nearest it.
    /// A success replaces it with the solved value.
    pub fn set_coordinate_seed(
        &mut self,
        id: JointId,
        coordinate: Coordinate,
        value: f64,
    ) -> Result<(), EditError> {
        let stored = self.stored(id)?;
        if !coordinate.angular() || !stored.definition.kind.coordinates().contains(&coordinate) {
            return Err(EditError::InvalidJoint(
                "only an angle of the joint has a seed",
            ));
        }
        if !value.is_finite() {
            return Err(EditError::InvalidJoint("a seed must be finite"));
        }
        self.joints
            .get_mut(&id)
            .expect("checked above")
            .seeds
            .insert(coordinate, value);
        self.revision += 1;
        Ok(())
    }
    /// The unwrapped value an angle of a joint starts from, if it has one.
    pub fn coordinate_seed(
        &self,
        id: JointId,
        coordinate: Coordinate,
    ) -> Result<Option<f64>, EditError> {
        Ok(self.stored(id)?.seeds.get(&coordinate).copied())
    }
    pub fn remove_joint(&mut self, id: JointId) -> Result<(), EditError> {
        self.joint(id)?;
        self.joints.remove(&id);
        self.revision += 1;
        Ok(())
    }

    /// Applies a batch atomically. What an aborted batch added is not in the assembly.
    ///
    /// A successful batch counts as one revision.
    pub fn edit<T>(
        &mut self,
        edit: impl FnOnce(&mut Self) -> Result<T, EditError>,
    ) -> Result<T, EditError> {
        let mut draft = Self {
            bodies: self.bodies.clone(),
            joints: self.joints.clone(),
            next_id: self.next_id,
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
        }
        *self = draft;
        Ok(output)
    }

    /// Solves from the seeds, publishing only a completely validated result.
    /// Underconstraint and compatible redundancy succeed and are described in the report.
    pub fn solve(&mut self) -> Result<Solution3d, SolveError> {
        self.solve_toward(&[])
    }
    /// Moves toward temporary targets while keeping every joint.
    ///
    /// A success becomes the seeds of the next solve. Failure preserves the seeds,
    /// accepted edits, and the last successful result.
    pub fn solve_toward(&mut self, targets: &[Target3d]) -> Result<Solution3d, SolveError> {
        if !self.options.valid() {
            return Err(SolveError::InvalidOptions);
        }
        for target in targets {
            self.get(target.body)?;
            if !target.valid() {
                return Err(EditError::InvalidTarget(
                    "target coordinates and positive weight must be finite",
                )
                .into());
            }
        }
        let solved = solving::solve(&self.bodies, &self.joints, targets, &self.options)?;
        for (id, pose) in &solved.poses {
            if let Some(body) = self.bodies.get_mut(id) {
                body.seed = *pose;
            }
        }
        for (id, coordinates) in &solved.coordinates {
            if let Some(joint) = self.joints.get_mut(id) {
                for (coordinate, value) in coordinates {
                    if coordinate.angular() {
                        joint.seeds.insert(*coordinate, *value);
                    }
                }
            }
        }
        self.sequence += 1;
        let solution = Solution3d(Arc::new(SolutionData {
            revision: self.revision,
            sequence: self.sequence,
            poses: solved.poses,
            coordinates: solved.coordinates,
            relative: solved.relative,
            report: solved.report,
        }));
        self.last = Some(solution.clone());
        Ok(solution)
    }

    fn get(&self, id: BodyId) -> Result<&Body, EditError> {
        self.bodies
            .get(&id)
            .ok_or(EditError::InvalidBodyReference(id))
    }
    fn get_mut(&mut self, id: BodyId) -> Result<&mut Body, EditError> {
        self.bodies
            .get_mut(&id)
            .ok_or(EditError::InvalidBodyReference(id))
    }
    fn joints_on(&self, body: BodyId) -> Vec<JointId> {
        self.joints
            .iter()
            .filter(|(_, joint)| joint.definition.bodies().contains(&body))
            .map(|(id, _)| *id)
            .collect()
    }
    fn validate_joint(&self, joint: &Joint3d) -> Result<(), EditError> {
        for body in joint.bodies() {
            self.get(body)?;
        }
        match joint.invalid() {
            Some(reason) => Err(EditError::InvalidJoint(reason)),
            None => Ok(()),
        }
    }
    /// A number this assembly never issued, whatever was removed since.
    fn issue_id(&mut self) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }
    /// What is inserted under a stored id must not collide with what is issued later.
    fn reserve_id(&mut self, value: u64) {
        self.next_id = self.next_id.max(value.saturating_add(1));
    }
    fn store_body(&mut self, id: BodyId, seed: Rigid) -> Result<(), EditError> {
        if self.bodies.contains_key(&id) {
            return Err(EditError::DuplicateBody(id));
        }
        self.bodies.insert(id, Body { seed, fixed: None });
        self.revision += 1;
        Ok(())
    }
    fn store_joint(&mut self, id: JointId, joint: Joint3d) -> Result<(), EditError> {
        if self.joints.contains_key(&id) {
            return Err(EditError::DuplicateJoint(id));
        }
        self.joints.insert(
            id,
            StoredJoint {
                definition: joint,
                seeds: BTreeMap::new(),
            },
        );
        self.revision += 1;
        Ok(())
    }
    fn stored(&self, id: JointId) -> Result<&StoredJoint, EditError> {
        self.joints
            .get(&id)
            .ok_or(EditError::InvalidJointReference(id))
    }
}

pub(super) fn valid_pose(pose: Rigid) -> Result<(), EditError> {
    let finite = |v: Vector3<f64>| v.iter().all(|c| c.is_finite());
    if finite(pose.apply(Point3::origin()).coords)
        && [Vector3::x(), Vector3::y(), Vector3::z()]
            .into_iter()
            .all(|axis| finite(pose.apply_vector(axis)))
    {
        Ok(())
    } else {
        Err(EditError::InvalidPose("a pose must be finite"))
    }
}
