use super::system::{Body, StoredJoint, valid_pose};
use super::{Assembly3d, BodyId, JointId, SolveOptions};
use serde::{Deserialize, Deserializer, Serialize, Serializer, de::Error};
use std::collections::BTreeMap;

/// What survives storage: the bodies with their seeds and fixings, and the joints with
/// the seeds of their angles. Revisions and solutions describe one session and start
/// afresh.
#[derive(Serialize, Deserialize)]
struct StoredAssembly {
    options: SolveOptions,
    next_id: u64,
    bodies: Vec<(BodyId, Body)>,
    joints: Vec<(JointId, StoredJoint)>,
}

impl Serialize for Assembly3d {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        StoredAssembly {
            options: self.options().clone(),
            next_id: self.next_id,
            bodies: self.bodies.clone().into_iter().collect(),
            joints: self.joints.clone().into_iter().collect(),
        }
        .serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Assembly3d {
    /// Refuses what an edit could not have produced, so that solving can rely on it.
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        StoredAssembly::deserialize(deserializer)?
            .restored()
            .map_err(D::Error::custom)
    }
}

impl StoredAssembly {
    fn restored(self) -> Result<Assembly3d, &'static str> {
        if !self.options.valid() {
            return Err("invalid solver tolerances or iteration budgets");
        }
        let mut bodies = BTreeMap::new();
        for (id, body) in self.bodies {
            if valid_pose(body.seed).is_err() || body.fixed.is_some_and(|f| valid_pose(f).is_err())
            {
                return Err("a stored pose is not finite");
            }
            if bodies.insert(id, body).is_some() {
                return Err("a body id is stored twice");
            }
        }
        let mut joints = BTreeMap::new();
        for (id, joint) in self.joints {
            let definition = &joint.definition;
            if definition
                .bodies()
                .iter()
                .any(|body| !bodies.contains_key(body))
                || definition.invalid().is_some()
            {
                return Err("a stored joint is invalid");
            }
            if !joint.seeds.iter().all(|(coordinate, value)| {
                coordinate.angular()
                    && definition.kind.coordinates().contains(coordinate)
                    && value.is_finite()
            }) {
                return Err("a stored seed is not a finite angle of its joint");
            }
            if joints.insert(id, joint).is_some() {
                return Err("a joint id is stored twice");
            }
        }
        let highest = bodies
            .keys()
            .map(|id: &BodyId| id.value())
            .chain(joints.keys().map(|id: &JointId| id.value()))
            .max();
        if highest.is_some_and(|highest| highest >= self.next_id) {
            return Err("the next id to issue is already in use");
        }
        Ok(Assembly3d::restored(
            self.options,
            bodies,
            joints,
            self.next_id,
        ))
    }
}
