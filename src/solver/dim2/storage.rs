use super::constraints::StoredConstraint;
use super::primitives::Primitive;
use super::{Constraint2d, ConstraintId, GeometryId, SolveOptions, System2d, TangencyMode};
use serde::{Deserialize, Deserializer, Serialize, Serializer, de::Error};
use std::collections::BTreeMap;

/// What survives storage: the problem, its seed and everything captured from it.
/// Revisions and solutions describe one session and start afresh.
#[derive(Serialize, Deserialize)]
struct StoredSystem {
    options: SolveOptions,
    next_id: u64,
    primitives: Vec<(GeometryId, Primitive)>,
    constraints: Vec<(ConstraintId, StoredConstraint)>,
}

impl Serialize for System2d {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        StoredSystem {
            options: self.options().clone(),
            next_id: self.next_id,
            primitives: self.geometry.clone().into_iter().collect(),
            constraints: self.constraints.clone().into_iter().collect(),
        }
        .serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for System2d {
    /// Refuses what an edit could not have produced, so that solving can rely on it.
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        StoredSystem::deserialize(deserializer)?
            .restored()
            .map_err(D::Error::custom)
    }
}

impl StoredSystem {
    fn restored(self) -> Result<System2d, &'static str> {
        if !self.options.valid() {
            return Err("invalid solver tolerances or iteration budgets");
        }
        let mut geometry = BTreeMap::new();
        for (id, primitive) in self.primitives {
            if !primitive.consistent_with(id) {
                return Err("a stored primitive is invalid");
            }
            if geometry.insert(id, primitive).is_some() {
                return Err("a primitive id is stored twice");
            }
        }
        let mut constraints = BTreeMap::new();
        for (id, constraint) in self.constraints {
            if !consistent(&constraint, &geometry) {
                return Err("a stored constraint is invalid");
            }
            if constraints.insert(id, constraint).is_some() {
                return Err("a constraint id is stored twice");
            }
        }
        let highest = geometry
            .keys()
            .map(|id: &GeometryId| id.value)
            .chain(constraints.keys().map(|id: &ConstraintId| id.value()))
            .max();
        if highest.is_some_and(|highest| highest >= self.next_id) {
            return Err("the next id to issue is already in use");
        }
        Ok(System2d::restored(
            self.options,
            geometry,
            constraints,
            self.next_id,
        ))
    }
}

fn consistent(constraint: &StoredConstraint, geometry: &BTreeMap<GeometryId, Primitive>) -> bool {
    let definition = &constraint.definition;
    if !definition.valid_values()
        || definition
            .geometries()
            .iter()
            .any(|id| !geometry.contains_key(id))
        || constraint.side.abs() != 1.
        || !constraint.captured.iter().all(|value| value.is_finite())
        || matches!(
            constraint.tangency,
            TangencyMode::Automatic | TangencyMode::Internal
        )
    {
        return false;
    }
    match definition {
        Constraint2d::Fixed(id) => {
            constraint.captured.len() == id.kind.layout().len()
                && constraint.captured_frame.is_some()
        }
        Constraint2d::FixedPoint(_) => constraint.captured.len() == 2,
        _ => constraint.captured.is_empty(),
    }
}
