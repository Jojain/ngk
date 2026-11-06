use super::SketchBuildError;
use super::build::Drawn;
use crate::geometry::LINEAR_TOLERANCE;
use crate::geometry::dim2::utils::Point2;
use crate::solver::{Constraint2d, PointRef, System2d};
use std::collections::HashMap;

/// Which curve ends are one point, as the system's coincidences say.
///
/// Positions alone cannot say it: two ends can touch without being joined, and a
/// coincidence on an arc's end holds only within tolerance.
pub(super) struct Joints {
    /// Where each joined end sits: every end of a joint gets the same position.
    positions: HashMap<PointRef, Point2>,
    /// The joint each joined end belongs to.
    joint_of: HashMap<PointRef, usize>,
}

impl Joints {
    /// Groups the curves' ends through the system's coincidences. A coincidence chain may
    /// run through what is not drawn, such as an independent point or construction line.
    pub fn of(system: &System2d, curves: &[Drawn]) -> Result<Self, SketchBuildError> {
        let mut parents = HashMap::new();
        for constraint in system.constraints.values() {
            if let Constraint2d::Coincident(a, b) | Constraint2d::Distance(a, b, 0.) =
                constraint.definition
            {
                let (a, b) = (root(&mut parents, a), root(&mut parents, b));
                if a != b {
                    parents.insert(b, a);
                }
            }
        }
        let mut members: Vec<(PointRef, Vec<usize>)> = vec![];
        for (index, curve) in curves.iter().enumerate() {
            for (end, _) in curve.ends() {
                let joint = root(&mut parents, end);
                match members
                    .iter_mut()
                    .find(|(candidate, _)| *candidate == joint)
                {
                    Some((_, curve_indices)) => curve_indices.push(index),
                    None => members.push((joint, vec![index])),
                }
            }
        }
        for (_, curve_indices) in &members {
            if curve_indices.len() > 2 {
                return Err(SketchBuildError::Branching {
                    curves: curve_indices.iter().map(|i| curves[*i].curve).collect(),
                });
            }
        }
        let mut positions = HashMap::new();
        let mut joint_of = HashMap::new();
        let mut first_at: Vec<Option<(Point2, usize)>> = vec![None; members.len()];
        for (index, curve) in curves.iter().enumerate() {
            for (end, position) in curve.ends() {
                let joint = root(&mut parents, end);
                let joint = members
                    .iter()
                    .position(|(candidate, _)| *candidate == joint)
                    .expect("every end was grouped above");
                let (shared, first) = *first_at[joint].get_or_insert((position, index));
                let gap = (position - shared).norm();
                if gap > LINEAR_TOLERANCE {
                    return Err(SketchBuildError::JointGap {
                        first: curves[first].curve,
                        second: curve.curve,
                        gap,
                    });
                }
                positions.insert(end, shared);
                joint_of.insert(end, joint);
            }
        }
        Ok(Self {
            positions,
            joint_of,
        })
    }

    /// The position an end takes in the topology.
    pub fn position(&self, end: PointRef) -> Point2 {
        self.positions[&end]
    }

    /// The joint an end belongs to; an end joined to nothing is a joint of its own.
    pub fn joint(&self, end: PointRef) -> usize {
        self.joint_of[&end]
    }
}

fn root(parents: &mut HashMap<PointRef, PointRef>, mut point: PointRef) -> PointRef {
    while let Some(parent) = parents.get(&point) {
        point = *parent;
    }
    point
}
