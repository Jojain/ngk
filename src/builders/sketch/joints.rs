use super::SketchError;
use super::drawn::Drawn;
use crate::geometry::LINEAR_TOLERANCE;
use crate::geometry::dim2::utils::Point2;
use crate::solver::{Constraint2d, CurveId, PointRef, System2d};
use std::collections::HashMap;

/// How far a reported crossing may sit from a joint and still be that joint. Curves leaving
/// a joint tangentially meet at a poorly located point, well outside the kernel's tolerance.
pub(super) const JOINT_CONTACT: f64 = 1e-6;

/// Which curve ends are one point, as the system's coincidences say.
///
/// Positions alone cannot say it: two ends can touch without being joined, and a
/// coincidence on an arc's end holds only within tolerance.
pub(super) struct Joints {
    /// Where each joined end sits: every end of a joint gets the same position.
    positions: HashMap<PointRef, Point2>,
    /// The joint each joined end belongs to.
    joint_of: HashMap<PointRef, usize>,
    /// The curves with an end at each joint, once per end.
    ends_at: Vec<Vec<CurveId>>,
}

impl Joints {
    /// Groups the curves' ends through the system's coincidences. A coincidence chain may
    /// run through what is not drawn, such as an independent point or construction line.
    pub fn of(system: &System2d, curves: &[Drawn]) -> Result<Self, SketchError> {
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
        let mut members: Vec<(PointRef, Vec<CurveId>)> = vec![];
        for curve in curves {
            for (end, _) in curve.ends() {
                let joint = root(&mut parents, end);
                match members
                    .iter_mut()
                    .find(|(candidate, _)| *candidate == joint)
                {
                    Some((_, ends)) => ends.push(curve.curve),
                    None => members.push((joint, vec![curve.curve])),
                }
            }
        }
        let mut positions = HashMap::new();
        let mut joint_of = HashMap::new();
        let mut first_at: Vec<Option<(Point2, CurveId)>> = vec![None; members.len()];
        for curve in curves {
            for (end, position) in curve.ends() {
                let joint = root(&mut parents, end);
                let joint = members
                    .iter()
                    .position(|(candidate, _)| *candidate == joint)
                    .expect("every end was grouped above");
                let (shared, first) = *first_at[joint].get_or_insert((position, curve.curve));
                let gap = (position - shared).norm();
                if gap > LINEAR_TOLERANCE {
                    return Err(SketchError::JointGap {
                        first,
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
            ends_at: members.into_iter().map(|(_, ends)| ends).collect(),
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

    /// The curves at a joint where more than two ends meet, if there is one.
    pub fn branching(&self) -> Option<&[CurveId]> {
        self.ends_at
            .iter()
            .find(|ends| ends.len() > 2)
            .map(Vec::as_slice)
    }
}

fn root(parents: &mut HashMap<PointRef, PointRef>, mut point: PointRef) -> PointRef {
    while let Some(parent) = parents.get(&point) {
        point = *parent;
    }
    point
}
