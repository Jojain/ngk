//! Regularized selection policy for both operand boundaries.
use super::{
    BooleanOperation, BooleanSide, classify::RelativeLocation, neighborhood::FragmentGraph,
};
use crate::topology::shape_keys::FaceKey;

pub(crate) struct SelectionPlan {
    pub(crate) kept: Vec<FaceKey>,
    pub(crate) reversed: Vec<FaceKey>,
    pub(crate) dropped: Vec<FaceKey>,
}

/// Applies the operation table, retaining A's copy of same-oriented coincidence.
pub(crate) fn run(
    operation: BooleanOperation,
    graph: &FragmentGraph,
    classes: &[RelativeLocation],
) -> SelectionPlan {
    use BooleanOperation::{Difference, Intersection, Union};
    use BooleanSide::{First, Second};
    use RelativeLocation::{Inside, OnBoundaryOpposite, OnBoundarySame, Outside};
    let mut plan = SelectionPlan {
        kept: Vec::new(),
        reversed: Vec::new(),
        dropped: Vec::new(),
    };
    for (fragment, location) in graph.fragments.iter().zip(classes) {
        let keep = matches!(
            (operation, fragment.side, location),
            (Union, _, Outside)
                | (Union, First, OnBoundarySame)
                | (Intersection, _, Inside)
                | (Intersection, First, OnBoundarySame)
                | (Difference, First, Outside | OnBoundaryOpposite)
                | (Difference, Second, Inside)
        );
        if keep {
            plan.kept.push(fragment.face);
            if operation == Difference && fragment.side == Second {
                plan.reversed.push(fragment.face);
            }
        } else {
            plan.dropped.push(fragment.face);
        }
    }
    plan
}
