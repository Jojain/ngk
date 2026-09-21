//! Applies one domain's operation table to the classified fragments.

use super::domain::{BooleanDomain, RelativeLocation, Selection};
use super::{BooleanOperation, neighborhood::FragmentGraph};

pub(crate) struct SelectionPlan<D: BooleanDomain> {
    pub(crate) kept: Vec<D::Fragment>,
    pub(crate) reversed: Vec<D::Fragment>,
    pub(crate) dropped: Vec<D::Fragment>,
}

/// Sorts every fragment into kept, kept-reversed or dropped.
///
/// The table itself belongs to the domain: what survives a union depends on
/// whether a fragment is a piece of boundary or a piece of interior, and only
/// the domain knows which it handed over.
pub(crate) fn run<D: BooleanDomain>(
    operation: BooleanOperation,
    graph: &FragmentGraph<D>,
    classes: &[RelativeLocation],
) -> SelectionPlan<D> {
    let mut plan = SelectionPlan {
        kept: Vec::new(),
        reversed: Vec::new(),
        dropped: Vec::new(),
    };
    for (fragment, &location) in graph.fragments.iter().zip(classes) {
        match D::keeps(operation, fragment.side, location) {
            Selection::Keep => plan.kept.push(fragment.fragment),
            Selection::KeepReversed => {
                plan.kept.push(fragment.fragment);
                plan.reversed.push(fragment.fragment);
            }
            Selection::Drop => plan.dropped.push(fragment.fragment),
        }
    }
    plan
}
