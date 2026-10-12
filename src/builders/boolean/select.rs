//! Decides which classified fragments bound the result.
//!
//! The rule is the same for every operation, every operand count and both
//! domains. Each fragment is read as a sheet with a front and a behind, and the
//! domain says, for every other operand, whether the points just in front and
//! just behind belong to it ([`super::domain::Membership`]). The operation's set
//! formula is evaluated on both sides:
//!
//! - **union**: `T ∪ t₁ ∪ … ∪ tₙ`;
//! - **intersection**: `T ∩ t₁ ∩ … ∩ tₙ`;
//! - **difference**: `T − (t₁ ∪ … ∪ tₙ)`.
//!
//! A fragment lies on the result's boundary exactly where the two answers
//! differ. It is kept as it stands when the result is behind it, and reversed
//! when the result is in front -- which only a tool's boundary under a
//! difference can be.
//!
//! Where several operands hold a copy of the same fragment, every copy reads
//! the same two answers, so one of them is kept: the one of the lowest-index
//! operand, the target first. With one tool this keeps the target's copy of a
//! boundary both operands share whenever the result keeps it at all.

use super::domain::{BooleanDomain, Membership, RelativeLocation, Selection};
use super::{BooleanOperation, neighborhood::FragmentGraph};

/// The target's operand index; the tools follow it.
const TARGET: usize = 0;

pub(crate) struct SelectionPlan<D: BooleanDomain> {
    pub(crate) kept: Vec<D::Fragment>,
    pub(crate) reversed: Vec<D::Fragment>,
    pub(crate) dropped: Vec<D::Fragment>,
}

/// Sorts every fragment into kept, kept-reversed or dropped.
///
/// `classes[i]` holds where fragment `i` lies against the other operands, by
/// operand index, in the order [`super::classify::run`] asked them. It stops
/// asking once [`settles`] says the answer is a drop, so a fragment need not
/// carry a location for every operand.
pub(crate) fn run<D: BooleanDomain>(
    operation: BooleanOperation,
    operands: usize,
    graph: &FragmentGraph<D>,
    classes: &[Vec<(usize, RelativeLocation)>],
) -> SelectionPlan<D> {
    let mut plan = SelectionPlan {
        kept: Vec::new(),
        reversed: Vec::new(),
        dropped: Vec::new(),
    };
    for (fragment, locations) in graph.fragments.iter().zip(classes) {
        let memberships = locations
            .iter()
            .map(|&(operand, location)| (operand, D::membership(location)));
        match decide(operation, operands, fragment.operand, memberships) {
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

/// Whether one location already drops a fragment of operand `own`, whatever
/// the remaining operands say.
///
/// That is the case when another operand's copy of the fragment wins, or when
/// one operand alone fixes the formula on both sides: inside any operand of a
/// union, outside any operand of an intersection, outside the target or inside
/// a tool of a difference.
pub(crate) fn settles<D: BooleanDomain>(
    operation: BooleanOperation,
    own: usize,
    other: usize,
    location: RelativeLocation,
) -> bool {
    settled(operation, own, other, D::membership(location))
}

fn settled(operation: BooleanOperation, own: usize, other: usize, membership: Membership) -> bool {
    let everywhere = membership.front && membership.behind;
    let nowhere = !membership.front && !membership.behind;
    (membership.coincident && other < own)
        || match operation {
            BooleanOperation::Union => everywhere,
            BooleanOperation::Intersection => nowhere,
            BooleanOperation::Difference if other == TARGET => nowhere,
            BooleanOperation::Difference => everywhere,
        }
}

/// Selects one fragment of operand `own` among `operands`, from its
/// membership in each other operand it was located against.
fn decide(
    operation: BooleanOperation,
    operands: usize,
    own: usize,
    memberships: impl IntoIterator<Item = (usize, Membership)>,
) -> Selection {
    let mut front = vec![false; operands];
    let mut behind = vec![false; operands];
    front[own] = Membership::OWN.front;
    behind[own] = Membership::OWN.behind;
    for (operand, membership) in memberships {
        if settled(operation, own, operand, membership) {
            return Selection::Drop;
        }
        front[operand] = membership.front;
        behind[operand] = membership.behind;
    }
    match (evaluate(operation, &front), evaluate(operation, &behind)) {
        (false, true) => Selection::Keep,
        (true, false) => Selection::KeepReversed,
        _ => Selection::Drop,
    }
}

/// The operation's set formula over one point's membership in each operand,
/// the target first.
fn evaluate(operation: BooleanOperation, members: &[bool]) -> bool {
    match operation {
        BooleanOperation::Union => members.iter().any(|&member| member),
        BooleanOperation::Intersection => members.iter().all(|&member| member),
        BooleanOperation::Difference => {
            members[TARGET] && !members[TARGET + 1..].iter().any(|&member| member)
        }
    }
}
