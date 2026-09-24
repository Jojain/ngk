//! Fragment identity derived from the split operands' lineage.

use super::domain::BooleanDomain;
use super::{BooleanOperandPreparation, BooleanSide};
use std::collections::BTreeMap;

/// One boundary cell of one operand after splitting, and where it came from.
pub(crate) struct BoundaryFragment<D: BooleanDomain> {
    pub(crate) fragment: D::Fragment,
    pub(crate) source: D::Fragment,
    pub(crate) side: BooleanSide,
}

// Derived implementations would demand `D: Clone` and `D: Copy`, which the
// domain marker has no reason to satisfy; only the fragment is copied.
impl<D: BooleanDomain> Clone for BoundaryFragment<D> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<D: BooleanDomain> Copy for BoundaryFragment<D> {}

pub(crate) struct FragmentGraph<D: BooleanDomain> {
    pub(crate) fragments: Vec<BoundaryFragment<D>>,
}

impl<D: BooleanDomain> FragmentGraph<D> {
    /// Collects every fragment both operands were split into, in key order.
    pub(crate) fn build(preparation: &BooleanOperandPreparation) -> Self {
        let mut ordered = BTreeMap::new();
        for (side, lineage) in [
            (BooleanSide::First, &preparation.first_lineage),
            (BooleanSide::Second, &preparation.second_lineage),
        ] {
            for (source, fragment) in D::fragments(lineage) {
                ordered.insert(
                    fragment,
                    BoundaryFragment {
                        fragment,
                        source,
                        side,
                    },
                );
            }
        }
        Self {
            fragments: ordered.into_values().collect(),
        }
    }
}
