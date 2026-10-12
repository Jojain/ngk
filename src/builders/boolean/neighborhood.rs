//! Fragment identity derived from the split operands' lineage.

use super::BooleanLineage;
use super::domain::BooleanDomain;
use std::collections::BTreeMap;

/// One boundary cell of one operand after splitting, and where it came from.
pub(crate) struct BoundaryFragment<D: BooleanDomain> {
    pub(crate) fragment: D::Fragment,
    pub(crate) source: D::Fragment,
    /// The operand's index, the target first.
    pub(crate) operand: usize,
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
    /// Collects every fragment every operand was split into, in key order.
    ///
    /// `lineages` is indexed by operand, the target first.
    pub(crate) fn build(lineages: &[BooleanLineage]) -> Self {
        let mut ordered = BTreeMap::new();
        for (operand, lineage) in lineages.iter().enumerate() {
            for (source, fragment) in D::fragments(lineage) {
                ordered.insert(
                    fragment,
                    BoundaryFragment {
                        fragment,
                        source,
                        operand,
                    },
                );
            }
        }
        Self {
            fragments: ordered.into_values().collect(),
        }
    }
}
