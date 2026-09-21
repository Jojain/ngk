//! Fragment identity and adjacency derived from the staged map.

use super::domain::BooleanDomain;
use super::{BooleanOperandPreparation, BooleanSide};
use crate::model::Model;
use crate::topology::payload::Payload;
use std::collections::{BTreeMap, HashMap, HashSet};

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
    pub(crate) components: Vec<Vec<usize>>,
}

impl<D: BooleanDomain> FragmentGraph<D> {
    /// Builds same-operand components, treating every realized section cell as a barrier.
    pub(crate) fn build<P: Payload>(
        map: &Model<P>,
        preparation: &BooleanOperandPreparation,
    ) -> Self {
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
        let fragments = ordered.into_values().collect::<Vec<_>>();
        let index = fragments
            .iter()
            .enumerate()
            .map(|(i, f)| (f.fragment, i))
            .collect::<HashMap<_, _>>();
        let barriers = D::barriers(map, preparation);
        let mut visited = HashSet::new();
        let mut components = Vec::new();
        for seed in 0..fragments.len() {
            if visited.contains(&seed) {
                continue;
            }
            let mut pending = vec![seed];
            let mut component = Vec::new();
            while let Some(i) = pending.pop() {
                if !visited.insert(i) {
                    continue;
                }
                component.push(i);
                for boundary in D::boundaries(map, fragments[i].fragment) {
                    if barriers.contains(&boundary) {
                        continue;
                    }
                    for neighbour in D::incident(map, boundary) {
                        if let Some(&j) = index.get(&neighbour)
                            && fragments[j].side == fragments[i].side
                        {
                            pending.push(j);
                        }
                    }
                }
            }
            component.sort_unstable();
            components.push(component);
        }
        Self {
            fragments,
            components,
        }
    }
}
