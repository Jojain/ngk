//! Face-fragment identity and adjacency derived from the staged map.

use super::{BooleanPreparation, BooleanSide};
use crate::topology::{gmap::GMap, payload::Payload, shape_keys::FaceKey};
use std::collections::{BTreeMap, HashMap, HashSet};

#[derive(Clone, Copy)]
pub(crate) struct BoundaryFragment {
    pub(crate) face: FaceKey,
    pub(crate) source_face: FaceKey,
    pub(crate) side: BooleanSide,
}

pub(crate) struct FragmentGraph {
    pub(crate) fragments: Vec<BoundaryFragment>,
    pub(crate) components: Vec<Vec<usize>>,
}

impl FragmentGraph {
    /// Builds same-operand components, treating every known intersection edge as a barrier.
    pub(crate) fn build<P: Payload>(map: &GMap<P>, preparation: &BooleanPreparation) -> Self {
        let mut ordered = BTreeMap::new();
        for (side, lineage) in [
            (BooleanSide::First, &preparation.first_lineage),
            (BooleanSide::Second, &preparation.second_lineage),
        ] {
            for (&source_face, faces) in &lineage.faces {
                for &face in faces {
                    ordered.insert(
                        face,
                        BoundaryFragment {
                            face,
                            source_face,
                            side,
                        },
                    );
                }
            }
        }
        let fragments = ordered.into_values().collect::<Vec<_>>();
        let index = fragments
            .iter()
            .enumerate()
            .map(|(i, f)| (f.face, i))
            .collect::<HashMap<_, _>>();
        let barriers = preparation
            .span_edges
            .values()
            .flatten()
            .flatten()
            .copied()
            .collect::<HashSet<_>>();
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
                for edge in map.face_unchecked(fragments[i].face).edges() {
                    if barriers.contains(&edge.key()) {
                        continue;
                    }
                    for face in edge.faces() {
                        if let Some(&j) = index.get(&face.key())
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
