use std::collections::{HashMap, HashSet, VecDeque};

use thiserror::Error;

use crate::topology::gmap::{Dart, Dim, GMap};

use super::cells::{EmbeddingIndex, EntityOwner};
use super::walk::below;

/// Every raw dart a logical entity's interior covers, and which way each reads.
///
/// The region is recovered, never stored: it is the orbit of the entity's
/// anchor under every involution but the entity's own. An entity occupies
/// exactly one raw cell of its own dimension, so that orbit *is* the entity —
/// there is no second cell to reach and nothing to flood.
///
/// Alongside membership the walk records a sense per dart, which every
/// involution step flips. Darts sharing the anchor's sense are the ones that
/// read the entity the way its anchor does, which is what keeps every loop of
/// a face wound the same way round.
#[derive(Debug, Clone)]
pub struct LogicalRegion {
    owner: EntityOwner,
    order: Vec<Dart>,
    sense: HashMap<Dart, bool>,
}

impl LogicalRegion {
    /// Returns the entity this region belongs to.
    pub fn owner(&self) -> EntityOwner {
        self.owner
    }

    /// Returns the dimension of the region's raw cells.
    pub fn dimension(&self) -> Dim {
        self.owner.dimension()
    }

    /// Reports whether `dart` lies in a raw cell this entity owns.
    pub fn contains(&self, dart: Dart) -> bool {
        self.sense.contains_key(&dart)
    }

    /// Reports whether `dart` reads the region the way its anchor does.
    pub fn is_aligned(&self, dart: Dart) -> bool {
        self.sense.get(&dart) == Some(&true)
    }

    /// Returns the number of darts in the region.
    pub fn len(&self) -> usize {
        self.order.len()
    }

    /// Reports whether the region covers no darts.
    pub fn is_empty(&self) -> bool {
        self.order.is_empty()
    }

    /// Iterates the region's darts in the order the walk first reached them.
    pub fn darts(&self) -> impl Iterator<Item = Dart> + '_ {
        self.order.iter().copied()
    }

    /// Returns one dart per raw cell of the region, in first-seen order.
    pub fn cells(&self, map: &GMap) -> Vec<Dart> {
        let indices = map.orbit_indices(self.dimension());
        let mut seen = HashSet::new();
        let mut cells = Vec::new();
        for dart in self.darts() {
            if !seen.insert(dart) {
                continue;
            }
            for member in map.orbit(dart, indices.clone()) {
                seen.insert(member);
            }
            cells.push(dart);
        }
        cells
    }

    /// Returns the darts of the region that lie on its frontier.
    ///
    /// A dart is on the frontier when the raw cell one dimension below it is
    /// not owned by this entity — the cell is a real boundary, not a cut the
    /// entity's own interior runs through.
    pub fn frontier(&self, index: &EmbeddingIndex) -> Vec<Dart> {
        let Some(below) = below(self.dimension()) else {
            return Vec::new();
        };
        self.darts()
            .filter(|&dart| index.owner(below, dart) != Some(self.owner))
            .collect()
    }
}

/// Recovers the region of `owner` from the cell its anchor sits in.
///
/// The walk is the anchor's orbit under every involution but the entity's own,
/// and it never leaves that one cell, because an entity has no second cell of
/// its own dimension to leave to. A cut the entity owns lies in the same cell
/// and is reached without being crossed.
pub fn recover_region(
    map: &GMap,
    index: &EmbeddingIndex,
    owner: EntityOwner,
    anchor: Dart,
) -> Result<LogicalRegion, RegionError> {
    let dimension = owner.dimension();
    if index.owner(dimension, anchor) != Some(owner) {
        return Err(RegionError::AnchorNotOwned { anchor, owner });
    }

    let within = map.orbit_indices(dimension);
    let mut order = Vec::new();
    let mut sense: HashMap<Dart, bool> = HashMap::new();
    let mut queue = VecDeque::from([(anchor, true)]);

    while let Some((dart, aligned)) = queue.pop_front() {
        match sense.get(&dart) {
            Some(&held) if held == aligned => continue,
            Some(_) => return Err(RegionError::InconsistentOrientation { dart, owner }),
            None => {
                sense.insert(dart, aligned);
                order.push(dart);
            }
        }

        for &i in &within {
            // A free involution is a loop back onto the same dart. It says
            // nothing about which way round the map reads, so it is not a step.
            let next = map.alpha(Dim::from_index(i), dart);
            if next != dart {
                queue.push_back((next, !aligned));
            }
        }
    }

    Ok(LogicalRegion {
        owner,
        order,
        sense,
    })
}

/// A labelling that no logical region can be recovered from.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum RegionError {
    #[error("{owner:?} does not own the cell at {anchor:?}, so no region starts there")]
    AnchorNotOwned {
        /// The dart the walk was asked to start from.
        anchor: Dart,
        /// The entity whose region was requested.
        owner: EntityOwner,
    },

    #[error("{owner:?} reaches {dart:?} both ways round, so it has no consistent sense")]
    InconsistentOrientation {
        /// The dart reached with two senses.
        dart: Dart,
        /// The entity whose region was being walked.
        owner: EntityOwner,
    },
}
