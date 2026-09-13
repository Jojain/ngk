use std::collections::{HashMap, HashSet, VecDeque};

use thiserror::Error;

use crate::topology::gmap::{Dart, Dim, GMap};

use super::ownership::{EntityOwner, OwnershipIndex, Subdivision};
use super::walk::{below, turn};

/// Every raw dart a logical entity's interior covers, and which way each reads.
///
/// The region is recovered, never stored. It is the answer to "which part of
/// the map is this entity", and it is recomputed from ownership labels plus the
/// map's own involutions, so a refinement nobody told the entity about still
/// yields the same region.
///
/// Alongside membership the walk records a sense per dart: every involution
/// step flips it, and so does every crossing of an interior cut, because the
/// turn across one is always an odd number of steps. Darts sharing the anchor's
/// sense are the ones that read the entity the way its anchor does, which is
/// what keeps every loop of a face wound the same way round.
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
    pub fn frontier(&self, index: &OwnershipIndex) -> Vec<Dart> {
        let Some(below) = below(self.dimension()) else {
            return Vec::new();
        };
        self.darts()
            .filter(|&dart| index.owner(below, dart) != Some(self.owner))
            .collect()
    }
}

/// Recovers the region of `owner`, starting from a dart of one of its cells.
///
/// The walk stays inside a raw cell using every involution but the cell's own,
/// and leaves it only across a lower cell that `owner` also owns, turning with
/// [`turn`]. Cells a higher-dimensional entity owns are scaffold the turn
/// passes over; anything else across an owned interior boundary is a labelling
/// mistake and is reported rather than absorbed.
pub fn recover_region(
    map: &GMap,
    index: &OwnershipIndex,
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

        let Some(below) = below(dimension) else {
            continue;
        };
        if index.owner(below, dart) != Some(owner) {
            continue;
        }

        let across =
            turn(map, index, dimension, dart).ok_or(RegionError::InteriorBoundaryNotShared {
                dimension: below,
                dart,
                owner,
            })?;
        match index.owner(dimension, across) {
            Some(found) if found == owner => queue.push_back((across, !aligned)),
            Some(found) => {
                return Err(RegionError::ForeignCell {
                    dimension,
                    dart: across,
                    expected: owner,
                    found,
                });
            }
            None => {
                return Err(RegionError::UnlabelledCell {
                    dimension,
                    dart: across,
                    across: owner,
                });
            }
        }
    }

    Ok(LogicalRegion {
        owner,
        order,
        sense,
    })
}

/// Recovers the region of every entity named by `subdivision`.
///
/// Each entity is anchored at its first recorded cell of its own dimension, and
/// every other record it holds must then touch the region that walk produces. A
/// same-dimension record left outside is two things sharing one key; a
/// lower-dimension record left outside is scaffold labelled for an entity it is
/// not inside.
pub fn recover_all_regions(
    map: &GMap,
    index: &OwnershipIndex,
    subdivision: &Subdivision,
) -> Result<Vec<LogicalRegion>, RegionError> {
    let mut seen = HashSet::new();
    let mut regions = Vec::new();

    for record in subdivision.records() {
        if record.dimension != record.owner.dimension() || !seen.insert(record.owner) {
            continue;
        }
        regions.push(recover_region(
            map,
            index,
            record.owner,
            record.representative,
        )?);
    }

    for record in subdivision.records() {
        let Some(region) = regions.iter().find(|r| r.owner() == record.owner) else {
            return Err(RegionError::NoAnchor {
                owner: record.owner,
            });
        };
        let touches = map
            .orbit(record.representative, map.orbit_indices(record.dimension))
            .any(|dart| region.contains(dart));
        if !touches {
            return Err(RegionError::Disconnected {
                owner: record.owner,
                representative: record.representative,
            });
        }
    }

    Ok(regions)
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

    #[error(
        "{across:?} owns an interior boundary leading to the unlabelled {dimension:?}-cell at {dart:?}"
    )]
    UnlabelledCell {
        /// Dimension of the cell reached.
        dimension: Dim,
        /// A dart of the cell reached.
        dart: Dart,
        /// The entity whose interior boundary was crossed.
        across: EntityOwner,
    },

    #[error(
        "{expected:?} owns an interior boundary leading into the {dimension:?}-cell at {dart:?}, which {found:?} owns"
    )]
    ForeignCell {
        /// Dimension of the cell reached.
        dimension: Dim,
        /// A dart of the cell reached.
        dart: Dart,
        /// The entity whose interior boundary was crossed.
        expected: EntityOwner,
        /// The entity that owns the cell on the far side.
        found: EntityOwner,
    },

    #[error(
        "{owner:?} owns the {dimension:?}-cell at {dart:?} as interior, but nothing lies across it"
    )]
    InteriorBoundaryNotShared {
        /// Dimension of the cell claimed as interior.
        dimension: Dim,
        /// A dart of that cell.
        dart: Dart,
        /// The entity claiming it.
        owner: EntityOwner,
    },

    #[error("{owner:?} reaches {dart:?} both ways round, so it has no consistent sense")]
    InconsistentOrientation {
        /// The dart reached with two senses.
        dart: Dart,
        /// The entity whose region was being walked.
        owner: EntityOwner,
    },

    #[error(
        "{owner:?} also labels the cell at {representative:?}, which its region does not reach"
    )]
    Disconnected {
        /// The entity with more than one region.
        owner: EntityOwner,
        /// A dart of the unreachable cell.
        representative: Dart,
    },

    #[error("{owner:?} labels scaffold but owns no cell of its own dimension to be anchored at")]
    NoAnchor {
        /// The entity with no cell of its own.
        owner: EntityOwner,
    },
}
