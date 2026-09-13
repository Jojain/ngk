use std::collections::{BTreeMap, HashMap};

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::topology::gmap::{Dart, Dim, GMAP_INVOLUTION_COUNT, GMap};
use crate::topology::shape_keys::{EdgeKey, FaceKey, SolidKey, VertexKey};

/// The logical entity whose interior contains a raw cell.
///
/// Profiles and sheets are absent on purpose: they are aggregates of logical
/// entities, not regions of the map, so nothing lies in a profile's interior
/// that does not already lie in an edge's.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum EntityOwner {
    /// A logical vertex.
    Vertex(VertexKey),
    /// A logical edge, open or closed.
    Edge(EdgeKey),
    /// A logical face, bounded or boundaryless.
    Face(FaceKey),
    /// A logical solid.
    Solid(SolidKey),
}

impl EntityOwner {
    /// Returns the dimension of the entity.
    ///
    /// A raw cell may only be owned by an entity of its own dimension or
    /// higher: a seam edge lies inside a face, but no cell lies inside
    /// something smaller than itself.
    pub fn dimension(self) -> Dim {
        match self {
            Self::Vertex(_) => Dim::Zero,
            Self::Edge(_) => Dim::One,
            Self::Face(_) => Dim::Two,
            Self::Solid(_) => Dim::Three,
        }
    }
}

/// One raw cell's ownership, read out of a [`Subdivision`].
///
/// This is a view assembled on the way out, not a stored row: the dimension is
/// the shelf the entry sits on and the representative is its key. The
/// representative locates the orbit; it is not the identity of anything. A
/// refinement that destroys this dart re-anchors the entry on a surviving dart
/// of the same orbit rather than allocating a second one.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct OrbitOwnership {
    /// Dimension of the raw cell this entry labels.
    pub dimension: Dim,
    /// A dart of the labelled orbit.
    pub representative: Dart,
    /// The logical entity whose interior contains the cell.
    pub owner: EntityOwner,
}

/// The authoritative classification of one map's raw cells.
///
/// One entry per labelled orbit and nothing else. The set of darts an entity
/// covers is not stored: it is recovered by walking the map, which is what
/// keeps the classification valid across a refinement the entity did not ask
/// for.
///
/// Entries live on one shelf per cell dimension, keyed by a representative
/// dart, so a cell's dimension is where its entry is rather than a field that
/// could disagree with it. Keying also makes labelling the same anchor twice a
/// replacement instead of a second entry, which is what promoting a scaffold
/// cell to a logical one does: an edge inside a face becoming an edge in its
/// own right rewrites one entry.
///
/// `BTreeMap` rather than `HashMap`: entries come back in dart order, so
/// enumeration and serialization are deterministic. Ordering here is a
/// property callers are entitled to, not an accident of a hasher.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Subdivision {
    cells: [BTreeMap<Dart, EntityOwner>; GMAP_INVOLUTION_COUNT],
}

impl Default for Subdivision {
    fn default() -> Self {
        Self {
            cells: std::array::from_fn(|_| BTreeMap::new()),
        }
    }
}

impl Subdivision {
    /// Creates a classification with no labelled orbits.
    pub fn new() -> Self {
        Self::default()
    }

    /// Labels the raw `dimension`-cell containing `representative`.
    ///
    /// Labelling the same anchor again replaces what it said. Two entries that
    /// reach one orbit by *different* anchors are still a contradiction, and
    /// [`OwnershipIndex::build`] is where that is caught.
    pub fn own(&mut self, dimension: Dim, representative: Dart, owner: EntityOwner) {
        self.cells[dimension.index()].insert(representative, owner);
    }

    /// Returns what the entry anchored exactly at `representative` says.
    ///
    /// This is the stored entry, not the answer for the whole orbit: a dart of
    /// the same cell that is not the anchor returns `None`. Ask
    /// [`OwnershipIndex::owner`] for the orbit-wide answer.
    pub fn owner_at(&self, dimension: Dim, representative: Dart) -> Option<EntityOwner> {
        self.cells[dimension.index()].get(&representative).copied()
    }

    /// Returns every entry, by ascending dimension and then by anchor.
    pub fn records(&self) -> impl Iterator<Item = OrbitOwnership> + '_ {
        self.cells.iter().enumerate().flat_map(|(index, shelf)| {
            let dimension = Dim::from_index(index);
            shelf
                .iter()
                .map(move |(&representative, &owner)| OrbitOwnership {
                    dimension,
                    representative,
                    owner,
                })
        })
    }

    /// Returns the entries naming `owner`, in the same order as [`Self::records`].
    pub fn records_of(&self, owner: EntityOwner) -> impl Iterator<Item = OrbitOwnership> + '_ {
        self.records().filter(move |record| record.owner == owner)
    }

    /// Returns how many orbits are labelled.
    pub fn len(&self) -> usize {
        self.cells.iter().map(BTreeMap::len).sum()
    }

    /// Reports whether nothing is labelled.
    pub fn is_empty(&self) -> bool {
        self.cells.iter().all(BTreeMap::is_empty)
    }

    /// Expands every record into the dart lookup the walkers read.
    ///
    /// This is the derived half of the classification: it is rebuilt from the
    /// records whenever the map changes, and is never serialized.
    pub fn index(&self, map: &GMap) -> Result<OwnershipIndex, SubdivisionError> {
        OwnershipIndex::build(map, self)
    }

    /// Rewrites every entry's anchor through `map`.
    ///
    /// An entry names an orbit, not a dart, so renumbering the map moves the
    /// anchor and changes nothing else about what the entry says.
    ///
    /// # Panics
    ///
    /// Panics if `map` sends two anchors of one dimension to the same dart.
    /// Renumbering is a bijection over retained darts, so that means the caller
    /// handed over a mapping its own map does not agree with.
    pub(crate) fn map_darts(&mut self, map: impl Fn(Dart) -> Dart) {
        for shelf in &mut self.cells {
            let mut moved = BTreeMap::new();
            for (representative, owner) in std::mem::take(shelf) {
                let landed = map(representative);
                assert!(
                    moved.insert(landed, owner).is_none(),
                    "renumbering should not land two ownership anchors on {landed:?}"
                );
            }
            *shelf = moved;
        }
    }

    /// Adds the entries of `source` whose anchors were copied, remapped.
    ///
    /// An entry whose anchor did not come across is dropped: the cell it named
    /// is not in this model, so nothing here is classified by it.
    pub(crate) fn extend_remapped(&mut self, source: &Subdivision, darts: &HashMap<Dart, Dart>) {
        for record in source.records() {
            if let Some(&representative) = darts.get(&record.representative) {
                self.own(record.dimension, representative, record.owner);
            }
        }
    }
}

/// Dart-to-owner lookup derived from a [`Subdivision`] and the map it labels.
#[derive(Debug, Clone)]
pub struct OwnershipIndex {
    owners: [HashMap<Dart, EntityOwner>; GMAP_INVOLUTION_COUNT],
}

impl OwnershipIndex {
    /// Expands every ownership record across its orbit.
    ///
    /// Rejects a record whose representative is not a dart of `map`, a record
    /// whose owner is of lower dimension than the cell it labels, and two
    /// records that disagree about one orbit.
    pub fn build(map: &GMap, subdivision: &Subdivision) -> Result<Self, SubdivisionError> {
        let mut owners: [HashMap<Dart, EntityOwner>; GMAP_INVOLUTION_COUNT] =
            std::array::from_fn(|_| HashMap::new());

        for record in subdivision.records() {
            let OrbitOwnership {
                dimension,
                representative,
                owner,
            } = record;

            if representative.id() >= map.dart_count() {
                return Err(SubdivisionError::DanglingRecord {
                    dimension,
                    representative,
                });
            }
            if owner.dimension().index() < dimension.index() {
                return Err(SubdivisionError::OwnerBelowCell {
                    dimension,
                    representative,
                    owner,
                });
            }

            let slot = &mut owners[dimension.index()];
            for dart in map.orbit(representative, map.orbit_indices(dimension)) {
                if let Some(&held) = slot.get(&dart)
                    && held != owner
                {
                    return Err(SubdivisionError::ConflictingOwnership {
                        dimension,
                        dart,
                        held,
                        claimed: owner,
                    });
                }
                slot.insert(dart, owner);
            }
        }

        Ok(Self { owners })
    }

    /// Returns the entity owning the raw `dimension`-cell that `dart` lies in.
    pub fn owner(&self, dimension: Dim, dart: Dart) -> Option<EntityOwner> {
        self.owners[dimension.index()].get(&dart).copied()
    }

    /// Reports whether the raw `dimension`-cell at `dart` is interior scaffold.
    ///
    /// A cell is scaffold when something of strictly higher dimension owns it:
    /// a cylinder's seam edge inside its wall, a cut face inside a solid. Such
    /// a cell is crossed by a traversal rather than emitted by it.
    pub fn is_scaffold(&self, dimension: Dim, dart: Dart) -> bool {
        self.owner(dimension, dart)
            .is_some_and(|owner| owner.dimension().index() > dimension.index())
    }
}

/// A classification that does not describe the map it labels.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum SubdivisionError {
    #[error(
        "ownership record for a {dimension:?}-cell names {representative:?}, which is not a dart of the map"
    )]
    DanglingRecord {
        /// Dimension the record claims to label.
        dimension: Dim,
        /// The dart the record is anchored at.
        representative: Dart,
    },

    #[error(
        "{owner:?} cannot own the {dimension:?}-cell at {representative:?}: an owner is never of lower dimension than the cell it contains"
    )]
    OwnerBelowCell {
        /// Dimension of the labelled cell.
        dimension: Dim,
        /// The dart the record is anchored at.
        representative: Dart,
        /// The owner the record names.
        owner: EntityOwner,
    },

    #[error("the {dimension:?}-cell at {dart:?} is owned by both {held:?} and {claimed:?}")]
    ConflictingOwnership {
        /// Dimension of the contested cell.
        dimension: Dim,
        /// A dart both records reach.
        dart: Dart,
        /// The owner recorded first.
        held: EntityOwner,
        /// The owner recorded second.
        claimed: EntityOwner,
    },
}
