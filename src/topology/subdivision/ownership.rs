use std::collections::HashMap;

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

/// One raw cell's ownership, stored against a representative dart.
///
/// The representative locates the orbit; it is not the identity of anything.
/// A refinement that destroys this dart re-anchors the record on a surviving
/// dart of the same orbit rather than allocating a new record.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct OrbitOwnership {
    /// Dimension of the raw cell this record labels.
    pub dimension: Dim,
    /// A dart of the labelled orbit.
    pub representative: Dart,
    /// The logical entity whose interior contains the cell.
    pub owner: EntityOwner,
}

/// The authoritative classification of one map's raw cells.
///
/// This holds one record per labelled orbit and nothing else. The set of darts
/// an entity covers is not stored: it is recovered by walking the map, which is
/// what keeps the classification valid across a refinement that the entity did
/// not ask for.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Subdivision {
    records: Vec<OrbitOwnership>,
}

impl Subdivision {
    /// Creates a classification with no labelled orbits.
    pub fn new() -> Self {
        Self::default()
    }

    /// Labels the raw `dimension`-cell containing `representative`.
    pub fn own(&mut self, dimension: Dim, representative: Dart, owner: EntityOwner) {
        self.records.push(OrbitOwnership {
            dimension,
            representative,
            owner,
        });
    }

    /// Returns every stored record, in the order they were added.
    pub fn records(&self) -> &[OrbitOwnership] {
        &self.records
    }

    /// Returns the records naming `owner`, in the order they were added.
    pub fn records_of(&self, owner: EntityOwner) -> impl Iterator<Item = &OrbitOwnership> {
        self.records.iter().filter(move |r| r.owner == owner)
    }

    /// Expands every record into the dart lookup the walkers read.
    ///
    /// This is the derived half of the classification: it is rebuilt from the
    /// records whenever the map changes, and is never serialized.
    pub fn index(&self, map: &GMap) -> Result<OwnershipIndex, SubdivisionError> {
        OwnershipIndex::build(map, self)
    }

    /// Rewrites every record's anchor through `map`.
    ///
    /// A record names an orbit, not a dart, so renumbering the map moves the
    /// anchor and changes nothing else about what the record says.
    pub(crate) fn map_darts(&mut self, map: impl Fn(Dart) -> Dart) {
        for record in &mut self.records {
            record.representative = map(record.representative);
        }
    }

    /// Adds the records of `source` whose anchors were copied, remapped.
    ///
    /// A record whose anchor did not come across is dropped: the cell it named
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
            } = *record;

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
