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

    /// Drops every entry naming `owner`, whatever dimension it labels.
    ///
    /// Removing a logical entity has to remove what it claimed, or the
    /// classification would go on describing cells as being inside something
    /// that no longer exists. Keyed by owner rather than by anchor on purpose:
    /// an entity may own cells of several dimensions, and a caller holding a
    /// deleted entity's attribute should not have to know which.
    pub(crate) fn disown(&mut self, owner: EntityOwner) {
        for shelf in &mut self.cells {
            shelf.retain(|_, held| *held != owner);
        }
    }

    /// Drops the entry anchored exactly at `representative`, if there is one.
    ///
    /// Keyed by anchor rather than by owner, because this unlabels one cell and
    /// leaves everything else the owner claims alone -- which is what promoting
    /// a single interior cell to a logical entity of its own needs.
    pub(crate) fn disown_at(&mut self, dimension: Dim, representative: Dart) {
        self.cells[dimension.index()].remove(&representative);
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

    /// Adds the entries of `source` whose anchor *and* owner were copied.
    ///
    /// Both ends of an entry have to be translated. The anchor is a dart of
    /// another map, and the owner is a key in another model's slotmaps — a
    /// `VertexKey(1v1)` there names a different vertex here, or none at all.
    /// Carrying an entry over with its original key would classify a cell as
    /// being inside whatever happened to land on that key, which is how a copy
    /// ends up with one entity claiming two unrelated cells.
    ///
    /// An entry missing either translation is dropped: the cell it named or the
    /// entity it named is not in this model, so nothing here is classified
    /// by it.
    pub(crate) fn extend_remapped(
        &mut self,
        source: &Subdivision,
        darts: &HashMap<Dart, Dart>,
        owners: &OwnerRemap<'_>,
    ) {
        for record in source.records() {
            let (Some(&representative), Some(owner)) = (
                darts.get(&record.representative),
                owners.translate(record.owner),
            ) else {
                continue;
            };
            self.own(record.dimension, representative, owner);
        }
    }
}

/// How one model's entity keys correspond to another's during a copy.
///
/// Held together rather than passed as four parallel maps: an ownership record
/// names exactly one entity, and which slotmap that is follows from the record
/// itself, not from the caller remembering to consult the matching map.
pub(crate) struct OwnerRemap<'a> {
    /// Source vertex key to destination vertex key.
    pub vertices: &'a HashMap<VertexKey, VertexKey>,
    /// Source edge key to destination edge key.
    pub edges: &'a HashMap<EdgeKey, EdgeKey>,
    /// Source face key to destination face key.
    pub faces: &'a HashMap<FaceKey, FaceKey>,
    /// Source solid key to destination solid key.
    pub solids: &'a HashMap<SolidKey, SolidKey>,
}

impl OwnerRemap<'_> {
    /// Returns `owner` as this model names it, or `None` if it was not copied.
    pub(crate) fn translate(&self, owner: EntityOwner) -> Option<EntityOwner> {
        match owner {
            EntityOwner::Vertex(key) => self.vertices.get(&key).copied().map(EntityOwner::Vertex),
            EntityOwner::Edge(key) => self.edges.get(&key).copied().map(EntityOwner::Edge),
            EntityOwner::Face(key) => self.faces.get(&key).copied().map(EntityOwner::Face),
            EntityOwner::Solid(key) => self.solids.get(&key).copied().map(EntityOwner::Solid),
        }
    }
}

/// Dart-to-owner lookup derived from a [`Subdivision`] and the map it labels.
#[derive(Debug, Clone)]
pub struct OwnershipIndex {
    owners: [HashMap<Dart, EntityOwner>; GMAP_INVOLUTION_COUNT],
}

impl OwnershipIndex {
    /// Expands the stored records across their orbits.
    ///
    /// Rejects a record whose representative is not a dart of `map`, a record
    /// whose owner is of lower dimension than the cell it labels, and two
    /// records that disagree about one orbit.
    pub fn build(map: &GMap, subdivision: &Subdivision) -> Result<Self, SubdivisionError> {
        Self::build_with_anchors(map, subdivision, std::iter::empty())
    }

    /// Expands the stored records *and* the cells entities are anchored at.
    ///
    /// The classification has two halves. An entity always contains the cell
    /// its own anchor sits in; that half is read from the entity stores through
    /// `anchors` and never written down, because a stored copy of it would be a
    /// second record of where an entity is that has to be re-anchored in step
    /// with the attribute every time an edit moves it -- and would silently
    /// claim a foreign cell the first time it was not. The stored half is
    /// everything else an entity contains: a closure point inside an edge, a
    /// seam inside a face, a buried corner inside a solid.
    ///
    /// Anchors are applied first so a stored record that contradicts one is
    /// reported as the conflict it is.
    pub fn build_with_anchors(
        map: &GMap,
        subdivision: &Subdivision,
        anchors: impl IntoIterator<Item = (Dim, Dart, EntityOwner)>,
    ) -> Result<Self, SubdivisionError> {
        let mut owners: [HashMap<Dart, EntityOwner>; GMAP_INVOLUTION_COUNT] =
            std::array::from_fn(|_| HashMap::new());

        let anchored = anchors
            .into_iter()
            .map(|(dimension, representative, owner)| OrbitOwnership {
                dimension,
                representative,
                owner,
            });
        for record in anchored.chain(subdivision.records()) {
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
