//! Cell removal — the inverse of cell splitting.
//!
//! This module implements the `i`-removal operation of Damiand & Lienhardt
//! (Defs. 58–59, Algs. 50–51): removing an `i`-cell `C` merges the at most two
//! `(i + 1)`-cells incident to it. A 0-removal therefore fuses the two edges
//! meeting at a vertex, and a 1-removal fuses the two faces sharing an edge.
//!
//! The operation is combinatorial plus identity bookkeeping. It rewrites
//! `alpha_i` for the darts linked to `C`, deletes the darts of `C`, reseeds
//! every attribute whose reference dart is deleted, declares the merged
//! identities so commit can reconcile them, and drops the parameter curves of
//! the deleted darts.
//!
//! It deliberately does **not** decide whether a cell should disappear, and it
//! does not build the merged geometry. Both belong to the caller;
//! [`crate::healing`] is the caller that supplies them.

use std::collections::{HashMap, HashSet};

use thiserror::Error;

use crate::topology::gmap::{Cell0, Cell1, Cell2, Dim, GMap};
use crate::topology::orientation::Orientation;
use crate::topology::shape_keys::{EdgeKey, FaceKey, ProfileKey};
use crate::topology::{Dart, IsolatedDart, Payload, TopologyEdit, TopologyEditError};

/// Failure raised while removing a cell from a staged map.
#[derive(Debug, Error)]
pub enum CellRemovalError {
    /// Only 0-cells and 1-cells can be removed by this operation.
    #[error("cell removal is implemented for dimensions 0 and 1, not {dim:?}")]
    UnsupportedDimension { dim: Dim },
    /// The cell fails the removability condition of Def. 58.
    #[error("the {dim:?}-cell at dart {dart:?} is not removable")]
    NotRemovable { dart: Dart, dim: Dim },
    /// The cell has no registered attribute, so its identity cannot be dropped.
    #[error("the {dim:?}-cell at dart {dart:?} has no registered attribute")]
    UnregisteredCell { dart: Dart, dim: Dim },
    /// A cell incident to the removed cell has no registered identity.
    #[error("a cell incident to the {dim:?}-cell at dart {dart:?} has no registered identity")]
    UnregisteredIncidence { dart: Dart, dim: Dim },
    /// Only one `(dim + 1)`-cell bounds the removed cell on both sides, and the
    /// reshaping form of the removal does not apply at this dimension.
    #[error("the {dim:?}-cell at dart {dart:?} has the same cell on both sides")]
    SameIncidentCell { dart: Dart, dim: Dim },
    /// Removing the edge would break one boundary loop into several.
    ///
    /// The face would gain a hole, or fall apart, and deciding which of the
    /// resulting loops bounds it from outside needs more than the combinatorics.
    #[error("removing the edge at dart {dart:?} would split its boundary into {loops} loops")]
    LoopWouldSplit { dart: Dart, loops: usize },
    /// Removing the cell would delete every dart of the map.
    #[error("removing the {dim:?}-cell at dart {dart:?} would empty the map")]
    WouldEmptyMap { dart: Dart, dim: Dim },
    /// A staged alpha edit was rejected.
    #[error(transparent)]
    Topology(#[from] TopologyEditError),
}

/// The pair of identities fused by a removal.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MergedCell {
    /// A 0-removal fused two edges.
    Edges {
        /// Identity that keeps describing the fused edge.
        survivor: EdgeKey,
        /// Identity consumed by the fusion.
        consumed: EdgeKey,
    },
    /// A 1-removal reshaped one face's boundary instead of fusing two faces.
    ///
    /// The edge bounded the same face on both sides, so removing it rejoins the
    /// boundary rather than merging identities. Nothing is consumed except the
    /// edge itself, and the loop it was traversed on keeps its identity.
    Loops {
        /// The face whose boundary was rejoined.
        face: FaceKey,
        /// Boundary loop identity that keeps describing the rejoined loop.
        survivor_loop: ProfileKey,
        /// A second loop identity the rejoin absorbed, when the edge separated
        /// two of the face's loops.
        consumed_loop: Option<ProfileKey>,
    },
    /// A 1-removal deleted the final edge of an inner boundary component.
    BoundaryRemoved {
        /// The face whose empty inner boundary disappeared.
        face: FaceKey,
        /// One identity that described the removed boundary.
        profile: ProfileKey,
    },
    /// A 1-removal fused two faces and their two boundary loops.
    Faces {
        /// Identity that keeps describing the fused face.
        survivor: FaceKey,
        /// Identity consumed by the fusion.
        consumed: FaceKey,
        /// Boundary loop identity that keeps describing the fused loop.
        survivor_loop: ProfileKey,
        /// Boundary loop identity consumed by the fusion.
        consumed_loop: ProfileKey,
        /// How the consumed face's default orientation relates to the
        /// survivor's, which is what re-keys the consumed face's parameter
        /// curves onto the fused face.
        orientation: Orientation,
    },
}

/// Outcome of one cell removal.
///
/// Dart removal compacts the map's dart ids, so every dart captured before the
/// removal must be translated through [`CellRemoval::remap`] before it is used
/// again.
#[derive(Debug, Clone)]
pub struct CellRemoval {
    /// Darts of the removed cell, in their pre-removal numbering.
    pub removed: Vec<Dart>,
    /// The two identities the removal fused.
    pub merged: MergedCell,
    remap: HashMap<Dart, Dart>,
}

impl CellRemoval {
    /// Translates a pre-removal dart to its post-removal identity.
    ///
    /// Returns `None` for a dart that belonged to the removed cell.
    pub fn remap(&self, dart: Dart) -> Option<Dart> {
        self.remap.get(&dart).copied()
    }
}

/// Def. 58: reports whether the `dim`-cell containing `dart` can be removed.
///
/// An `(n - 1)`-cell is always removable in an `n`-Gmap. A lower-dimensional
/// cell is removable when `alpha(i+1)` and `alpha(i+2)` commute on every one of
/// its darts, which is what bounds the number of incident `(i + 1)`-cells to
/// two.
pub fn is_removable<P: Payload>(g: &GMap<P>, dart: Dart, dim: Dim) -> bool {
    match dim {
        Dim::Three => false,
        Dim::Two => true,
        Dim::Zero | Dim::One => {
            let next = Dim::from_index(dim.index() + 1);
            let after = Dim::from_index(dim.index() + 2);
            g.orbit(dart, g.orbit_indices(dim))
                .all(|d| g.alpha(next, g.alpha(after, d)) == g.alpha(after, g.alpha(next, d)))
        }
    }
}

/// Removes the `dim`-cell containing `dart`, merging the two `(dim + 1)`-cells
/// that were incident to it.
///
/// The caller owns every domain decision around this operation: it must have
/// checked that the merge is meaningful, and it must write the merged geometry
/// afterwards. In particular the surviving edge of a 0-removal keeps whichever
/// curve it had, which no longer spans the fused edge, and the fused boundary
/// has no parameter curve until the caller supplies one.
pub fn remove_cell_staged<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    dart: Dart,
    dim: Dim,
) -> Result<CellRemoval, CellRemovalError> {
    let preflight = Preflight::resolve(g.map(), dart, dim)?;
    let Preflight {
        cell,
        cell_set,
        pairs,
        seeds,
        plan,
    } = preflight;

    drop_removed_cell_attribute(g, &cell_set, dart, dim)?;
    reseed_attributes(g, &cell_set, &seeds);
    drop_pcurves(g, &cell_set);

    for &d in &cell {
        for dimension in [Dim::Zero, Dim::One, Dim::Two, Dim::Three] {
            if !g.is_free(d, dimension) {
                g.unlink(dimension, d)?;
            }
        }
    }
    for (first, second) in pairs {
        g.link(dim, first, second)?;
    }

    // Identity bookkeeping runs on the rewired map but before dart ids are
    // compacted, so every dart the plan captured is still addressable.
    let merged = plan.apply(g);

    let mut removed = cell;
    removed.sort_by_key(|d| d.id());
    let remap = g.remove_isolated_darts(removed.iter().copied().map(IsolatedDart::new).collect());
    Ok(CellRemoval {
        removed,
        merged,
        remap,
    })
}

/// Reports whether [`remove_cell_staged`] would accept this cell.
///
/// Every rejection the removal can raise is decided before it mutates
/// anything, so a caller that must not disturb the map on refusal — a healing
/// pass choosing candidates — asks here first.
pub fn can_remove_cell<P: Payload>(
    g: &GMap<P>,
    dart: Dart,
    dim: Dim,
) -> Result<(), CellRemovalError> {
    Preflight::resolve(g, dart, dim).map(|_| ())
}

/// Everything the removal decides before it touches the map.
struct Preflight {
    cell: Vec<Dart>,
    cell_set: HashSet<Dart>,
    pairs: Vec<(Dart, Dart)>,
    seeds: HashMap<Dart, Option<Dart>>,
    plan: MergePlan,
}

impl Preflight {
    fn resolve<P: Payload>(g: &GMap<P>, dart: Dart, dim: Dim) -> Result<Self, CellRemovalError> {
        if !matches!(dim, Dim::Zero | Dim::One) {
            return Err(CellRemovalError::UnsupportedDimension { dim });
        }
        if !is_removable(g, dart, dim) {
            return Err(CellRemovalError::NotRemovable { dart, dim });
        }

        let cell = g.orbit(dart, g.orbit_indices(dim)).collect::<Vec<_>>();
        let cell_set = cell.iter().copied().collect::<HashSet<_>>();
        if cell_set.len() == g.dart_count() {
            return Err(CellRemovalError::WouldEmptyMap { dart, dim });
        }

        let pairs = removal_pairs(g, &cell, &cell_set, dim)
            .ok_or(CellRemovalError::NotRemovable { dart, dim })?;
        let seeds = replacement_seeds(g, &cell, &cell_set, dim);
        let plan = MergePlan::build(g, dart, dim, &cell, &cell_set, &pairs)?;
        Ok(Self {
            cell,
            cell_set,
            pairs,
            seeds,
            plan,
        })
    }
}

/// Removes the `dim`-cell containing `dart` in its own transaction.
///
/// This is the standalone form of [`remove_cell_staged`] and carries the same
/// caller obligations. Because it commits immediately, use it only when the
/// stale geometry it leaves behind does not matter — most callers want the
/// staged form inside a healing pass.
pub fn remove_cell<P: Payload>(
    g: &mut GMap<P>,
    dart: Dart,
    dim: Dim,
) -> Result<CellRemoval, CellRemovalError> {
    g.transaction(|edit| remove_cell_staged(edit, dart, dim))
}

/// Identity bookkeeping decided before the topology changes and applied after.
enum MergePlan {
    Edges {
        survivor: EdgeKey,
        consumed: EdgeKey,
    },
    Faces {
        survivor: FaceKey,
        consumed: FaceKey,
        survivor_loop: ProfileKey,
        consumed_loop: ProfileKey,
        /// The consumed face's loops that are not the fused one.
        transferred: Vec<Dart>,
    },
    FilledBoundaryFaces {
        survivor: FaceKey,
        consumed: FaceKey,
        survivor_loop: ProfileKey,
        consumed_loop: ProfileKey,
        survivor_outer: Dart,
        remaining_inner: Vec<Dart>,
        transferred: Vec<Dart>,
    },
    Loops {
        face: FaceKey,
        survivor_loop: ProfileKey,
        consumed_loop: Option<ProfileKey>,
        /// The seed the rejoined loop keeps, chosen for its orientation.
        seed: Dart,
        /// The face's complete loop list once the rejoin has happened.
        boundaries: Vec<Dart>,
    },
    BoundaryRemoved {
        face: FaceKey,
        profiles: Vec<ProfileKey>,
        face_aliases: Vec<FaceKey>,
        boundaries: Vec<Dart>,
    },
}

impl MergePlan {
    /// Resolves the identities the removal will fuse or rejoin.
    fn build<P: Payload>(
        g: &GMap<P>,
        dart: Dart,
        dim: Dim,
        cell: &[Dart],
        cell_set: &HashSet<Dart>,
        pairs: &[(Dart, Dart)],
    ) -> Result<Self, CellRemovalError> {
        match dim {
            Dim::Zero => {
                let (first, second) = incident_pair(g, dart, dim, |d| g.cell_key::<Cell1>(d))?;
                let (survivor, consumed) = ordered(first, second);
                Ok(MergePlan::Edges { survivor, consumed })
            }
            _ => match incident_keys(g, dart, dim, |d| g.cell_key::<Cell2>(d))?.as_slice() {
                [face] => Self::loops(g, dart, dim, cell, cell_set, pairs, *face),
                [first, second] => {
                    let (survivor, consumed) = match (
                        incident_loop_is_outer(g, cell, *first),
                        incident_loop_is_outer(g, cell, *second),
                    ) {
                        (Some(false), Some(true)) => (*first, *second),
                        (Some(true), Some(false)) => (*second, *first),
                        _ => ordered(*first, *second),
                    };
                    Self::faces(g, dart, dim, cell, cell_set, survivor, consumed)
                }
                _ => Err(CellRemovalError::NotRemovable { dart, dim }),
            },
        }
    }

    /// Collects the loop bookkeeping for an edge the same face bounds twice.
    ///
    /// Removing such an edge rejoins boundary rather than merging identities:
    /// a slit closes up, or an inner loop opens into the outer one. The
    /// rejoined loop must come out as a single component — a removal that
    /// leaves two would have to decide which of them bounds the face from
    /// outside, which the combinatorics alone cannot answer.
    fn loops<P: Payload>(
        g: &GMap<P>,
        dart: Dart,
        dim: Dim,
        cell: &[Dart],
        cell_set: &HashSet<Dart>,
        pairs: &[(Dart, Dart)],
        face: FaceKey,
    ) -> Result<Self, CellRemovalError> {
        let missing = || CellRemovalError::UnregisteredIncidence { dart, dim };
        let attr = g.face_attr(face).ok_or_else(missing)?;
        let boundaries = std::iter::once(attr.outer_loop)
            .chain(attr.inner_loops.iter().copied())
            .collect::<Vec<_>>();

        let touched = cell
            .iter()
            .filter_map(|&d| g.profile_key(d))
            .collect::<HashSet<_>>();
        let affected = boundaries
            .iter()
            .enumerate()
            .filter(|&(_, &seed)| {
                g.profile_key(seed)
                    .is_some_and(|key| touched.contains(&key))
            })
            .map(|(index, _)| index)
            .collect::<Vec<_>>();
        let [rejoined, absorbed @ ..] = affected.as_slice() else {
            return Err(missing());
        };

        let reference = boundaries[*rejoined];
        let surviving = affected
            .iter()
            .flat_map(|&index| g.orbit(boundaries[index], vec![0, 1]))
            .filter(|d| !cell_set.contains(d))
            .collect::<HashSet<_>>();
        if surviving.is_empty() && affected.len() == 1 && affected[0] != 0 {
            let profiles = g
                .iter_profiles()
                .filter(|(_, attr)| cell_set.contains(&attr.dart))
                .map(|(key, _)| key)
                .collect::<Vec<_>>();
            if profiles.is_empty() {
                return Err(missing());
            }
            let face_aliases = g
                .iter_faces()
                .filter(|(_, attr)| {
                    std::iter::once(attr.outer_loop)
                        .chain(attr.inner_loops.iter().copied())
                        .any(|seed| cell_set.contains(&seed))
                })
                .map(|(key, _)| key)
                .collect();
            let boundaries = boundaries
                .into_iter()
                .enumerate()
                .filter_map(|(index, seed)| (index != affected[0]).then_some(seed))
                .collect();
            return Ok(MergePlan::BoundaryRemoved {
                face,
                profiles,
                face_aliases,
                boundaries,
            });
        }
        let components = rejoined_components(g, &surviving, cell_set, dim, pairs);
        if components != 1 {
            return Err(CellRemovalError::LoopWouldSplit {
                dart,
                loops: components,
            });
        }

        // The seed carries the loop's traversal direction, so the replacement
        // has to sit in the same orientation class as the one it replaces.
        let seed = surviving
            .iter()
            .copied()
            .filter(|&d| {
                g.cell_orientation_from_seed(reference, d, Dim::Two) == Some(Orientation::Same)
            })
            .min()
            .ok_or_else(missing)?;

        let survivor_loop = g.profile_key(reference).ok_or_else(missing)?;
        let consumed_loop = absorbed
            .first()
            .and_then(|&index| g.profile_key(boundaries[index]))
            .filter(|key| *key != survivor_loop);
        let boundaries = boundaries
            .into_iter()
            .enumerate()
            .filter(|(index, _)| !absorbed.contains(index))
            .map(|(index, seed_dart)| if index == *rejoined { seed } else { seed_dart })
            .collect();

        Ok(MergePlan::Loops {
            face,
            survivor_loop,
            consumed_loop,
            seed,
            boundaries,
        })
    }

    /// Collects the loop bookkeeping for a face fusion.
    fn faces<P: Payload>(
        g: &GMap<P>,
        dart: Dart,
        dim: Dim,
        cell: &[Dart],
        cell_set: &HashSet<Dart>,
        survivor: FaceKey,
        consumed: FaceKey,
    ) -> Result<Self, CellRemovalError> {
        let loop_seed = |face: FaceKey| {
            cell.iter()
                .copied()
                .filter(|&d| g.cell_key::<Cell2>(d) == Some(face))
                .find_map(|d| g.profile_key(d).map(|profile| (d, profile)))
        };
        let (_, survivor_loop) =
            loop_seed(survivor).ok_or(CellRemovalError::UnregisteredIncidence { dart, dim })?;
        let (_, consumed_loop) =
            loop_seed(consumed).ok_or(CellRemovalError::UnregisteredIncidence { dart, dim })?;
        if survivor_loop == consumed_loop {
            return Err(CellRemovalError::SameIncidentCell { dart, dim });
        }

        let survivor_attr = g
            .face_attr(survivor)
            .ok_or(CellRemovalError::UnregisteredIncidence { dart, dim })?;
        let attr = g
            .face_attr(consumed)
            .ok_or(CellRemovalError::UnregisteredIncidence { dart, dim })?;
        let transferred = std::iter::once(attr.outer_loop)
            .chain(attr.inner_loops.iter().copied())
            .filter(|&seed| g.profile_key(seed) != Some(consumed_loop))
            .collect::<Vec<_>>();

        let survivor_loop_is_inner = survivor_attr
            .inner_loops
            .iter()
            .any(|&seed| g.profile_key(seed) == Some(survivor_loop));
        let consumed_loop_is_outer = g.profile_key(attr.outer_loop) == Some(consumed_loop);
        let loop_disappears = [survivor_loop, consumed_loop].into_iter().all(|profile| {
            let seed = g.profile_attr_unchecked(profile).dart;
            g.orbit(seed, vec![0, 1])
                .all(|loop_dart| cell_set.contains(&loop_dart))
        });
        if survivor_loop_is_inner && consumed_loop_is_outer && loop_disappears {
            let remaining_inner = survivor_attr
                .inner_loops
                .iter()
                .copied()
                .filter(|&seed| g.profile_key(seed) != Some(survivor_loop))
                .collect();
            return Ok(MergePlan::FilledBoundaryFaces {
                survivor,
                consumed,
                survivor_loop,
                consumed_loop,
                survivor_outer: survivor_attr.outer_loop,
                remaining_inner,
                transferred,
            });
        }

        Ok(MergePlan::Faces {
            survivor,
            consumed,
            survivor_loop,
            consumed_loop,
            transferred,
        })
    }

    /// Declares the merged identities and moves the consumed face's other loops.
    fn apply<P: Payload>(self, g: &mut TopologyEdit<'_, P>) -> MergedCell {
        match self {
            MergePlan::Edges { survivor, consumed } => {
                g.merge_edges_into(survivor, consumed);
                MergedCell::Edges { survivor, consumed }
            }
            MergePlan::Loops {
                face,
                survivor_loop,
                consumed_loop,
                seed,
                boundaries,
            } => {
                g.profile_attr_mut_unchecked(survivor_loop).dart = seed;
                if let Some(consumed) = consumed_loop {
                    // The absorbed identity is dropped at commit, but until then
                    // it must still name a dart the map holds.
                    g.profile_attr_mut_unchecked(consumed).dart = seed;
                    g.merge_profiles_into(survivor_loop, consumed);
                }
                let attr = g.face_attr_mut_unchecked(face);
                attr.outer_loop = boundaries[0];
                attr.inner_loops = boundaries[1..].to_vec();
                MergedCell::Loops {
                    face,
                    survivor_loop,
                    consumed_loop,
                }
            }
            MergePlan::BoundaryRemoved {
                face,
                profiles,
                face_aliases,
                boundaries,
            } => {
                let profile = profiles[0];
                for key in profiles {
                    g.remove_profile(key);
                }
                let attr = g.face_attr_mut_unchecked(face);
                attr.outer_loop = boundaries[0];
                attr.inner_loops = boundaries[1..].to_vec();
                for alias in face_aliases.into_iter().filter(|key| *key != face) {
                    let attr = g.face_attr_mut_unchecked(alias);
                    attr.outer_loop = boundaries[0];
                    attr.inner_loops.clear();
                    attr.pcurves.clear();
                }
                MergedCell::BoundaryRemoved { face, profile }
            }
            MergePlan::Faces {
                survivor,
                consumed,
                survivor_loop,
                consumed_loop,
                transferred,
            } => {
                // The fused-loop seeds belonged to the removed cell, so the
                // reseeded darts the two face attributes now carry are what
                // relates the two faces' default orientations.
                let survivor_reference = g.face_attr_unchecked(survivor).outer_loop;
                let consumed_reference = g.face_attr_unchecked(consumed).outer_loop;
                let orientation = g
                    .map()
                    .cell_orientation_from_seed(survivor_reference, consumed_reference, Dim::Two)
                    .unwrap_or(Orientation::Same);

                let moved = transferred
                    .into_iter()
                    .map(|seed| match orientation {
                        Orientation::Same => seed,
                        Orientation::Reversed => g.alpha(Dim::Zero, seed),
                    })
                    .collect::<Vec<_>>();
                g.face_attr_mut_unchecked(survivor)
                    .inner_loops
                    .extend(moved);

                g.merge_profiles_into(survivor_loop, consumed_loop);
                g.merge_faces_into(survivor, consumed);
                MergedCell::Faces {
                    survivor,
                    consumed,
                    survivor_loop,
                    consumed_loop,
                    orientation,
                }
            }
            MergePlan::FilledBoundaryFaces {
                survivor,
                consumed,
                survivor_loop,
                consumed_loop,
                survivor_outer,
                mut remaining_inner,
                transferred,
            } => {
                let consumed_reference = g.face_attr_unchecked(consumed).outer_loop;
                let orientation = g
                    .map()
                    .cell_orientation_from_seed(survivor_outer, consumed_reference, Dim::Two)
                    .unwrap_or(Orientation::Same);
                remaining_inner.extend(transferred.into_iter().map(|seed| match orientation {
                    Orientation::Same => seed,
                    Orientation::Reversed => g.alpha(Dim::Zero, seed),
                }));

                let survivor_attr = g.face_attr_mut_unchecked(survivor);
                survivor_attr.outer_loop = survivor_outer;
                survivor_attr.inner_loops = remaining_inner;
                let consumed_attr = g.face_attr_mut_unchecked(consumed);
                consumed_attr.outer_loop = survivor_outer;
                consumed_attr.inner_loops.clear();
                consumed_attr.pcurves.clear();
                g.remove_profile(survivor_loop);
                g.remove_profile(consumed_loop);
                g.merge_faces_into(survivor, consumed);
                MergedCell::Faces {
                    survivor,
                    consumed,
                    survivor_loop,
                    consumed_loop,
                    orientation,
                }
            }
        }
    }
}

/// Classifies the loop of `face` touched by `cell` as outer or inner.
fn incident_loop_is_outer<P: Payload>(g: &GMap<P>, cell: &[Dart], face: FaceKey) -> Option<bool> {
    let profile = cell
        .iter()
        .copied()
        .filter(|&dart| g.cell_key::<Cell2>(dart) == Some(face))
        .find_map(|dart| g.profile_key(dart))?;
    let attr = g.face_attr(face)?;
    Some(g.profile_key(attr.outer_loop) == Some(profile))
}

/// Returns the two distinct `(dim + 1)`-cell identities incident to the cell.
fn incident_pair<P, K, F>(
    g: &GMap<P>,
    dart: Dart,
    dim: Dim,
    key_of: F,
) -> Result<(K, K), CellRemovalError>
where
    P: Payload,
    K: Copy + Ord,
    F: Fn(Dart) -> Option<K>,
{
    match incident_keys(g, dart, dim, key_of)?.as_slice() {
        [first, second] => Ok((*first, *second)),
        [_] => Err(CellRemovalError::SameIncidentCell { dart, dim }),
        _ => Err(CellRemovalError::NotRemovable { dart, dim }),
    }
}

/// Returns the distinct `(dim + 1)`-cell identities incident to the cell.
fn incident_keys<P, K, F>(
    g: &GMap<P>,
    dart: Dart,
    dim: Dim,
    key_of: F,
) -> Result<Vec<K>, CellRemovalError>
where
    P: Payload,
    K: Copy + Ord,
    F: Fn(Dart) -> Option<K>,
{
    let target = Dim::from_index(dim.index() + 1);
    let mut keys = Vec::new();
    for incident in g.incident_cells(dart, dim, target) {
        let key = key_of(incident).ok_or(CellRemovalError::UnregisteredIncidence { dart, dim })?;
        if !keys.contains(&key) {
            keys.push(key);
        }
    }
    Ok(keys)
}

/// Counts the boundary loops `surviving` would form once the cell is gone.
///
/// The rewiring is the only thing the removal changes, so the components can be
/// counted on the map as it stands by substituting the replacement links.
fn rejoined_components<P: Payload>(
    g: &GMap<P>,
    surviving: &HashSet<Dart>,
    cell: &HashSet<Dart>,
    dim: Dim,
    pairs: &[(Dart, Dart)],
) -> usize {
    let replacements = pairs
        .iter()
        .flat_map(|&(first, second)| [(first, second), (second, first)])
        .collect::<HashMap<_, _>>();
    let step = |d: Dart, along: Dim| {
        let linked = g.alpha(along, d);
        if along == dim && cell.contains(&linked) {
            replacements.get(&d).copied().unwrap_or(d)
        } else {
            linked
        }
    };

    let mut unvisited = surviving.clone();
    let mut components = 0;
    while let Some(&start) = unvisited.iter().next() {
        components += 1;
        let mut queue = vec![start];
        unvisited.remove(&start);
        while let Some(current) = queue.pop() {
            for along in [Dim::Zero, Dim::One] {
                let next = step(current, along);
                if unvisited.remove(&next) {
                    queue.push(next);
                }
            }
        }
    }
    components
}

/// Orders two identities so the survivor is deterministic across runs.
fn ordered<K: Ord>(first: K, second: K) -> (K, K) {
    if first <= second {
        (first, second)
    } else {
        (second, first)
    }
}

/// Follows the Def. 59 path `(alpha_i o alpha_(i+1))^k` from `dart` until it
/// leaves `cell`.
///
/// The same path both repairs `alpha_i` for the darts linked to the removed
/// cell and gives the replacement seed for an attribute whose reference dart is
/// about to disappear: it applies two involutions per step, so it preserves the
/// dart's orientation class.
fn removal_partner<P: Payload>(
    g: &GMap<P>,
    cell: &HashSet<Dart>,
    dim: Dim,
    dart: Dart,
) -> Option<Dart> {
    let next = Dim::from_index(dim.index() + 1);
    let mut current = g.alpha(dim, g.alpha(next, dart));
    for _ in 0..=cell.len() {
        if !cell.contains(&current) {
            return Some(current);
        }
        current = g.alpha(dim, g.alpha(next, current));
    }
    None
}

/// Def. 59: the `alpha_dim` pairs that replace the links broken by the removal.
///
/// Each surviving dart linked to the cell has exactly one preimage inside it,
/// so normalizing on dart id yields every unordered pair exactly once. A dart
/// whose path returns to itself simply becomes `dim`-free.
fn removal_pairs<P: Payload>(
    g: &GMap<P>,
    cell: &[Dart],
    cell_set: &HashSet<Dart>,
    dim: Dim,
) -> Option<Vec<(Dart, Dart)>> {
    let mut pairs = Vec::new();
    for &inner in cell {
        let linked = g.alpha(dim, inner);
        if cell_set.contains(&linked) {
            continue;
        }
        let partner = removal_partner(g, cell_set, dim, inner)?;
        if linked.id() < partner.id() {
            pairs.push((linked, partner));
        }
    }
    Some(pairs)
}

/// Replacement reference darts for every seed the removal would invalidate.
///
/// A dart maps to `None` when the Def. 59 path never leaves the removed cell.
/// That happens when the removal takes the last cell that bounded something —
/// the vertex where a slit's two edges met, once both are gone — and the
/// attribute seeded there has nothing left to describe.
fn replacement_seeds<P: Payload>(
    g: &GMap<P>,
    cell: &[Dart],
    cell_set: &HashSet<Dart>,
    dim: Dim,
) -> HashMap<Dart, Option<Dart>> {
    cell.iter()
        .map(|&d| (d, removal_partner(g, cell_set, dim, d)))
        .collect()
}

/// Drops every identity describing the cell that the removal deletes.
///
/// A cell can carry more than one identity part-way through an operation: a
/// fusion earlier in the same transaction leaves the consumed key in place
/// until commit reconciles it. All of them go together, and commit treats the
/// merge that named them as spent.
fn drop_removed_cell_attribute<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    cell: &HashSet<Dart>,
    dart: Dart,
    dim: Dim,
) -> Result<(), CellRemovalError> {
    let dropped = match dim {
        Dim::Zero => {
            let keys = g
                .map()
                .iter_vertices()
                .filter(|(_, attr)| cell.contains(&attr.dart))
                .map(|(key, _)| key)
                .collect::<Vec<_>>();
            keys.iter()
                .filter(|&&key| g.remove_vertex(key).is_some())
                .count()
        }
        _ => {
            let keys = g
                .map()
                .iter_edges()
                .filter(|(_, attr)| cell.contains(&attr.dart))
                .map(|(key, _)| key)
                .collect::<Vec<_>>();
            keys.iter()
                .filter(|&&key| g.remove_edge(key).is_some())
                .count()
        }
    };
    (dropped > 0)
        .then_some(())
        .ok_or(CellRemovalError::UnregisteredCell { dart, dim })
}

/// Repoints every attribute reference dart that the removal would delete.
///
/// Reference darts carry contextual orientation, so a replacement must stay in
/// the same cell *and* the same orientation class. The Def. 59 path provides
/// both. A vertex or edge with no replacement has lost its last dart and goes
/// with it; a loop or shell seed with none is left for [`MergePlan`], which is
/// the only caller that can produce one and already knows the answer.
fn reseed_attributes<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    cell: &HashSet<Dart>,
    seeds: &HashMap<Dart, Option<Dart>>,
) {
    let replace = |dart: &mut Dart| {
        if let Some(Some(seed)) = seeds.get(dart) {
            *dart = *seed;
        }
    };

    let vertices = reseeded(
        g.map().iter_vertices().map(|(key, attr)| (key, attr.dart)),
        seeds,
    );
    for (key, dart) in vertices {
        match dart {
            Some(dart) => g.vertex_attr_mut_unchecked(key).dart = dart,
            None => {
                g.remove_vertex(key);
            }
        }
    }

    let edges = reseeded(
        g.map().iter_edges().map(|(key, attr)| (key, attr.dart)),
        seeds,
    );
    for (key, dart) in edges {
        match dart {
            Some(dart) => g.edge_attr_mut_unchecked(key).dart = dart,
            None => {
                g.remove_edge(key);
            }
        }
    }

    let profiles = reseeded(
        g.map().iter_profiles().map(|(key, attr)| (key, attr.dart)),
        seeds,
    );
    for (key, dart) in profiles
        .into_iter()
        .filter_map(|(key, dart)| Some((key, dart?)))
    {
        g.profile_attr_mut_unchecked(key).dart = dart;
    }

    // A shell keeps every dart the removal does not delete, so a seed that has
    // no Def. 59 replacement can still be re-rooted anywhere in the same shell.
    let sheets = reseeded(
        g.map().iter_sheets().map(|(key, attr)| (key, attr.dart)),
        seeds,
    );
    for (key, dart) in sheets {
        let dart =
            dart.or_else(|| shell_fallback(g.map(), cell, g.map().sheet_attr_unchecked(key).dart));
        if let Some(dart) = dart {
            g.sheet_attr_mut_unchecked(key).dart = dart;
        }
    }

    let faces = g
        .map()
        .iter_faces()
        .filter(|(_, attr)| {
            std::iter::once(attr.outer_loop)
                .chain(attr.inner_loops.iter().copied())
                .any(|dart| seeds.contains_key(&dart))
        })
        .map(|(key, _)| key)
        .collect::<Vec<_>>();
    for key in faces {
        let attr = g.face_attr_mut_unchecked(key);
        replace(&mut attr.outer_loop);
        for dart in &mut attr.inner_loops {
            replace(dart);
        }
    }

    let solids = g
        .map()
        .iter_solids()
        .filter(|(_, attr)| {
            std::iter::once(attr.outer_shell)
                .chain(attr.inner_shells.iter().flatten().copied())
                .any(|dart| seeds.contains_key(&dart))
        })
        .map(|(key, _)| key)
        .collect::<Vec<_>>();
    for key in solids {
        let attr = g.solid_attr_mut_unchecked(key);
        replace(&mut attr.outer_shell);
        for dart in attr.inner_shells.iter_mut().flatten() {
            replace(dart);
        }
    }
}

/// Returns a surviving dart of the shell rooted at `dart`.
fn shell_fallback<P: Payload>(g: &GMap<P>, cell: &HashSet<Dart>, dart: Dart) -> Option<Dart> {
    g.orbit(dart, g.orbit_indices(Dim::Three))
        .find(|d| !cell.contains(d))
}

/// Collects the `(key, replacement)` pairs for single-dart attribute seeds.
fn reseeded<K>(
    attributes: impl Iterator<Item = (K, Dart)>,
    seeds: &HashMap<Dart, Option<Dart>>,
) -> Vec<(K, Option<Dart>)> {
    attributes
        .filter_map(|(key, dart)| seeds.get(&dart).map(|&seed| (key, seed)))
        .collect()
}

/// Drops parameter curves keyed by a dart the removal deletes.
///
/// A pcurve describes one boundary dart of one face. When that dart disappears
/// the entry has no meaning left, and the caller re-inserts the fused
/// boundary's pcurve after the removal.
fn drop_pcurves<P: Payload>(g: &mut TopologyEdit<'_, P>, cell: &HashSet<Dart>) {
    let faces = g
        .map()
        .iter_faces()
        .filter(|(_, attr)| attr.pcurves.keys().any(|dart| cell.contains(dart)))
        .map(|(key, _)| key)
        .collect::<Vec<_>>();
    for key in faces {
        g.face_attr_mut_unchecked(key)
            .pcurves
            .retain(|dart, _| !cell.contains(dart));
    }
}
