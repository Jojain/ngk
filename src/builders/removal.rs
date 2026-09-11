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

use crate::geometry::{Axis2, LINEAR_TOLERANCE, Surface, SurfacePeriodicity};
use crate::topology::attributes::{FaceAttr, LoopDefinition, LoopKind, ProfileAttr, ShellRoot};
use crate::topology::gmap::{Cell1, Cell2, Dim, GMap};
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
    /// Removing the edge would leave one loop spanning a closed direction.
    ///
    /// The face would be bounded by a loop on one side and by a parametric
    /// degeneracy on the other — a spherical cap. That loop is neither outer nor
    /// inner, and no face can say so yet.
    #[error("removing the edge at dart {dart:?} would leave a lone wrapping loop")]
    WouldLeaveWrappingLoop { dart: Dart },
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
    /// A 1-removal removed a seam, leaving the face a ring.
    ///
    /// The edge was walked twice by one loop, and dropping it let that loop fall
    /// into the two the face really has — each spanning a whole period of a
    /// closed axis. Nothing is consumed: one identity is kept and one is added,
    /// because the face now genuinely has two boundaries where it had one.
    Ring {
        /// The face that became a ring.
        face: FaceKey,
        /// Boundary loop identity kept by one of the two wrapping loops.
        survivor_loop: ProfileKey,
        /// Identity created for the other, split from `survivor_loop`.
        added_loop: ProfileKey,
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
    edit: &mut TopologyEdit<'_, P>,
    dart: Dart,
    dim: Dim,
) -> Result<CellRemoval, CellRemovalError> {
    let preflight = Preflight::resolve(edit.map(), dart, dim)?;
    let Preflight {
        cell,
        cell_set,
        pairs,
        seeds,
        plan,
    } = preflight;

    drop_removed_cell_attribute(edit, &cell_set, dart, dim)?;
    reseed_attributes(edit, &cell_set, &seeds);
    drop_pcurves(edit, &cell_set);

    for &d in &cell {
        for dimension in [Dim::Zero, Dim::One, Dim::Two, Dim::Three] {
            if !edit.is_free(d, dimension) {
                edit.unlink(dimension, d)?;
            }
        }
    }
    for (first, second) in pairs {
        edit.link(dim, first, second)?;
    }

    // Identity bookkeeping runs on the rewired map but before dart ids are
    // compacted, so every dart the plan captured is still addressable.
    let merged = plan.apply(edit);

    let mut removed = cell;
    removed.sort_by_key(|d| d.id());
    let remap =
        edit.remove_isolated_darts(removed.iter().copied().map(IsolatedDart::new).collect());
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
    /// Removing a seam left the face bounded by two wrapping loops.
    Ring {
        face: FaceKey,
        /// The loop identity the seam was walked on, kept by `seeds[0]`.
        survivor_loop: ProfileKey,
        /// The closed axis each of the two loops spans.
        axis: Axis2,
        /// The seed kept by `survivor_loop`, then the one needing its own.
        seeds: [Dart; 2],
        /// The face's other loops, which the removal does not touch.
        untouched: Vec<LoopDefinition>,
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
        let boundaries = attr.darts().collect::<Vec<_>>();

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
                .filter(|(_, attr)| attr.darts().any(|seed| cell_set.contains(&seed)))
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
        if components.len() != 1 {
            // Two components is not always undecidable. Removing a seam leaves
            // exactly two, and neither bounds the face from outside: each runs a
            // whole period and bounds the axis across it. That is a ring, and it
            // is the one shape the combinatorics *can* answer for here.
            if let Some(ring) =
                Self::ring(g, face, attr, reference, &components, &boundaries, absorbed)?
            {
                return Ok(ring);
            }
            return Err(CellRemovalError::LoopWouldSplit {
                dart,
                loops: components.len(),
            });
        }
        // One surviving component that still spans a whole period belongs to a
        // face closed on its far side by a degeneracy rather than by a loop — a
        // spherical cap. Its loop is neither outer nor inner, and calling it
        // either would put a winding test on a loop that has no inside, so the
        // removal declines until a face can say it is closed by a pole.
        if wrapping_axis(attr, &components[0]).is_some() {
            return Err(CellRemovalError::WouldLeaveWrappingLoop { dart });
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

    /// Reads a two-way boundary split as a ring, when that is what it is.
    ///
    /// A seam is not part of the shape: it is where the parameterization was cut
    /// open so a closed direction could be walked as a loop. Remove it and the
    /// one loop falls into the two the face really has, each spanning a whole
    /// period of the closed axis. Neither is outer or inner — the question does
    /// not arise, because a period-spanning loop bounds the axis across it and
    /// carries no inside.
    ///
    /// Returns `None` when the split is anything else, leaving the caller to
    /// refuse it: two components that do not each wrap really are undecidable.
    fn ring<P: Payload>(
        g: &GMap<P>,
        face: FaceKey,
        attr: &FaceAttr<P::F>,
        reference: Dart,
        components: &[HashSet<Dart>],
        boundaries: &[Dart],
        absorbed: &[usize],
    ) -> Result<Option<Self>, CellRemovalError> {
        let [first, second] = components else {
            return Ok(None);
        };
        // A rejoin that also absorbs another of the face's loops is a different
        // edit; a seam removal touches the one loop it is walked on twice.
        if !absorbed.is_empty() {
            return Ok(None);
        }
        let Some(axis) = wrapping_axis(attr, first) else {
            return Ok(None);
        };
        if wrapping_axis(attr, second) != Some(axis) {
            return Ok(None);
        }

        let seed = |component: &HashSet<Dart>| {
            component
                .iter()
                .copied()
                .filter(|&d| {
                    g.cell_orientation_from_seed(reference, d, Dim::Two) == Some(Orientation::Same)
                })
                .min()
        };
        let (Some(kept), Some(added)) = (seed(first), seed(second)) else {
            return Ok(None);
        };
        let survivor_loop =
            g.profile_key(reference)
                .ok_or(CellRemovalError::UnregisteredIncidence {
                    dart: reference,
                    dim: Dim::One,
                })?;
        Ok(Some(MergePlan::Ring {
            face,
            survivor_loop,
            axis,
            seeds: [kept, added],
            untouched: boundaries
                .iter()
                .filter(|&&seed| seed != reference)
                .filter_map(|&seed| Some(LoopDefinition::from_kind(seed, attr.kind_of(seed)?)))
                .collect(),
        }))
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
        let transferred = attr
            .darts()
            .filter(|&seed| g.profile_key(seed) != Some(consumed_loop))
            .collect::<Vec<_>>();

        let survivor_loop_is_inner = survivor_attr
            .inner()
            .any(|seed| g.profile_key(seed) == Some(survivor_loop));
        let consumed_loop_is_outer = g.profile_key(attr.outer_unchecked()) == Some(consumed_loop);
        let loop_disappears = [survivor_loop, consumed_loop].into_iter().all(|profile| {
            let seed = g.profile_attr_unchecked(profile).dart;
            g.orbit(seed, vec![0, 1])
                .all(|loop_dart| cell_set.contains(&loop_dart))
        });
        if survivor_loop_is_inner && consumed_loop_is_outer && loop_disappears {
            let remaining_inner = survivor_attr
                .inner()
                .filter(|&seed| g.profile_key(seed) != Some(survivor_loop))
                .collect();
            return Ok(MergePlan::FilledBoundaryFaces {
                survivor,
                consumed,
                survivor_loop,
                consumed_loop,
                survivor_outer: survivor_attr.outer_unchecked(),
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
    fn apply<P: Payload>(self, edit: &mut TopologyEdit<'_, P>) -> MergedCell {
        match self {
            MergePlan::Edges { survivor, consumed } => {
                edit.merge_edges_into(survivor, consumed);
                MergedCell::Edges { survivor, consumed }
            }
            MergePlan::Loops {
                face,
                survivor_loop,
                consumed_loop,
                seed,
                boundaries,
            } => {
                edit.profile_attr_mut_unchecked(survivor_loop).dart = seed;
                if let Some(consumed) = consumed_loop {
                    // The absorbed identity is dropped at commit, but until then
                    // it must still name a dart the map holds.
                    edit.profile_attr_mut_unchecked(consumed).dart = seed;
                    edit.merge_profiles_into(survivor_loop, consumed);
                }
                let attr = edit.face_attr_mut_unchecked(face);
                attr.set_outer(boundaries[0]);
                attr.set_inner(boundaries[1..].to_vec());
                MergedCell::Loops {
                    face,
                    survivor_loop,
                    consumed_loop,
                }
            }
            MergePlan::Ring {
                face,
                survivor_loop,
                axis,
                seeds: [kept, added],
                untouched,
            } => {
                edit.profile_attr_mut_unchecked(survivor_loop).dart = kept;
                // The second loop is not a new boundary, it is the half of the
                // old one the seam was hiding, so it descends from that identity.
                let added_loop = edit.add_profile_split_from(
                    survivor_loop,
                    ProfileAttr::new(added, P::Profile::default()),
                );
                let mut loops = vec![
                    LoopDefinition::from_kind(kept, LoopKind::Wrapping { axis }),
                    LoopDefinition::from_kind(added, LoopKind::Wrapping { axis }),
                ];
                loops.extend(untouched);
                edit.face_attr_mut_unchecked(face).loops = loops;
                MergedCell::Ring {
                    face,
                    survivor_loop,
                    added_loop,
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
                    edit.remove_profile(key);
                }
                let attr = edit.face_attr_mut_unchecked(face);
                attr.set_outer(boundaries[0]);
                attr.set_inner(boundaries[1..].to_vec());
                for alias in face_aliases.into_iter().filter(|key| *key != face) {
                    let attr = edit.face_attr_mut_unchecked(alias);
                    attr.set_outer(boundaries[0]);
                    attr.clear_inner();
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
                let survivor_reference = edit.face_attr_unchecked(survivor).outer_unchecked();
                let consumed_reference = edit.face_attr_unchecked(consumed).outer_unchecked();
                let orientation = edit
                    .map()
                    .cell_orientation_from_seed(survivor_reference, consumed_reference, Dim::Two)
                    .unwrap_or(Orientation::Same);

                let moved = transferred
                    .into_iter()
                    .map(|seed| match orientation {
                        Orientation::Same => seed,
                        Orientation::Reversed => edit.alpha(Dim::Zero, seed),
                    })
                    .collect::<Vec<_>>();
                edit.face_attr_mut_unchecked(survivor).extend_inner(moved);

                edit.merge_profiles_into(survivor_loop, consumed_loop);
                edit.merge_faces_into(survivor, consumed);
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
                let consumed_reference = edit.face_attr_unchecked(consumed).outer_unchecked();
                let orientation = edit
                    .map()
                    .cell_orientation_from_seed(survivor_outer, consumed_reference, Dim::Two)
                    .unwrap_or(Orientation::Same);
                remaining_inner.extend(transferred.into_iter().map(|seed| match orientation {
                    Orientation::Same => seed,
                    Orientation::Reversed => edit.alpha(Dim::Zero, seed),
                }));

                let survivor_attr = edit.face_attr_mut_unchecked(survivor);
                survivor_attr.set_outer(survivor_outer);
                survivor_attr.set_inner(remaining_inner);
                let consumed_attr = edit.face_attr_mut_unchecked(consumed);
                consumed_attr.set_outer(survivor_outer);
                consumed_attr.clear_inner();
                consumed_attr.pcurves.clear();
                edit.remove_profile(survivor_loop);
                edit.remove_profile(consumed_loop);
                edit.merge_faces_into(survivor, consumed);
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
    Some(g.profile_key(attr.outer_unchecked()) == Some(profile))
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
) -> Vec<HashSet<Dart>> {
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
    let mut components = Vec::new();
    while let Some(&start) = unvisited.iter().next() {
        let mut component = HashSet::from([start]);
        let mut queue = vec![start];
        unvisited.remove(&start);
        while let Some(current) = queue.pop() {
            for along in [Dim::Zero, Dim::One] {
                let next = step(current, along);
                if unvisited.remove(&next) {
                    component.insert(next);
                    queue.push(next);
                }
            }
        }
        components.push(component);
    }
    components
}

/// The support's periods, in parameter order.
fn periods_of(surface: &Surface) -> [Option<f64>; 2] {
    match surface.periodicity() {
        SurfacePeriodicity::None => [None, None],
        SurfacePeriodicity::UPeriodic(period) => [Some(period), None],
        SurfacePeriodicity::VPeriodic(period) => [None, Some(period)],
        SurfacePeriodicity::UVPeriodic(u, v) => [Some(u), Some(v)],
    }
}

/// The closed axis a boundary component travels one whole period of, if any.
///
/// Only one dart per edge occurrence carries a pcurve, so summing over the
/// component's stored pcurves counts each exactly once; travel is signed, and
/// addition commutes, so the component need not be walked in order.
fn wrapping_axis<D>(attr: &FaceAttr<D>, component: &HashSet<Dart>) -> Option<Axis2> {
    let periods = periods_of(&attr.surface);
    [Axis2::U, Axis2::V].into_iter().find(|axis| {
        periods[axis.index()].is_some_and(|period| {
            let travel = component
                .iter()
                .filter_map(|dart| attr.pcurves.get(dart))
                .map(|pcurve| axis.of(pcurve.point_at(1.0)) - axis.of(pcurve.point_at(0.0)))
                .sum::<f64>();
            (travel.abs() - period).abs() <= LINEAR_TOLERANCE
        })
    })
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
    edit: &mut TopologyEdit<'_, P>,
    cell: &HashSet<Dart>,
    dart: Dart,
    dim: Dim,
) -> Result<(), CellRemovalError> {
    let dropped = match dim {
        Dim::Zero => {
            let keys = edit
                .map()
                .iter_vertices()
                .filter(|(_, attr)| cell.contains(&attr.dart))
                .map(|(key, _)| key)
                .collect::<Vec<_>>();
            keys.iter()
                .filter(|&&key| edit.remove_vertex(key).is_some())
                .count()
        }
        _ => {
            let keys = edit
                .map()
                .iter_edges()
                .filter(|(_, attr)| cell.contains(&attr.dart))
                .map(|(key, _)| key)
                .collect::<Vec<_>>();
            keys.iter()
                .filter(|&&key| edit.remove_edge(key).is_some())
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
    edit: &mut TopologyEdit<'_, P>,
    cell: &HashSet<Dart>,
    seeds: &HashMap<Dart, Option<Dart>>,
) {
    let replaced = |dart: Dart| match seeds.get(&dart) {
        Some(Some(seed)) => *seed,
        _ => dart,
    };

    let vertices = reseeded(
        edit.map()
            .iter_vertices()
            .map(|(key, attr)| (key, attr.dart)),
        seeds,
    );
    for (key, dart) in vertices {
        match dart {
            Some(dart) => edit.vertex_attr_mut_unchecked(key).dart = dart,
            None => {
                edit.remove_vertex(key);
            }
        }
    }

    let edges = reseeded(
        edit.map().iter_edges().map(|(key, attr)| (key, attr.dart)),
        seeds,
    );
    for (key, dart) in edges {
        match dart {
            Some(dart) => edit.edge_attr_mut_unchecked(key).dart = dart,
            None => {
                edit.remove_edge(key);
            }
        }
    }

    let profiles = reseeded(
        edit.map()
            .iter_profiles()
            .map(|(key, attr)| (key, attr.dart)),
        seeds,
    );
    for (key, dart) in profiles
        .into_iter()
        .filter_map(|(key, dart)| Some((key, dart?)))
    {
        edit.profile_attr_mut_unchecked(key).dart = dart;
    }

    // A shell keeps every dart the removal does not delete, so a seed that has
    // no Def. 59 replacement can still be re-rooted anywhere in the same shell.
    let sheets = reseeded(
        edit.map()
            .iter_sheets()
            .filter_map(|(key, attr)| Some((key, attr.dart()?))),
        seeds,
    );
    for (key, dart) in sheets {
        let dart = dart.or_else(|| {
            let root = edit.map().sheet_attr_unchecked(key).dart()?;
            shell_fallback(edit.map(), cell, root)
        });
        if let Some(dart) = dart {
            edit.sheet_attr_mut_unchecked(key).root = ShellRoot::Dart(dart);
        }
    }

    let faces = edit
        .map()
        .iter_faces()
        .filter(|(_, attr)| attr.darts().any(|dart| seeds.contains_key(&dart)))
        .map(|(key, _)| key)
        .collect::<Vec<_>>();
    for key in faces {
        let attr = edit.face_attr_mut_unchecked(key);
        attr.map_darts(|dart| match seeds.get(&dart) {
            Some(Some(seed)) => *seed,
            _ => dart,
        });
    }

    let solids = edit
        .map()
        .iter_solids()
        .filter(|(_, attr)| attr.shell_darts().any(|dart| seeds.contains_key(&dart)))
        .map(|(key, _)| key)
        .collect::<Vec<_>>();
    for key in solids {
        let attr = edit.solid_attr_mut_unchecked(key);
        attr.map_shell_darts(|dart| replaced(dart));
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
fn drop_pcurves<P: Payload>(edit: &mut TopologyEdit<'_, P>, cell: &HashSet<Dart>) {
    let faces = edit
        .map()
        .iter_faces()
        .filter(|(_, attr)| attr.pcurves.keys().any(|dart| cell.contains(dart)))
        .map(|(key, _)| key)
        .collect::<Vec<_>>();
    for key in faces {
        edit.face_attr_mut_unchecked(key)
            .pcurves
            .retain(|dart, _| !cell.contains(dart));
    }
}
