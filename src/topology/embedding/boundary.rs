use std::collections::{HashSet, VecDeque};

use thiserror::Error;

use crate::topology::gmap::{Dart, Dim, GMap};
use crate::topology::shape_keys::EdgeKey;

use super::cells::{EntityOwner, EmbeddingIndex};
use super::region::LogicalRegion;
use super::walk::turn;

/// One closed oriented boundary cycle of a logical face.
///
/// Each dart names one oriented edge the face runs along, read from the vertex
/// the traversal leaves. Cuts interior to the face never appear: the walk turns
/// across their paired dart instead of emitting them, which is what makes
/// a bridged annulus come back as the two cycles it really has.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BoundaryCycle {
    darts: Vec<Dart>,
}

impl BoundaryCycle {
    /// Returns the cycle's darts in traversal order.
    pub fn darts(&self) -> &[Dart] {
        &self.darts
    }

    /// Returns the number of darts in the cycle.
    pub fn len(&self) -> usize {
        self.darts.len()
    }

    /// Reports whether the cycle holds no dart.
    pub fn is_empty(&self) -> bool {
        self.darts.is_empty()
    }

    /// Returns the logical edge each dart of the cycle runs along, in order.
    ///
    /// One edge per dart. A logical edge occupies exactly one raw cell, so a
    /// dart names a whole edge rather than a piece of one, and an edge the
    /// cycle runs twice appears twice — which is how a seam reads.
    pub fn edge_keys(&self, index: &EmbeddingIndex) -> Result<Vec<EdgeKey>, BoundaryError> {
        self.darts
            .iter()
            .map(|&dart| match index.owner(Dim::One, dart) {
                Some(EntityOwner::Edge(edge)) => Ok(edge),
                other => Err(BoundaryError::BoundaryNotAnEdge { dart, owner: other }),
            })
            .collect()
    }
}

/// One connected boundary component of a logical solid.
#[derive(Debug, Clone)]
pub struct BoundaryShell {
    darts: Vec<Dart>,
}

impl BoundaryShell {
    /// Returns the shell's darts in the order the walk first reached them.
    pub fn darts(&self) -> &[Dart] {
        &self.darts
    }

    /// Counts the 2-cells of the shell.
    pub fn face_count(&self, map: &GMap) -> usize {
        self.components(|dart| vec![map.alpha(Dim::Zero, dart), map.alpha(Dim::One, dart)])
    }

    /// Counts the 1-cells of the shell.
    pub fn edge_count(&self, map: &GMap, index: &EmbeddingIndex) -> usize {
        self.components(|dart| {
            let mut out = vec![map.alpha(Dim::Zero, dart)];
            out.extend(turn(map, index, Dim::Two, dart));
            out
        })
    }

    /// Counts the 0-cells of the shell.
    pub fn vertex_count(&self, map: &GMap, index: &EmbeddingIndex) -> usize {
        self.components(|dart| {
            let mut out = vec![map.alpha(Dim::One, dart)];
            out.extend(turn(map, index, Dim::Two, dart));
            out
        })
    }

    /// Returns `V - E + F` over the shell's raw cells.
    ///
    /// This is a property of the scaffold, not of the logical entities on it: a
    /// sphere shell answers 2 and a torus shell answers 0 however few logical
    /// faces happen to cover them.
    pub fn euler_characteristic(&self, map: &GMap, index: &EmbeddingIndex) -> i64 {
        self.vertex_count(map, index) as i64 - self.edge_count(map, index) as i64
            + self.face_count(map) as i64
    }

    /// Counts the connected components of the shell's darts under `neighbours`.
    fn components(&self, neighbours: impl Fn(Dart) -> Vec<Dart>) -> usize {
        let members: HashSet<Dart> = self.darts.iter().copied().collect();
        let mut visited = HashSet::new();
        let mut count = 0;

        for &start in &self.darts {
            if visited.contains(&start) {
                continue;
            }
            count += 1;
            let mut queue = VecDeque::from([start]);
            while let Some(dart) = queue.pop_front() {
                if !visited.insert(dart) {
                    continue;
                }
                for neighbour in neighbours(dart) {
                    if members.contains(&neighbour) && !visited.contains(&neighbour) {
                        queue.push_back(neighbour);
                    }
                }
            }
        }
        count
    }
}

/// Extracts the oriented boundary cycles of a logical face.
///
/// Only darts that read the face the way its anchor does seed a cycle, so
/// every loop of one face comes back wound the same way round, whichever order
/// the region walk happened to reach them in.
pub fn boundary_cycles(
    map: &GMap,
    index: &EmbeddingIndex,
    region: &LogicalRegion,
) -> Result<Vec<BoundaryCycle>, BoundaryError> {
    if region.dimension() != Dim::Two {
        return Err(BoundaryError::WrongDimension {
            expected: Dim::Two,
            found: region.dimension(),
        });
    }

    let owner = region.owner();
    let frontier: Vec<Dart> = region
        .frontier(index)
        .into_iter()
        .filter(|&dart| region.is_aligned(dart))
        .collect();
    let mut consumed = HashSet::new();
    let mut cycles = Vec::new();

    for &start in &frontier {
        if consumed.contains(&start) {
            continue;
        }
        let mut darts = Vec::new();
        let mut current = start;
        loop {
            consumed.insert(current);
            darts.push(current);
            current = next_on_cycle(map, index, owner, current)?;
            if current == start {
                break;
            }
            if darts.len() > frontier.len() {
                return Err(BoundaryError::CycleDoesNotClose { start, owner });
            }
        }
        cycles.push(BoundaryCycle { darts });
    }

    Ok(cycles)
}

/// Extracts the connected boundary components of a logical solid.
pub fn boundary_shells(
    map: &GMap,
    index: &EmbeddingIndex,
    region: &LogicalRegion,
) -> Result<Vec<BoundaryShell>, BoundaryError> {
    if region.dimension() != Dim::Three {
        return Err(BoundaryError::WrongDimension {
            expected: Dim::Three,
            found: region.dimension(),
        });
    }

    let frontier = region.frontier(index);
    let members: HashSet<Dart> = frontier.iter().copied().collect();
    let mut visited = HashSet::new();
    let mut shells = Vec::new();

    for &start in &frontier {
        if visited.contains(&start) {
            continue;
        }
        let mut darts = Vec::new();
        let mut queue = VecDeque::from([start]);
        while let Some(dart) = queue.pop_front() {
            if !visited.insert(dart) {
                continue;
            }
            darts.push(dart);

            let across = turn(map, index, Dim::Two, dart).ok_or(BoundaryError::OpenShell {
                dart,
                owner: region.owner(),
            })?;
            for neighbour in [
                map.alpha(Dim::Zero, dart),
                map.alpha(Dim::One, dart),
                across,
            ] {
                if members.contains(&neighbour) {
                    queue.push_back(neighbour);
                }
            }
        }
        shells.push(BoundaryShell { darts });
    }

    Ok(shells)
}

/// Returns one dart per 0-cell bounding an edge.
///
/// A vertex the edge owns is a closure point interior to it and is not
/// returned, so a vertexless circle answers nothing and a segment answers two.
pub fn boundary_vertices(
    map: &GMap,
    index: &EmbeddingIndex,
    region: &LogicalRegion,
) -> Result<Vec<Dart>, BoundaryError> {
    if region.dimension() != Dim::One {
        return Err(BoundaryError::WrongDimension {
            expected: Dim::One,
            found: region.dimension(),
        });
    }

    let indices = map.orbit_indices(Dim::Zero);
    let mut seen = HashSet::new();
    let mut vertices = Vec::new();
    for dart in region.frontier(index) {
        if !seen.insert(dart) {
            continue;
        }
        for member in map.orbit(dart, indices.clone()) {
            seen.insert(member);
        }
        vertices.push(dart);
    }
    Ok(vertices)
}

/// Steps to the next dart on a logical face's boundary cycle.
///
/// Flipping to the far end of the current dart and turning once around
/// the vertex there gives the next one, unless the turn lands on a cut the face
/// owns, in which case the walk crosses to the cut's paired dart and
/// keeps turning.
fn next_on_cycle(
    map: &GMap,
    index: &EmbeddingIndex,
    owner: EntityOwner,
    dart: Dart,
) -> Result<Dart, BoundaryError> {
    let mut current = map.alpha(Dim::Zero, dart);
    let mut visited = HashSet::new();

    loop {
        current = map.alpha(Dim::One, current);
        if index.owner(Dim::One, current) != Some(owner) {
            return Ok(current);
        }
        if !visited.insert(current) {
            return Err(BoundaryError::CutWalkDoesNotTerminate {
                dart: current,
                owner,
            });
        }
        let across = turn(map, index, Dim::Two, current).ok_or(BoundaryError::DanglingCut {
            dart: current,
            owner,
        })?;
        if index.owner(Dim::Two, across) != Some(owner) {
            return Err(BoundaryError::CutSeparatesEntities {
                dart: across,
                owner,
                found: index.owner(Dim::Two, across),
            });
        }
        current = across;
    }
}

/// A region whose boundary cannot be walked.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum BoundaryError {
    #[error("this boundary walk needs a {expected:?}-dimensional region, not a {found:?} one")]
    WrongDimension {
        /// Dimension the walk requires.
        expected: Dim,
        /// Dimension of the region it was given.
        found: Dim,
    },

    #[error("{owner:?} owns the cut at {dart:?}, but nothing is paired across it")]
    DanglingCut {
        /// A dart of the cut.
        dart: Dart,
        /// The entity that owns it.
        owner: EntityOwner,
    },

    #[error("{owner:?} owns the cut at {dart:?}, which separates it from {found:?}")]
    CutSeparatesEntities {
        /// A dart across the cut.
        dart: Dart,
        /// The entity whose boundary was being walked.
        owner: EntityOwner,
        /// The entity found on the far side.
        found: Option<EntityOwner>,
    },

    #[error("turning across cuts at {dart:?} never leaves {owner:?}")]
    CutWalkDoesNotTerminate {
        /// The dart the walk returned to.
        dart: Dart,
        /// The entity whose boundary was being walked.
        owner: EntityOwner,
    },

    #[error("the boundary cycle of {owner:?} from {start:?} does not close")]
    CycleDoesNotClose {
        /// The dart the cycle started at.
        start: Dart,
        /// The entity whose boundary was being walked.
        owner: EntityOwner,
    },

    #[error("the boundary of {owner:?} is open at {dart:?}")]
    OpenShell {
        /// A dart on the open edge.
        dart: Dart,
        /// The entity whose boundary was being walked.
        owner: EntityOwner,
    },

    #[error(
        "the boundary dart at {dart:?} is owned by {owner:?}, which is not a logical edge"
    )]
    BoundaryNotAnEdge {
        /// The dart on the boundary.
        dart: Dart,
        /// What owns its 1-cell, if anything.
        owner: Option<EntityOwner>,
    },
}
