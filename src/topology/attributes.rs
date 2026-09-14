use std::collections::HashMap;

use serde::{Deserialize, Serialize};

use crate::geometry::dim2::trimmed::TrimmedCurve2;
use crate::geometry::{Axis2, Curve, DomainSide, Point3, Surface};
use crate::model::{Cell0, Cell2, Model};
use crate::topology::dart::Dart;
use crate::topology::edge::Edge;
use crate::topology::face::Face;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::EdgeKey;
use crate::topology::vertex::Vertex;

/// Stored data for a keyed vertex 0-cell.
#[derive(Clone, Serialize, Deserialize)]
pub struct VertexAttr<T> {
    /// Representative dart of the vertex orbit.
    pub dart: Dart,
    /// Geometric point attached to the vertex.
    pub point: Point3,
    /// User payload attached to the vertex.
    pub data: T,
}

impl<T> VertexAttr<T> {
    /// Creates a vertex attribute rooted at `dart`.
    pub fn new(dart: Dart, point: Point3, data: T) -> Self {
        Self { dart, point, data }
    }

    /// Returns a typed vertex view over this attribute in `model`.
    pub fn vertex<'a, P: Payload>(&self, model: &'a Model<P>) -> Vertex<'a, P> {
        let key = model
            .cell_key::<Cell0>(self.dart)
            .expect("VertexAttr must be registered to produce a Vertex view");
        Vertex::new(model, key)
    }
}

/// Stored data for a keyed edge 1-cell.
#[derive(Clone, Serialize, Deserialize)]
pub struct EdgeAttr<T> {
    /// Representative dart of the edge orbit.
    pub dart: Dart,
    /// Geometric curve attached to the edge.
    pub curve: Curve,
    /// User payload attached to the edge.
    pub data: T,
}

impl<T> EdgeAttr<T> {
    /// Creates an edge attribute rooted at `dart`.
    ///
    /// The caller's `dart` defines the edge's default orientation.
    pub fn new(dart: Dart, curve: Curve, data: T) -> Self {
        Self { dart, curve, data }
    }

    /// Returns a typed edge view over this attribute in `model`.
    pub fn edge<'a, P: Payload>(&self, model: &'a Model<P>, key: EdgeKey) -> Edge<'a, P> {
        Edge::new(model, key)
    }
}

/// Stored data and default orientation for a profile.
#[derive(Clone, Serialize, Deserialize)]
pub struct ProfileAttr<T> {
    /// Oriented dart used as the profile's default traversal root.
    pub dart: Dart,
    /// User payload attached to the profile.
    pub data: T,
}

impl<T> ProfileAttr<T> {
    /// Creates a profile attribute rooted at the given oriented dart.
    pub fn new(dart: Dart, data: T) -> Self {
        Self { dart, data }
    }
}

/// What one face loop bounds in its face's parameter domain.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum LoopKind {
    /// A loop that closes in parameter space and bounds the face from outside.
    Outer,
    /// A loop that closes in parameter space and cuts a hole in the face.
    Inner,
    /// One of a pair of loops that each span exactly one period of `axis`,
    /// together bounding the axis they are transverse to.
    ///
    /// Such a loop closes on the periodic quotient, not in parameter space: a
    /// cylinder wall's base is a horizontal line one period long, never a
    /// closed polygon. It carries no winding, and it does not need to: the face
    /// is the band between the pair, whichever way round they are traversed.
    Wrapping { axis: Axis2 },
    /// A lone loop spanning one period of `axis`, with the transverse
    /// direction closed on its far side by a parametric degeneracy on `side`.
    ///
    /// A spherical cap — what every sphere-plane cut produces. Unlike a
    /// [`Self::Wrapping`] pair there is no second loop to bound the other side,
    /// and travel direction cannot stand in for one: reversing the loop has to
    /// flip the face's normal, so if it also chose the side it would move the
    /// face to the opposite pole. Which degeneracy closes the face is therefore
    /// said outright.
    Capping { axis: Axis2, side: DomainSide },
}

impl LoopKind {
    /// Returns the axis this loop spans, when it spans one.
    ///
    /// Both periodic kinds run a whole period; they differ only in what closes
    /// the transverse direction on the far side.
    pub fn wrapped_axis(self) -> Option<Axis2> {
        match self {
            LoopKind::Wrapping { axis } | LoopKind::Capping { axis, .. } => Some(axis),
            LoopKind::Outer | LoopKind::Inner => None,
        }
    }

    /// Returns the domain end whose degeneracy closes the face, if one does.
    pub fn capped_side(self) -> Option<DomainSide> {
        match self {
            LoopKind::Capping { side, .. } => Some(side),
            LoopKind::Outer | LoopKind::Inner | LoopKind::Wrapping { .. } => None,
        }
    }
}

/// Stored definition of one loop on a face.
///
/// A definition contains only the data a face attribute can persist: the
/// oriented seed that locates a closed profile and its parameter-domain role.
/// [`crate::topology::face::Loop`] resolves it into the face-facing topology
/// view that offers traversal operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LoopDefinition {
    /// A loop that closes in parameter space and bounds the face from outside.
    Outer { seed: Dart },
    /// A loop that closes in parameter space and cuts a hole in the face.
    Inner { seed: Dart },
    /// One of a pair of loops each spanning one whole period of `axis`.
    Wrapping { seed: Dart, axis: Axis2 },
    /// A lone loop spanning one whole period of `axis`, the transverse
    /// direction closed on its far side by the degeneracy on `side`.
    Capping {
        seed: Dart,
        axis: Axis2,
        side: DomainSide,
    },
}

impl LoopDefinition {
    pub(crate) fn from_kind(seed: Dart, kind: LoopKind) -> Self {
        match kind {
            LoopKind::Outer => Self::outer(seed),
            LoopKind::Inner => Self::inner(seed),
            LoopKind::Wrapping { axis } => Self::wrapping(seed, axis),
            LoopKind::Capping { axis, side } => Self::capping(seed, axis, side),
        }
    }

    /// Defines a chart-closed exterior loop.
    pub fn outer(seed: Dart) -> Self {
        Self::Outer { seed }
    }

    /// Defines a chart-closed hole loop.
    pub fn inner(seed: Dart) -> Self {
        Self::Inner { seed }
    }

    /// Defines a loop spanning one whole period of `axis`.
    pub fn wrapping(seed: Dart, axis: Axis2) -> Self {
        Self::Wrapping { seed, axis }
    }

    /// Defines a lone period-spanning loop closed at `end` by a degeneracy.
    pub fn capping(seed: Dart, axis: Axis2, side: DomainSide) -> Self {
        Self::Capping { seed, axis, side }
    }

    /// Returns this definition's oriented traversal seed.
    pub fn seed(self) -> Dart {
        match self {
            Self::Outer { seed }
            | Self::Inner { seed }
            | Self::Wrapping { seed, .. }
            | Self::Capping { seed, .. } => seed,
        }
    }

    /// Returns this definition's parameter-domain classification.
    pub fn kind(self) -> LoopKind {
        match self {
            Self::Outer { .. } => LoopKind::Outer,
            Self::Inner { .. } => LoopKind::Inner,
            Self::Wrapping { axis, .. } => LoopKind::Wrapping { axis },
            Self::Capping { axis, side, .. } => LoopKind::Capping { axis, side },
        }
    }

    /// Replaces this definition's oriented traversal seed.
    pub fn set_seed(&mut self, seed: Dart) {
        match self {
            Self::Outer { seed: current }
            | Self::Inner { seed: current }
            | Self::Wrapping { seed: current, .. }
            | Self::Capping { seed: current, .. } => *current = seed,
        }
    }
}

/// What bounds a face, and where the face's one raw 2-cell is read from.
///
/// A face occupies exactly one raw 2-cell and therefore always has an anchor,
/// but it does not always have a *loop*: a whole sphere or a whole torus covers
/// a closed support, so every cell its 2-cell touches is embedded in it and its
/// boundary walk emits nothing.
///
/// One value rather than a loop list beside an anchor dart. A face bounded by
/// loops anchors at the first of them, so the anchor is derived and cannot go
/// stale; a face bounded by nothing stores the only dart it has. Healing a seam
/// away moves a face from the first case to the second, and this makes that
/// move name the dart it survives at instead of leaving one behind to rot.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub(crate) enum FaceBoundary {
    /// One definition per boundary, never empty. The face anchors at the first.
    Loops(Vec<LoopDefinition>),
    /// The face bounds nothing and anchors at this dart of its 2-cell.
    Closed(Dart),
}

impl FaceBoundary {
    /// Wraps a loop list, or records a boundaryless face anchored at `fallback`.
    fn from_loops(loops: Vec<LoopDefinition>, fallback: Dart) -> Self {
        match loops.is_empty() {
            true => Self::Closed(fallback),
            false => Self::Loops(loops),
        }
    }

    /// Returns the dart the face is anchored at.
    fn seed(&self) -> Dart {
        match self {
            Self::Loops(loops) => loops[0].seed(),
            Self::Closed(dart) => *dart,
        }
    }

    /// Returns the stored loop definitions; empty for a boundaryless face.
    fn loops(&self) -> &[LoopDefinition] {
        match self {
            Self::Loops(loops) => loops,
            Self::Closed(_) => &[],
        }
    }

    /// Opens the loop list for in-place editing.
    ///
    /// A boundaryless face edits an empty list — it has no loop yet — and its
    /// anchor is held aside while that happens. Whichever state the edit leaves
    /// is decided when the guard is dropped, so an edit that adds the first loop
    /// and one that removes the last both land on the right variant without the
    /// caller naming a dart it already stores.
    fn edit(&mut self) -> BoundaryEdit<'_> {
        let anchor = self.seed();
        let loops = match self {
            Self::Loops(loops) => std::mem::take(loops),
            Self::Closed(_) => Vec::new(),
        };
        BoundaryEdit {
            boundary: self,
            loops,
            anchor,
        }
    }
}

/// A face's loop list open for editing, settling back into a [`FaceBoundary`].
///
/// Held by [`FaceBoundary::edit`] and written back when it is dropped: a list
/// left empty becomes a boundaryless face anchored where it already was, and
/// one left with loops becomes a bounded face anchored at the first of them.
pub(crate) struct BoundaryEdit<'a> {
    boundary: &'a mut FaceBoundary,
    loops: Vec<LoopDefinition>,
    anchor: Dart,
}

impl std::ops::Deref for BoundaryEdit<'_> {
    type Target = Vec<LoopDefinition>;

    fn deref(&self) -> &Self::Target {
        &self.loops
    }
}

impl std::ops::DerefMut for BoundaryEdit<'_> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.loops
    }
}

impl Drop for BoundaryEdit<'_> {
    fn drop(&mut self) {
        *self.boundary = FaceBoundary::from_loops(std::mem::take(&mut self.loops), self.anchor);
    }
}

/// Stored data for a keyed domain face.
///
/// # Boundary orientation
///
/// Face loop definitions hold oriented boundary seeds, not arbitrary
/// representatives of their loop cells. Traversing a stored seed determines
/// the direction of the corresponding boundary. Choosing `alpha0(seed)`
/// traverses the same loop in the opposite direction and reverses the face.
///
/// Each pcurve is keyed by the directed boundary dart that uses it, and its
/// parameter direction must match that dart from start vertex to end vertex.
/// In the support surface's UV space, the outer loop and every inner loop must
/// have opposite winding:
///
/// - outer CCW and inner CW: the face follows the support-surface orientation;
/// - outer CW and inner CCW: the face opposes the support-surface orientation.
///
/// Whether a loop is an outer boundary or a hole is determined structurally by
/// its [`LoopDefinition`], not by winding alone.
///
/// Reversing a face is an atomic operation: replace every loop seed `d` with
/// `alpha0(d)`, and replace each pcurve entry `(d, curve)` with
/// `(alpha0(d), curve.reversed())`. Copy, merge, and topology-edit operations
/// must preserve these oriented darts rather than substitute canonical cell
/// representatives.
#[derive(Clone, Serialize, Deserialize)]
pub struct FaceAttr<T> {
    /// Geometric support surface of the face.
    pub surface: Surface,
    /// User payload attached to the face.
    pub data: T,
    /// What bounds this face, and where its one raw 2-cell is read from.
    pub(crate) boundary: FaceBoundary,
    /// Directed boundary pcurves keyed by their oriented boundary darts.
    pub pcurves: HashMap<Dart, TrimmedCurve2>,
}

impl<T> FaceAttr<T> {
    /// Creates a face attribute without boundary pcurves.
    ///
    /// The loop darts must follow the orientation contract documented on
    /// [`FaceAttr`].
    pub fn new(surface: Surface, data: T, outer_loop: Dart, inner_loops: Vec<Dart>) -> Self {
        Self {
            surface,
            data,
            boundary: FaceBoundary::Loops(
                std::iter::once(LoopDefinition::outer(outer_loop))
                    .chain(inner_loops.into_iter().map(LoopDefinition::inner))
                    .collect(),
            ),
            pcurves: HashMap::new(),
        }
    }

    /// Creates a face attribute with explicit boundary pcurves.
    ///
    /// The loop darts and pcurves must follow the orientation contract
    /// documented on [`FaceAttr`].
    pub fn with_pcurves(
        surface: Surface,
        data: T,
        outer_loop: Dart,
        inner_loops: Vec<Dart>,
        pcurves: HashMap<Dart, TrimmedCurve2>,
    ) -> Self {
        Self {
            surface,
            data,
            boundary: FaceBoundary::Loops(
                std::iter::once(LoopDefinition::outer(outer_loop))
                    .chain(inner_loops.into_iter().map(LoopDefinition::inner))
                    .collect(),
            ),
            pcurves,
        }
    }

    /// Creates a face attribute from explicit loop definitions.
    ///
    /// This constructor supports faces with no outer loop, such as a ring
    /// bounded only by [`LoopDefinition::Wrapping`] loops. The face anchors at
    /// its first loop's seed; a face bounded by nothing at all has no loop to
    /// take one from and is built with [`Self::closed`] instead.
    ///
    /// # Panics
    ///
    /// Panics on an empty loop list.
    pub fn with_loops(
        surface: Surface,
        data: T,
        loops: Vec<LoopDefinition>,
        pcurves: HashMap<Dart, TrimmedCurve2>,
    ) -> Self {
        assert!(
            !loops.is_empty(),
            "a face with no loop is built with FaceAttr::closed"
        );
        Self {
            surface,
            data,
            boundary: FaceBoundary::Loops(loops),
            pcurves,
        }
    }

    /// Creates a face that bounds nothing, anchored at a dart of its 2-cell.
    ///
    /// A whole sphere or a whole torus covers a closed support: every cell its
    /// 2-cell touches is embedded in the face, so its boundary walk emits
    /// nothing and it has no loop to store. It still occupies a raw 2-cell,
    /// and `dart` is where that cell is read from.
    pub fn closed(
        surface: Surface,
        data: T,
        dart: Dart,
        pcurves: HashMap<Dart, TrimmedCurve2>,
    ) -> Self {
        Self {
            surface,
            data,
            boundary: FaceBoundary::Closed(dart),
            pcurves,
        }
    }

    /// Replaces this face's boundaries, anchoring at `fallback` when there are
    /// none left.
    ///
    /// Healing a seam away empties a face's loops, and the face still occupies
    /// a 2-cell afterwards. Naming the surviving dart here is what stops an
    /// anchor from outliving the loop it came from.
    pub(crate) fn set_boundary(&mut self, loops: Vec<LoopDefinition>, fallback: Dart) {
        self.boundary = FaceBoundary::from_loops(loops, fallback);
    }

    /// Returns a typed face view over this attribute in `model`.
    pub fn face<'a, P: Payload<F = T>>(&'a self, model: &'a Model<P>) -> Face<'a, P> {
        let key = model
            .cell_key::<Cell2>(self.seed())
            .expect("FaceAttr must be registered to produce a Face view");
        Face::new(model, key)
    }

    pub(crate) fn wrapping(&self) -> impl Iterator<Item = (Dart, Axis2)> + '_ {
        self.loops()
            .iter()
            .filter_map(|loop_| loop_.kind().wrapped_axis().map(|axis| (loop_.seed(), axis)))
    }
    pub(crate) fn inner(&self) -> impl Iterator<Item = Dart> + '_ {
        self.loops()
            .iter()
            .filter(|loop_| loop_.kind() == LoopKind::Inner)
            .map(|loop_| loop_.seed())
    }
    pub(crate) fn inner_vec(&self) -> Vec<Dart> {
        self.inner().collect()
    }
    pub(crate) fn darts(&self) -> impl Iterator<Item = Dart> + '_ {
        self.outer_seed().into_iter().chain(
            self.loops()
                .iter()
                .filter(|loop_| loop_.kind() != LoopKind::Outer)
                .map(|loop_| loop_.seed()),
        )
    }
    /// Returns whether this face bounds nothing at all.
    ///
    /// It still occupies a raw 2-cell, which [`Self::seed`] reads: a face with
    /// no loop is a face covering a closed support, not a face with no
    /// topology.
    pub(crate) fn is_boundaryless(&self) -> bool {
        self.loops().is_empty()
    }
    pub(crate) fn kind_of(&self, seed: Dart) -> Option<LoopKind> {
        self.loop_definition(seed).map(|loop_| loop_.kind())
    }
    pub(crate) fn outer_unchecked(&self) -> Dart {
        self.outer_seed().expect("face should have an outer loop")
    }
    pub(crate) fn set_outer(&mut self, seed: Dart) {
        if let Some(loop_) = self
            .boundary
            .edit()
            .iter_mut()
            .find(|loop_| loop_.kind() == LoopKind::Outer)
        {
            loop_.set_seed(seed)
        } else {
            self.boundary.edit().insert(0, LoopDefinition::outer(seed));
        }
    }
    pub(crate) fn set_inner(&mut self, seeds: Vec<Dart>) {
        self.clear_inner();
        self.extend_inner(seeds);
    }
    pub(crate) fn push_inner(&mut self, seed: Dart) {
        self.boundary.edit().push(LoopDefinition::inner(seed));
    }
    pub(crate) fn extend_inner(&mut self, seeds: impl IntoIterator<Item = Dart>) {
        self.boundary
            .edit()
            .extend(seeds.into_iter().map(LoopDefinition::inner));
    }
    pub(crate) fn clear_inner(&mut self) {
        self.boundary
            .edit()
            .retain(|loop_| loop_.kind() != LoopKind::Inner);
    }
    pub(crate) fn map_darts(&mut self, map: impl Fn(Dart) -> Dart) {
        if let FaceBoundary::Closed(dart) = &mut self.boundary {
            *dart = map(*dart);
            return;
        }
        for loop_ in self.boundary.edit().iter_mut() {
            loop_.set_seed(map(loop_.seed()));
        }
    }
    /// Rewrites every dart this face names through `map`, dropping the loops it
    /// does not cover.
    ///
    /// A boundaryless face names only its anchor, so it is rewritten whole:
    /// there is no loop for a partial answer to drop, and leaving the anchor
    /// unmapped would point the copy back into the model it came from.
    pub(crate) fn retain_mapped(&mut self, map: &HashMap<Dart, Dart>) {
        if let FaceBoundary::Closed(dart) = &mut self.boundary {
            if let Some(&mapped) = map.get(dart) {
                *dart = mapped;
            }
            return;
        }
        self.boundary
            .edit()
            .retain_mut(|loop_| match map.get(&loop_.seed()) {
                Some(&seed) => {
                    loop_.set_seed(seed);
                    true
                }
                None => false,
            });
    }
    /// Returns the stored loop definition whose seed is seed.
    pub(crate) fn loop_definition(&self, seed: Dart) -> Option<LoopDefinition> {
        self.loops()
            .iter()
            .copied()
            .find(|loop_| loop_.seed() == seed)
    }

    /// Returns the seed of the chart-closed outer loop, if this face has one.
    pub(crate) fn outer_seed(&self) -> Option<Dart> {
        self.loops().iter().find_map(|loop_| match loop_ {
            LoopDefinition::Outer { seed } => Some(*seed),
            _ => None,
        })
    }

    /// Returns the oriented dart this face is anchored at.
    ///
    /// Total: a face occupies exactly one raw 2-cell, and this reads it. The
    /// loop seeds say where the face's *boundaries* are, which is a different
    /// question and may have no answer at all.
    pub(crate) fn seed(&self) -> Dart {
        self.boundary.seed()
    }

    /// Returns this face's stored loop definitions; empty when it bounds nothing.
    pub(crate) fn loops(&self) -> &[LoopDefinition] {
        self.boundary.loops()
    }
}

/// Stored data and default orientation for a sheet.
#[derive(Clone, Serialize, Deserialize)]
pub struct SheetAttr<T> {
    /// The dart the sheet is anchored at, carrying its traversal direction.
    pub root: Dart,
    /// User payload attached to the sheet.
    pub data: T,
}

impl<T> SheetAttr<T> {
    /// Creates a sheet attribute anchored at `root`.
    pub fn new(root: Dart, data: T) -> Self {
        Self { root, data }
    }

    /// Returns the sheet's anchoring dart.
    ///
    /// Total: every face occupies a raw 2-cell, so a sheet always has a dart
    /// to point at — a sheet that is one boundaryless face points into the
    /// polygon that face owns.
    pub fn dart(&self) -> Dart {
        self.root
    }
}

/// Stored data for a keyed domain solid.
#[derive(Clone, Serialize, Deserialize)]
pub struct SolidAttr<T> {
    /// User payload attached to the solid.
    pub data: T,
    /// The dart the outer shell is anchored at.
    pub outer_shell: Dart,
    /// The darts inner shells are anchored at, when cavities are stored.
    pub inner_shells: Option<Vec<Dart>>,
}

impl<T> SolidAttr<T> {
    /// Creates a solid attribute from an outer shell and optional inner shells.
    pub fn new(data: T, outer_shell: Dart, inner_shells: Option<Vec<Dart>>) -> Self {
        Self {
            data,
            outer_shell,
            inner_shells,
        }
    }

    /// Returns every shell's anchoring dart, the outer shell first.
    pub fn shells(&self) -> impl Iterator<Item = Dart> + '_ {
        std::iter::once(self.outer_shell).chain(self.inner_shells.iter().flatten().copied())
    }

    /// Rewrites every shell's anchoring dart through `map`.
    pub(crate) fn map_shell_darts(&mut self, map: impl Fn(Dart) -> Dart) {
        self.outer_shell = map(self.outer_shell);
        for shell in self.inner_shells.iter_mut().flatten() {
            *shell = map(*shell);
        }
    }
}
