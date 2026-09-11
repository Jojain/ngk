use std::collections::HashMap;

use serde::{Deserialize, Serialize};

use crate::geometry::dim2::trimmed::TrimmedCurve2;
use crate::geometry::{Axis2, Curve, DomainSide, Point3, Surface};
use crate::topology::dart::Dart;
use crate::topology::edge::Edge;
use crate::topology::face::Face;
use crate::topology::gmap::{Cell0, Cell2, GMap};
use crate::topology::orientation::Orientation;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey};
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

    /// Returns a typed vertex view over this attribute in `gmap`.
    pub fn vertex<'a, P: Payload>(&self, gmap: &'a GMap<P>) -> Vertex<'a, P> {
        let key = gmap
            .cell_key::<Cell0>(self.dart)
            .expect("VertexAttr must be registered to produce a Vertex view");
        Vertex::new(gmap, key)
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

    /// Returns a typed edge view over this attribute in `gmap`.
    pub fn edge<'a, P: Payload>(&self, gmap: &'a GMap<P>, key: EdgeKey) -> Edge<'a, P> {
        Edge::new(gmap, key)
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
    /// direction closed on its far side by a parametric degeneracy at `end`.
    ///
    /// A spherical cap — what every sphere-plane cut produces. Unlike a
    /// [`Self::Wrapping`] pair there is no second loop to bound the other side,
    /// and travel direction cannot stand in for one: reversing the loop has to
    /// flip the face's normal, so if it also chose the side it would move the
    /// face to the opposite pole. Which degeneracy closes the face is therefore
    /// said outright.
    Capping { axis: Axis2, end: DomainSide },
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
    pub fn capped_end(self) -> Option<DomainSide> {
        match self {
            LoopKind::Capping { end, .. } => Some(end),
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
    /// direction closed on its far side by the degeneracy at `end`.
    Capping {
        seed: Dart,
        axis: Axis2,
        end: DomainSide,
    },
}

impl LoopDefinition {
    pub(crate) fn from_kind(seed: Dart, kind: LoopKind) -> Self {
        match kind {
            LoopKind::Outer => Self::outer(seed),
            LoopKind::Inner => Self::inner(seed),
            LoopKind::Wrapping { axis } => Self::wrapping(seed, axis),
            LoopKind::Capping { axis, end } => Self::capping(seed, axis, end),
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
    pub fn capping(seed: Dart, axis: Axis2, end: DomainSide) -> Self {
        Self::Capping { seed, axis, end }
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
            Self::Capping { axis, end, .. } => LoopKind::Capping { axis, end },
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
    /// Internal storage for this face's loop definitions.
    pub(crate) loops: Vec<LoopDefinition>,
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
            loops: std::iter::once(LoopDefinition::outer(outer_loop))
                .chain(inner_loops.into_iter().map(LoopDefinition::inner))
                .collect(),
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
            loops: std::iter::once(LoopDefinition::outer(outer_loop))
                .chain(inner_loops.into_iter().map(LoopDefinition::inner))
                .collect(),
            pcurves,
        }
    }

    /// Creates a face attribute from explicit loop definitions.
    ///
    /// This constructor supports faces with no outer loop, such as a ring
    /// bounded only by [`LoopDefinition::Wrapping`] loops.
    pub fn with_loops(
        surface: Surface,
        data: T,
        loops: Vec<LoopDefinition>,
        pcurves: HashMap<Dart, TrimmedCurve2>,
    ) -> Self {
        Self {
            surface,
            data,
            loops,
            pcurves,
        }
    }

    /// Returns a typed face view over this attribute in `gmap`.
    pub fn face<'a, P: Payload<F = T>>(&'a self, gmap: &'a GMap<P>) -> Face<'a, P> {
        let key = gmap
            .cell_key::<Cell2>(self.seed_unchecked())
            .expect("FaceAttr must be registered to produce a Face view");
        Face::new(gmap, key)
    }

    pub(crate) fn wrapping(&self) -> impl Iterator<Item = (Dart, Axis2)> + '_ {
        self.loops
            .iter()
            .filter_map(|loop_| loop_.kind().wrapped_axis().map(|axis| (loop_.seed(), axis)))
    }
    pub(crate) fn inner(&self) -> impl Iterator<Item = Dart> + '_ {
        self.loops
            .iter()
            .filter(|loop_| loop_.kind() == LoopKind::Inner)
            .map(|loop_| loop_.seed())
    }
    pub(crate) fn inner_vec(&self) -> Vec<Dart> {
        self.inner().collect()
    }
    pub(crate) fn darts(&self) -> impl Iterator<Item = Dart> + '_ {
        self.outer_seed().into_iter().chain(
            self.loops
                .iter()
                .filter(|loop_| loop_.kind() != LoopKind::Outer)
                .map(|loop_| loop_.seed()),
        )
    }
    pub(crate) fn is_empty(&self) -> bool {
        self.loops.is_empty()
    }
    pub(crate) fn kind_of(&self, seed: Dart) -> Option<LoopKind> {
        self.loop_definition(seed).map(|loop_| loop_.kind())
    }
    pub(crate) fn outer_unchecked(&self) -> Dart {
        self.outer_seed().expect("face should have an outer loop")
    }
    pub(crate) fn set_outer(&mut self, seed: Dart) {
        if let Some(loop_) = self
            .loops
            .iter_mut()
            .find(|loop_| loop_.kind() == LoopKind::Outer)
        {
            loop_.set_seed(seed)
        } else {
            self.loops.insert(0, LoopDefinition::outer(seed));
        }
    }
    pub(crate) fn set_inner(&mut self, seeds: Vec<Dart>) {
        self.clear_inner();
        self.extend_inner(seeds);
    }
    pub(crate) fn push_inner(&mut self, seed: Dart) {
        self.loops.push(LoopDefinition::inner(seed));
    }
    pub(crate) fn extend_inner(&mut self, seeds: impl IntoIterator<Item = Dart>) {
        self.loops
            .extend(seeds.into_iter().map(LoopDefinition::inner));
    }
    pub(crate) fn clear_inner(&mut self) {
        self.loops.retain(|loop_| loop_.kind() != LoopKind::Inner);
    }
    pub(crate) fn map_darts(&mut self, map: impl Fn(Dart) -> Dart) {
        for loop_ in &mut self.loops {
            loop_.set_seed(map(loop_.seed()));
        }
    }
    pub(crate) fn retain_mapped(&mut self, map: &HashMap<Dart, Dart>) {
        self.loops.retain_mut(|loop_| match map.get(&loop_.seed()) {
            Some(&seed) => {
                loop_.set_seed(seed);
                true
            }
            None => false,
        });
    }
    /// Returns the stored loop definition whose seed is seed.
    pub(crate) fn loop_definition(&self, seed: Dart) -> Option<LoopDefinition> {
        self.loops
            .iter()
            .copied()
            .find(|loop_| loop_.seed() == seed)
    }

    /// Returns the seed of the chart-closed outer loop, if this face has one.
    pub(crate) fn outer_seed(&self) -> Option<Dart> {
        self.loops.iter().find_map(|loop_| match loop_ {
            LoopDefinition::Outer { seed } => Some(*seed),
            _ => None,
        })
    }

    /// Returns an oriented seed locating this face, when it has a boundary.
    ///
    /// A boundaryless face has no loop and so no seed: nothing is incident to
    /// it, and it is reached by key alone.
    pub(crate) fn seed(&self) -> Option<Dart> {
        self.outer_seed()
            .or_else(|| self.loops.first().map(|loop_| loop_.seed()))
    }

    /// Returns an oriented seed suitable for locating this dart-backed face.
    ///
    /// # Panics
    ///
    /// Panics if the face has no loops.
    pub(crate) fn seed_unchecked(&self) -> Dart {
        self.seed().expect("dart-backed face should have a loop")
    }
}

/// Where a sheet, or one of a solid's shells, is anchored.
///
/// A shell is normally located by one of its darts: its faces, its orientation
/// and its extent all follow from walking the map from there. A boundaryless
/// face has no darts at all, so a sheet holding one has no incidence to point
/// at, and names the face instead.
///
/// > A root is a dart whenever any dart exists in the cell. It is a key only
/// > when there is no dart to point at.
///
/// That invariant is what makes the key variant self-eliminating: the moment a
/// sheet gains topology — a boundaryless face split by a plane — the commit
/// re-roots it at a dart, so a stored key never outlives the face it names.
/// A dart root carries its shell's direction in the dart itself; a face root
/// has no dart to `alpha0`-flip, and no loop seeds either, so it spells the
/// direction out. That is what lets a spherical cavity — an inner shell facing
/// inward — be written at all.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ShellRoot {
    /// An oriented dart of the shell, giving its default traversal direction.
    Dart(Dart),
    /// The shell is exactly one boundaryless face, read in this orientation.
    Face {
        /// The face that is the whole shell.
        face: FaceKey,
        /// How the shell faces relative to the support surface's own normal.
        sense: Orientation,
    },
}

impl ShellRoot {
    /// Anchors a shell at a boundaryless face, facing as its support does.
    pub fn at_face(face: FaceKey) -> Self {
        Self::Face {
            face,
            sense: Orientation::Same,
        }
    }

    /// Returns the anchoring dart, or `None` for a boundaryless shell.
    pub fn dart(self) -> Option<Dart> {
        match self {
            Self::Dart(dart) => Some(dart),
            Self::Face { .. } => None,
        }
    }

    /// Returns the anchoring face, or `None` for a dart-rooted shell.
    pub fn face(self) -> Option<FaceKey> {
        match self {
            Self::Face { face, .. } => Some(face),
            Self::Dart(_) => None,
        }
    }

    /// Returns the same root read in the opposite orientation.
    ///
    /// # Panics
    ///
    /// Panics on a dart root, whose reversal needs the map to `alpha0` it.
    pub fn reversed_face(self) -> Self {
        match self {
            Self::Face { face, sense } => Self::Face {
                face,
                sense: sense.flip(),
            },
            Self::Dart(_) => panic!("reversing a dart root needs the map it belongs to"),
        }
    }

    /// Returns the anchoring dart of a dart-rooted shell.
    ///
    /// # Panics
    ///
    /// Panics on a boundaryless shell, which has no dart to return.
    pub fn dart_unchecked(self) -> Dart {
        self.dart()
            .expect("dart-rooted shell should have an anchoring dart")
    }

    /// Rewrites the anchoring dart through `map`, leaving a face root alone.
    pub(crate) fn map_dart(&mut self, map: impl FnOnce(Dart) -> Dart) {
        if let Self::Dart(dart) = self {
            *dart = map(*dart);
        }
    }
}

/// Stored data and default orientation for a sheet.
#[derive(Clone, Serialize, Deserialize)]
pub struct SheetAttr<T> {
    /// Where the sheet is anchored, carrying its default traversal direction.
    pub root: ShellRoot,
    /// User payload attached to the sheet.
    pub data: T,
}

impl<T> SheetAttr<T> {
    /// Creates a sheet attribute anchored at `root`.
    pub fn new(root: ShellRoot, data: T) -> Self {
        Self { root, data }
    }

    /// Returns the sheet's anchoring dart, or `None` when it is boundaryless.
    pub fn dart(&self) -> Option<Dart> {
        self.root.dart()
    }
}

/// Stored data for a keyed domain solid.
#[derive(Clone, Serialize, Deserialize)]
pub struct SolidAttr<T> {
    /// User payload attached to the solid.
    pub data: T,
    /// Anchor of the outer shell.
    pub outer_shell: ShellRoot,
    /// Anchors of inner shells, when cavities are stored.
    pub inner_shells: Option<Vec<ShellRoot>>,
}

impl<T> SolidAttr<T> {
    /// Creates a solid attribute from an outer shell and optional inner shells.
    pub fn new(data: T, outer_shell: ShellRoot, inner_shells: Option<Vec<ShellRoot>>) -> Self {
        Self {
            data,
            outer_shell,
            inner_shells,
        }
    }

    /// Returns every shell anchor, the outer shell first.
    pub fn shells(&self) -> impl Iterator<Item = ShellRoot> + '_ {
        std::iter::once(self.outer_shell).chain(self.inner_shells.iter().flatten().copied())
    }

    /// Returns the anchoring dart of every dart-rooted shell, outer first.
    pub fn shell_darts(&self) -> impl Iterator<Item = Dart> + '_ {
        self.shells().filter_map(ShellRoot::dart)
    }

    /// Rewrites every shell's anchoring dart through `map`.
    pub(crate) fn map_shell_darts(&mut self, map: impl Fn(Dart) -> Dart) {
        self.outer_shell.map_dart(&map);
        for shell in self.inner_shells.iter_mut().flatten() {
            shell.map_dart(&map);
        }
    }
}
