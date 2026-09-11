use std::collections::HashMap;

use serde::{Deserialize, Serialize};

use crate::geometry::dim2::trimmed::TrimmedCurve2;
use crate::geometry::{Axis2, Curve, Point3, Surface};
use crate::topology::dart::Dart;
use crate::topology::edge::Edge;
use crate::topology::face::Face;
use crate::topology::gmap::{Cell0, Cell2, GMap};
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
    /// A loop that spans exactly one period of `axis`, bounding the axis it is
    /// transverse to.
    ///
    /// Such a loop closes on the periodic quotient, not in parameter space: a
    /// cylinder wall's base is a horizontal line one period long, never a
    /// closed polygon. It carries no winding, so which side of it holds
    /// material is read from its travel direction instead.
    Wrapping { axis: Axis2 },
}

impl LoopKind {
    /// Returns the axis this loop spans, when it wraps one.
    pub fn wrapped_axis(self) -> Option<Axis2> {
        match self {
            LoopKind::Wrapping { axis } => Some(axis),
            LoopKind::Outer | LoopKind::Inner => None,
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
    /// A loop that spans exactly one period of `axis`.
    Wrapping { seed: Dart, axis: Axis2 },
}

impl LoopDefinition {
    pub(crate) fn from_kind(seed: Dart, kind: LoopKind) -> Self {
        match kind {
            LoopKind::Outer => Self::outer(seed),
            LoopKind::Inner => Self::inner(seed),
            LoopKind::Wrapping { axis } => Self::wrapping(seed, axis),
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

    /// Returns this definition's oriented traversal seed.
    pub fn seed(self) -> Dart {
        match self {
            Self::Outer { seed } | Self::Inner { seed } | Self::Wrapping { seed, .. } => seed,
        }
    }

    /// Returns this definition's parameter-domain classification.
    pub fn kind(self) -> LoopKind {
        match self {
            Self::Outer { .. } => LoopKind::Outer,
            Self::Inner { .. } => LoopKind::Inner,
            Self::Wrapping { axis, .. } => LoopKind::Wrapping { axis },
        }
    }

    /// Replaces this definition's oriented traversal seed.
    pub fn set_seed(&mut self, seed: Dart) {
        match self {
            Self::Outer { seed: current }
            | Self::Inner { seed: current }
            | Self::Wrapping { seed: current, .. } => *current = seed,
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

    /// Returns an oriented seed suitable for locating this dart-backed face.
    ///
    /// # Panics
    ///
    /// Panics if the face has no loops.
    pub(crate) fn seed_unchecked(&self) -> Dart {
        self.outer_seed()
            .or_else(|| self.loops.first().map(|loop_| loop_.seed()))
            .expect("dart-backed face should have a loop")
    }
}

/// Stored data and default orientation for a sheet.
#[derive(Clone, Serialize, Deserialize)]
pub struct SheetAttr<T> {
    /// Oriented dart used as the sheet's default traversal root.
    pub dart: Dart,
    /// User payload attached to the sheet.
    pub data: T,
}

impl<T> SheetAttr<T> {
    /// Creates a sheet attribute rooted at the given oriented dart.
    pub fn new(dart: Dart, data: T) -> Self {
        Self { dart, data }
    }
}

/// Stored data for a keyed domain solid.
#[derive(Clone, Serialize, Deserialize)]
pub struct SolidAttr<T> {
    /// User payload attached to the solid.
    pub data: T,
    /// Representative dart of the outer shell.
    pub outer_shell: Dart,
    /// Representative darts of inner shells, when cavities are stored.
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
}
