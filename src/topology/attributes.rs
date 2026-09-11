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

/// What one boundary loop bounds on its face.
///
/// The kind lives on each loop rather than on the boundary as a whole, so that
/// configurations combine freely: a face may carry an outer loop and holes, or
/// several wrapping loops and a hole, without a variant per combination.
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

/// One boundary loop of a face: its oriented seed dart and what it bounds.
///
/// The dart is an oriented seed, not an arbitrary representative of its loop
/// cell. Traversing it determines the direction of the boundary; `alpha0` of it
/// traverses the same loop backwards and reverses the face.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct BoundaryLoop {
    /// Oriented seed dart of this loop.
    pub dart: Dart,
    /// What this loop bounds.
    pub kind: LoopKind,
}

impl BoundaryLoop {
    /// Creates a boundary loop seeded at `dart`.
    pub fn new(dart: Dart, kind: LoopKind) -> Self {
        Self { dart, kind }
    }
}

/// The boundary loops of one face, in storage order.
///
/// Order is preserved because callers index inner loops positionally, but the
/// outer loop is found by kind rather than by position.
///
/// A face need not have an outer loop. A ring face — a cylinder wall — is
/// bounded by two [`LoopKind::Wrapping`] loops and nothing else, so the
/// face-level question "is this a disk or a ring" is answered by reading the
/// kinds rather than by a stored flag that could disagree with them.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct FaceBoundary {
    loops: Vec<BoundaryLoop>,
}

impl FaceBoundary {
    /// Creates a boundary from an outer loop seed and inner loop seeds.
    pub fn new(outer: Dart, inner: Vec<Dart>) -> Self {
        let mut loops = Vec::with_capacity(1 + inner.len());
        loops.push(BoundaryLoop::new(outer, LoopKind::Outer));
        loops.extend(
            inner
                .into_iter()
                .map(|dart| BoundaryLoop::new(dart, LoopKind::Inner)),
        );
        Self { loops }
    }

    /// Creates a boundary from already-kinded loops.
    pub fn from_loops(loops: Vec<BoundaryLoop>) -> Self {
        Self { loops }
    }

    /// Returns every boundary loop in storage order.
    pub fn loops(&self) -> &[BoundaryLoop] {
        &self.loops
    }

    /// Returns the oriented seed dart of the outer loop, if the face has one.
    pub fn outer(&self) -> Option<Dart> {
        self.loops
            .iter()
            .find(|boundary| boundary.kind == LoopKind::Outer)
            .map(|boundary| boundary.dart)
    }

    /// Returns an oriented seed dart of the face, preferring the outer loop.
    ///
    /// This is the face's representative: any code that only needs *a* dart of
    /// the face — to resolve its key, to root a shell, to carry its
    /// orientation — wants this rather than the outer loop, because a ring
    /// face has no outer loop to give.
    pub fn seed(&self) -> Option<Dart> {
        self.outer().or_else(|| self.loops.first().map(|l| l.dart))
    }

    /// Returns an oriented seed dart of the face, preferring the outer loop.
    ///
    /// # Panics
    ///
    /// Panics if the face has no boundary loops at all.
    pub fn seed_unchecked(&self) -> Dart {
        self.seed().expect("face should have a boundary loop")
    }

    /// Iterates the oriented seed darts of the wrapping loops, with their axes.
    pub fn wrapping(&self) -> impl Iterator<Item = (Dart, Axis2)> + '_ {
        self.loops
            .iter()
            .filter_map(|boundary| Some((boundary.dart, boundary.kind.wrapped_axis()?)))
    }

    /// Returns whether any loop wraps a periodic direction.
    pub fn is_ring(&self) -> bool {
        self.wrapping().next().is_some()
    }

    /// Returns the oriented seed dart of the outer loop.
    ///
    /// # Panics
    ///
    /// Panics if the face has no outer loop.
    pub fn outer_unchecked(&self) -> Dart {
        self.outer().expect("face should have an outer loop")
    }

    /// Iterates the oriented seed darts of the inner loops, in storage order.
    pub fn inner(&self) -> impl Iterator<Item = Dart> + '_ {
        self.loops
            .iter()
            .filter(|boundary| boundary.kind == LoopKind::Inner)
            .map(|boundary| boundary.dart)
    }

    /// Collects the oriented seed darts of the inner loops.
    pub fn inner_vec(&self) -> Vec<Dart> {
        self.inner().collect()
    }

    /// Iterates every loop's oriented seed dart, outer first when one exists.
    ///
    /// Storage order is preserved among the rest, so a ring face's wrapping
    /// loops come back in the order they were registered.
    pub fn darts(&self) -> impl Iterator<Item = Dart> + '_ {
        self.outer().into_iter().chain(
            self.loops
                .iter()
                .filter(|boundary| boundary.kind != LoopKind::Outer)
                .map(|boundary| boundary.dart),
        )
    }

    /// Returns whether the face has no boundary loops at all.
    pub fn is_empty(&self) -> bool {
        self.loops.is_empty()
    }

    /// Returns the kind of the loop seeded at `dart`, if it is stored.
    pub fn kind_of(&self, dart: Dart) -> Option<LoopKind> {
        self.loops
            .iter()
            .find(|boundary| boundary.dart == dart)
            .map(|boundary| boundary.kind)
    }

    /// Replaces the outer loop seed, adding one if the face had none.
    pub fn set_outer(&mut self, dart: Dart) {
        match self
            .loops
            .iter_mut()
            .find(|boundary| boundary.kind == LoopKind::Outer)
        {
            Some(boundary) => boundary.dart = dart,
            None => self
                .loops
                .insert(0, BoundaryLoop::new(dart, LoopKind::Outer)),
        }
    }

    /// Replaces every inner loop seed, keeping the outer loop untouched.
    pub fn set_inner(&mut self, darts: Vec<Dart>) {
        self.loops
            .retain(|boundary| boundary.kind != LoopKind::Inner);
        self.loops.extend(
            darts
                .into_iter()
                .map(|dart| BoundaryLoop::new(dart, LoopKind::Inner)),
        );
    }

    /// Appends one inner loop seed.
    pub fn push_inner(&mut self, dart: Dart) {
        self.loops.push(BoundaryLoop::new(dart, LoopKind::Inner));
    }

    /// Appends several inner loop seeds.
    pub fn extend_inner(&mut self, darts: impl IntoIterator<Item = Dart>) {
        self.loops.extend(
            darts
                .into_iter()
                .map(|dart| BoundaryLoop::new(dart, LoopKind::Inner)),
        );
    }

    /// Removes every inner loop, keeping the outer loop untouched.
    pub fn clear_inner(&mut self) {
        self.loops
            .retain(|boundary| boundary.kind != LoopKind::Inner);
    }

    /// Rewrites every loop seed through `map`, preserving kinds and order.
    ///
    /// Used when an edit renumbers darts: the boundary structure is unchanged,
    /// only the darts naming it.
    pub fn map_darts(&mut self, map: impl Fn(Dart) -> Dart) {
        for boundary in &mut self.loops {
            boundary.dart = map(boundary.dart);
        }
    }

    /// Keeps only the loops `map` names, rewriting their seeds.
    ///
    /// A merge that brings part of a map across drops the loops whose darts
    /// did not come with it; the survivors keep their kinds and order.
    pub fn retain_mapped(&mut self, map: &HashMap<Dart, Dart>) {
        self.loops
            .retain_mut(|boundary| match map.get(&boundary.dart) {
                Some(&dart) => {
                    boundary.dart = dart;
                    true
                }
                None => false,
            });
    }
}

/// Stored data for a keyed domain face.
///
/// # Boundary orientation
///
/// A [`FaceBoundary`] holds oriented boundary seeds, not arbitrary
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
/// its [`LoopKind`], not by winding alone.
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
    /// Oriented seed darts of the face's boundary loops.
    pub boundary: FaceBoundary,
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
            boundary: FaceBoundary::new(outer_loop, inner_loops),
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
            boundary: FaceBoundary::new(outer_loop, inner_loops),
            pcurves,
        }
    }

    /// Creates a face attribute from already-kinded boundary loops.
    ///
    /// This is the constructor a face without an outer loop needs — a ring
    /// face bounded only by [`LoopKind::Wrapping`] loops.
    pub fn with_boundary(
        surface: Surface,
        data: T,
        boundary: FaceBoundary,
        pcurves: HashMap<Dart, TrimmedCurve2>,
    ) -> Self {
        Self {
            surface,
            data,
            boundary,
            pcurves,
        }
    }

    /// Returns a typed face view over this attribute in `gmap`.
    pub fn face<'a, P: Payload<F = T>>(&'a self, gmap: &'a GMap<P>) -> Face<'a, P> {
        let key = gmap
            .cell_key::<Cell2>(self.boundary.seed_unchecked())
            .expect("FaceAttr must be registered to produce a Face view");
        Face::new(gmap, key)
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

#[cfg(test)]
mod tests {
    use super::*;

    fn darts(count: usize) -> Vec<Dart> {
        (0..count).map(Dart::new).collect()
    }

    #[test]
    fn a_boundary_reports_its_outer_loop_and_holes_separately() {
        let d = darts(3);
        let boundary = FaceBoundary::new(d[0], vec![d[1], d[2]]);

        assert_eq!(boundary.outer(), Some(d[0]));
        assert_eq!(boundary.inner_vec(), vec![d[1], d[2]]);
        assert_eq!(boundary.darts().collect::<Vec<_>>(), vec![d[0], d[1], d[2]]);
        assert_eq!(boundary.kind_of(d[0]), Some(LoopKind::Outer));
        assert_eq!(boundary.kind_of(d[2]), Some(LoopKind::Inner));
    }

    #[test]
    fn setting_inner_loops_leaves_the_outer_loop_in_place() {
        let d = darts(4);
        let mut boundary = FaceBoundary::new(d[0], vec![d[1]]);

        boundary.set_inner(vec![d[2], d[3]]);

        assert_eq!(boundary.outer(), Some(d[0]));
        assert_eq!(boundary.inner_vec(), vec![d[2], d[3]]);

        boundary.clear_inner();

        assert_eq!(boundary.outer(), Some(d[0]));
        assert!(boundary.inner_vec().is_empty());
    }

    /// A face with no loops is the boundaryless case seamless periodic faces
    /// need; it must round-trip rather than be mistaken for a malformed one.
    #[test]
    fn an_empty_boundary_has_no_outer_loop_and_gains_one_on_demand() {
        let d = darts(1);
        let mut boundary = FaceBoundary::default();

        assert!(boundary.is_empty());
        assert_eq!(boundary.outer(), None);
        assert_eq!(boundary.darts().count(), 0);

        boundary.set_outer(d[0]);

        assert_eq!(boundary.outer(), Some(d[0]));
        assert_eq!(boundary.loops().len(), 1);
    }

    #[test]
    fn remapping_darts_preserves_kinds_and_order() {
        let d = darts(6);
        let mut boundary = FaceBoundary::new(d[0], vec![d[1], d[2]]);

        boundary.map_darts(|dart| Dart::new(dart.id() + 3));

        assert_eq!(boundary.outer(), Some(d[3]));
        assert_eq!(boundary.inner_vec(), vec![d[4], d[5]]);
    }
}
