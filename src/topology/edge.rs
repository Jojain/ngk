use std::collections::HashSet;
use std::ops::Deref;

use crate::geometry::parameter::NativeParam;
use crate::geometry::{Curve, Interval, PointCoincidence, TrimmedCurve};
use crate::model::{Cell1, Cell2, MergeTopology, TopologyMerge};
use crate::topology::attributes::EdgeAttr;
use crate::topology::closed::Closeable;
use crate::topology::face::Face;
use crate::topology::gmap::Dim;
use crate::topology::orientation::Orientation;
use crate::topology::shape_keys::EdgeKey;

use super::gmap::Dart;
use super::payload::{Payload, StandardPayload};
use super::sheet::Sheet;
use super::vertex::Vertex;
use crate::model::Model;

/// A typed view over a 1-cell of a [`Model`], in whichever of the three shapes
/// an edge can have.
///
/// The shapes are **bounded** (two distinct corners), **marked** (one corner,
/// which is both its start and its end) and **unmarked** (no corner at all).
/// All three are read off the combinatorics; none is stored.
///
/// Marked and unmarked are both *closed*, but they are separate variants rather
/// than one with an `Option` inside, and the enum is flat rather than nested for
/// the same reason: a site that must tell them apart should not be able to write
/// one pattern that silently covers both. A site that genuinely means "closed"
/// writes `Marked(_) | Unmarked(_)`, which stays exhaustive-checked, or asks
/// [`Closeable::is_closed`](crate::topology::closed::Closeable::is_closed).
///
/// The distinction is load-bearing for geometry, not decoration. A bounded edge
/// says which part of its support it is by naming the two points that cut it. A
/// closed edge cannot: its ends coincide, so they name no arc. **A closed edge
/// *is* its support** — it spans the whole period — which is why
/// [`parameter_interval`](EdgeCore::parameter_interval) stays total without
/// endpoints, and why storing an `Interval` on `EdgeAttr` for this one case
/// would be the loose pair the codebase avoids.
///
/// Everything an edge can answer without knowing which shape it has lives on
/// [`EdgeCore`], which every one of these types derefs to, so `key`, `curve`,
/// `faces` and the rest read the same on all of them. Only the corners differ:
/// two, inherent and total, on [`BoundedEdge`]; one, inherent and total, on
/// [`MarkedEdge`]; none on [`UnmarkedEdge`], which offers no way to ask.
/// [`has_corner_at`](EdgeCore::has_corner_at) answers across all three, so a
/// caller asking only whether a cut lands on an existing corner never branches.
///
/// # Narrowing
///
/// A caller that already knows what it built — a freshly sewn polyline, a
/// chamfer's own walls, a section the solver just cut between two points —
/// narrows once with [`bounded_unchecked`](Edge::bounded_unchecked) and is done:
/// the rest of the function holds a [`BoundedEdge`] and never asks again. A
/// caller that can meet arbitrary model topology matches on the two variants, or
/// narrows with the total [`bounded`](Edge::bounded).
///
/// Better still, a function that only ever handles bounded edges should *take* a
/// [`BoundedEdge`]. Then the check happens once, at the one caller that can
/// actually decide what to do about a closed one.
///
/// # Default orientation
///
/// `Model::edge(edge_key)` uses the dart stored in `EdgeAttr`. Traversals such as
/// [`Profile::edges`](crate::topology::profile::Profile::edges) or
/// [`Face::edges`](crate::topology::face::Face::edges) preserve the exact dart
/// reached in that traversal context.
pub enum Edge<'a, P: Payload = StandardPayload> {
    /// Two distinct corners cut a section out of the support.
    Bounded(BoundedEdge<'a, P>),
    /// One corner, which is both the edge's start and its end.
    Marked(MarkedEdge<'a, P>),
    /// No corner at all; the edge spans its support entirely.
    Unmarked(UnmarkedEdge<'a, P>),
}

impl<'a, P: Payload> Edge<'a, P> {
    /// Creates an edge view with the default (`Same`) orientation.
    pub fn new(model: &'a Model<P>, key: EdgeKey) -> Self {
        let dart = model.edge_attr_unchecked(key).dart;
        EdgeCore { model, key, dart }.classify()
    }

    /// Creates an edge view from a dart, resolving the edge key and
    /// orientation relative to the stored default direction.
    ///
    /// Returns `None` if the dart does not belong to a registered edge.
    pub fn from_dart(model: &'a Model<P>, dart: Dart) -> Option<Self> {
        let key = model.cell_key::<Cell1>(dart)?;
        Some(EdgeCore { model, key, dart }.classify())
    }

    /// Returns this view as a bounded edge, or `None` when it closes on itself.
    pub fn bounded(self) -> Option<BoundedEdge<'a, P>> {
        match self {
            Self::Bounded(edge) => Some(edge),
            Self::Marked(_) | Self::Unmarked(_) => None,
        }
    }

    /// Returns this view as a bounded edge, for a caller that knows it is one.
    ///
    /// # Panics
    ///
    /// Panics if the edge is closed.
    pub fn bounded_unchecked(self) -> BoundedEdge<'a, P> {
        self.bounded()
            .expect("edge should be bounded by two distinct vertices")
    }

    /// Returns a new edge view with the opposite orientation.
    ///
    /// Reversal does not change what bounds an edge, so the variant is kept.
    pub fn reversed(&self) -> Self {
        match self {
            Self::Bounded(edge) => Self::Bounded(edge.reversed()),
            Self::Marked(edge) => Self::Marked(edge.reversed()),
            Self::Unmarked(edge) => Self::Unmarked(edge.reversed()),
        }
    }
}

/// An edge cut out of its support by two distinct bounding vertices.
///
/// Reached from [`Edge::bounded`] or [`Edge::bounded_unchecked`]. Its
/// [`start`](Self::start) and [`end`](Self::end) are total: a value of this type
/// is proof that the two ends exist and differ, so code below the narrowing
/// never unwraps them again.
pub struct BoundedEdge<'a, P: Payload = StandardPayload>(EdgeCore<'a, P>);

impl<'a, P: Payload> BoundedEdge<'a, P> {
    /// Returns the vertex this edge view leaves.
    pub fn start(&self) -> Vertex<'a, P> {
        self.vertices().0
    }

    /// Returns the vertex this edge view arrives at.
    pub fn end(&self) -> Vertex<'a, P> {
        self.vertices().1
    }

    /// Returns both bounding vertices in this view's traversal order.
    pub fn vertices(&self) -> (Vertex<'a, P>, Vertex<'a, P>) {
        vertices_at_dart(self.0.model, self.0.dart).expect("a bounded edge has two distinct ends")
    }

    /// Returns a new view of the same edge traversed the other way.
    pub fn reversed(&self) -> Self {
        Self(self.0.reversed())
    }

    /// Widens back to the shape-agnostic view.
    pub fn into_edge(self) -> Edge<'a, P> {
        Edge::Bounded(self)
    }
}

/// A closed edge carrying one corner, which is both its start and its end.
///
/// [`corner`](Self::corner) is total: a value of this type is proof the corner
/// exists, so nothing below the narrowing unwraps it again. That corner is where
/// the edge *begins* — which is not in general where its support's own
/// parameterization starts, and the difference is load-bearing.
pub struct MarkedEdge<'a, P: Payload = StandardPayload>(EdgeCore<'a, P>);

impl<'a, P: Payload> MarkedEdge<'a, P> {
    /// Returns the corner the edge leaves and arrives at.
    pub fn corner(&self) -> Vertex<'a, P> {
        Vertex::from_dart(self.0.model, self.0.dart).expect("a marked edge has a corner")
    }

    /// Returns a new view of the same edge traversed the other way.
    pub fn reversed(&self) -> Self {
        Self(self.0.reversed())
    }

    /// Widens back to the shape-agnostic view.
    pub fn into_edge(self) -> Edge<'a, P> {
        Edge::Marked(self)
    }
}

/// A closed edge with no corner anywhere on it.
///
/// The place its parameterization closes is a raw cell classified as interior to
/// the edge, not a corner anything meets at, so this type deliberately offers no
/// way to ask for one. Cutting such an edge marks it rather than separating it:
/// there is no second corner to separate it from.
pub struct UnmarkedEdge<'a, P: Payload = StandardPayload>(EdgeCore<'a, P>);

impl<'a, P: Payload> UnmarkedEdge<'a, P> {
    /// Returns a new view of the same edge traversed the other way.
    pub fn reversed(&self) -> Self {
        Self(self.0.reversed())
    }

    /// Widens back to the shape-agnostic view.
    pub fn into_edge(self) -> Edge<'a, P> {
        Edge::Unmarked(self)
    }
}

/// What every edge view answers, whatever bounds it.
///
/// [`Edge`], [`BoundedEdge`] and [`ClosedEdge`] all deref here, so these methods
/// are written once and read identically on all three. The type is never named
/// at a call site; it exists so that narrowing an [`Edge`] costs nothing and
/// gains only the endpoints.
pub struct EdgeCore<'a, P: Payload = StandardPayload> {
    model: &'a Model<P>,
    key: EdgeKey,
    dart: Dart,
}

impl<'a, P: Payload> EdgeCore<'a, P> {
    /// Sorts this view into the variant its combinatorics put it in.
    fn classify(self) -> Edge<'a, P> {
        if vertices_at_dart(self.model, self.dart).is_some() {
            return Edge::Bounded(BoundedEdge(self));
        }
        match Vertex::from_dart(self.model, self.dart) {
            Some(_) => Edge::Marked(MarkedEdge(self)),
            None => Edge::Unmarked(UnmarkedEdge(self)),
        }
    }

    /// Returns the same core traversed the other way.
    fn reversed(&self) -> Self {
        Self {
            model: self.model,
            key: self.key,
            dart: self.model.alpha(Dim::Zero, self.dart),
        }
    }

    /// Returns the stable key of this edge.
    pub fn key(&self) -> EdgeKey {
        self.key
    }

    /// Returns the store edge attribute
    pub fn attr(&self) -> &EdgeAttr<P::E> {
        self.model.edge_attr_unchecked(self.key)
    }

    /// Returns the dart that represents this edge view in the current
    /// traversal context.
    pub fn dart(&self) -> Dart {
        self.dart
    }

    /// Iterates every dart in this edge's 1-cell orbit.
    pub fn darts(&self) -> impl Iterator<Item = Dart> + '_ {
        let dart = self.dart();
        self.model.orbit(dart, self.model.orbit_indices(Dim::One))
    }

    /// Returns the distinct vertices incident to this edge.
    ///
    /// One for a closed edge whose ends still meet at a vertex, none for one
    /// that has lost it, two for a bounded edge. Narrow to a [`BoundedEdge`] to
    /// ask which end is which; this answers only what the edge touches.
    pub fn vertices(&self) -> Vec<Vertex<'a, P>> {
        self.model
            .incident_cells(self.dart(), Dim::One, Dim::Zero)
            .filter_map(|d| Vertex::from_dart(self.model, d))
            .collect()
    }

    /// Returns the distinct domain faces incident to this edge.
    pub fn faces(&self) -> Vec<Face<'a, P>> {
        let mut seen = HashSet::new();
        self.model
            .incident_cells(self.dart(), Dim::One, Dim::Two)
            .filter_map(|dart| {
                let key = self.model.cell_key::<Cell2>(dart)?;
                seen.insert(key)
                    .then(|| Face::from_dart(self.model, dart))
                    .flatten()
            })
            .collect()
    }

    /// Returns all 2-dimensional sheets incident to this edge.
    ///
    /// Wrap a returned sheet with [`Closed::new`](super::closed::Closed::new)
    /// when the caller needs the stronger shell invariant.
    pub fn sheets(&self) -> Vec<Sheet<'a, P>> {
        self.model
            .incident_cells(self.dart(), Dim::One, Dim::Three)
            .filter_map(|d| Sheet::from_dart(self.model, d))
            .collect()
    }

    /// Returns the geometric curve attached to this edge.
    pub fn curve(&self) -> &'a Curve {
        &self.model.edge_attr_unchecked(self.key).curve
    }

    /// Whether a corner already sits where `parameter` falls on this edge.
    ///
    /// This is what "would this cut land on an end of the edge?" actually asks,
    /// and it is total over all three shapes: a bounded edge has two corners, a
    /// marked edge one, an unmarked edge none — so on an unmarked edge the
    /// answer is always `false`, because there is nothing there to land on.
    ///
    /// Comparing *points* rather than parameters on purpose. The ends of an
    /// edge's span coincide with its corners only when it has corners there, and
    /// reconstructing the answer from `domain.start.value()` is the mistake this exists
    /// to stop. `tolerance` is a distance.
    pub fn has_corner_at(&self, parameter: NativeParam, tolerance: f64) -> bool {
        let at = self.curve().point_at(parameter);
        self.vertices()
            .iter()
            .map(|corner| corner.point())
            .any(|corner| corner.coincides(at, tolerance))
    }

    /// Returns the curve-parameter span followed by this oriented edge view.
    ///
    /// The reference span is recovered from the corners selected by the edge's
    /// stored dart. A view reached in the opposite direction swaps that span
    /// without applying periodic wrapping again, so it traverses the same
    /// geometric section backward rather than its complement.
    pub fn parameter_interval(&self) -> Interval {
        let attr = self.model.edge_attr_unchecked(self.key);
        // The reference span belongs to the stored dart, not to this view: the
        // view's orientation is applied to it below.
        //
        // Asked of the corners directly rather than through `vertices_at_dart`,
        // which folds a marked edge's coincident ends to `None`. A marked edge
        // runs a whole period *from its corner*, which `interval_between`
        // answers for two coincident points, and that corner is not in general
        // where the support's own domain starts.
        let ends = Vertex::from_dart(self.model, attr.dart).zip(Vertex::from_dart(
            self.model,
            self.model.alpha(Dim::Zero, attr.dart),
        ));
        let reference = match ends {
            Some((start, end)) => attr.curve.interval_between(*start.point(), *end.point()),
            // An unmarked edge is its support, and has no corner to ask.
            None => attr.curve.domain(),
        };
        match self.model.edge_orientation_at_dart(self.key, self.dart) {
            Orientation::Same => reference,
            Orientation::Reversed => reference.reversed(),
        }
    }

    /// Returns this edge's curve paired with the span its vertices bound.
    ///
    /// An edge stores only a support — a whole line, a whole circle — so this
    /// is the value that says which part of it the edge actually is. The span
    /// is *derived*, never stored: the bounding vertices give its ends and the
    /// view's orientation gives its direction, which is why an edge needs no
    /// interval in its attribute. Geometry that has no vertices to derive from,
    /// such as a solver's section, must carry its span instead.
    pub fn trimmed_curve(&self) -> TrimmedCurve {
        TrimmedCurve::new(self.curve().clone(), self.parameter_interval())
    }

    /// Returns the curve length over this edge view's
    /// [`parameter_interval`](Self::parameter_interval).
    pub fn length(&self) -> f64 {
        self.trimmed_curve().length()
    }
}

/// Returns the two vertices at the ends of the edge occurrence at `dart`, or
/// `None` when they are the same vertex or either is unregistered.
///
/// This is the single place closedness is decided, and it is decided
/// combinatorially rather than by comparing endpoints within a tolerance: two
/// *distinct* vertices that happen to sit at one point are a degenerate model,
/// not a circle, and no tolerance can tell the difference the map already
/// records exactly.
fn vertices_at_dart<P: Payload>(
    model: &Model<P>,
    dart: Dart,
) -> Option<(Vertex<'_, P>, Vertex<'_, P>)> {
    let start = Vertex::from_dart(model, dart)?;
    let end = Vertex::from_dart(model, model.alpha(Dim::Zero, dart))?;
    (start.key() != end.key()).then_some((start, end))
}

// An edge view is three `Copy` words over a borrowed map, so it is `Copy` too —
// written out rather than derived because `derive` would demand `P: Copy`, and
// the payload type is never touched by a view.
impl<P: Payload> Copy for Edge<'_, P> {}
impl<P: Payload> Copy for BoundedEdge<'_, P> {}
impl<P: Payload> Copy for MarkedEdge<'_, P> {}
impl<P: Payload> Copy for UnmarkedEdge<'_, P> {}
impl<P: Payload> Copy for EdgeCore<'_, P> {}

impl<P: Payload> Clone for Edge<'_, P> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<P: Payload> Clone for BoundedEdge<'_, P> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<P: Payload> Clone for MarkedEdge<'_, P> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<P: Payload> Clone for UnmarkedEdge<'_, P> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<P: Payload> Clone for EdgeCore<'_, P> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<'a, P: Payload> Deref for Edge<'a, P> {
    type Target = EdgeCore<'a, P>;

    fn deref(&self) -> &Self::Target {
        match self {
            Self::Bounded(edge) => &edge.0,
            Self::Marked(edge) => &edge.0,
            Self::Unmarked(edge) => &edge.0,
        }
    }
}

impl<'a, P: Payload> Deref for BoundedEdge<'a, P> {
    type Target = EdgeCore<'a, P>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<'a, P: Payload> Deref for MarkedEdge<'a, P> {
    type Target = EdgeCore<'a, P>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<'a, P: Payload> Deref for UnmarkedEdge<'a, P> {
    type Target = EdgeCore<'a, P>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<P: Payload> MergeTopology<P> for EdgeCore<'_, P> {
    fn merge_topology(&self) -> TopologyMerge<'_, P> {
        TopologyMerge::new(self.model, self.darts().collect(), self.dart())
    }
}

impl<P: Payload> MergeTopology<P> for Edge<'_, P> {
    fn merge_topology(&self) -> TopologyMerge<'_, P> {
        (**self).merge_topology()
    }
}

impl<P: Payload> MergeTopology<P> for BoundedEdge<'_, P> {
    fn merge_topology(&self) -> TopologyMerge<'_, P> {
        self.0.merge_topology()
    }
}

impl<P: Payload> MergeTopology<P> for MarkedEdge<'_, P> {
    fn merge_topology(&self) -> TopologyMerge<'_, P> {
        self.0.merge_topology()
    }
}

impl<P: Payload> MergeTopology<P> for UnmarkedEdge<'_, P> {
    fn merge_topology(&self) -> TopologyMerge<'_, P> {
        self.0.merge_topology()
    }
}

impl<P: Payload> Closeable for Edge<'_, P> {
    fn is_closed(&self) -> bool {
        matches!(self, Self::Marked(_) | Self::Unmarked(_))
    }
}

impl<P: Payload> Closeable for BoundedEdge<'_, P> {
    fn is_closed(&self) -> bool {
        false
    }
}

impl<P: Payload> Closeable for MarkedEdge<'_, P> {
    fn is_closed(&self) -> bool {
        true
    }
}

impl<P: Payload> Closeable for UnmarkedEdge<'_, P> {
    fn is_closed(&self) -> bool {
        true
    }
}
