use super::edge::Edge;
use super::gmap::{Dart, Dim};
use super::orientation::Orientation;
use super::payload::{Payload, StandardPayload};
use super::vertex::Vertex;
use crate::geometry::Surface;
use crate::geometry::dim2::curves::Curve2;
use crate::geometry::dim2::trimmed::TrimmedCurve2;
use crate::geometry::parameter::Fraction;
use crate::geometry::{LINEAR_TOLERANCE, Point2, Point3};
use crate::model::{Cell2, MergeTopology, Model, RealizationPurpose, TopologyMerge};
use crate::topology::attributes::{FaceAttr, LoopKind};
use crate::topology::embedding::{EntityOwner, boundary_cycles, recover_region};
use crate::topology::profile::LoopCorner;
use crate::topology::shape_keys::{FaceKey, ProfileKey};
use nalgebra::UnitVector3;
use std::collections::HashSet;

/// Samples per pcurve used to read a boundary's winding.
///
/// The winding only needs a sign, never a measurement, so a handful of
/// samples per pcurve settles it at any model scale.
const BOUNDARY_WINDING_SAMPLES: usize = 8;

/// One closed oriented boundary of a face, as the map itself reports it.
///
/// A loop combines a boundary walk with the face-domain role that gives that
/// walk meaning: a chart-closed exterior, a hole, or a periodic wrapping
/// boundary. The role belongs here rather than on [`Profile`], because it
/// depends on this face's support and parameterization.
///
/// The walk is **derived, never stored**. Its darts come from the frontier of
/// the face's own region, so a cut the face owns — a bridge to a hole, a
/// periodic seam — is turned across rather than emitted, and refining the
/// scaffold changes which raw darts appear without changing the loop. A
/// profile cannot stand in for this: once a face reaches its hole along a
/// bridge, one raw boundary chain carries every loop the face has, and only
/// turning across the face's own cuts separates them again.
pub struct Loop<'a, P: Payload = StandardPayload> {
    model: &'a Model<P>,
    /// The loop's boundary darts, in traversal order.
    darts: Vec<Dart>,
    kind: LoopKind,
}

impl<'a, P: Payload> Loop<'a, P> {
    /// Creates a loop from a derived boundary walk and its face-domain role.
    fn new(model: &'a Model<P>, darts: Vec<Dart>, kind: LoopKind) -> Self {
        Self { model, darts, kind }
    }

    /// Returns the loop's boundary darts in traversal order.
    ///
    /// One dart per oriented edge the walk runs along. An edge the loop runs
    /// twice yields two darts even where the two runs are adjacent, which is
    /// how a seam is walked: up one side of it and straight back down the
    /// other.
    pub fn darts(&self) -> impl Iterator<Item = Dart> + '_ {
        self.darts.iter().copied()
    }

    /// Returns the same loop read from `dart`, or `None` when the walk does
    /// not run along it.
    ///
    /// A caller that located something by counting along this loop has to read
    /// it back from where it counted, which is the dart it named and not
    /// wherever the walk happened to begin. Reversing when the walk runs the
    /// other way round means the result travels the way `dart` points too.
    pub fn starting_at(&self, dart: Dart) -> Option<Self> {
        let rotated = |darts: &[Dart]| {
            darts.iter().position(|&walked| walked == dart).map(|at| {
                let mut darts = darts.to_vec();
                darts.rotate_left(at);
                darts
            })
        };
        let darts = rotated(&self.darts).or_else(|| rotated(&self.reversed().darts))?;
        Some(Self::new(self.model, darts, self.kind))
    }

    /// Returns this loop's edges, one per oriented dart, in walk order.
    ///
    /// # Panics
    ///
    /// Panics on a boundary dart carrying no logical edge. The walk turns
    /// across every cell the face owns and emits none of them, so each dart it
    /// does emit bounds the face and must name the edge it runs along; one that
    /// does not is a face whose boundary was never registered, and dropping it
    /// would hand back a loop shorter than the walk.
    pub fn edges(&self) -> Vec<Edge<'a, P>> {
        self.darts()
            .map(|dart| {
                Edge::from_dart(self.model, dart)
                    .expect("a face's boundary dart names the edge it runs along")
            })
            .collect()
    }

    /// Returns the logical vertices this loop meets, in walk order.
    ///
    /// A loop of closed edges meets none: the place an unmarked circle closes
    /// is interior to it and is not a corner anything meets at.
    pub fn vertices(&self) -> Vec<Vertex<'a, P>> {
        self.darts()
            .filter_map(|dart| Vertex::from_dart(self.model, dart))
            .collect()
    }

    /// Returns the loop's corners in traversal order.
    ///
    /// Each corner pairs the dart arriving at it with the one leaving.
    pub fn corners(&self) -> Vec<LoopCorner<'a, P>> {
        let count = self.darts.len();
        self.darts
            .iter()
            .enumerate()
            .map(|(index, &outgoing)| {
                LoopCorner::new(
                    self.model,
                    self.darts[(index + count - 1) % count],
                    outgoing,
                )
            })
            .collect()
    }

    /// Returns a dart of this loop's walk, carrying its traversal orientation.
    ///
    /// # Panics
    ///
    /// Panics on a loop with no boundary dart, which no walked cycle is.
    pub fn dart(&self) -> Dart {
        *self
            .darts
            .first()
            .expect("a walked boundary cycle has at least one dart")
    }

    /// Returns the registered profile this loop's walk runs along, if any.
    pub fn profile_key(&self) -> Option<ProfileKey> {
        self.model.profile_key(self.dart())
    }

    /// Returns the number of oriented edge darts in this loop.
    pub fn len(&self) -> usize {
        self.darts.len()
    }

    /// Reports whether the walk found no boundary at all.
    pub fn is_empty(&self) -> bool {
        self.darts.is_empty()
    }

    /// Returns how this loop bounds its face in parameter space.
    pub fn kind(&self) -> LoopKind {
        self.kind
    }

    /// Returns whether this loop bounds the face's outer region.
    pub fn is_outer(&self) -> bool {
        self.kind == LoopKind::Outer
    }
    /// Returns whether this loop bounds a hole.
    pub fn is_inner(&self) -> bool {
        self.kind == LoopKind::Inner
    }
    /// Returns whether this loop bounds a wrapping seam.
    pub fn is_wrapping(&self) -> bool {
        matches!(self.kind, LoopKind::Wrapping { .. })
    }

    /// Returns the periodic direction this loop spans, if any.
    pub fn wrapping_axis(&self) -> Option<crate::geometry::Axis2> {
        self.kind.wrapped_axis()
    }

    /// Returns the same face loop with the opposite traversal orientation.
    ///
    /// Walking the other way round means visiting the darts in reverse
    /// and reading each from its far end, which is its `alpha0` partner.
    pub fn reversed(&self) -> Self {
        let darts = self
            .darts
            .iter()
            .rev()
            .map(|&dart| self.model.alpha(Dim::Zero, dart))
            .collect();
        Self::new(self.model, darts, self.kind)
    }
}

/// A domain-level face view with stable identity and contextual orientation.
///
/// A face is a surface region backed by a stored [`FaceAttr`]. Its loops are
/// a list of kinded loops: usually an outer loop and holes, but a ring face —
/// a cylinder wall — is bounded by two wrapping loops and has no outer loop.
///
/// The view carries a *sense* rather than a dart. A dart would record how the
/// face was reached, but no use of it here locates anything: each one either
/// asks which way round the view is, or flips a stored loop seed with `alpha0`.
/// Resolving that question once at construction removes the locator, and lets a
/// face be viewed whether or not it owns a dart to be reached by.
///
/// The sense is relative to the default orientation defined by the outer loop
/// stored in [`FaceAttr`]. Opposite volume-side uses of a sewn face
/// therefore share one [`FaceKey`] while producing oppositely oriented views.
pub struct Face<'g, P: Payload = StandardPayload> {
    model: &'g Model<P>,
    key: FaceKey,
    sense: Orientation,
}

impl<'g, P: Payload> Clone for Face<'g, P> {
    fn clone(&self) -> Self {
        Self {
            model: self.model,
            key: self.key,
            sense: self.sense,
        }
    }
}

impl<'g, P: Payload> Face<'g, P> {
    /// Creates a face view with the default (`Same`) orientation.
    pub fn new(model: &'g Model<P>, key: FaceKey) -> Self {
        Self {
            model,
            key,
            sense: Orientation::Same,
        }
    }

    /// Creates a face view from a dart, resolving the face key and orientation
    /// relative to the face's stored default direction.
    ///
    /// Returns `None` if the dart does not belong to a registered face.
    pub fn from_dart(model: &'g Model<P>, dart: Dart) -> Option<Self> {
        let key = model.cell_key::<Cell2>(dart)?;
        let sense = model.face_orientation_at_dart(key, dart);
        Some(Self { model, key, sense })
    }

    /// Returns the stored face attribute.
    ///
    /// # Panics
    ///
    /// Panics if the key is not present in the map.
    fn attr(&self) -> &'g FaceAttr<P::F> {
        self.model.face_attr_unchecked(self.key)
    }

    /// Returns this face's stored loop definitions.
    ///
    /// The seeds locate each loop's stored direction, which pcurve keys and
    /// boundary indices are written against; [`Self::loops`] answers what the
    /// map says bounds the face.
    pub(crate) fn attr_loops(&self) -> &'g [crate::topology::attributes::LoopDefinition] {
        self.attr().loops()
    }

    /// Returns the stable key of this face.
    pub fn key(&self) -> FaceKey {
        self.key
    }

    /// Returns this view's orientation relative to the face's stored default.
    pub fn sense(&self) -> Orientation {
        self.sense
    }

    /// Returns a boundary dart carrying this face view's contextual orientation.
    ///
    /// This is the face's seed loop — the outer loop when it has one, its first
    /// loop otherwise — `alpha0`-flipped when the view is reversed, so it
    /// round-trips through [`Self::from_dart`] to an identical view.
    ///
    /// A face that bounds nothing -- a whole sphere, a whole torus -- still has
    /// one: it occupies a raw 2-cell like any other face, and this reads it.
    /// What such a face has none of is a *loop*.
    pub fn dart(&self) -> Dart {
        self.oriented_seed(self.attr().seed())
    }

    /// Returns a new face view with the opposite orientation.
    pub fn reversed(&self) -> Self {
        Self {
            model: self.model,
            key: self.key,
            sense: self.sense.flip(),
        }
    }

    /// Reads a stored loop seed in this view's orientation.
    fn oriented_seed(&self, seed: Dart) -> Dart {
        match self.sense {
            Orientation::Same => seed,
            Orientation::Reversed => self.model.alpha(Dim::Zero, seed),
        }
    }

    /// Returns the face loop whose walk passes through `seed`.
    ///
    /// # Panics
    ///
    /// Panics if `seed` is not a boundary dart of this face.
    pub fn loop_from_seed(&self, seed: Dart) -> Loop<'g, P> {
        self.loops()
            .into_iter()
            .find(|boundary| {
                boundary.darts().any(|dart| {
                    self.model.cell_representative(dart, Dim::One)
                        == self.model.cell_representative(seed, Dim::One)
                })
            })
            .expect("face loop seed must lie on a boundary of its face")
    }

    /// Returns every raw dart this face covers, cuts included.
    ///
    /// This is the face as the map holds it, which is what copying or merging
    /// one has to move; [`Self::loops`] answers the different question of what
    /// bounds it.
    ///
    /// Asked of the involutions alone, never of the classification. Copying a
    /// face is something builders do half way through an edit, where the
    /// classification is allowed to be inconsistent and a region walk has
    /// nothing to report — and where a wrong answer would silently copy part of
    /// a face. Every 2-cell the face's own loops touch is the whole of it,
    /// whether the scaffold joins them into one cell or not yet.
    pub(crate) fn region_darts(&self) -> Vec<Dart> {
        let mut seen = HashSet::new();
        let mut darts = Vec::new();
        // Every 2-cell the face's own loops touch, taken from the involutions
        // alone. Copying a face happens half way through an edit, where the
        // classification is allowed to be inconsistent and a boundary walk has
        // nothing to report -- and where a wrong answer silently copies part of
        // a face. A cut the face owns lies in the same 2-cell and travels with
        // it without being asked for separately.
        let within = self.model.topology().orbit_indices(Dim::Two);
        for loop_ in self.attr().loops() {
            for dart in self.model.topology().orbit(loop_.seed(), within.clone()) {
                if seen.insert(dart) {
                    darts.push(dart);
                }
            }
        }
        darts
    }

    /// Walks this face's boundary and returns one dart list per closed loop.
    ///
    /// The walk is the authority on which boundaries the face has: it reads the
    /// frontier of the face's own region and turns across every cut the face
    /// owns, so a bridge to a hole or a periodic seam separates the loops
    /// instead of joining them.
    fn boundary_walks(&self) -> Vec<Vec<Dart>> {
        let anchor = self.attr().seed();
        let ownership = self.model.embedding_index();
        // A face that cannot be walked is a face whose scaffold does not hold
        // together, and answering "no boundary" would hand the caller a shape
        // it never built. Say which face and why instead.
        let region = recover_region(
            self.model.topology(),
            ownership,
            EntityOwner::Face(self.key),
            anchor,
        )
        .unwrap_or_else(|error| {
            panic!("face {:?} covers no recoverable region: {error}", self.key)
        });
        boundary_cycles(self.model.topology(), ownership, &region)
            .unwrap_or_else(|error| panic!("face {:?} has no walkable boundary: {error}", self.key))
            .into_iter()
            .map(|cycle| cycle.darts().to_vec())
            .collect()
    }

    /// Returns the outer boundary loop of the face, if it has one.
    ///
    /// A ring face — a cylinder wall — is bounded by wrapping loops and has no
    /// outer loop at all, so a caller that needs the whole boundary should
    /// reach for [`Self::loops`] rather than treat this as infallible.
    pub fn outer_loop(&self) -> Option<Loop<'g, P>> {
        self.loops().into_iter().find(Loop::is_outer)
    }

    /// Returns every inner boundary loop of the face.
    ///
    /// Inner loops represent holes in the face region.
    pub fn inner_loops(&self) -> Vec<Loop<'g, P>> {
        self.loops().into_iter().filter(Loop::is_inner).collect()
    }

    /// Returns all boundary loops, outer first when there is one.
    ///
    /// Which boundaries exist is derived from the map; what each one *means* in
    /// the face's parameter domain is not derivable from connectivity and is
    /// read from the stored [`LoopDefinition`] whose seed the walk passes
    /// through.
    pub fn loops(&self) -> Vec<Loop<'g, P>> {
        let definitions = self.attr().loops();
        // Each stored definition describes one loop, so the pairing is
        // one-to-one: a definition already claimed cannot describe a second
        // cycle, and letting it would leave another cycle unnamed and silently
        // read as an outer boundary.
        let mut claimed = vec![false; definitions.len()];
        let mut loops: Vec<Loop<'g, P>> = self
            .boundary_walks()
            .into_iter()
            .map(|darts| {
                let matched = definitions
                    .iter()
                    .enumerate()
                    .position(|(index, definition)| {
                        !claimed[index]
                            && darts.iter().any(|&dart| {
                                self.model.cell_representative(dart, Dim::One)
                                    == self.model.cell_representative(definition.seed(), Dim::One)
                            })
                    });
                let kind = match matched {
                    Some(index) => {
                        claimed[index] = true;
                        definitions[index].kind()
                    }
                    None => LoopKind::Outer,
                };
                let boundary = Loop::new(self.model, darts, kind);
                match self.sense {
                    Orientation::Same => boundary,
                    Orientation::Reversed => boundary.reversed(),
                }
            })
            .collect();
        loops.sort_by_key(|boundary| boundary.kind() != LoopKind::Outer);
        loops
    }

    /// Returns all boundary edges of the face.
    ///
    /// Edges are returned by loop order: all outer-loop edges first, followed
    /// by each inner loop's edges in storage order.
    pub fn edges(&self) -> Vec<Edge<'g, P>> {
        let mut edges = Vec::new();
        for loop_ in self.loops() {
            edges.extend(loop_.edges());
        }
        edges
    }

    /// Returns all boundary vertices of the face.
    ///
    /// Vertices are returned by loop order and are not globally deduplicated
    /// across separate loops.
    pub fn vertices(&self) -> Vec<Vertex<'g, P>> {
        let mut vertices = Vec::new();
        for loop_ in self.loops() {
            vertices.extend(loop_.vertices());
        }
        vertices
    }

    /// Returns the geometric support surface of the face.
    pub fn surface(&self) -> &Surface {
        &self.attr().surface
    }

    /// Evaluates the face's support surface at `(u, v)`.
    ///
    /// This does not test the face's trimming loops. The returned point is
    /// therefore defined even when `(u, v)` lies outside the outer loop or
    /// inside a hole.
    pub fn point_at(&self, u: f64, v: f64) -> Point3 {
        self.attr().surface.point_at(u, v)
    }

    /// Returns the support point at the middle of the surface's own domain.
    ///
    /// A boundaryless face has no vertex to name a point on it, so a caller
    /// that only needs *some* point of the face reads one off the surface.
    /// Returns `None` when the domain is unbounded in either direction, which
    /// no closed surface is.
    pub(crate) fn domain_center(&self) -> Option<Point3> {
        let (u, v) = self.surface().domain();
        (u.is_finite() && v.is_finite()).then(|| {
            self.point_at(
                u.at(Fraction::new(0.5)).value(),
                v.at(Fraction::new(0.5)).value(),
            )
        })
    }

    /// Approximates this oriented face's signed tetrahedral volume contribution.
    ///
    /// Signed parameter-space triangle fans include concave boundaries and holes.
    /// Curved triangles are subdivided on the support surface; this is an
    /// orientation estimate, not a certified mass-property calculation.
    pub(crate) fn signed_volume_contribution(&self, reference: Point3) -> Option<f64> {
        // A face nothing encloses spans its whole support, and any loop it does
        // carry is a hole in that. Fanning its loops alone would measure the
        // bite taken out of a torus instead of what is left of it — and measure
        // it with the wrong sign, a hole being wound against the face. Starting
        // from the support and letting each hole's own fan take its region back
        // off is the same statement the bounded case makes, from the other end.
        let enclosed = self
            .loops()
            .iter()
            .any(|boundary| !matches!(boundary.kind(), LoopKind::Inner));
        let mut volume = match enclosed {
            true => 0.0,
            false => self.boundaryless_signed_volume(reference)?,
        };
        if self.attr().is_boundaryless() {
            return volume.is_finite().then_some(volume);
        }
        let planar = matches!(self.surface(), Surface::Plane(_));
        for boundary in self.loops() {
            let mut uvs = Vec::new();
            for edge in boundary.edges() {
                let curve = self.pcurve(edge.dart())?;
                // A straight pcurve is a straight 3D segment only on a plane.
                // Elsewhere -- a sphere's seam meridian, say -- one sample per
                // edge leaves the loop with too few points to span a fan at
                // all, and the face contributes no volume at all.
                let count = if planar && matches!(curve.curve(), Curve2::Line(_)) {
                    1
                } else {
                    32
                };
                uvs.extend(curve.sample(count).into_iter().take(count));
            }
            let origin = *uvs.first()?;
            for pair in uvs[1..].windows(2) {
                let count = if planar { 1 } else { 16 };
                let point = |i: usize, j: usize| {
                    let uv = origin
                        + (pair[0] - origin) * (i as f64 / count as f64)
                        + (pair[1] - origin) * (j as f64 / count as f64);
                    self.point_at(uv.x, uv.y) - reference
                };
                for i in 0..count {
                    for j in 0..count - i {
                        let (a, b, c) = (point(i, j), point(i + 1, j), point(i, j + 1));
                        volume += a.dot(&b.cross(&c)) / 6.0;
                        if i + j + 1 < count {
                            volume += b.dot(&point(i + 1, j + 1).cross(&c)) / 6.0;
                        }
                    }
                }
            }
        }
        volume.is_finite().then_some(volume)
    }

    /// Signed tetrahedral volume of a face that covers its whole support.
    ///
    /// With no loops there is no boundary to fan from, and none is needed: the
    /// surface's own domain *is* the region, so the integral runs over a grid
    /// of it. The face's sense, which on a bounded face is carried by the
    /// direction its pcurves run, has to be applied here explicitly — there are
    /// no pcurves to carry it.
    fn boundaryless_signed_volume(&self, reference: Point3) -> Option<f64> {
        /// Grid cells per parameter direction. The integral converges on a
        /// sign, not on a mass property, so a coarse grid is enough.
        const STEPS: usize = 24;

        let (u_span, v_span) = self.surface().domain();
        if !u_span.is_finite() || !v_span.is_finite() {
            return None;
        }
        let corner = |i: usize, j: usize| {
            let u = u_span.at(Fraction::new(i as f64 / STEPS as f64));
            let v = v_span.at(Fraction::new(j as f64 / STEPS as f64));
            self.point_at(u.value(), v.value()) - reference
        };
        let mut volume = 0.0;
        for i in 0..STEPS {
            for j in 0..STEPS {
                let (a, b, c, d) = (
                    corner(i, j),
                    corner(i + 1, j),
                    corner(i + 1, j + 1),
                    corner(i, j + 1),
                );
                volume += a.dot(&b.cross(&c)) / 6.0;
                volume += a.dot(&c.cross(&d)) / 6.0;
            }
        }
        let volume = match self.sense {
            Orientation::Same => volume,
            Orientation::Reversed => -volume,
        };
        volume.is_finite().then_some(volume)
    }

    /// Returns the oriented face normal at a surface parameter.
    ///
    /// Counter-clockwise outer-loop pcurves keep the support-surface normal;
    /// clockwise outer-loop pcurves flip it. If the winding cannot be sampled,
    /// the support-surface normal is returned unchanged. Like [`Self::point_at`],
    /// this does not test whether `(u, v)` belongs to the trimmed face region.
    ///
    /// A face with no boundary at all is not a winding that failed to sample:
    /// it covers a closed support and has no loop to read, so which side is out
    /// is what [`Self::sense`] says and nothing else does.
    pub fn normal_at(&self, u: f64, v: f64) -> UnitVector3<f64> {
        let surface_normal = self.attr().surface.normal_at(u, v);
        let flipped = match self.boundary_signed_area() {
            Some(area) => area < -LINEAR_TOLERANCE,
            None => self.attr().loops().is_empty() && self.sense == Orientation::Reversed,
        };
        if flipped {
            -surface_normal
        } else {
            surface_normal
        }
    }

    /// Signed area of the face's outer boundary, read on a synthesized domain.
    ///
    /// A ring face's loops carry no winding of their own — each is a line
    /// exactly one period long — so the domain closes them across its own cut
    /// and the winding is read from the closed result. That is the same
    /// rectangle a stored seam used to spell out, computed rather than
    /// recorded.
    pub(crate) fn boundary_signed_area(&self) -> Option<f64> {
        let realization = self
            .model
            .realize_face(self.key, self.sense, RealizationPurpose::Geometry)
            .ok()?;
        let points = realization
            .domain()
            .loops()
            .first()?
            .polyline(BOUNDARY_WINDING_SAMPLES);
        (!points.is_empty()).then(|| signed_area(&points))
    }

    /// Returns the user payload attached to this face.
    pub fn data(&self) -> &P::F {
        self.attr().data()
    }

    /// Returns the pcurve assigned to a boundary dart, if present.
    ///
    /// The pcurve is expressed in this face's support-surface parameter space.
    /// The lookup first tries `dart` directly, then its `alpha0` and `alpha2`
    /// partners, so callers may pass any dart from the edge orbit. The pcurve
    /// key is a profile boundary dart of the face, which may differ from the
    /// edge's default orientation dart when edges are shared between faces.
    ///
    /// The returned pcurve respects the face's current orientation: if the
    /// face is reversed relative to default, the pcurve is reversed.
    pub fn pcurve(&self, dart: Dart) -> Option<TrimmedCurve2> {
        let attr = self.attr();
        let g = self.model;
        let candidates = [dart, g.alpha(Dim::Zero, dart), g.alpha(Dim::Two, dart)];
        let cached = candidates.iter().find_map(|&d| attr.pcurves.get(&d));
        cached.cloned().map(|pc| match self.sense {
            Orientation::Same => pc,
            Orientation::Reversed => pc.reversed(),
        })
    }
}

fn signed_area(points: &[Point2]) -> f64 {
    if points.len() < 3 {
        return 0.0;
    }

    0.5 * points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .take(points.len())
        .map(|(a, b)| a.x * b.y - b.x * a.y)
        .sum::<f64>()
}

impl<P: Payload> MergeTopology<P> for Face<'_, P> {
    fn merge_topology(&self) -> TopologyMerge<'_, P> {
        // Every dart the face covers, not just the ones its boundary walk
        // emits: a walk names one dart per oriented edge, while copying a face
        // has to carry its whole raw region, cuts and all.
        TopologyMerge::new(self.model, self.region_darts(), self.dart())
    }
}
