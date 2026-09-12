use super::closed::Closed;
use super::edge::Edge;
use super::gmap::{Cell2, Dart, Dim, GMap, MergeTopology, TopologyMerge};
use super::orientation::Orientation;
use super::payload::{Payload, StandardPayload};
use super::profile::Profile;
use super::vertex::Vertex;
use crate::geometry::Surface;
use crate::geometry::dim2::curves::Curve2;
use crate::geometry::dim2::trimmed::TrimmedCurve2;
use crate::geometry::{LINEAR_TOLERANCE, Point2, Point3};
use crate::topology::attributes::{FaceAttr, LoopDefinition, LoopKind};
use crate::topology::shape_keys::FaceKey;
use crate::topology::unwrapped_face_domain::UnwrappedFaceDomain;
use nalgebra::UnitVector3;
use std::ops::Deref;

/// Samples per pcurve used to read a boundary's winding.
///
/// The winding only needs a sign, never a measurement, so a handful of
/// samples per pcurve settles it at any model scale.
const BOUNDARY_WINDING_SAMPLES: usize = 8;

/// A closed profile as used by one face.
///
/// A loop combines the profile traversal with the face-domain role that gives
/// that traversal meaning: a chart-closed exterior, a hole, or a periodic
/// wrapping boundary. The role belongs here rather than on [`Profile`], because
/// it depends on this face's support and parameterization.
pub struct Loop<'a, P: Payload = StandardPayload> {
    profile: Closed<Profile<'a, P>>,
    kind: LoopKind,
}

impl<'a, P: Payload> Loop<'a, P> {
    /// Creates a loop from a closed profile and its face-domain role.
    fn new(profile: Closed<Profile<'a, P>>, kind: LoopKind) -> Self {
        Self { profile, kind }
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
    pub fn reversed(&self) -> Self {
        Self::new(self.profile.reversed(), self.kind)
    }
}

impl<'a, P: Payload> Deref for Loop<'a, P> {
    type Target = Closed<Profile<'a, P>>;

    fn deref(&self) -> &Self::Target {
        &self.profile
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
    gmap: &'g GMap<P>,
    key: FaceKey,
    sense: Orientation,
}

impl<'g, P: Payload> Clone for Face<'g, P> {
    fn clone(&self) -> Self {
        Self {
            gmap: self.gmap,
            key: self.key,
            sense: self.sense,
        }
    }
}

impl<'g, P: Payload> Face<'g, P> {
    /// Creates a face view with the default (`Same`) orientation.
    pub fn new(gmap: &'g GMap<P>, key: FaceKey) -> Self {
        Self {
            gmap,
            key,
            sense: Orientation::Same,
        }
    }

    /// Creates a face view from a dart, resolving the face key and orientation
    /// relative to the face's stored default direction.
    ///
    /// Returns `None` if the dart does not belong to a registered face.
    pub fn from_dart(gmap: &'g GMap<P>, dart: Dart) -> Option<Self> {
        let key = gmap.cell_key::<Cell2>(dart)?;
        let sense = gmap.face_orientation_at_dart(key, dart);
        Some(Self { gmap, key, sense })
    }

    /// Returns the stored face attribute.
    ///
    /// # Panics
    ///
    /// Panics if the key is not present in the map.
    fn attr(&self) -> &'g FaceAttr<P::F> {
        self.gmap.face_attr_unchecked(self.key)
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
    /// A boundaryless face has no loop, therefore no dart: it covers a closed
    /// support and touches nothing. Its orientation lives in [`Self::sense`]
    /// alone.
    pub fn dart(&self) -> Option<Dart> {
        self.attr().seed().map(|seed| self.oriented_seed(seed))
    }

    /// Returns a boundary dart carrying this face view's contextual orientation.
    ///
    /// # Panics
    ///
    /// Panics on a boundaryless face, which has no boundary dart to return.
    pub fn dart_unchecked(&self) -> Dart {
        self.dart()
            .expect("dart-backed face should have a boundary dart")
    }

    /// Returns a new face view with the opposite orientation.
    pub fn reversed(&self) -> Self {
        Self {
            gmap: self.gmap,
            key: self.key,
            sense: self.sense.flip(),
        }
    }

    /// Reads a stored loop seed in this view's orientation.
    fn oriented_seed(&self, seed: Dart) -> Dart {
        match self.sense {
            Orientation::Same => seed,
            Orientation::Reversed => self.gmap.alpha(Dim::Zero, seed),
        }
    }

    /// Returns the face loop seeded at `seed`, read in this view's orientation.
    ///
    /// The loop is trusted as closed because face attributes are created from
    /// closed boundary profiles. A wrapping loop is closed too — on the
    /// periodic quotient rather than in parameter space.
    pub fn loop_from_seed(&self, seed: Dart) -> Loop<'g, P> {
        let definition = self
            .attr()
            .loop_definition(seed)
            .expect("face loop seed must be stored on its face");
        self.loop_from_definition(definition)
    }

    /// Resolves one stored loop definition into this face's oriented loop view.
    fn loop_from_definition(&self, definition: LoopDefinition) -> Loop<'g, P> {
        let profile = Closed::new_unchecked(
            Profile::from_dart(self.gmap, self.oriented_seed(definition.seed()))
                .expect("face loop must have a registered profile"),
        );
        Loop::new(profile, definition.kind())
    }

    /// Returns the outer boundary loop of the face, if it has one.
    ///
    /// A ring face — a cylinder wall — is bounded by wrapping loops and has no
    /// outer loop at all, so a caller that needs the whole boundary should
    /// reach for [`Self::loops`] rather than treat this as infallible.
    pub fn outer_loop(&self) -> Option<Loop<'g, P>> {
        self.attr()
            .outer_seed()
            .map(|seed| self.loop_from_seed(seed))
    }

    /// Returns every inner boundary loop of the face.
    ///
    /// Inner loops represent holes in the face region. The returned order is
    /// the storage order from the face attribute.
    pub fn inner_loops(&self) -> Vec<Loop<'g, P>> {
        self.loops().into_iter().filter(Loop::is_inner).collect()
    }

    /// Returns all boundary loops, outer first when there is one.
    pub fn loops(&self) -> Vec<Loop<'g, P>> {
        let definitions = &self.attr().loops;
        let mut loops = Vec::with_capacity(definitions.len());
        if let Some(outer) = self.attr().outer_seed() {
            loops.push(self.loop_from_seed(outer));
        }
        loops.extend(
            definitions
                .iter()
                .copied()
                .filter(|definition| definition.kind() != LoopKind::Outer)
                .map(|definition| self.loop_from_definition(definition)),
        );
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
        (u.is_finite() && v.is_finite()).then(|| self.point_at(u.at(0.5), v.at(0.5)))
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
        if self.attr().is_empty() {
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
            let u = u_span.at(i as f64 / STEPS as f64);
            let v = v_span.at(j as f64 / STEPS as f64);
            self.point_at(u, v) - reference
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
    pub fn normal_at(&self, u: f64, v: f64) -> UnitVector3<f64> {
        let surface_normal = self.attr().surface.normal_at(u, v);
        match self.boundary_signed_area() {
            Some(area) if area < -LINEAR_TOLERANCE => -surface_normal,
            _ => surface_normal,
        }
    }

    /// Signed area of the face's outer boundary, read on a synthesized domain.
    ///
    /// A ring face's loops carry no winding of their own — each is a line
    /// exactly one period long — so the domain closes them across its own cut
    /// and the winding is read from the closed result. That is the same
    /// rectangle a stored seam used to spell out, computed rather than
    /// recorded.
    fn boundary_signed_area(&self) -> Option<f64> {
        let domain = UnwrappedFaceDomain::of_face(self).ok()?;
        let points = domain.loops().first()?.polyline(BOUNDARY_WINDING_SAMPLES);
        (!points.is_empty()).then(|| signed_area(&points))
    }

    /// Returns the user payload attached to this face.
    pub fn data(&self) -> &P::F {
        &self.attr().data
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
        let g = self.gmap;
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
        let mut darts = Vec::new();
        for loop_ in self.loops() {
            darts.extend(loop_.darts());
        }
        TopologyMerge::new(self.gmap, darts, self.dart_unchecked())
    }
}
