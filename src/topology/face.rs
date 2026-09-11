use super::closed::Closed;
use super::edge::Edge;
use super::gmap::{Cell2, Dart, Dim, GMap, MergeTopology, TopologyMerge};
use super::orientation::Orientation;
use super::payload::{Payload, StandardPayload};
use super::profile::{Loop, Profile};
use super::vertex::Vertex;
use crate::geometry::Surface;
use crate::geometry::dim2::curves::Curve2;
use crate::geometry::dim2::trimmed::TrimmedCurve2;
use crate::geometry::{LINEAR_TOLERANCE, Point2, Point3};
use crate::topology::attributes::{FaceAttr, FaceBoundary};
use crate::topology::chart::Chart;
use crate::topology::shape_keys::FaceKey;
use nalgebra::UnitVector3;

/// Samples per pcurve used to read a boundary's winding.
///
/// The winding only needs a sign, never a measurement, so a handful of
/// samples per pcurve settles it at any model scale.
const BOUNDARY_WINDING_SAMPLES: usize = 8;

/// A domain-level face view with stable identity and contextual orientation.
///
/// A face is a surface region backed by a stored [`FaceAttr`]. Its boundary is
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
/// stored in [`FaceAttr::boundary`]. Opposite volume-side uses of a sewn face
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
    pub fn dart(&self) -> Dart {
        self.oriented_seed(self.attr().boundary.seed_unchecked())
    }

    /// Returns the stored boundary of this face: its loop seeds and their kinds.
    ///
    /// The seeds are as stored, in the face's default orientation. Read one in
    /// this view's orientation with [`Self::loop_from_seed`].
    pub fn boundary(&self) -> &'g FaceBoundary {
        &self.attr().boundary
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

    /// Returns the loop seeded at `seed`, read in this view's orientation.
    ///
    /// The loop is trusted as closed because face attributes are created from
    /// closed boundary profiles. A wrapping loop is closed too — on the
    /// periodic quotient rather than in parameter space.
    pub fn loop_from_seed(&self, seed: Dart) -> Loop<'g, P> {
        Closed::new_unchecked(
            Profile::from_dart(self.gmap, self.oriented_seed(seed))
                .expect("face loop must have a registered profile"),
        )
    }

    /// Returns the outer boundary loop of the face, if it has one.
    ///
    /// A ring face — a cylinder wall — is bounded by wrapping loops and has no
    /// outer loop at all, so a caller that needs the whole boundary should
    /// reach for [`Self::loops`] rather than treat this as infallible.
    pub fn outer_loop(&self) -> Option<Loop<'g, P>> {
        self.attr()
            .boundary
            .outer()
            .map(|seed| self.loop_from_seed(seed))
    }

    /// Returns every inner boundary loop of the face.
    ///
    /// Inner loops represent holes in the face region. The returned order is
    /// the storage order from the face attribute.
    pub fn inner_loops(&self) -> Vec<Loop<'g, P>> {
        self.attr()
            .boundary
            .inner()
            .map(|seed| self.loop_from_seed(seed))
            .collect()
    }

    /// Returns all boundary loops, outer first when there is one.
    pub fn loops(&self) -> Vec<Loop<'g, P>> {
        self.attr()
            .boundary
            .darts()
            .map(|seed| self.loop_from_seed(seed))
            .collect()
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

    /// Approximates this oriented face's signed tetrahedral volume contribution.
    ///
    /// Signed parameter-space triangle fans include concave boundaries and holes.
    /// Curved triangles are subdivided on the support surface; this is an
    /// orientation estimate, not a certified mass-property calculation.
    pub(crate) fn signed_volume_contribution(&self, reference: Point3) -> Option<f64> {
        let planar = matches!(self.surface(), Surface::Plane(_));
        let mut volume = 0.0;
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

    /// Signed area of the face's outer boundary, read on a synthesized chart.
    ///
    /// A ring face's loops carry no winding of their own — each is a line
    /// exactly one period long — so the chart closes them across its own cut
    /// and the winding is read from the closed result. That is the same
    /// rectangle a stored seam used to spell out, computed rather than
    /// recorded.
    fn boundary_signed_area(&self) -> Option<f64> {
        let chart = Chart::of_face(self).ok()?;
        let points = chart.loops().first()?.polyline(BOUNDARY_WINDING_SAMPLES);
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
        TopologyMerge::new(self.gmap, darts, self.dart())
    }
}
