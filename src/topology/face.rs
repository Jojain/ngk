use super::closed::Closed;
use super::edge::Edge;
use super::gmap::{Cell2, Dart, Dim, GMap, MergeTopology, TopologyMerge};
use super::orientation::Orientation;
use super::payload::{Payload, StandardPayload};
use super::profile::{Loop, Profile};
use super::vertex::Vertex;
use crate::geometry::Surface;
use crate::geometry::dim2::curves::Curve2;
use crate::geometry::{LINEAR_TOLERANCE, Point2, Point3};
use crate::topology::attributes::FaceAttr;
use crate::topology::shape_keys::FaceKey;
use nalgebra::UnitVector3;

/// A domain-level face view with stable identity and contextual orientation.
///
/// A face is a surface region backed by a stored [`FaceAttr`]. It has one
/// outer boundary loop and zero or more inner loops for holes.
///
/// The view's dart records how the face was reached by the current traversal.
/// Its orientation is derived relative to the default orientation defined by
/// [`FaceAttr::outer_loop`]. Opposite volume-side uses of a sewn face therefore
/// share one [`FaceKey`] while producing oppositely oriented views.
pub struct Face<'g, P: Payload = StandardPayload> {
    gmap: &'g GMap<P>,
    key: FaceKey,
    dart: Dart,
}

impl<'g, P: Payload> Clone for Face<'g, P> {
    fn clone(&self) -> Self {
        Self {
            gmap: self.gmap,
            key: self.key,
            dart: self.dart,
        }
    }
}

impl<'g, P: Payload> Face<'g, P> {
    /// Creates a face view with the default (`Same`) orientation.
    pub fn new(gmap: &'g GMap<P>, key: FaceKey) -> Self {
        let dart = gmap.face_attr_unchecked(key).outer_loop;
        Self { gmap, key, dart }
    }

    /// Creates a face view from a dart, resolving the face key and orientation
    /// relative to the face's stored default direction.
    ///
    /// Returns `None` if the dart does not belong to a registered face.
    pub fn from_dart(gmap: &'g GMap<P>, dart: Dart) -> Option<Self> {
        let key = gmap.cell_key::<Cell2>(dart)?;
        Some(Self { gmap, key, dart })
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

    /// Returns the dart carrying this face view's contextual orientation.
    pub fn dart(&self) -> Dart {
        self.dart
    }

    /// Returns a new face view with the opposite orientation.
    pub fn reversed(&self) -> Self {
        Self {
            gmap: self.gmap,
            key: self.key,
            dart: self.gmap.alpha(Dim::Zero, self.dart),
        }
    }

    /// Returns the outer boundary loop of the face.
    ///
    /// The loop is trusted as closed because face attributes are created from
    /// closed boundary profiles.
    pub fn outer_loop(&self) -> Loop<'g, P> {
        let d = self.outer_loop_dart();
        Closed::new_unchecked(
            Profile::from_dart(self.gmap, d).expect("face outer loop must have a profile"),
        )
    }

    fn outer_loop_dart(&self) -> Dart {
        let attr = self.attr();
        match self.gmap.face_orientation_at_dart(self.key, self.dart) {
            Orientation::Same => attr.outer_loop,
            Orientation::Reversed => self.gmap.alpha(Dim::Zero, attr.outer_loop),
        }
    }

    /// Returns every inner boundary loop of the face.
    ///
    /// Inner loops represent holes in the face region. The returned order is
    /// the storage order from the face attribute.
    pub fn inner_loops(&self) -> Vec<Loop<'g, P>> {
        let attr = self.attr();
        attr.inner_loops
            .iter()
            .map(|d| {
                let dart = match self.gmap.face_orientation_at_dart(self.key, self.dart) {
                    Orientation::Same => *d,
                    Orientation::Reversed => self.gmap.alpha(Dim::Zero, *d),
                };
                Closed::new_unchecked(
                    Profile::from_dart(self.gmap, dart)
                        .expect("face loop must have a registered profile"),
                )
            })
            .collect()
    }

    /// Returns all boundary loops, outer first followed by inner loops.
    pub fn loops(&self) -> Vec<Loop<'g, P>> {
        let mut loops = vec![self.outer_loop()];
        loops.extend(self.inner_loops());
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
                let count = if matches!(curve, Curve2::Line(_)) {
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
        match self.outer_loop_signed_area() {
            Some(area) if area < -LINEAR_TOLERANCE => -surface_normal,
            _ => surface_normal,
        }
    }

    fn outer_loop_signed_area(&self) -> Option<f64> {
        let points = self.sample_loop_pcurves(&self.outer_loop())?;
        Some(signed_area(&points))
    }

    fn sample_loop_pcurves(&self, loop_: &Loop<'_, P>) -> Option<Vec<Point2>> {
        let mut points = Vec::new();
        for edge in loop_.edges() {
            let samples = self.pcurve(edge.dart())?.sample(8);
            let n = samples.len();
            points.extend(samples.into_iter().take(n.saturating_sub(1)));
        }
        (!points.is_empty()).then_some(points)
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
    pub fn pcurve(&self, dart: Dart) -> Option<Curve2> {
        let attr = self.attr();
        let g = self.gmap;
        let candidates = [dart, g.alpha(Dim::Zero, dart), g.alpha(Dim::Two, dart)];
        let cached = candidates.iter().find_map(|&d| attr.pcurves.get(&d));
        cached.cloned().map(
            |pc| match self.gmap.face_orientation_at_dart(self.key, self.dart) {
                Orientation::Same => pc,
                Orientation::Reversed => pc.reversed(),
            },
        )
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
        let mut darts = self.outer_loop().darts().collect::<Vec<_>>();
        for loop_ in self.inner_loops() {
            darts.extend(loop_.darts());
        }
        TopologyMerge::new(self.gmap, darts, self.outer_loop_dart())
    }
}
