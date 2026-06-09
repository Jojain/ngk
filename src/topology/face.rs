use super::closed::Closed;
use super::edge::Edge;
use super::gmap::Dart;
use super::gmap::GMap;
use super::gmap::{MergeTopology, TopologyMerge};
use super::payload::{Payload, StandardPayload};
use super::profile::{Loop, Profile};
use super::vertex::Vertex;
use crate::geometry::Surface;
use crate::geometry::dim2::curves::Curve2;
use crate::geometry::{LINEAR_TOLERANCE, Point2};
use crate::topology::attributes::FaceAttr;
use crate::topology::gmap::Cell2;
use crate::topology::shape_keys::FaceKey;
use nalgebra::UnitVector3;

/// A domain-level face view.
///
/// A face is a surface region backed by a stored [`FaceAttr`]. It has one
/// outer boundary loop and zero or more inner loops for holes. This differs
/// from [`Facet`](crate::topology::facet::Facet), which is the raw gmap 2-cell.
pub struct Face<'g, P: Payload = StandardPayload> {
    gmap: &'g GMap<P>,
    attr: &'g FaceAttr<P::F>,
}

impl<'g, P: Payload> Clone for Face<'g, P> {
    fn clone(&self) -> Self {
        Self {
            gmap: self.gmap,
            attr: self.attr,
        }
    }
}

impl<'g, P: Payload> Face<'g, P> {
    /// Creates a face view from a stored face attribute.
    pub fn new(gmap: &'g GMap<P>, attr: &'g FaceAttr<P::F>) -> Self {
        Self { gmap, attr }
    }

    /// Returns the stable key of this face in the source map.
    ///
    /// # Panics
    ///
    /// Panics if the face's outer loop is not registered in the map's face
    /// index.
    pub fn key(&self) -> FaceKey {
        *self
            .gmap
            .attribute::<Cell2>(self.attr.outer_loop)
            .expect("face view must have a registered face key")
    }

    /// Returns the outer boundary loop of the face.
    ///
    /// The loop is trusted as closed because face attributes are created from
    /// closed boundary profiles.
    pub fn outer_loop(&self) -> Loop<'g, P> {
        let d = self.attr.outer_loop;
        Closed::new_unchecked(Profile::new(self.gmap, d))
    }

    /// Returns every inner boundary loop of the face.
    ///
    /// Inner loops represent holes in the face region. The returned order is
    /// the storage order from the face attribute.
    pub fn inner_loops(&self) -> Vec<Loop<'g, P>> {
        self.attr
            .inner_loops
            .iter()
            .map(|d| Closed::new_unchecked(Profile::new(self.gmap, *d)))
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
        &self.attr.surface
    }

    /// Returns the oriented face normal at a surface parameter.
    ///
    /// Counter-clockwise outer-loop pcurves keep the support-surface normal;
    /// clockwise outer-loop pcurves flip it. If the winding cannot be sampled,
    /// the support-surface normal is returned unchanged.
    pub fn normal_at(&self, u: f64, v: f64) -> UnitVector3<f64> {
        let normal = self.attr.surface.normal_at(u, v);
        match self.outer_loop_signed_area() {
            Some(area) if area < -LINEAR_TOLERANCE => -normal,
            _ => normal,
        }
    }

    fn outer_loop_signed_area(&self) -> Option<f64> {
        let points = self.sample_loop_pcurves(&self.outer_loop())?;
        Some(signed_area(&points))
    }

    fn sample_loop_pcurves(&self, loop_: &Loop<'_, P>) -> Option<Vec<Point2>> {
        let mut points = Vec::new();
        for edge in loop_.edges() {
            let samples = self.pcurve(edge.dart)?.sample(8);
            let n = samples.len();
            points.extend(samples.into_iter().take(n.saturating_sub(1)));
        }
        (!points.is_empty()).then_some(points)
    }

    /// Returns the user payload attached to this face.
    pub fn data(&self) -> &P::F {
        &self.attr.data
    }

    /// Returns the pcurve assigned to a boundary dart, if present.
    ///
    /// The pcurve is expressed in this face's support-surface parameter space.
    pub fn pcurve(&self, dart: Dart) -> Option<&Curve2> {
        self.attr.pcurves.get(&dart)
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
        TopologyMerge::new(self.gmap, darts, self.attr.outer_loop)
    }
}
