use std::borrow::Cow;
use std::collections::HashMap;
use std::collections::VecDeque;

use nalgebra::UnitVector3;

use super::closed::Closed;
use super::edge::Edge;
use super::gmap::{Dart, Dim, GMap, MergeTopology, TopologyMerge};
use super::payload::{Payload, StandardPayload};
use super::profile::{Loop, Profile};
use super::vertex::Vertex;
use crate::geometry::dim2::curves::Curve2;
use crate::geometry::{LINEAR_TOLERANCE, Point2, Point3, Surface};
use crate::topology::attributes::{FaceAttr, FacetAttr};
use crate::topology::shape_keys::{FaceKey, FacetKey};

/// An oriented domain-level face view.
///
/// A face resolves a stored [`FaceAttr`] for connectivity and a shared
/// [`FacetAttr`] for geometry.
pub struct Face<'g, P: Payload = StandardPayload> {
    gmap: &'g GMap<P>,
    key: FaceKey,
    attr: &'g FaceAttr,
    facet_attr: &'g FacetAttr<P::F>,
}

impl<'g, P: Payload> Clone for Face<'g, P> {
    fn clone(&self) -> Self {
        Self {
            gmap: self.gmap,
            key: self.key,
            attr: self.attr,
            facet_attr: self.facet_attr,
        }
    }
}

impl<'g, P: Payload> Face<'g, P> {
    pub(crate) fn new(
        gmap: &'g GMap<P>,
        key: FaceKey,
        attr: &'g FaceAttr,
        facet_attr: &'g FacetAttr<P::F>,
    ) -> Self {
        Self {
            gmap,
            key,
            attr,
            facet_attr,
        }
    }

    /// Returns the stable key of this oriented face occurrence.
    pub fn key(&self) -> FaceKey {
        self.key
    }

    /// Returns the shared geometric facet key.
    pub fn facet_key(&self) -> FacetKey {
        self.attr.facet
    }

    /// Returns the oriented outer boundary loop.
    pub fn outer_loop(&self) -> Loop<'g, P> {
        Closed::new_unchecked(Profile::new(self.gmap, self.attr.outer_loop))
    }

    /// Returns the oriented inner boundary loops.
    pub fn inner_loops(&self) -> Vec<Loop<'g, P>> {
        self.attr
            .inner_loops
            .iter()
            .map(|dart| Closed::new_unchecked(Profile::new(self.gmap, *dart)))
            .collect()
    }

    /// Returns all boundary loops, outer first.
    pub fn loops(&self) -> Vec<Loop<'g, P>> {
        let mut loops = vec![self.outer_loop()];
        loops.extend(self.inner_loops());
        loops
    }

    /// Returns all boundary edges in loop order.
    pub fn edges(&self) -> Vec<Edge<'g, P>> {
        self.loops()
            .into_iter()
            .flat_map(|loop_| loop_.edges())
            .collect()
    }

    /// Returns all boundary vertices in loop order.
    pub fn vertices(&self) -> Vec<Vertex<'g, P>> {
        self.loops()
            .into_iter()
            .flat_map(|loop_| loop_.vertices())
            .collect()
    }

    /// Returns the shared support surface.
    pub fn surface(&self) -> &Surface {
        &self.facet_attr.surface
    }

    /// Evaluates the support surface at `(u, v)`.
    pub fn point_at(&self, u: f64, v: f64) -> Point3 {
        self.surface().point_at(u, v)
    }

    /// Returns the face normal oriented by the outer-loop pcurve winding.
    pub fn normal_at(&self, u: f64, v: f64) -> UnitVector3<f64> {
        let normal = self.surface().normal_at(u, v);
        let canonical = self.canonical_outer_dart();
        let canonical_reversed = canonical
            .and_then(|dart| self.loop_signed_area(dart))
            .is_some_and(|area| area < -LINEAR_TOLERANCE);
        let occurrence_reversed = canonical
            .and_then(|dart| orientation_parity(self.gmap, dart, self.attr.outer_loop))
            .unwrap_or(false);
        if canonical_reversed ^ occurrence_reversed {
            -normal
        } else {
            normal
        }
    }

    fn canonical_outer_dart(&self) -> Option<Dart> {
        let dart = self.attr.outer_loop;
        let alpha3 = self.gmap.alpha(Dim::Three, dart);
        let alpha0 = self.gmap.alpha(Dim::Zero, dart);
        let alpha30 = self.gmap.alpha(Dim::Three, alpha0);
        [dart, alpha3, alpha0, alpha30]
            .into_iter()
            .find(|candidate| self.facet_attr.pcurves.contains_key(candidate))
    }

    fn loop_signed_area(&self, dart: Dart) -> Option<f64> {
        let mut points = Vec::new();
        for edge in Profile::new(self.gmap, dart).edges() {
            let samples = self.pcurve(edge.dart)?.sample(8);
            let count = samples.len();
            points.extend(samples.into_iter().take(count.saturating_sub(1)));
        }
        (!points.is_empty()).then(|| signed_area(&points))
    }

    /// Returns the user payload attached to the shared facet.
    pub fn data(&self) -> &P::F {
        &self.facet_attr.data
    }

    /// Returns all directed pcurves stored on the shared facet.
    pub fn pcurves(&self) -> &HashMap<Dart, Curve2> {
        &self.facet_attr.pcurves
    }

    /// Returns the pcurve assigned to a boundary dart.
    ///
    /// Alpha3-related darts share a pcurve. Alpha0-related darts receive the
    /// reversed pcurve so its parameter direction follows the requested dart.
    pub fn pcurve(&self, dart: Dart) -> Option<Cow<'g, Curve2>> {
        if let Some(curve) = self.facet_attr.pcurves.get(&dart) {
            return Some(Cow::Borrowed(curve));
        }

        let opposite = self.gmap.alpha(Dim::Three, dart);
        if let Some(curve) = self.facet_attr.pcurves.get(&opposite) {
            return Some(Cow::Owned(curve.reversed()));
        }

        let reversed = self.gmap.alpha(Dim::Zero, dart);
        if let Some(curve) = self.facet_attr.pcurves.get(&reversed) {
            return Some(Cow::Owned(curve.reversed()));
        }

        let reversed_opposite = self.gmap.alpha(Dim::Three, reversed);
        self.facet_attr
            .pcurves
            .get(&reversed_opposite)
            .map(Cow::Borrowed)
    }

    pub(crate) fn pcurve_source_dart(&self, dart: Dart) -> Option<Dart> {
        let opposite = self.gmap.alpha(Dim::Three, dart);
        let reversed = self.gmap.alpha(Dim::Zero, dart);
        let reversed_opposite = self.gmap.alpha(Dim::Three, reversed);
        [dart, opposite, reversed, reversed_opposite]
            .into_iter()
            .find(|candidate| self.facet_attr.pcurves.contains_key(candidate))
    }
}

fn orientation_parity<P: Payload>(
    gmap: &GMap<P>,
    reference: Dart,
    candidate: Dart,
) -> Option<bool> {
    let mut parity = vec![None; gmap.dart_count()];
    let mut queue = VecDeque::from([reference]);
    parity[reference.id()] = Some(false);

    while let Some(dart) = queue.pop_front() {
        let current = parity[dart.id()]?;
        if dart == candidate {
            return Some(current);
        }
        for dim in [Dim::Zero, Dim::One, Dim::Three] {
            let linked = gmap.alpha(dim, dart);
            if linked == dart {
                continue;
            }
            let linked_parity = !current;
            match parity[linked.id()] {
                Some(existing) if existing != linked_parity => return None,
                Some(_) => {}
                None => {
                    parity[linked.id()] = Some(linked_parity);
                    queue.push_back(linked);
                }
            }
        }
    }
    None
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
        TopologyMerge::new(self.gmap, darts, self.attr.outer_loop)
    }
}
