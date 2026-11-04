//! Edge discretisation: every edge is cut into samples once, and every face
//! along it meshes against exactly those samples.
//!
//! An edge and the faces it bounds meet only where they share points. So the
//! samples of an edge are a function of the edge and of the faces incident to
//! it, never of the face asking: whichever face is meshed first, and whether a
//! face is meshed alone or with its whole solid, the edge comes out cut at the
//! same parameters, at the same 3D points, bit for bit.
//!
//! Three things decide where an edge is cut:
//!
//! - its own turning: enough samples that the tangent turns through at most
//!   one `opts.curve.segments`-th of a whole turn between two of them, as
//!   [`segments_for`] measures it;
//! - each incident face's turning across it: where a surface's normal turns
//!   along the edge faster than the edge itself does -- a straight ruling of a
//!   twisted ruled surface -- the edge is bisected until the normal turns no
//!   more than the same angle between samples, so the face can be refined
//!   right up to its boundary without ever splitting it;
//! - where an incident face's unwrapped domain starts or ends a pcurve along
//!   it: a rim a ring face is cut open at is read from the cut, so the cut
//!   point is a boundary point of that face and must be one of the edge's.

use std::collections::HashMap;
use std::f64::consts::TAU;

use super::CurveOpts;
use super::curve::segments_for;
use crate::geometry::parameter::Fraction;
use crate::geometry::{LINEAR_TOLERANCE, Point3, TrimmedCurve};
use crate::model::Model;
use crate::topology::edge::Edge;
use crate::topology::face::Face;
use crate::topology::orientation::Orientation;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey, VertexKey};
use crate::topology::unwrapped_face_domain::UnwrappedFaceDomain;

/// The most samples any one edge is cut into.
const MAX_SAMPLES: usize = 16_384;
/// How many times one interval may be halved to follow an incident surface.
const MAX_BISECTIONS: u32 = 12;

/// A point of a shape that more than one face, or a face and an edge, share.
///
/// Two faces meshed apart name the same boundary point by the same identity,
/// which is what makes their meshes one closed surface once assembled.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum SharedPoint {
    /// A vertex of the shape.
    Vertex(VertexKey),
    /// The point where a closed edge with no corner closes.
    Closure(EdgeKey),
    /// The `index`th interior sample of an edge, in the edge's stored order.
    Sample(EdgeKey, u32),
    /// A point of a face's unwrapped domain that lies on no edge: on a seam
    /// the domain synthesised, or a pole. Shared by the two sides of the seam
    /// within the face, and by nothing outside it.
    Interior(FaceKey, u32),
}

/// One edge cut into samples, in the edge's stored direction.
#[derive(Debug, Clone)]
pub struct EdgeSamples {
    /// The edge's span, as stored.
    pub span: TrimmedCurve,
    /// Fractions of `span`, increasing from `0` to `1`.
    pub fractions: Vec<f64>,
    /// The 3D point at each fraction.
    pub points: Vec<Point3>,
    /// The identity of each point.
    pub ids: Vec<SharedPoint>,
    /// Whether the edge closes on itself (its two ends are one point).
    pub closed: bool,
}

/// Edge samples and face domains, computed once each.
///
/// Meshing a whole shape reuses this across faces; meshing one face builds a
/// fresh one, which arrives at the same samples because they depend only on
/// the model.
#[derive(Default)]
pub struct Discretisation {
    edges: HashMap<EdgeKey, EdgeSamples>,
    domains: HashMap<(FaceKey, Orientation), Option<UnwrappedFaceDomain>>,
}

impl Discretisation {
    /// The unwrapped domain of `face` as the view reads it -- its loops run
    /// the way its sense says -- or `None` where it cannot be read.
    pub fn domain<P: Payload>(&mut self, face: &Face<'_, P>) -> Option<&UnwrappedFaceDomain> {
        self.domains
            .entry((face.key(), face.sense()))
            .or_insert_with(|| UnwrappedFaceDomain::of_face(face).ok())
            .as_ref()
    }

    /// The samples of the edge `key`, in the edge's stored direction whichever
    /// way a caller walks it.
    pub fn edge<P: Payload>(
        &mut self,
        model: &Model<P>,
        key: EdgeKey,
        opts: CurveOpts,
    ) -> &EdgeSamples {
        if !self.edges.contains_key(&key) {
            let edge = Edge::new(model, key);
            let samples = self.discretise(model, &edge, opts);
            self.edges.insert(key, samples);
        }
        &self.edges[&key]
    }

    fn discretise<P: Payload>(
        &mut self,
        model: &Model<P>,
        edge: &Edge<'_, P>,
        opts: CurveOpts,
    ) -> EdgeSamples {
        let span = edge.trimmed_curve();
        let start = span.start();
        let end = span.end();
        let length = span.length().max(LINEAR_TOLERANCE);
        let closed = (start - end).norm() <= LINEAR_TOLERANCE.max(1e-12 * length);
        let faces = edge.faces();
        let count = segments_for(&span, opts).max(if closed { 3 } else { 1 });
        let mut fractions = (0..=count)
            .map(|index| index as f64 / count as f64)
            .collect::<Vec<_>>();

        // Every place an incident face's domain starts a pcurve along this
        // edge is a boundary point of that face.
        for face in &faces {
            let Some(domain) = self.domain(face) else { continue };
            for loop_ in domain.loops() {
                for curve in loop_.curves() {
                    let Some(of) = Edge::from_dart(model, curve.dart()) else {
                        continue;
                    };
                    if of.key() != edge.key() {
                        continue;
                    }
                    for point in [curve.span().start(), curve.span().end()] {
                        let fraction = span.parameter_at(point).value();
                        let fraction = if closed { fraction.rem_euclid(1.0) } else { fraction };
                        if (span.point_at(Fraction::new(fraction)) - point).norm()
                            <= 1e-9 * length.max(1.0)
                        {
                            fractions.push(fraction.clamp(0.0, 1.0));
                        }
                    }
                }
            }
        }
        fractions.sort_by(f64::total_cmp);
        let merge = 1e-12;
        fractions.dedup_by(|b, a| (*b - *a).abs() <= merge);
        if let Some(last) = fractions.last_mut() {
            *last = 1.0;
        }
        fractions[0] = 0.0;

        // Each incident surface turns along the edge no faster than the edge
        // is sampled.
        let step = TAU / opts.segments.max(1) as f64 * (1.0 + 1e-6);
        let normals = |fraction: f64| -> Vec<Option<nalgebra::Vector3<f64>>> {
            let point = span.point_at(Fraction::new(fraction));
            faces
                .iter()
                .map(|face| {
                    let surface = face.surface();
                    let uv = surface.param_at(point).ok()?;
                    if surface.is_degenerate_at(uv.x, uv.y) {
                        return None;
                    }
                    let (_, du, dv) = surface.point_and_derivative_at(uv.x, uv.y);
                    let normal = du.cross(&dv);
                    let norm = normal.norm();
                    (norm.is_finite() && norm > 0.0).then(|| normal / norm)
                })
                .collect()
        };
        let turns_too_far = |a: &[Option<nalgebra::Vector3<f64>>], b: &[Option<nalgebra::Vector3<f64>>]| {
            a.iter().zip(b).any(|(a, b)| match (a, b) {
                (Some(a), Some(b)) => a.dot(b).clamp(-1.0, 1.0).acos() > step,
                _ => false,
            })
        };
        if !faces.is_empty() {
            let mut refined = Vec::with_capacity(fractions.len());
            let mut stack = Vec::new();
            for pair in fractions.windows(2) {
                refined.push(pair[0]);
                stack.clear();
                stack.push((pair[0], pair[1], 0u32));
                let mut inserted = Vec::new();
                while let Some((a, b, depth)) = stack.pop() {
                    if depth >= MAX_BISECTIONS || refined.len() + inserted.len() >= MAX_SAMPLES {
                        continue;
                    }
                    if turns_too_far(&normals(a), &normals(b)) {
                        let middle = 0.5 * (a + b);
                        inserted.push(middle);
                        stack.push((a, middle, depth + 1));
                        stack.push((middle, b, depth + 1));
                    }
                }
                inserted.sort_by(f64::total_cmp);
                refined.extend(inserted);
            }
            refined.push(1.0);
            fractions = refined;
        }

        let points = fractions
            .iter()
            .map(|fraction| span.point_at(Fraction::new(*fraction)))
            .collect::<Vec<_>>();
        let vertices = edge.vertices();
        let start_id = vertex_at(&vertices, start).unwrap_or(SharedPoint::Closure(edge.key()));
        let end_id = if closed {
            start_id
        } else {
            vertex_at(&vertices, end).unwrap_or(SharedPoint::Closure(edge.key()))
        };
        let last = fractions.len() - 1;
        let ids = (0..fractions.len())
            .map(|index| match index {
                0 => start_id,
                _ if index == last => end_id,
                _ => SharedPoint::Sample(edge.key(), index as u32),
            })
            .collect();
        // The ends are the stored corner points, exactly.
        let mut points = points;
        points[0] = vertex_point(&vertices, start).unwrap_or(start);
        points[last] = if closed {
            points[0]
        } else {
            vertex_point(&vertices, end).unwrap_or(end)
        };
        EdgeSamples {
            span,
            fractions,
            points,
            ids,
            closed,
        }
    }
}

/// The vertex of `vertices` at `point`, nearest first.
fn nearest_vertex<'a, 'g, P: Payload>(
    vertices: &'a [crate::topology::vertex::Vertex<'g, P>],
    point: Point3,
) -> Option<&'a crate::topology::vertex::Vertex<'g, P>> {
    vertices.iter().min_by(|a, b| {
        (*a.point() - point)
            .norm()
            .total_cmp(&(*b.point() - point).norm())
    })
}

fn vertex_at<P: Payload>(
    vertices: &[crate::topology::vertex::Vertex<'_, P>],
    point: Point3,
) -> Option<SharedPoint> {
    nearest_vertex(vertices, point).map(|vertex| SharedPoint::Vertex(vertex.key()))
}

fn vertex_point<P: Payload>(
    vertices: &[crate::topology::vertex::Vertex<'_, P>],
    point: Point3,
) -> Option<Point3> {
    nearest_vertex(vertices, point).map(|vertex| *vertex.point())
}
