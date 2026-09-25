//! A selected edge read as the crease between two oriented surfaces.
//!
//! The closed-form rows read a face's normal once, because a plane has one.
//! Everything else reads it where it is needed: at the foot of a point on the
//! face's support, turned outward by the face's sense. That, the direction
//! each face runs along the edge, and the edge's span are all a general
//! section solver needs to know about the model.

use std::collections::HashSet;

use nalgebra::Vector3;

use super::super::errors::BlendError;
use super::super::network::NetworkEdge;
use crate::geometry::parameter::Fraction;
use crate::geometry::{LINEAR_TOLERANCE, Point2, Point3, Surface, TrimmedCurve};
use crate::model::Model;
use crate::topology::gmap::Dart;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey};

/// Fractions of the edge at which a crease is checked for flatness and for
/// a change of convexity.
const CONVEXITY_SAMPLES: [f64; 5] = [0.1, 0.3, 0.5, 0.7, 0.9];

/// Where a point lands on one face's support.
#[derive(Debug, Clone, Copy)]
pub(super) struct Foot {
    pub(super) uv: Point2,
    pub(super) point: Point3,
    /// The face's outward unit normal there.
    pub(super) normal: Vector3<f64>,
}

/// The crease at one point of the edge.
#[derive(Debug, Clone, Copy)]
pub(super) struct CreaseFrame {
    pub(super) point: Point3,
    /// The edge's unit tangent, in the span's direction.
    pub(super) tangent: Vector3<f64>,
    pub(super) feet: [Foot; 2],
    /// The unit direction from the edge into each face, square to the edge.
    pub(super) inward: [Vector3<f64>; 2],
}

/// A selected edge and its two faces, each read as an oriented surface.
#[derive(Debug)]
pub(super) struct Crease<'a> {
    pub(super) key: EdgeKey,
    pub(super) span: &'a TrimmedCurve,
    pub(super) surfaces: [&'a Surface; 2],
    /// `1` where a face's outward normal is its support's, `-1` where the face
    /// lies on its support reversed.
    senses: [f64; 2],
    /// `1` where a face's loop runs along the edge in the span's direction,
    /// `-1` where it runs against it.
    walks: [f64; 2],
}

impl<'a> Crease<'a> {
    /// Reads the crease of a selected edge off the model.
    pub(super) fn capture<P: Payload>(
        model: &'a Model<P>,
        edge: &'a NetworkEdge,
    ) -> Result<Self, BlendError> {
        let middle = edge.span.point_at(Fraction::new(0.5));
        let mut surfaces = Vec::with_capacity(2);
        let mut senses = [1.0; 2];
        let mut walks = [1.0; 2];
        for (index, side) in edge.sides.iter().enumerate() {
            let surface = &model.face_attr_unchecked(side.face).surface;
            let uv = surface
                .param_at(middle)
                .map_err(|_| BlendError::UnsupportedEdge {
                    edge: edge.key,
                    reason: "it cannot be located on its faces",
                })?;
            let face_normal = model.face_unchecked(side.face).normal_at(uv.x, uv.y);
            if face_normal.dot(&surface.normal_at(uv.x, uv.y)) < 0.0 {
                senses[index] = -1.0;
            }
            if !runs_along(model, side.face, side.start) {
                walks[index] = -1.0;
            }
            surfaces.push(surface);
        }
        Ok(Self {
            key: edge.key,
            span: &edge.span,
            surfaces: [surfaces[0], surfaces[1]],
            senses,
            walks,
        })
    }

    /// Where `point` lands on `side`'s support, searching from `hint` when
    /// one is given.
    pub(super) fn foot(&self, side: usize, point: Point3, hint: Option<Point2>) -> Option<Foot> {
        let surface = self.surfaces[side];
        let uv = match hint {
            Some(hint) => surface.param_near(point, hint, LINEAR_TOLERANCE),
            None => surface.param_at(point),
        }
        .ok()?;
        let normal = surface.normal_at(uv.x, uv.y).into_inner() * self.senses[side];
        Some(Foot {
            uv,
            point: surface.point_at(uv.x, uv.y),
            normal,
        })
    }

    /// The crease at fraction `t` of the edge.
    pub(super) fn frame_at(&self, t: Fraction) -> Option<CreaseFrame> {
        let point = self.span.point_at(t);
        let derivative = self.span.derivative_at(t, 1);
        if derivative.norm() <= LINEAR_TOLERANCE {
            return None;
        }
        let tangent = derivative.normalize();
        let feet = [self.foot(0, point, None)?, self.foot(1, point, None)?];
        let inward = std::array::from_fn(|side| {
            feet[side]
                .normal
                .cross(&(tangent * self.walks[side]))
                .normalize()
        });
        Some(CreaseFrame {
            point,
            tangent,
            feet,
            inward,
        })
    }

    /// Whether the solid's material lies inside the crease.
    ///
    /// Refuses a crease whose faces continue each other anywhere along it,
    /// and one that turns from convex to concave: a blend of constant sense
    /// cannot follow either.
    pub(super) fn convexity(&self) -> Result<bool, BlendError> {
        let mut convex = None;
        for fraction in CONVEXITY_SAMPLES {
            let frame =
                self.frame_at(Fraction::new(fraction))
                    .ok_or(BlendError::UnsupportedEdge {
                        edge: self.key,
                        reason: "it cannot be located on its faces",
                    })?;
            let crease = frame.inward[0].dot(&frame.feet[1].normal);
            if crease.abs() <= LINEAR_TOLERANCE.sqrt() {
                return Err(BlendError::FlatEdge { edge: self.key });
            }
            let here = crease < 0.0;
            if convex.is_some_and(|convex| convex != here) {
                return Err(BlendError::UnsupportedEdge {
                    edge: self.key,
                    reason: "it turns from convex to concave along its length",
                });
            }
            convex = Some(here);
        }
        Ok(convex.unwrap_or(true))
    }
}

/// Whether `face`'s stored loops run along `dart` in its own direction.
pub(crate) fn runs_along<P: Payload>(model: &Model<P>, face: FaceKey, dart: Dart) -> bool {
    loop_darts(model, face).contains(&dart)
}

/// Every dart `face`'s stored loops walk.
pub(crate) fn loop_darts<P: Payload>(model: &Model<P>, face: FaceKey) -> HashSet<Dart> {
    model
        .face_unchecked(face)
        .loops()
        .iter()
        .flat_map(|loop_| loop_.darts())
        .collect()
}

/// The unit direction from the edge into each face, square to `tangent`,
/// for faces whose outward normals along the edge are `normals`.
///
/// A face's interior lies to the left of its boundary as the face walks it,
/// seen from its outward normal, so the direction is read off the way each
/// face's stored loop runs along the edge.
pub(super) fn inward_directions<P: Payload>(
    model: &Model<P>,
    edge: &NetworkEdge,
    tangent: Vector3<f64>,
    normals: [Vector3<f64>; 2],
) -> [Vector3<f64>; 2] {
    std::array::from_fn(|index| {
        let side = edge.sides[index];
        let walked = if runs_along(model, side.face, side.start) {
            tangent
        } else {
            -tangent
        };
        normals[index].cross(&walked).normalize()
    })
}
