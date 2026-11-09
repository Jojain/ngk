//! The parts of a span whose image lands inside a planar face.
//!
//! The image of a span on a face's support plane is exact (see the plane
//! carrier); the face keeps of it only what is inside its trim. The image is
//! cut where it crosses the trim's boundary, and each part is kept when its
//! midpoint is on the face, the boundary included.

use super::error::ProjectionError;
use super::options::ProjectOptions;
use super::plane_carrier::{PARAMETER_EPSILON, PlaneCarrier};
use super::point_face::on_face;
use super::point_surface::surface_kind;
use crate::geometry::parameter::Fraction;
use crate::geometry::{CurveIntersectionOptions, Interval, Plane, Surface, TrimmedCurve};
use crate::model::Model;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::FaceKey;
use crate::topology::trim_domain::FaceTrimDomain;

/// A planar face: its key, its support and its trim.
pub(super) struct PlanarFace {
    pub(super) key: FaceKey,
    pub(super) plane: Plane,
    trim: FaceTrimDomain,
}

impl PlanarFace {
    /// The faces `keys` of `model`, each with its support plane.
    ///
    /// A face whose support is not a plane is refused with
    /// [`UnsupportedSurface`](ProjectionError::UnsupportedSurface): a curve's
    /// image on any other surface has no exact form here.
    pub(super) fn all<P: Payload>(
        model: &Model<P>,
        keys: &[FaceKey],
        options: &ProjectOptions,
    ) -> Result<Vec<Self>, ProjectionError> {
        keys.iter()
            .map(|&key| {
                let face = model.face_unchecked(key);
                let Surface::Plane(plane) = face.surface() else {
                    return Err(ProjectionError::UnsupportedSurface {
                        surface: surface_kind(face.surface()),
                    });
                };
                Ok(Self {
                    key,
                    plane: plane.clone(),
                    trim: FaceTrimDomain::new(&face, options.tolerance)?,
                })
            })
            .collect()
    }

    /// The parts of `span` whose image through `carrier` is inside the face,
    /// as intervals of the span's parameter in increasing order, apart from
    /// one another.
    ///
    /// `carrier` carries onto this face's plane; only the parts of the span in
    /// front of the plane have an image to clip.
    pub(super) fn cover(
        &self,
        carrier: &PlaneCarrier<'_>,
        span: &TrimmedCurve,
    ) -> Result<Vec<Interval>, ProjectionError> {
        let mut inside: Vec<Interval> = Vec::new();
        for piece in carrier
            .pieces(span)?
            .into_iter()
            .filter(|piece| piece.front)
        {
            let (_, pcurve) = carrier.project_piece(span.curve(), piece.interval)?;
            let mut cuts = vec![Fraction::START, Fraction::END];
            self.trim
                .crossings(&pcurve, CurveIntersectionOptions::default(), &mut cuts)?;
            cuts.sort_by(|a, b| a.value().total_cmp(&b.value()));
            for pair in cuts.windows(2) {
                let part = Interval::new(piece.interval.at(pair[0]), piece.interval.at(pair[1]));
                if part.length() <= PARAMETER_EPSILON {
                    continue;
                }
                let middle = Fraction::new(0.5 * (pair[0].value() + pair[1].value()));
                if on_face(&self.trim, pcurve.point_at(middle)) {
                    inside.push(part.ordered());
                }
            }
        }
        inside.sort_by(|a, b| a.start.value().total_cmp(&b.start.value()));
        let mut merged: Vec<Interval> = Vec::new();
        for part in inside {
            match merged.last_mut() {
                Some(last) if part.start.value() - last.end.value() <= PARAMETER_EPSILON => {
                    if part.end.value() > last.end.value() {
                        last.end = part.end;
                    }
                }
                _ => merged.push(part),
            }
        }
        Ok(merged)
    }
}
