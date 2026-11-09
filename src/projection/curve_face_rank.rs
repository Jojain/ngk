//! Which planar face a span lands on, at each of its parameters.
//!
//! Each face covers some parts of the span (see [`PlanarFace::cover`]). Where
//! several cover the same parameter, one wins as it does for a point: the first
//! wall of a directional projection, the nearest point of a perpendicular one.
//! Between two planes the order of the candidates can change only where the
//! two are equally far, and that locus is itself a plane, so the span is cut
//! where it crosses those planes and one winner is read off each part.

use nalgebra::Vector3;

use super::curve_face_clip::PlanarFace;
use super::error::ProjectionError;
use super::mode::Projection;
use super::options::ProjectOptions;
use super::plane_carrier::{PARAMETER_EPSILON, PlaneCarrier, crossings_with_plane};
use super::point_face_list::nearest;
use super::result::PointProjection;
use crate::geometry::{ANGULAR_TOLERANCE, Interval, NativeParam, Plane, Point3, TrimmedCurve};

/// A part of a span and the face that wins on it, if any face covers it.
#[derive(Debug, Clone, Copy)]
pub(super) struct Part {
    /// The part, with its start below its end.
    pub(super) interval: Interval,
    /// The index of the winning face in the list.
    pub(super) face: Option<usize>,
}

/// The winner on every part of `span`, in increasing parameter, neighbours with
/// the same winner merged.
///
/// `carriers[i]` carries onto `faces[i]`, and is `None` for a face no point of
/// the span can reach.
///
/// Where the nearest points are equally far at different places over a part,
/// the answer is [`Ambiguous`](ProjectionError::Ambiguous). Equal reach of a
/// directional projection is one point, and the earlier face in the list wins.
pub(super) fn winners(
    faces: &[PlanarFace],
    carriers: &[Option<PlaneCarrier<'_>>],
    span: &TrimmedCurve,
    mode: Projection,
    options: &ProjectOptions,
) -> Result<Vec<Part>, ProjectionError> {
    let mut covers = Vec::with_capacity(faces.len());
    for (face, carrier) in faces.iter().zip(carriers) {
        covers.push(match carrier {
            Some(carrier) => face.cover(carrier, span)?,
            None => Vec::new(),
        });
    }

    let whole = span.interval().ordered();
    let mut cuts = vec![whole.start.value(), whole.end.value()];
    for cover in &covers {
        for part in cover {
            cuts.extend([part.start.value(), part.end.value()]);
        }
    }
    for first in 0..faces.len() {
        for second in first + 1..faces.len() {
            if !overlap(&covers[first], &covers[second]) {
                continue;
            }
            for plane in equal_reach_planes(&faces[first].plane, &faces[second].plane, mode) {
                cuts.extend(crossings_with_plane(span, &plane)?);
            }
        }
    }
    cuts.retain(|cut| *cut >= whole.start.value() && *cut <= whole.end.value());
    cuts.sort_by(f64::total_cmp);
    cuts.dedup_by(|a, b| (*a - *b).abs() <= PARAMETER_EPSILON);

    let mut parts: Vec<Part> = Vec::new();
    for pair in cuts.windows(2) {
        let interval = Interval::new(pair[0], pair[1]);
        let middle = interval.midpoint();
        let point = span.curve().point_at(NativeParam::new(middle.value()));
        let mut candidates = Vec::new();
        for (index, cover) in covers.iter().enumerate() {
            if cover.iter().any(|part| part.contains(middle, 0.0)) {
                candidates.push(reach(&faces[index], mode, point));
            }
        }
        let face = if candidates.is_empty() {
            None
        } else {
            let wall = nearest(candidates, matches!(mode, Projection::Normal), options)?;
            faces.iter().position(|face| face.key == wall.hit)
        };
        match parts.last_mut() {
            Some(last) if last.face == face => last.interval.end = interval.end,
            _ => parts.push(Part { interval, face }),
        }
    }
    Ok(parts)
}

/// Whether some part of `first` meets some part of `second`.
fn overlap(first: &[Interval], second: &[Interval]) -> bool {
    first
        .iter()
        .any(|a| second.iter().any(|b| a.intersects(*b, 0.0)))
}

/// Where `point` lands on `face`'s plane, and how far it travels to get there.
fn reach(
    face: &PlanarFace,
    mode: Projection,
    point: Point3,
) -> PointProjection<crate::topology::shape_keys::FaceKey> {
    let normal = *face.plane.normal();
    let signed = (point - face.plane.origin()).dot(&normal);
    let (distance, landing) = match mode {
        Projection::Normal => (signed.abs(), point - normal * signed),
        Projection::Along(direction) => {
            let travel = -signed / direction.dot(&normal);
            (travel, point + direction.into_inner() * travel)
        }
    };
    PointProjection {
        point: landing,
        parameters: None,
        distance,
        hit: face.key,
    }
}

/// The planes where a point is as far from `first`'s plane as from `second`'s.
///
/// A directional projection reaches both planes at the same travel on one
/// plane; a perpendicular one is equally near them on the two bisectors. Planes
/// that are parallel have none, or are the same everywhere, and cut nothing.
fn equal_reach_planes(first: &Plane, second: &Plane, mode: Projection) -> Vec<Plane> {
    let (a, b) = (*first.normal(), *second.normal());
    let (oa, ob) = (first.origin().coords, second.origin().coords);
    match mode {
        Projection::Along(direction) => {
            let (slope_a, slope_b) = (direction.dot(&a), direction.dot(&b));
            let normal = a / slope_a - b / slope_b;
            let offset = a.dot(&oa) / slope_a - b.dot(&ob) / slope_b;
            plane_where(normal, offset).into_iter().collect()
        }
        Projection::Normal => [1.0, -1.0]
            .into_iter()
            .filter_map(|sign| {
                let normal = a - b * sign;
                let offset = a.dot(&oa) - sign * b.dot(&ob);
                plane_where(normal, offset)
            })
            .collect(),
    }
}

/// The plane of the points `p` with `normal . p = offset`, unless `normal`
/// vanishes.
fn plane_where(normal: Vector3<f64>, offset: f64) -> Option<Plane> {
    let length = normal.norm();
    if length <= ANGULAR_TOLERANCE {
        return None;
    }
    let unit = normal / length;
    // Any direction across the normal will do for the plane's first axis.
    let across = if unit.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let x_dir = across - unit * across.dot(&unit);
    Some(Plane::new(
        Point3::from(unit * (offset / length)),
        x_dir,
        unit,
    ))
}
