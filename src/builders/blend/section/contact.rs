//! One cross-section of a blend, solved at one point of its edge.
//!
//! Both solvers work in space rather than in either face's parameters, so they
//! need nothing from a surface but where a point lands on it: the foot and the
//! outward normal there. The cross-section is pinned to the plane square to
//! the edge at the point asked for, which is what makes one fraction of the
//! edge name one section.
//!
//! - A fillet's ball has its centre `radius` behind both faces on a convex
//!   crease and `radius` in front of both on a concave one. The centre is the
//!   unknown; each face contributes its signed distance, whose gradient is the
//!   face's normal, and the section plane the third equation.
//! - A chamfer's rail on each face is where the face meets the circle of
//!   radius `distance` about the edge point in that plane: a true setback of
//!   `distance`, which is what the planar strip already is.

use nalgebra::{Matrix3, Vector3};

use super::crease::{Crease, CreaseFrame, Foot};
use crate::geometry::parameter::Fraction;
use crate::geometry::{LINEAR_TOLERANCE, Point3};

/// Newton steps a section may take before it is given up on.
const MAX_STEPS: usize = 64;

/// A residual this small is a solved section.
const RESIDUAL: f64 = LINEAR_TOLERANCE * 0.1;

/// A fillet's cross-section: the ball's centre and where it touches each face.
#[derive(Debug, Clone, Copy)]
pub(super) struct BallSection {
    pub(super) centre: Point3,
    pub(super) contacts: [Foot; 2],
}

/// Solves the ball of `radius` touching both faces in the plane square to
/// the edge at `t`, starting from `guess` when one is known nearby.
///
/// `None` when the ball cannot be placed, or touches a face on the far side
/// of the edge from the face's own interior.
pub(super) fn ball_at(
    crease: &Crease<'_>,
    t: Fraction,
    radius: f64,
    convex: bool,
    guess: Option<&BallSection>,
) -> Option<BallSection> {
    let frame = crease.frame_at(t)?;
    let sign = if convex { 1.0 } else { -1.0 };
    let normals = frame.feet.map(|foot| foot.normal);
    let mut centre = match guess {
        Some(guess) => guess.centre,
        None => {
            let fold = 1.0 + normals[0].dot(&normals[1]);
            if fold <= LINEAR_TOLERANCE.sqrt() {
                return None;
            }
            frame.point - (normals[0] + normals[1]) * (sign * radius / fold)
        }
    };
    let mut hints = guess.map_or(frame.feet.map(|foot| foot.uv), |guess| {
        guess.contacts.map(|foot| foot.uv)
    });
    for _ in 0..MAX_STEPS {
        let feet = [
            crease.foot(0, centre, Some(hints[0]))?,
            crease.foot(1, centre, Some(hints[1]))?,
        ];
        hints = feet.map(|foot| foot.uv);
        let residual = Vector3::new(
            (centre - feet[0].point).dot(&feet[0].normal) + sign * radius,
            (centre - feet[1].point).dot(&feet[1].normal) + sign * radius,
            (centre - frame.point).dot(&frame.tangent),
        );
        if residual.amax() <= RESIDUAL {
            return accept_ball(&frame, centre, feet);
        }
        let jacobian = Matrix3::from_rows(&[
            feet[0].normal.transpose(),
            feet[1].normal.transpose(),
            frame.tangent.transpose(),
        ]);
        let step = jacobian.lu().solve(&(-residual))?;
        centre += clamp_step(step, radius);
    }
    None
}

/// Solves where each face meets the circle of radius `distance` about the
/// edge point at `t`, square to the edge, on the side of the face's interior.
pub(super) fn setbacks_at(
    crease: &Crease<'_>,
    t: Fraction,
    distance: f64,
    guess: Option<&[Foot; 2]>,
) -> Option<[Foot; 2]> {
    let frame = crease.frame_at(t)?;
    let first = setback(crease, &frame, 0, distance, guess.map(|feet| feet[0]))?;
    let second = setback(crease, &frame, 1, distance, guess.map(|feet| feet[1]))?;
    Some([first, second])
}

/// One face's chamfer rail point at the edge point `frame` stands at.
fn setback(
    crease: &Crease<'_>,
    frame: &CreaseFrame,
    side: usize,
    distance: f64,
    guess: Option<Foot>,
) -> Option<Foot> {
    let mut point = guess.map_or(frame.point + frame.inward[side] * distance, |foot| {
        foot.point
    });
    let mut hint = guess.map_or(frame.feet[side].uv, |foot| foot.uv);
    for _ in 0..MAX_STEPS {
        let foot = crease.foot(side, point, Some(hint))?;
        hint = foot.uv;
        let offset = point - frame.point;
        let residual = Vector3::new(
            (point - foot.point).dot(&foot.normal),
            offset.dot(&frame.tangent),
            offset.norm_squared() - distance * distance,
        );
        let scaled = Vector3::new(residual.x, residual.y, residual.z / (2.0 * distance));
        if scaled.amax() <= RESIDUAL {
            return ((foot.point - frame.point).dot(&frame.inward[side]) > 0.0).then_some(foot);
        }
        let jacobian = Matrix3::from_rows(&[
            foot.normal.transpose(),
            frame.tangent.transpose(),
            (offset * 2.0).transpose(),
        ]);
        let step = jacobian.lu().solve(&(-residual))?;
        point += clamp_step(step, distance);
    }
    None
}

/// Accepts a converged ball only if it touches each face on that face's own
/// side of the edge.
fn accept_ball(frame: &CreaseFrame, centre: Point3, feet: [Foot; 2]) -> Option<BallSection> {
    let beside = (0..2).all(|side| (feet[side].point - frame.point).dot(&frame.inward[side]) > 0.0);
    beside.then_some(BallSection {
        centre,
        contacts: feet,
    })
}

/// Shortens a Newton step longer than `limit`, which keeps a poor first guess
/// from throwing the search onto another sheet of a face.
fn clamp_step(step: Vector3<f64>, limit: f64) -> Vector3<f64> {
    let length = step.norm();
    if length > limit {
        step * (limit / length)
    } else {
        step
    }
}
