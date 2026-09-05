//! Deciding whether two faces sit on one support surface.
//!
//! Fusing two faces keeps the survivor's surface, so the question is not only
//! whether the two surfaces cover the same points but also what happens to the
//! consumed face's parameter curves. Two answers are useful:
//!
//! - [`SurfaceMatch::Identical`] — the surfaces are the same value, so every
//!   parameter curve stays valid and only needs re-keying;
//! - [`SurfaceMatch::Coplanar`] — both faces are planar on one plane but with
//!   different parameterizations, so the fused boundary's parameter curves are
//!   rebuilt in the survivor's plane.
//!
//! Every other pair is reported as no match. A cylinder split by an imprint
//! yields two fragments that share one `Surface` value, so it is covered by
//! `Identical`; a genuinely reparameterized curved pair is left alone rather
//! than approximated.

use crate::geometry::{Cylinder, Plane, Surface};
use crate::topology::orientation::Orientation;

/// How two support surfaces relate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SurfaceMatch {
    /// Same surface value: parameter curves carry over unchanged.
    Identical,
    /// Same plane, different parameterization: parameter curves are rebuilt.
    Coplanar,
}

/// Reports how two support surfaces relate, or `None` when they do not.
pub fn surfaces_match(
    first: &Surface,
    second: &Surface,
    linear: f64,
    angular: f64,
) -> Option<SurfaceMatch> {
    match (first, second) {
        (Surface::Plane(first), Surface::Plane(second)) => {
            planes_match(first, second, linear, angular)
        }
        (Surface::Cylinder(first), Surface::Cylinder(second)) => {
            cylinders_identical(first, second, linear, angular).then_some(SurfaceMatch::Identical)
        }
        _ => None,
    }
}

/// Classifies two planes as identical, merely coplanar, or unrelated.
///
/// Opposite normals still describe one plane. A face's outward direction comes
/// from its boundary winding rather than from the support normal, so an
/// antiparallel pair is a legitimate fusion whose parameter curves must be
/// rebuilt.
fn planes_match(first: &Plane, second: &Plane, linear: f64, angular: f64) -> Option<SurfaceMatch> {
    let parallel = first.normal().dot(&second.normal()).abs() >= 1.0 - angular;
    let coplanar = (second.origin() - first.origin())
        .dot(&first.normal())
        .abs()
        <= linear;
    if !parallel || !coplanar {
        return None;
    }
    let identical = (second.origin() - first.origin()).norm() <= linear
        && directions_match(
            first.x_dir().into_inner(),
            second.x_dir().into_inner(),
            angular,
        ) == Some(Orientation::Same)
        && directions_match(
            first.y_dir().into_inner(),
            second.y_dir().into_inner(),
            angular,
        ) == Some(Orientation::Same);
    Some(if identical {
        SurfaceMatch::Identical
    } else {
        SurfaceMatch::Coplanar
    })
}

/// Reports whether two cylinders share one parameterization.
fn cylinders_identical(first: &Cylinder, second: &Cylinder, linear: f64, angular: f64) -> bool {
    (first.radius - second.radius).abs() <= linear
        && (second.frame.origin - first.frame.origin).norm() <= linear
        && [
            (first.frame.x_dir, second.frame.x_dir),
            (first.frame.y_dir, second.frame.y_dir),
            (first.frame.z_dir, second.frame.z_dir),
        ]
        .into_iter()
        .all(|(a, b)| {
            directions_match(a.into_inner(), b.into_inner(), angular) == Some(Orientation::Same)
        })
}

/// Compares two unit directions up to sign.
fn directions_match(
    first: nalgebra::Vector3<f64>,
    second: nalgebra::Vector3<f64>,
    angular: f64,
) -> Option<Orientation> {
    let alignment = first.dot(&second);
    if alignment >= 1.0 - angular {
        Some(Orientation::Same)
    } else if alignment <= -1.0 + angular {
        Some(Orientation::Reversed)
    } else {
        None
    }
}
