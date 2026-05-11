use std::collections::{HashMap, HashSet};

use nalgebra::Vector3;
use thiserror::Error;

use crate::geometry::{Curve2, LINEAR_TOLERANCE, Point2};
use crate::topology::closed::Closed;

use super::attributes::FaceAttr;
use super::gmap::{Cell0, Dart, Dim, GMap};
use super::payload::Payload;
use super::profile::Profile;
use super::shape_keys::{FaceKey, SolidKey};
use super::sheet::Sheet;

#[derive(Debug, Error, PartialEq, Eq)]
pub enum GMapValidationError {
    #[error("alpha{dim}({dart:?}) points outside the dart set: {linked:?}")]
    AlphaOutOfBounds {
        dim: usize,
        dart: Dart,
        linked: Dart,
    },

    #[error("alpha{dim} is not an involution at {dart:?}: alpha{dim}({linked:?}) = {back:?}")]
    AlphaNotInvolution {
        dim: usize,
        dart: Dart,
        linked: Dart,
        back: Dart,
    },

    #[error(
        "alpha{left} o alpha{right} is not an involution at {dart:?}: applying it twice gives {back:?}"
    )]
    AlphaCompositionNotInvolution {
        left: usize,
        right: usize,
        dart: Dart,
        back: Dart,
    },

    #[error("solid {solid:?} does not exist")]
    MissingSolid { solid: SolidKey },

    #[error("solid {solid:?} shell representative {shell:?} points outside the dart set")]
    SolidShellOutOfBounds { solid: SolidKey, shell: Dart },

    #[error("solid {solid:?} shell at {shell:?} is open: {dart:?} is alpha{dim}-free")]
    SolidShellOpen {
        solid: SolidKey,
        shell: Dart,
        dart: Dart,
        dim: usize,
    },

    #[error("solid {solid:?} shell at {shell:?} face {face:?} has no usable orientation data")]
    SolidFaceOrientationUnavailable {
        solid: SolidKey,
        shell: Dart,
        face: FaceKey,
    },

    #[error("solid {solid:?} shell at {shell:?} face {face:?} normal does not point outward")]
    SolidFaceNormalNotOutward {
        solid: SolidKey,
        shell: Dart,
        face: FaceKey,
    },
}

/// Validate the structural axioms of the stored n-GMap involutions.
///
/// This checks the book definition used by this crate: every alpha is an
/// involution, and every alpha_i o alpha_j is an involution when i + 2 <= j.
pub fn validate_gmap<P: Payload>(g: &GMap<P>) -> Result<(), GMapValidationError> {
    let dart_count = g.dart_count();

    for i in 0..g.dimension() {
        let dim = Dim::from_index(i);
        for dart in g.darts() {
            let linked = g.alpha(dim, dart);
            if linked.id() >= dart_count {
                return Err(GMapValidationError::AlphaOutOfBounds {
                    dim: i,
                    dart,
                    linked,
                });
            }
            let back = g.alpha(dim, linked);
            if back != dart {
                return Err(GMapValidationError::AlphaNotInvolution {
                    dim: i,
                    dart,
                    linked,
                    back,
                });
            }
        }
    }

    for left in 0..g.dimension() {
        for right in (left + 2)..g.dimension() {
            let left_dim = Dim::from_index(left);
            let right_dim = Dim::from_index(right);
            for dart in g.darts() {
                let once = g.alpha(left_dim, g.alpha(right_dim, dart));
                let back = g.alpha(left_dim, g.alpha(right_dim, once));
                if back != dart {
                    return Err(GMapValidationError::AlphaCompositionNotInvolution {
                        left,
                        right,
                        dart,
                        back,
                    });
                }
            }
        }
    }

    Ok(())
}

/// Validate one registered solid as a closed surface shell.
///
/// In this codebase a solid is represented by one outer closed 2-sheet and
/// optional closed inner 2-sheets; the surrounding alpha3 volume pairing is not
/// required for this boundary-representation style.
pub fn validate_solid_manifold<P: Payload>(
    g: &GMap<P>,
    solid: SolidKey,
) -> Result<(), GMapValidationError> {
    validate_gmap(g)?;

    let attr = g
        .solid(solid)
        .ok_or(GMapValidationError::MissingSolid { solid })?;
    validate_shell(g, solid, attr.outer_shell)?;
    if let Some(inner_shells) = &attr.inner_shells {
        for &shell in inner_shells {
            validate_shell(g, solid, shell)?;
        }
    }

    Ok(())
}

/// Validate every registered solid in the map as a closed surface shell.
pub fn validate_all_solid_manifolds<P: Payload>(g: &GMap<P>) -> Result<(), GMapValidationError> {
    validate_gmap(g)?;
    for (solid, _) in g.iter_solids() {
        validate_solid_manifold(g, solid)?;
    }
    Ok(())
}

/// Validate that every face surface normal of one solid's shell points outside.
pub fn validate_solid_orientation<P: Payload>(
    g: &GMap<P>,
    solid: SolidKey,
) -> Result<(), GMapValidationError> {
    validate_gmap(g)?;

    let attr = g
        .solid(solid)
        .ok_or(GMapValidationError::MissingSolid { solid })?;
    validate_shell(g, solid, attr.outer_shell)?;
    validate_shell_orientation(g, solid, attr.outer_shell, ShellSide::Outer)?;
    if let Some(inner_shells) = &attr.inner_shells {
        for &shell in inner_shells {
            validate_shell(g, solid, shell)?;
            validate_shell_orientation(g, solid, shell, ShellSide::Inner)?;
        }
    }

    Ok(())
}

/// Validate every registered solid's face surface normals.
pub fn validate_all_solid_orientations<P: Payload>(g: &GMap<P>) -> Result<(), GMapValidationError> {
    validate_gmap(g)?;
    for (solid, _) in g.iter_solids() {
        validate_solid_orientation(g, solid)?;
    }
    Ok(())
}

fn validate_shell<P: Payload>(
    g: &GMap<P>,
    solid: SolidKey,
    shell: Dart,
) -> Result<(), GMapValidationError> {
    if shell.id() >= g.dart_count() {
        return Err(GMapValidationError::SolidShellOutOfBounds { solid, shell });
    }

    Closed::new(Sheet::new(g, shell)).ok_or(GMapValidationError::SolidShellOpen {
        solid,
        shell,
        dart: shell,
        dim: 2,
    })?;

    Ok(())
}

fn validate_shell_orientation<P: Payload>(
    g: &GMap<P>,
    solid: SolidKey,
    shell: Dart,
    side: ShellSide,
) -> Result<(), GMapValidationError> {
    let shell_darts = Sheet::new(g, shell).darts().collect::<Vec<_>>();
    let Some(shell_center) = shell_centroid(g, &shell_darts) else {
        return Ok(());
    };

    let shell_darts = shell_darts.into_iter().collect::<HashSet<_>>();
    for (face, attr) in g.iter_faces() {
        if !shell_darts.contains(&attr.outer_loop) {
            continue;
        }
        let Some((face_center, normal)) = face_orientation_sample(g, attr) else {
            return Err(GMapValidationError::SolidFaceOrientationUnavailable {
                solid,
                shell,
                face,
            });
        };
        let outward = face_center - shell_center;
        let normal_dot_outward = normal.dot(&outward);
        let is_valid = match side {
            ShellSide::Outer => normal_dot_outward > LINEAR_TOLERANCE,
            ShellSide::Inner => normal_dot_outward < -LINEAR_TOLERANCE,
        };
        if !is_valid {
            return Err(GMapValidationError::SolidFaceNormalNotOutward { solid, shell, face });
        }
    }

    Ok(())
}

#[derive(Clone, Copy)]
enum ShellSide {
    Outer,
    Inner,
}

fn shell_centroid<P: Payload>(g: &GMap<P>, shell_darts: &[Dart]) -> Option<Vector3<f64>> {
    let mut vertex_representatives = HashSet::new();
    let mut sum = Vector3::zeros();
    let mut count = 0;

    for &dart in shell_darts {
        let repr = g.cell_representative(dart, Dim::Zero);
        if !vertex_representatives.insert(repr) {
            continue;
        }
        let vertex = g.attribute::<Cell0>(dart)?;
        sum += vertex.point.coords;
        count += 1;
    }

    (count > 0).then_some(sum / count as f64)
}

fn face_orientation_sample<P: Payload>(
    g: &GMap<P>,
    attr: &FaceAttr<P::F>,
) -> Option<(Vector3<f64>, Vector3<f64>)> {
    let outer_uv = sample_loop_pcurve(g, attr.outer_loop, &attr.pcurves)?;
    if outer_uv.is_empty() {
        return None;
    }

    let uv_center = uv_centroid(&outer_uv);
    let face_center = attr.surface.point_at(uv_center.x, uv_center.y).coords;
    let normal = *attr.surface.normal_at(uv_center.x, uv_center.y);

    Some((face_center, normal))
}

fn sample_loop_pcurve(
    g: &GMap<impl Payload>,
    loop_dart: Dart,
    pcurves: &HashMap<Dart, Curve2>,
) -> Option<Vec<Point2>> {
    let profile = Profile::new(g, loop_dart);
    let edge_darts = profile.darts().step_by(2);
    let mut points = Vec::new();
    for dart in edge_darts {
        let samples = pcurves.get(&dart)?.sample(1);
        let n = samples.len();
        points.extend(samples.into_iter().take(n.saturating_sub(1)));
    }

    (!points.is_empty()).then_some(points)
}

fn uv_centroid(points: &[Point2]) -> Point2 {
    let sum = points
        .iter()
        .fold(nalgebra::Vector2::zeros(), |sum, point| sum + point.coords);
    Point2::from(sum / points.len() as f64)
}
