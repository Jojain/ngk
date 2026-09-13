use std::collections::HashSet;

use thiserror::Error;

use crate::geometry::Surface;
use crate::topology::closed::Closed;

use super::attributes::ShellRoot;
use super::face::Face;
use super::gmap::{Dart, Dim, GMap};
use super::payload::Payload;
use super::shape_keys::{FaceKey, SolidKey};
use super::sheet::Sheet;
use crate::model::Model;

/// A map that is not a generalized map.
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
}

/// What a model's entities claim, checked against the map beneath them.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum ModelValidationError {
    /// The map itself is not a generalized map.
    #[error(transparent)]
    Topology(#[from] GMapValidationError),

    #[error("solid {solid:?} does not exist")]
    MissingSolid { solid: SolidKey },

    #[error("solid {solid:?} shell representative {shell:?} points outside the dart set")]
    SolidShellOutOfBounds { solid: SolidKey, shell: ShellRoot },

    #[error("solid {solid:?} shell at {shell:?} is open: {dart:?} is alpha{dim}-free")]
    SolidShellOpen {
        solid: SolidKey,
        shell: ShellRoot,
        dart: Dart,
        dim: usize,
    },

    #[error(
        "solid {solid:?} shell at {shell:?} is the boundaryless face {face:?}, \
         whose surface does not close on itself"
    )]
    SolidShellSurfaceOpen {
        solid: SolidKey,
        shell: ShellRoot,
        face: FaceKey,
    },

    #[error("solid {solid:?} shell at {shell:?} face {face:?} has no usable orientation data")]
    SolidFaceOrientationUnavailable {
        solid: SolidKey,
        shell: ShellRoot,
        face: FaceKey,
    },

    #[error("solid {solid:?} shell at {shell:?} face {face:?} normal does not point outward")]
    SolidFaceNormalNotOutward {
        solid: SolidKey,
        shell: ShellRoot,
        face: FaceKey,
    },
}

/// Validate the structural axioms of a generalized map's involutions.
///
/// This checks the book definition used by this crate: every alpha is an
/// involution, and every alpha_i o alpha_j is an involution when i + 2 <= j.
/// It reads no entity and no geometry, so it answers for a bare map.
pub fn validate_gmap(g: &GMap) -> Result<(), GMapValidationError> {
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
    g: &Model<P>,
    solid: SolidKey,
) -> Result<(), ModelValidationError> {
    validate_gmap(g.topology())?;

    let attr = g
        .solid_attr(solid)
        .ok_or(ModelValidationError::MissingSolid { solid })?;
    validate_shell(g, solid, attr.outer_shell)?;
    if let Some(inner_shells) = &attr.inner_shells {
        for &shell in inner_shells {
            validate_shell(g, solid, shell)?;
        }
    }

    Ok(())
}

/// Validate every registered solid in the map as a closed surface shell.
pub fn validate_all_solid_manifolds<P: Payload>(g: &Model<P>) -> Result<(), ModelValidationError> {
    validate_gmap(g.topology())?;
    for (solid, _) in g.iter_solids() {
        validate_solid_manifold(g, solid)?;
    }
    Ok(())
}

/// Validate that every face surface normal of one solid's shell points outside.
pub fn validate_solid_orientation<P: Payload>(
    g: &Model<P>,
    solid: SolidKey,
) -> Result<(), ModelValidationError> {
    validate_gmap(g.topology())?;

    let attr = g
        .solid_attr(solid)
        .ok_or(ModelValidationError::MissingSolid { solid })?;
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
pub fn validate_all_solid_orientations<P: Payload>(
    g: &Model<P>,
) -> Result<(), ModelValidationError> {
    validate_gmap(g.topology())?;
    for (solid, _) in g.iter_solids() {
        validate_solid_orientation(g, solid)?;
    }
    Ok(())
}

fn validate_shell<P: Payload>(
    g: &Model<P>,
    solid: SolidKey,
    shell: ShellRoot,
) -> Result<(), ModelValidationError> {
    match shell {
        ShellRoot::Dart(dart) => {
            if dart.id() >= g.dart_count() {
                return Err(ModelValidationError::SolidShellOutOfBounds { solid, shell });
            }
            let sheet =
                Sheet::from_dart(g, dart).expect("solid shell must have a registered sheet");
            Closed::new(sheet).ok_or(ModelValidationError::SolidShellOpen {
                solid,
                shell,
                dart,
                dim: 2,
            })?;
        }
        // A boundaryless shell has no free dart to find, because it has no
        // darts; what has to close is the surface itself.
        ShellRoot::Face { face, .. } => {
            let attr = g
                .face_attr(face)
                .ok_or(ModelValidationError::SolidShellOutOfBounds { solid, shell })?;
            if !attr.surface.is_closed() {
                return Err(ModelValidationError::SolidShellSurfaceOpen { solid, shell, face });
            }
        }
    }

    Ok(())
}

fn validate_shell_orientation<P: Payload>(
    g: &Model<P>,
    solid: SolidKey,
    shell: ShellRoot,
    side: ShellSide,
) -> Result<(), ModelValidationError> {
    validate_oriented_shell_volume(g, solid, shell, side)
}

/// Checks local winding and global signed volume without a star-shaped-shell assumption.
fn validate_oriented_shell_volume<P: Payload>(
    g: &Model<P>,
    solid: SolidKey,
    shell: ShellRoot,
    side: ShellSide,
) -> Result<(), ModelValidationError> {
    let sheet = g.shell_sheet(shell).expect("validated shell");
    let faces = match sheet.boundaryless_face() {
        // A shell that is one boundaryless face says which way it faces on its
        // root and nowhere else — the face has no winding to carry the answer
        // — so the shell's own reading of it is the only one there is.
        Some(_) => sheet.faces(),
        // Every other face states its sense in its boundary's winding, which
        // is what the rest of this reads, and reading it as the shell reached
        // it would state the same thing a second time.
        None => sheet
            .faces()
            .into_iter()
            .map(|face| g.face_unchecked(face.key()))
            .collect(),
    };
    let mut directed = HashSet::new();
    let mut owner = std::collections::HashMap::<Dart, FaceKey>::new();
    let mut volume = 0.0;
    let unavailable = |face: &Face<'_, P>| ModelValidationError::SolidFaceOrientationUnavailable {
        solid,
        shell,
        face: face.key(),
    };
    // A boundaryless face has no vertex to take a reference point from, so the
    // point comes off the surface itself — anywhere will do, the divergence
    // integral is reference-independent for a closed shell.
    let reference = match faces[0].vertices().first() {
        Some(vertex) => vertex
            .point()
            .copied()
            .ok_or_else(|| unavailable(&faces[0]))?,
        None => faces[0]
            .domain_center()
            .ok_or_else(|| unavailable(&faces[0]))?,
    };
    for face in &faces {
        let planar = matches!(face.surface(), Surface::Plane(_))
            && face.edges().iter().all(|edge| {
                edge.curve().is_some_and(|curve| {
                    curve
                        .to_nurbs()
                        .is_ok_and(|curve| curve.degree().get() == 1)
                })
            });
        if !planar {
            volume += face.signed_volume_contribution(reference).ok_or(
                ModelValidationError::SolidFaceOrientationUnavailable {
                    solid,
                    shell,
                    face: face.key(),
                },
            )?;
        }
        for boundary in face.loops() {
            let mut points = Vec::new();
            for edge in boundary.edges() {
                directed.insert(edge.dart());
                owner.insert(edge.dart(), face.key());
                points.push(
                    edge.trimmed_curve()
                        .map(|section| section.point_at(0.0))
                        .ok_or(ModelValidationError::SolidFaceOrientationUnavailable {
                            solid,
                            shell,
                            face: face.key(),
                        })?,
                );
            }
            for pair in points[1..].windows(2).filter(|_| planar) {
                volume += (points[0] - reference)
                    .dot(&(pair[0] - reference).cross(&(pair[1] - reference)))
                    / 6.0;
            }
        }
    }
    // Consistent winding is a statement about neighbours across an edge. A
    // boundaryless face has no neighbour and no edge, so it has nothing to
    // agree with, and only the volume sign below decides which way it faces.
    for face in &faces {
        for boundary in face.loops() {
            for edge in boundary.edges() {
                if !directed.contains(&g.alpha(Dim::Zero, g.alpha(Dim::Two, edge.dart()))) {
                    return Err(ModelValidationError::SolidFaceNormalNotOutward {
                        solid,
                        shell,
                        face: face.key(),
                    });
                }
            }
        }
    }
    let valid = volume.is_finite()
        && match side {
            ShellSide::Outer => volume > 0.0,
            ShellSide::Inner => volume < 0.0,
        };
    if !valid {
        return Err(ModelValidationError::SolidFaceNormalNotOutward {
            solid,
            shell,
            face: faces[0].key(),
        });
    }
    Ok(())
}
#[derive(Clone, Copy)]
enum ShellSide {
    Outer,
    Inner,
}
