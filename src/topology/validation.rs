use thiserror::Error;

use crate::topology::closed::Closed;

use super::gmap::{Dart, Dim, GMap};
use super::payload::Payload;
use super::shape_keys::SolidKey;
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
