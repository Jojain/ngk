use std::collections::HashSet;

use thiserror::Error;

use crate::geometry::Surface;
use crate::geometry::parameter::Fraction;
use crate::topology::closed::Closed;

use super::embedding::{EntityOwner, turn};
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
    SolidShellOutOfBounds { solid: SolidKey, shell: Dart },

    #[error("solid {solid:?} shell at {shell:?} is open: some dart of it is free")]
    SolidShellOpen { solid: SolidKey, shell: Dart },

    #[error("solid {solid:?} shell at {shell:?} reaches no registered face")]
    SolidShellHasNoFace { solid: SolidKey, shell: Dart },

    #[error(
        "solid {solid:?} shell at {shell:?} is the boundaryless face {face:?}, \
         whose surface does not close on itself"
    )]
    SolidShellSurfaceOpen {
        solid: SolidKey,
        shell: Dart,
        face: FaceKey,
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
    shell: Dart,
) -> Result<(), ModelValidationError> {
    if shell.id() >= g.dart_count() {
        return Err(ModelValidationError::SolidShellOutOfBounds { solid, shell });
    }
    let sheet = Sheet::from_dart(g, shell).expect("solid shell must have a registered sheet");
    Closed::new(sheet).ok_or(ModelValidationError::SolidShellOpen { solid, shell })?;

    Ok(())
}

fn validate_shell_orientation<P: Payload>(
    g: &Model<P>,
    solid: SolidKey,
    shell: Dart,
    side: ShellSide,
) -> Result<(), ModelValidationError> {
    validate_oriented_shell_volume(g, solid, shell, side)
}

/// Checks local winding and global signed volume without a star-shaped-shell assumption.
fn validate_oriented_shell_volume<P: Payload>(
    g: &Model<P>,
    solid: SolidKey,
    shell: Dart,
    side: ShellSide,
) -> Result<(), ModelValidationError> {
    let sheet = Sheet::from_dart(g, shell).expect("validated shell");
    let faces = sheet
        .faces()
        .into_iter()
        .map(|face| match face.loops().is_empty() {
            // A boundaryless face has no winding to say which way it points,
            // so the shell's own reading of it is the only one there is.
            true => face,
            // Every other face states its sense in its boundary's winding,
            // which is what the rest of this reads, and reading it as the
            // shell reached it would state the same thing a second time.
            false => g.face_unchecked(face.key()),
        })
        .collect::<Vec<_>>();
    let mut directed = HashSet::new();
    let mut owner = std::collections::HashMap::<Dart, FaceKey>::new();
    let mut volume = 0.0;
    let unavailable = |face: &Face<'_, P>| ModelValidationError::SolidFaceOrientationUnavailable {
        solid,
        shell,
        face: face.key(),
    };
    // A shell is faces, and every face occupies a 2-cell the walk reaches. One
    // that reports none is a shell whose 2-cells carry no registered face, and
    // measuring the volume of nothing would call any solid well oriented.
    let [first, ..] = faces.as_slice() else {
        return Err(ModelValidationError::SolidShellHasNoFace { solid, shell });
    };
    // Anywhere on the shell will do: the divergence integral is
    // reference-independent for a closed one. A vertex is the cheapest source.
    // A face whose rim is a whole circle has no vertex at all -- the point
    // where the circle closes is inside the edge -- so its rim's own curve
    // answers instead. A boundaryless face has neither, and the surface does.
    let reference = first
        .vertices()
        .first()
        .map(|vertex| *vertex.point())
        .or_else(|| {
            let boundary = faces[0].loops().into_iter().next()?;
            let edge = boundary.edges().into_iter().next()?;
            Some(edge.trimmed_curve().point_at(Fraction::new(0.0)))
        })
        .or_else(|| first.domain_center())
        .ok_or_else(|| unavailable(first))?;
    for face in &faces {
        let planar = matches!(face.surface(), Surface::Plane(_))
            && face.edges().iter().all(|edge| {
                edge.curve()
                    .to_nurbs()
                    .is_ok_and(|curve| curve.degree().get() == 1)
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
                points.push(edge.trimmed_curve().point_at(Fraction::new(0.0)));
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
                let across = turn(g.topology(), g.embedding_index(), Dim::Two, edge.dart());
                if !across.is_some_and(|dart| directed.contains(&g.alpha(Dim::Zero, dart))) {
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

/// An entity that does not occupy exactly one raw cell of its own dimension.
///
/// The dimension is not a field: [`EntityOwner`] already carries it, and a
/// second copy could disagree with the key it sits next to.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum CellOccupancyError {
    #[error(
        "{entity:?} spans more than one raw cell of its own dimension: {first:?} and {second:?}"
    )]
    SpansSeveralCells {
        /// The entity holding darts in two cells.
        entity: EntityOwner,
        /// A representative of the first cell reached.
        first: Dart,
        /// A representative of the second, which should not exist.
        second: Dart,
    },

    #[error("{entity:?} occupies no raw cell of its own dimension: it names no dart")]
    OccupiesNoCell {
        /// The entity with no topology at all.
        entity: EntityOwner,
    },
}

/// Reports every entity that does not occupy exactly one raw cell of its own
/// dimension.
///
/// Only faces and solids can fail. A vertex and an edge each name exactly one
/// anchor dart, so the cell they occupy is that dart's and there is nothing to
/// disagree with; profiles and sheets are aggregates of entities rather than
/// entities with a cell of their own, so the rule does not reach them. A face
/// names its anchor plus one dart per loop and one per pcurve, and a solid
/// names one per shell, and those are what can land in two cells or in none.
///
/// Reads the involutions alone, never the classification: an entity's own cell
/// is the question being asked, so an answer derived from ownership labels
/// would assume it. Darts outside the map are skipped rather than reported —
/// [`validate_gmap`] is what names those — so a model whose darts all dangle
/// reports its entities as occupying no cell, which is true of the map it has.
pub fn cell_occupancy_violations<P: Payload>(g: &Model<P>) -> Vec<CellOccupancyError> {
    let mut violations = Vec::new();

    for (key, attr) in g.iter_faces() {
        let darts = std::iter::once(attr.seed())
            .chain(attr.darts())
            .chain(attr.pcurves.keys().copied());
        check_one_cell(g, EntityOwner::Face(key), Dim::Two, darts, &mut violations);
    }

    for (key, attr) in g.iter_solids() {
        let darts = attr.shells();
        check_one_cell(
            g,
            EntityOwner::Solid(key),
            Dim::Three,
            darts,
            &mut violations,
        );
    }

    violations
}

/// Returns the first entity that breaks the rule, for a caller that only needs
/// to refuse.
///
/// [`cell_occupancy_violations`] is what to reach for when the whole list is
/// the point, such as an inventory of what a tree still has to fix.
pub fn validate_cell_occupancy<P: Payload>(g: &Model<P>) -> Result<(), CellOccupancyError> {
    match cell_occupancy_violations(g).into_iter().next() {
        Some(violation) => Err(violation),
        None => Ok(()),
    }
}

/// Records whether `darts` all lie in one `dimension`-cell, and that there is
/// at least one of them.
fn check_one_cell<P: Payload>(
    g: &Model<P>,
    entity: EntityOwner,
    dimension: Dim,
    darts: impl Iterator<Item = Dart>,
    violations: &mut Vec<CellOccupancyError>,
) {
    let mut held: Option<(Dart, Dart)> = None;
    for dart in darts.filter(|dart| dart.id() < g.dart_count()) {
        let cell = g.cell_representative(dart, dimension);
        match held {
            None => held = Some((dart, cell)),
            Some((_, first)) if first == cell => {}
            Some((first_dart, _)) => {
                violations.push(CellOccupancyError::SpansSeveralCells {
                    entity,
                    first: first_dart,
                    second: dart,
                });
                return;
            }
        }
    }
    if held.is_none() {
        violations.push(CellOccupancyError::OccupiesNoCell { entity });
    }
}
