//! Bounding a solid with closed sheets.

use std::collections::HashSet;

use thiserror::Error;

use crate::builders::boolean::{
    BooleanError, BooleanOperand, BooleanOptions, compute_boolean_intersections,
    shell_encloses_point,
};
use crate::builders::faces::reverse_face_winding_edit;
use crate::builders::scaffold::cut_between_shells;
use crate::geometry::{LINEAR_TOLERANCE, Point3};
use crate::measure::{MeasureError, signed_shell_volume};
use crate::model::Model;
use crate::topology::ModelEdit;
use crate::topology::attributes::SolidAttr;
use crate::topology::closed::Closed;
use crate::topology::edit::{EditKey, ModelEditError};
use crate::topology::gmap::{Dart, Dim};
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{SheetKey, SolidKey};
use crate::topology::validation::{
    ModelValidationError, validate_solid_manifold, validate_solid_orientation,
};

/// Why a set of sheets could not bound a solid.
#[derive(Debug, Error)]
pub enum SolidFromSheetsError {
    #[error("sheet {sheet:?} does not exist")]
    MissingSheet { sheet: SheetKey },
    #[error("sheet {sheet:?} is passed more than once")]
    RepeatedSheet { sheet: SheetKey },
    #[error("sheet {sheet:?} is open, so it bounds no region")]
    OpenSheet { sheet: SheetKey },
    #[error("sheet {sheet:?} encloses no volume")]
    DegenerateSheet { sheet: SheetKey },
    /// Two of the shells meet: a solid's shells are disjoint.
    #[error("sheets {first:?} and {second:?} touch")]
    SheetsTouch { first: SheetKey, second: SheetKey },
    #[error("void {void:?} is not inside the outer sheet")]
    VoidOutside { void: SheetKey },
    /// A void inside another void would leave material floating in it, which
    /// is a second solid rather than part of this one.
    #[error("void {inner:?} lies inside void {outer:?}")]
    NestedVoid { inner: SheetKey, outer: SheetKey },
    #[error("could not certify how the sheets lie against each other")]
    Classification(#[source] BooleanError),
    #[error("could not measure sheet {sheet:?}")]
    Measure {
        sheet: SheetKey,
        #[source]
        source: MeasureError,
    },
    #[error(transparent)]
    ModelEdit(#[from] ModelEditError),
    #[error(transparent)]
    Invalid(#[from] ModelValidationError),
}

/// Adds the solid `outer` bounds, less the regions `voids` bound.
///
/// Every sheet must be closed, and they must be disjoint: each void inside
/// `outer`, and none inside another. Which way a sheet is wound does not
/// matter: its role decides it, so `outer` is wound outward and every void
/// toward its own inside, out of the material, reversing any sheet wound the
/// other way. Failure leaves the model unchanged.
pub fn add_solid_from_sheets<P: Payload>(
    g: &mut Model<P>,
    outer: SheetKey,
    voids: &[SheetKey],
) -> Result<SolidKey, SolidFromSheetsError> {
    g.transaction(|edit| add_solid_from_sheets_edit(edit, outer, voids))
}

/// Bounds a solid with closed sheets inside the caller's transaction.
pub(crate) fn add_solid_from_sheets_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    outer: SheetKey,
    voids: &[SheetKey],
) -> Result<SolidKey, SolidFromSheetsError> {
    let sheets = std::iter::once(outer)
        .chain(voids.iter().copied())
        .collect::<Vec<_>>();
    let mut seen = HashSet::new();
    for &sheet in &sheets {
        if !seen.insert(sheet) {
            return Err(SolidFromSheetsError::RepeatedSheet { sheet });
        }
        let view = edit
            .sheet(sheet)
            .ok_or(SolidFromSheetsError::MissingSheet { sheet })?;
        if Closed::new(view).is_none() {
            return Err(SolidFromSheetsError::OpenSheet { sheet });
        }
    }
    // A sheet copied out of a solid brings that solid along; the one built
    // here replaces it.
    let registered = sheets
        .iter()
        .filter_map(|&sheet| edit.solid_key(edit.sheet_unchecked(sheet).dart()))
        .collect::<HashSet<_>>();
    for solid in registered {
        edit.remove_solid(solid);
    }

    // Every shell is wound outward first: that is the winding the ray
    // classifier reads a region from.
    let mut roots = Vec::with_capacity(sheets.len());
    for &sheet in &sheets {
        roots.push(wind_outward(edit, sheet)?);
    }
    for (index, &first) in sheets.iter().enumerate() {
        for &second in &sheets[index + 1..] {
            if touch(edit, first, second)? {
                return Err(SolidFromSheetsError::SheetsTouch { first, second });
            }
        }
    }
    for &void in voids {
        let inside = |shell| {
            shell_encloses_point(edit, shell, sample_point(edit, void), options())
                .map_err(SolidFromSheetsError::Classification)
        };
        if !inside(outer)? {
            return Err(SolidFromSheetsError::VoidOutside { void });
        }
        for &other in voids.iter().filter(|&&other| other != void) {
            if inside(other)? {
                return Err(SolidFromSheetsError::NestedVoid {
                    inner: void,
                    outer: other,
                });
            }
        }
    }
    for root in &mut roots[1..] {
        *root = reverse_shell(edit, *root);
    }

    let inner = (!voids.is_empty()).then(|| roots[1..].to_vec());
    let solid = edit.add_solid_derived_from(
        sheets.iter().map(|&sheet| EditKey::Sheet(sheet)).collect(),
        SolidAttr::new(roots[0], inner),
    );
    cut_between_shells(edit, solid, &roots)?;
    validate_solid_manifold(edit, solid)?;
    validate_solid_orientation(edit, solid)?;
    Ok(solid)
}

fn options() -> BooleanOptions {
    BooleanOptions::default()
}

/// Winds `sheet` so it encloses a positive volume, and returns its root in
/// that winding.
fn wind_outward<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    sheet: SheetKey,
) -> Result<Dart, SolidFromSheetsError> {
    let root = edit.sheet_unchecked(sheet).dart();
    let volume = signed_shell_volume(edit, root)
        .map_err(|source| SolidFromSheetsError::Measure { sheet, source })?;
    if !volume.is_finite() || volume.abs() <= LINEAR_TOLERANCE.powi(3) {
        return Err(SolidFromSheetsError::DegenerateSheet { sheet });
    }
    Ok(if volume > 0.0 {
        root
    } else {
        reverse_shell(edit, root)
    })
}

/// Reverses every face of the shell at `root`, and returns the root read the
/// other way.
///
/// A face with a boundary states its sense in its loops' winding, so that is
/// what is reversed. A face that bounds nothing has no winding and takes its
/// sense from the dart the shell is read at, so the root flips with it.
fn reverse_shell<P: Payload>(edit: &mut ModelEdit<'_, P>, root: Dart) -> Dart {
    let faces = edit
        .sheet_unchecked(edit.sheet_key(root).expect("a shell root is registered"))
        .faces()
        .into_iter()
        .filter(|face| !face.loops().is_empty())
        .map(|face| face.key())
        .collect::<Vec<_>>();
    for face in faces {
        reverse_face_winding_edit(edit, face);
    }
    edit.alpha(Dim::Zero, root)
}

/// Whether the two sheets meet anywhere, certified by the Boolean contact
/// search.
fn touch<P: Payload>(
    model: &Model<P>,
    first: SheetKey,
    second: SheetKey,
) -> Result<bool, SolidFromSheetsError> {
    let plan = compute_boolean_intersections(
        model,
        BooleanOperand::Sheet(first),
        BooleanOperand::Sheet(second),
        options(),
    )
    .map_err(SolidFromSheetsError::Classification)?;
    let network = &plan.network;
    Ok(
        !network.events().is_empty()
            || !network.spans().is_empty()
            || !network.regions().is_empty(),
    )
}

/// A point on `sheet`, which lies on no other sheet once they are known not
/// to touch.
fn sample_point<P: Payload>(model: &Model<P>, sheet: SheetKey) -> Point3 {
    let view = model.sheet_unchecked(sheet);
    if let Some(vertex) = view.vertices().first() {
        return *vertex.point();
    }
    // Only a sheet of one face that bounds nothing has no vertex, and such a
    // face covers its whole support.
    let face = view.faces()[0].clone();
    let (u, v) = face.surface().domain();
    face.surface()
        .point_at(u.midpoint().value(), v.midpoint().value())
}
