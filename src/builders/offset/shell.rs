//! Shelling: a solid emptied to a wall of given thickness, open where the
//! caller removes faces.
//!
//! **The wall is what is left of the solid once its inner offset is cut
//! away.** The inner solid is the solid's offset with a distance per face:
//! every face moves inward by the thickness, and every open face moves
//! *outward* by it, so the inner solid pokes through where the shell is open
//! instead of grazing the outer face. The Boolean difference then leaves the
//! wall, and an open face's region becomes the opening. With no open face the
//! inner solid lies wholly inside and is left as a cavity.
//!
//! The offset and the Boolean are two transactions, so both run on a copy of
//! the model, which replaces the caller's only once both have succeeded: the
//! call commits whole or leaves the model as it found it.

use std::collections::BTreeSet;

use thiserror::Error;

use super::errors::OffsetBuildError;
use super::reembed::{face_sign, plan_faces};
use crate::builders::boolean::{BooleanError, BooleanOperation, BooleanOptions, boolean};
use crate::geometry::Point2;
use crate::geometry::parameter::Fraction;
use crate::model::{Model, OpResult, StaleResult};
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{FaceKey, SolidKey};
use crate::topology::solid::Solid;

/// A solid was shelled to a wall of given thickness.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SolidShelling {
    /// The shelled solid.
    pub solid: SolidKey,
    revision: Option<u64>,
}

impl OpResult for SolidShelling {
    fn stamp(&mut self, revision: u64) {
        self.revision = Some(revision);
    }
}

impl SolidShelling {
    /// Resolves the shelled solid at the result's committed revision.
    pub fn view<'m, P: Payload>(&self, model: &'m Model<P>) -> Result<Solid<'m, P>, StaleResult> {
        StaleResult::check(self.revision, model)?;
        Ok(model.solid_unchecked(self.solid))
    }
}

/// Why a solid could not be shelled.
#[derive(Debug, Error)]
pub enum ShellError {
    #[error("solid {solid:?} does not exist")]
    MissingSolid { solid: SolidKey },
    #[error("shell thickness must be finite and positive, got {thickness}")]
    InvalidThickness { thickness: f64 },
    #[error("face {face:?} is not a face of the shelled solid")]
    ForeignOpenFace { face: FaceKey },
    #[error("every face of the solid is open, which leaves no wall")]
    EveryFaceOpen,
    /// The inner offset could not be built.
    #[error("the solid's inner offset failed: {0}")]
    InnerOffset(#[from] OffsetBuildError),
    /// Cutting the inner offset away failed.
    #[error("cutting the inner offset away failed: {0}")]
    Cut(#[from] BooleanError),
}

/// Empties `solid` to a wall `thickness` thick, measured inward from its
/// faces, and opens it where `open_faces` were.
///
/// With no open face, the result is the solid with a cavity of its inner
/// offset's shape.
pub fn shell_solid<P: Payload>(
    model: &mut Model<P>,
    solid: SolidKey,
    open_faces: &[FaceKey],
    thickness: f64,
) -> Result<SolidShelling, ShellError> {
    if !thickness.is_finite() || thickness <= 0.0 {
        return Err(ShellError::InvalidThickness { thickness });
    }
    let source = model
        .solid(solid)
        .ok_or(ShellError::MissingSolid { solid })?;
    let faces: BTreeSet<FaceKey> = source.faces().iter().map(|face| face.key()).collect();
    let open: BTreeSet<FaceKey> = open_faces.iter().copied().collect();
    if let Some(&face) = open.iter().find(|face| !faces.contains(face)) {
        return Err(ShellError::ForeignOpenFace { face });
    }
    if open.len() == faces.len() {
        return Err(ShellError::EveryFaceOpen);
    }
    // Each open face is recognized on the copy by its support and its
    // boundary: merging a copy gives every face a new key.
    let open_fingerprints = open
        .iter()
        .map(|&face| fingerprint(model, face))
        .collect::<Vec<_>>();

    let mut work = model.clone();
    let inner = work.transaction(|edit| {
        let copy = super::operations::copy_solid(edit, solid)?;
        let distances = edit
            .model()
            .solid_unchecked(copy)
            .faces()
            .iter()
            .map(|face| face.key())
            .collect::<BTreeSet<_>>()
            .into_iter()
            .map(|face| {
                let print = fingerprint(edit.model(), face);
                let distance = if open_fingerprints.contains(&print) {
                    thickness
                } else {
                    -thickness
                };
                (face, face_sign(edit.model(), face), distance)
            })
            .collect::<Vec<_>>();
        plan_faces(edit.model(), &distances)?.apply(edit);
        Ok::<_, OffsetBuildError>(copy)
    })?;
    let cut = boolean(
        &mut work,
        solid,
        &[inner],
        BooleanOperation::Difference,
        BooleanOptions::default(),
    )?;
    *model = work;
    let mut shelling = SolidShelling {
        solid: cut.solid,
        revision: None,
    };
    shelling.stamp(model.revision());
    Ok(shelling)
}

/// What tells a face from its copy's siblings: its support and the middles of
/// its pcurves, rounded so a copy's bit-identical values compare equal.
fn fingerprint<P: Payload>(model: &Model<P>, face: FaceKey) -> (String, Vec<(i64, i64)>) {
    let attr = model.face_attr_unchecked(face);
    let mut middles = attr
        .pcurves
        .values()
        .map(|pcurve| {
            let middle: Point2 = pcurve.point_at(Fraction::new(0.5));
            (
                (middle.x * 1.0e9).round() as i64,
                (middle.y * 1.0e9).round() as i64,
            )
        })
        .collect::<Vec<_>>();
    middles.sort_unstable();
    (format!("{:?}", attr.surface), middles)
}
