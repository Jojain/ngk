//! One blend call: resolve, plan, execute, check.

use super::check::{check_faces, check_solids, record_windings};
use super::corner::plan_corners;
use super::errors::BlendError;
use super::execute::execute;
use super::law::BlendLaw;
use super::solid::plan_solid;
use super::surgery::Surgery;
use super::target::{BlendTarget, resolve};
use crate::topology::ModelEdit;
use crate::topology::payload::Payload;
use crate::topology::shape_keys::{EdgeKey, FaceKey};

/// What one blend added and consumed.
#[derive(Debug, Default)]
pub(crate) struct BlendOutcome {
    /// Every face the blend added: blend faces, corner patches and cut faces.
    pub(crate) faces: Vec<FaceKey>,
    /// Every selected edge the blend replaced. These keys no longer exist.
    pub(crate) consumed_edges: Vec<EdgeKey>,
}

/// Blends `target` under `law` inside an open edit.
///
/// Everything is planned against the model as the call found it and built in
/// one surgery, so the result does not depend on the order the target lists
/// its selections in.
pub(crate) fn blend_edit<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    target: BlendTarget,
    law: BlendLaw,
) -> Result<BlendOutcome, BlendError> {
    law.validate()?;
    let resolution = resolve(edit.model(), &target, law)?;
    if resolution.is_empty() {
        return Err(BlendError::EmptyTarget);
    }

    let mut surgery = Surgery::default();
    plan_corners(edit.model(), &resolution.corners, law, &mut surgery)?;
    plan_solid(edit.model(), &resolution, law, &mut surgery)?;

    let windings = record_windings(edit.model(), &surgery);
    let executed = execute(edit, &surgery)?;
    check_faces(edit.model(), &executed, &windings)?;
    check_solids(edit.model(), &executed)?;
    Ok(BlendOutcome {
        faces: executed.faces,
        consumed_edges: surgery.cuts.iter().map(|cut| cut.edge).collect(),
    })
}
