//! Owned-shape healing operations.

use crate::healing::{
    HealingError, HealingOptions, HealingReport, HealingScope, remove_redundant_cells,
};
use crate::topology::payload::Payload;
use crate::topology::shape::{Shape, SolidTag};

/// An owned solid and the report from its healing pass.
pub struct HealedSolid<P: Payload> {
    pub shape: Shape<SolidTag, P>,
    pub report: HealingReport,
}

/// Removes redundant topology from a solid in one atomic edit.
pub fn solid<P: Payload>(
    mut shape: Shape<SolidTag, P>,
    mut options: HealingOptions,
) -> Result<HealedSolid<P>, HealingError> {
    options.scope = HealingScope::Solid(shape.key());
    let report = remove_redundant_cells(shape.model_mut(), options)?;
    Ok(HealedSolid { shape, report })
}
