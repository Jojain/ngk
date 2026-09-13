//! The seam pass: dropping the cut a periodic parameterization was opened along.
//!
//! A seam is not an edge of the shape. A cylinder's lateral surface closes in
//! `u`, and a representation that insists a face boundary be a closed loop in
//! the *planar* parameter domain has to cut it open somewhere and walk the cut
//! twice. Where that cut lands is arbitrary — rotate the cylinder about its own
//! axis and it moves — so a model carrying one is a model whose topology depends
//! on its parameterization.
//!
//! No builder in this tree makes a seam. STEP AP242 and every other B-Rep
//! interchange format does, so this pass is what an import runs: it is the same
//! 1-removal the edge pass uses, asked for on different grounds. The edge pass
//! removes an edge because the faces on either side turn out to be one face;
//! this one removes an edge because it was never a boundary, and what is left is
//! the face the parameterization was hiding — a ring, a cap, or a face with no
//! boundary at all, depending on what closes the direction the seam ran across.
//!
//! Both passes decide by asking [`crate::builders::removal::planned_merge`] what
//! the removal would do, so an edge belongs to exactly one of them and neither
//! has to guess.

use crate::topology::ModelEdit;
use crate::topology::payload::Payload;

use super::super::errors::HealingError;
use super::super::options::HealingOptions;
use super::super::report::HealingReport;

/// Offers every scoped seam edge to the 1-removal operation.
pub(in crate::healing) fn run<P: Payload>(
    edit: &mut ModelEdit<'_, P>,
    options: &HealingOptions,
    report: &mut HealingReport,
) -> Result<(), HealingError> {
    super::edges::run_over(edit, options, report, |kind| kind.is_seam_removal())
}
