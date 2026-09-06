//! Shape healing: removing topology that carries no shape.
//!
//! Boolean evaluation, and any operation that imprints one shape onto another,
//! leaves cells that are combinatorially valid but semantically redundant — an
//! edge split into halves of one curve, or a face split into fragments of one
//! surface. This module removes them.
//!
//! There is only one operation underneath. Damiand & Lienhardt's `i`-removal
//! deletes an `i`-cell and merges the two `(i + 1)`-cells that were incident to
//! it, so *fusing edges* is what 0-removal does to a vertex and *fusing faces*
//! is what 1-removal does to an edge. Healing is therefore two passes over the
//! one primitive in [`crate::builders::removal`]:
//!
//! 1. remove edges that separate two faces on one support surface, which fuses
//!    those faces, including closed interfaces where an island fills an inner
//!    loop when [`HealingOptions::remove_filled_inner_loops`] is enabled;
//! 2. remove vertices where two edges continue each other on one support
//!    curve, which fuses those edges.
//!
//! Edges go first, because fusing faces can leave a vertex with only two edges.
//! The two passes then repeat until nothing changes. Every accepted removal
//! deletes one cell of each of two adjacent dimensions, so `V - E + F` is
//! unchanged and the run is guaranteed to terminate.
//!
//! The whole run is one transaction, and every candidate is proposed only after
//! its geometry has been rebuilt successfully, so a model that cannot be healed
//! is left exactly as it was.
//!
//! # Current limitations
//!
//! - Face fusion needs the two faces to be coplanar, or to share one surface
//!   value; a reparameterized curved pair is reported as skipped.
//! - An edge the same face bounds on both sides is removed only when the
//!   boundary rejoins into a single loop. One that would fall into two — a
//!   cylinder's seam, an annulus closing up — is reported as skipped, because
//!   which of the two then bounds the face from outside is not a combinatorial
//!   question. A seam on a periodic surface is refused outright.
//! - Edge fusion rebuilds lines and arcs; a free-form pair is reported as
//!   skipped rather than approximated.
//!
//! Every skip is recorded in [`HealingReport::skipped`] with its reason, which
//! is the first thing to read when a model still looks redundant.

mod errors;
mod options;
mod passes;
pub mod predicates;
mod report;

pub use errors::HealingError;
pub use options::{HealingOptions, HealingScope};
pub use report::{HealedCell, HealingReport, HealingSkip, SkipReason};

use crate::topology::TopologyEdit;
use crate::topology::gmap::GMap;
use crate::topology::payload::Payload;

/// Removes every cell in scope whose removal does not change the shape.
///
/// The run is atomic: a failure restores the map exactly as it was.
pub fn remove_redundant_cells<P: Payload>(
    g: &mut GMap<P>,
    options: HealingOptions,
) -> Result<HealingReport, HealingError> {
    g.transaction(|edit| remove_redundant_cells_staged(edit, &options))
}

/// Runs the healing passes inside an operation's own transaction.
///
/// Use this from a builder that already knows which cells it created, so the
/// run stays proportional to the edit instead of to the model.
pub fn remove_redundant_cells_staged<P: Payload>(
    g: &mut TopologyEdit<'_, P>,
    options: &HealingOptions,
) -> Result<HealingReport, HealingError> {
    let mut report = HealingReport::default();
    for iteration in 1..=options.max_iterations {
        report.iterations = iteration;
        let before = report.changes();
        report.skipped.clear();
        if options.remove_redundant_edges {
            passes::edges::run(g, options, &mut report)?;
        }
        if options.remove_redundant_vertices {
            passes::vertices::run(g, options, &mut report)?;
        }
        if report.changes() == before {
            return Ok(report);
        }
    }
    Err(HealingError::NoConvergence {
        iterations: options.max_iterations,
    })
}
