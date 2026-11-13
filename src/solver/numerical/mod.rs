//! The numerical core every solver shares: differentiable expressions, the residual
//! rows they emit, damped minimum-norm corrections, objectives projected onto the
//! permanent relations' null space, and the diagnostics read off the Jacobian.
//!
//! A solver supplies a [`Problem`]: its unknowns' domain and its permanent and
//! temporary rows. Normalizing those unknowns, choosing them, and reading results
//! back stay with the solver.

mod evaluation;
mod factor;
mod iterate;
mod jet;

pub(crate) use evaluation::{Evaluation, Row, dependent_rows, residuals};
pub(crate) use factor::Factor;
pub(crate) use iterate::{Problem, improve_targets, restore};
pub(crate) use jet::Jet;
