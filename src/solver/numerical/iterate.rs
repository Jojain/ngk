use super::{Evaluation, Factor};
use crate::solver::SolveOptions;
use nalgebra::DVector;

/// What a solver hands the numerical core: a domain for its unknowns and the rows
/// they must satisfy.
pub(crate) trait Problem {
    /// What a group of permanent rows comes from.
    type Key;
    /// The length that linear rows are measured in units of.
    fn scale(&self) -> f64;
    /// Rejects unknowns outside the supported domain before relations are evaluated.
    fn valid(&self, x: &DVector<f64>) -> bool;
    /// What a configuration whose permanent rows hold must also satisfy to be accepted.
    fn admissible(&self, x: &DVector<f64>) -> bool;
    /// The relations that must hold, grouped by what emitted them.
    fn permanent(&self, x: &DVector<f64>) -> Evaluation<Self::Key>;
    /// The temporary objectives, pursued only along the permanent rows' null space.
    fn objectives(&self, x: &DVector<f64>) -> Evaluation<Self::Key>;
}

/// A configuration reached by [`restore`], with its permanent evaluation.
pub(crate) struct Attempt<K> {
    pub x: DVector<f64>,
    pub evaluation: Evaluation<K>,
    pub iterations: usize,
    pub feasible: bool,
}

/// Damped minimum-norm corrections with adaptive step bounds and domain rejection.
pub(crate) fn restore<P: Problem>(
    problem: &P,
    mut x: DVector<f64>,
    options: &SolveOptions,
) -> Attempt<P::Key> {
    let mut evaluation = problem.permanent(&x);
    let mut damping = 1e-6_f64;
    let mut radius = 1_f64;
    let mut iterations = 0;
    for _ in 0..options.max_iterations {
        if !problem.valid(&x) || !evaluation.finite() {
            break;
        }
        if evaluation.feasible(problem.scale(), options) {
            break;
        }
        iterations += 1;
        let factor = Factor::new(&evaluation.jacobian, options.rank_tolerance);
        let mut step = factor.correction(&(-&evaluation.values), damping);
        let norm = step.norm();
        if !norm.is_finite() || norm < 1e-15 {
            break;
        }
        if norm > radius {
            step *= radius / norm;
        }
        let trial = &x + step;
        if !problem.valid(&trial) {
            damping = (damping * 10.).min(1e12);
            radius *= 0.5;
            continue;
        }
        let next = problem.permanent(&trial);
        if next.finite() && next.values.norm() < evaluation.values.norm() {
            x = trial;
            evaluation = next;
            damping = (damping * 0.2).max(1e-14);
            radius = (radius * 2.).min(1e6);
        } else {
            damping = (damping * 10.).min(1e12);
            radius *= 0.5;
        }
    }
    let feasible = problem.valid(&x)
        && evaluation.feasible(problem.scale(), options)
        && problem.admissible(&x);
    Attempt {
        x,
        evaluation,
        iterations,
        feasible,
    }
}

/// The share of its predicted decrease a target step must deliver to be taken.
///
/// With an unreachable target the residual does not vanish, so the linear model
/// overshoots along a curved constraint: dragged at twice a segment's length, a full
/// step lands on the mirror angle. Taking any decrease would then accept each jump
/// across the optimum for a negligible gain; a shorter step lands near it instead.
const SUFFICIENT_DECREASE: f64 = 0.25;

/// The work [`improve_targets`] spent.
#[derive(Default)]
pub(crate) struct TargetWork {
    /// Iterations of the restorations after each trial step.
    pub restore_iterations: usize,
    pub iterations: usize,
    pub budget_exhausted: bool,
}

/// Projects the objectives onto the permanent Jacobian's null space, then restores
/// feasibility after each step.
pub(crate) fn improve_targets<P: Problem>(
    problem: &P,
    best: &mut Attempt<P::Key>,
    options: &SolveOptions,
) -> TargetWork {
    let mut work = TargetWork::default();
    let mut radius = 1_f64;
    for iteration in 0..options.max_target_iterations {
        let soft = problem.objectives(&best.x);
        if !soft.finite() {
            return work;
        }
        let factor = Factor::new(&best.evaluation.jacobian, options.rank_tolerance);
        let mut projected = soft.jacobian.clone();
        for row in 0..projected.nrows() {
            let v = projected.row(row).transpose();
            let v = factor.null_projection(&v);
            projected.row_mut(row).copy_from(&v.transpose());
        }
        let target_factor = Factor::new(&projected, options.rank_tolerance);
        let mut step = factor.null_projection(&target_factor.correction(&(-&soft.values), 1e-10));
        let norm = step.norm();
        if !norm.is_finite() || norm < 1e-11 {
            return work;
        }
        if norm > radius {
            step *= radius / norm;
        }
        let old_error = soft.values.norm();
        let change = &soft.jacobian * &step;
        let mut accepted = false;
        let mut fraction = 1_f64;
        for _ in 0..16 {
            let trial = &best.x + &step * fraction;
            if problem.valid(&trial) {
                let restored = restore(problem, trial, options);
                work.restore_iterations += restored.iterations;
                if restored.feasible {
                    let error = problem.objectives(&restored.x).values.norm();
                    let predicted =
                        old_error.powi(2) - (&soft.values + &change * fraction).norm_squared();
                    if error < old_error - 1e-13 * old_error.max(1e-6)
                        && old_error.powi(2) - error.powi(2) >= SUFFICIENT_DECREASE * predicted
                    {
                        *best = restored;
                        accepted = true;
                        break;
                    }
                }
            }
            fraction *= 0.5;
        }
        work.iterations += 1;
        if !accepted {
            return work;
        }
        radius = (radius * 1.5).min(1e6);
        if iteration + 1 == options.max_target_iterations {
            work.budget_exhausted = true;
        }
    }
    work
}
